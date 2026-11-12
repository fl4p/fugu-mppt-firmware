// Flash chip driver list for CONFIG_SPI_FLASH_AUTO_SUSPEND builds.
// IDF asserts at boot unless the chip driver reports SPI_FLASH_CHIP_CAP_SUSPEND, and with auto-suspend
// built in it never disables the caches around flash ops. The dispatcher below probes the stock
// drivers, runs the matching one from a RAM copy that reports suspend, and wraps the OS start/end
// hooks so caches are still disabled until fugu_flash_sus_set(true) (after a passed self-test).

#include <string.h>
#include "sdkconfig.h"
#include "esp_attr.h"
#include "fugu_flash_chip.h"

static DRAM_ATTR volatile bool s_sus;

#if CONFIG_SPI_FLASH_OVERRIDE_CHIP_DRIVER_LIST

#include "esp_flash.h"
#if __has_include("esp_flash_chips/spi_flash_chip_driver.h")
#include "esp_flash_chips/spi_flash_chip_driver.h"
#include "esp_flash_chips/spi_flash_chip_generic.h"
#include "esp_flash_chips/spi_flash_chip_issi.h"
#include "esp_flash_chips/spi_flash_chip_mxic.h"
#include "esp_flash_chips/spi_flash_chip_gd.h"
#include "esp_flash_chips/spi_flash_chip_winbond.h"
#include "esp_flash_chips/spi_flash_chip_boya.h"
#include "esp_flash_chips/spi_flash_chip_th.h"
#else // IDF 5.5
#include "spi_flash_chip_driver.h"
#include "spi_flash_chip_generic.h"
#include "spi_flash_chip_issi.h"
#include "spi_flash_chip_mxic.h"
#include "spi_flash_chip_gd.h"
#include "spi_flash_chip_winbond.h"
#include "spi_flash_chip_boya.h"
#include "spi_flash_chip_th.h"
#endif
#include "esp_private/cache_utils.h"

spi_flash_caps_t spi_flash_chip_generic_get_caps(esp_flash_t *chip);

// IDF's default list, same order
static const spi_flash_chip_t *const s_stock[] = {
#ifdef CONFIG_SPI_FLASH_SUPPORT_ISSI_CHIP
    &esp_flash_chip_issi,
#endif
#ifdef CONFIG_SPI_FLASH_SUPPORT_GD_CHIP
    &esp_flash_chip_gd,
#endif
#ifdef CONFIG_SPI_FLASH_SUPPORT_MXIC_CHIP
    &esp_flash_chip_mxic,
#endif
#ifdef CONFIG_SPI_FLASH_SUPPORT_WINBOND_CHIP
    &esp_flash_chip_winbond,
#endif
#ifdef CONFIG_SPI_FLASH_SUPPORT_BOYA_CHIP
    &esp_flash_chip_boya,
#endif
#ifdef CONFIG_SPI_FLASH_SUPPORT_TH_CHIP
    &esp_flash_chip_th,
#endif
#ifdef CONFIG_SPI_FLASH_SUPPORT_MXIC_OPI_CHIP
    &esp_flash_chip_mxic_opi,
#endif
    &esp_flash_chip_generic,
};

static DRAM_ATTR spi_flash_chip_t s_drv;
static DRAM_ATTR spi_flash_caps_t (*s_caps)(esp_flash_t *);
static DRAM_ATTR esp_flash_os_functions_t s_os;
static DRAM_ATTR const esp_flash_os_functions_t *s_orig;
static DRAM_ATTR bool s_opOff; // the running op disabled the caches; ops are serialized by s_orig->start

static esp_err_t wStart(void *arg, uint32_t flags) {
    esp_err_t e = s_orig->start(arg, flags);
    s_opOff = !s_sus;
    if (s_opOff) spi_flash_disable_interrupts_caches_and_other_cpu();
    return e;
}

static esp_err_t wEnd(void *arg) {
    if (s_opOff) {
        s_opOff = false;
        spi_flash_enable_interrupts_caches_and_other_cpu();
    }
    return s_orig->end(arg);
}

static spi_flash_caps_t capsSus(esp_flash_t *chip) {
    return (s_caps ? s_caps(chip) : 0) | SPI_FLASH_CHIP_CAP_SUSPEND;
}

// Runs inside IDF's detection start/end; the os_func swap is safe there because wEnd only re-enables
// caches that wStart disabled.
static esp_err_t dispatchProbe(esp_flash_t *chip, uint32_t id) {
    for (size_t i = 0; i < sizeof(s_stock) / sizeof(s_stock[0]); ++i) {
        const spi_flash_chip_t *d = s_stock[i];
        chip->chip_drv = d;
        if (d->probe(chip, id) != ESP_OK) continue;
        memcpy(&s_drv, d, sizeof(s_drv));
        s_caps = d->get_chip_caps;
        s_drv.get_chip_caps = capsSus;
        if (!s_drv.sus_setup) s_drv.sus_setup = spi_flash_chip_generic_suspend_cmd_conf;
        chip->chip_drv = &s_drv;
#if CONFIG_SPI_FLASH_AUTO_SUSPEND
        if (chip->os_func && chip->os_func != &s_os && chip->os_func->start && chip->os_func->end) {
            s_orig = chip->os_func;
            s_os = *s_orig;
            s_os.start = wStart;
            s_os.end = wEnd;
            chip->os_func = &s_os;
        }
#endif
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

static const spi_flash_chip_t s_dispatch = {
    .name = "fugu",
    .probe = dispatchProbe,
};

const spi_flash_chip_t *default_registered_chips[] = {&s_dispatch, NULL};

bool fugu_flash_sus_built(void) {
#if CONFIG_SPI_FLASH_AUTO_SUSPEND
    return s_orig != NULL && esp_flash_default_chip && esp_flash_default_chip->os_func == &s_os;
#else
    return false;
#endif
}

#else

bool fugu_flash_sus_built(void) { return false; }

#endif

void fugu_flash_sus_set(bool on) { s_sus = on && fugu_flash_sus_built(); }

bool fugu_flash_sus_on(void) { return s_sus; }
