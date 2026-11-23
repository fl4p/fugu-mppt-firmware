# Power-management experiment build: `pwr` knobs + PM/DFS + auto light sleep support.
# idf.py -B build/pm -DSDKCONFIG=build/pm/sdkconfig -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.pm" build
# Boots without DFS or light sleep (esp_pm_configure is never called, PM_DFS_INIT_AUTO off) until `pwr pm ...`.
# Bench only: in light sleep every GPIO switches to its isolated sleep config (ESP_SLEEP_GPIO_RESET_WORKAROUND
# selects PM_SLP_DISABLE_GPIO with tickless idle), gate and enable pins included.
CONFIG_FUGU_WITH_PWR_KNOBS=y
CONFIG_PM_ENABLE=y
CONFIG_FREERTOS_USE_TICKLESS_IDLE=y
CONFIG_FREERTOS_IDLE_TIME_BEFORE_SLEEP=3
CONFIG_BT_CTRL_MAIN_XTAL_PU_DURING_LIGHT_SLEEP=y
CONFIG_ESP_PHY_MAC_BB_PD=y
CONFIG_USJ_NO_AUTO_LS_ON_CONNECTION=y
# esp_pm_dump_locks time-in-mode for `pwr locks` (one esp_timer read per lock edge and mode switch)
CONFIG_PM_PROFILING=y
