idf_component_register(SRCS "flash_chip.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES spi_flash
                       LDFRAGMENTS linker.lf)
# spi_flash references default_registered_chips from here
idf_component_add_link_dependency(FROM spi_flash)
