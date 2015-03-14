/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00020000, LENGTH = 0x20000 
  RAM (rwx) :  ORIGIN = 0x20002400, LENGTH = 0x1C00
  NRF_UICR_GIT_VERSION (rwx) : ORIGIN = 0x10001080, LENGTH = 0x10
  NRF_UICR_APPLICATION_SETTINGS (rwx) : ORIGIN = 0x10001090, LENGTH = 0x70
}

SECTIONS
{
    .NRF_UICR_GIT_VERSION_BLOCK 0x10001080 :
    {
        KEEP(*(.uicrGitVersion))
    } > NRF_UICR_GIT_VERSION
    
    .NRF_UICR_APPLICATION_SETTINGS_BLOCK 0x10001090 :
    {
        KEEP(*(.uicrApplicationSettings))
    } > NRF_UICR_APPLICATION_SETTINGS
}


INCLUDE "gcc_nrf51_common.ld"