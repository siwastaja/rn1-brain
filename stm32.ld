MEMORY
{
  ram (rwx)        : ORIGIN = 0x20000000, LENGTH = 128K
  stack(rwx)       : ORIGIN = 0x2001FFF8, LENGTH = 0K

  sector0 (rx)     : ORIGIN = 0x00000000, LENGTH = 16K   /* Sector 0: Vector table required here, but could put other stuff too */
  sector1 (rx)     : ORIGIN = 0x00004000, LENGTH = 16K   /* Sector 1:  */
  sector2 (rx)     : ORIGIN = 0x00008000, LENGTH = 16K   /* Sector 2:  */
  sector3 (rx)     : ORIGIN = 0x0000C000, LENGTH = 16K   /* Sector 3:  */

  sector4 (rx)     : ORIGIN = 0x00010000, LENGTH = 64K   /* Sector 4:  */

  sector5 (rx)     : ORIGIN = 0x00020000, LENGTH = 128K  /* Sector 5: .text, .data, etc. */
  sector6 (rx)     : ORIGIN = 0x00040000, LENGTH = 128K  /* Sector 6: Includes flasher program etc., cannot be rewritten in-application (STM32 bootloader must be used)*/
}

SECTIONS
{
    .nvic_vector : 
    {
        *(vectors)    /* Vector table */
    } >sector0

    . = ALIGN(8);

    .text : 
    {
        *(.text)      /* Program code */
        *(.text.*)
        *(.rodata)    /* Read only data */
        *(.rodata.*)
        *(.sin_lut)
        *(.sin_lut.*)
    } >sector5

    . = ALIGN(8);

    _DATAI_BEGIN = LOADADDR(.data);

    .data : 
    {
        _DATA_BEGIN = .;
        *(.data)      /* Data memory */
        *(.data.*)        
        _DATA_END = .;
    } >ram AT>sector5

    . = ALIGN(8);

    .bss :
    {
        _BSS_BEGIN = .;
        *(.bss)
        *(COMMON)        
        _BSS_END = .;
    } >ram 

    . = ALIGN(8);


    _SETTINGSI_BEGIN = LOADADDR(.settings);

    .settings :
    {
        _SETTINGS_BEGIN = .;
        *(.settings)
        *(.settings.*)
        _SETTINGS_END = .;
    } >ram AT>sector1

    . = ALIGN(8);


    .heap :
    {
        _HEAP = .;
    } >ram

    .stack :
    {
        _STACKTOP = .;
    } >stack


    . = ALIGN(8);

    .flasher : 
    {
        *(.flasher)
        *(.flasher.*)
    } >sector6

    . = ALIGN(8);

}  
