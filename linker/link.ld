/*=============================================================================
  centipede_robot/linker/link.ld
  Linker Script for Centipede Robot Firmware
  =============================================================================
  
  Memory Layout:
  - .text:   0x1000 - Code segment
  - .data:   0x8000 - Initialized data
  - .bss:    0x9000 - Uninitialized data
  - Stack:   0xA000 - Stack space
  
  Total ROM usage target: < 32KB
  Total RAM usage target: < 8KB
  ============================================================================*/

MEMORY
{
    ROM (rx)  : ORIGIN = 0x00001000, LENGTH = 32K
    RAM (rwx) : ORIGIN = 0x00009000, LENGTH = 8K
}

SECTIONS
{
    /* Code section */
    . = 0x1000;
    
    .text : {
        *(.text)
        *(.text.*)
        . = ALIGN(16);
    } > ROM

    /* Read-only data */
    .rodata : {
        *(.rodata)
        *(.rodata.*)
        . = ALIGN(16);
    } > ROM

    /* Initialized data (copied to RAM at startup) */
    .data : {
        _data_start = .;
        *(.data)
        *(.data.*)
        . = ALIGN(16);
        _data_end = .;
    } > RAM AT > ROM

    /* Uninitialized data (zeroed at startup) */
    .bss : {
        _bss_start = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(16);
        _bss_end = .;
    } > RAM

    /* Stack */
    .stack : {
        . = ALIGN(4);
        _stack_start = .;
        . = . + 0x0400;      /* 1KB stack */
        _stack_end = .;
        . = ALIGN(4);
    } > RAM

    /* Heap (optional) */
    .heap : {
        _heap_start = .;
        . = . + 0x0400;      /* 1KB heap */
        _heap_end = .;
    } > RAM

    /* Symbol table for debug */
    _rom_start = ORIGIN(ROM);
    _rom_end   = ORIGIN(ROM) + LENGTH(ROM);
    _ram_start = ORIGIN(RAM);
    _ram_end   = ORIGIN(RAM) + LENGTH(RAM);
}

