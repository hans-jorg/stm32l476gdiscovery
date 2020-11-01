/**
 * @file stm32l476.ld
 *
 * @note script for linker
 */

/**
 * @brief Memory layout for the STM32L476VG
 */

MEMORY
{
    FLASH (rx)   : ORIGIN = 0x00000000, LENGTH = 1024K
    SRAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 96K
    CCMRAM (rwx) : ORIGIN = 0x10000000, LENGTH = 32K
}

STACK_SIZE   = 4K;
STACK_BASE   = ORIGIN(SRAM) + LENGTH(SRAM) - STACK_SIZE;
STACK_END    = ORIGIN(SRAM) + LENGTH(SRAM) - 4;
HEAP_SIZE    = 0x400;
_stack_start = STACK_BASE;
_stack_end   = STACK_END; /* Initial value */
_stack_init  = STACK_END;

/**
 * Sections used in C code
 * .text  - instructions
 * .rodata- constants
 * .data  - initialized data
 * .bss   - noninitialized data
 * .stack - pointer to highest address of RAM
 */

SECTIONS
{
    /**
     * Stored in  Flash starting at 0
     * 1) Interrupt table
     *    But @0 :  Initial value for SP 
     *        @4 :  Initial value for PC
     * 2) Code (Instructions)
     * 3) Constants
     * 4) Initial values for initialized data
     */
  _text       = .;
  _text_start = .; /* endereco do inicio (=0) */
    .text :
    {

     KEEP(*(.isr_vector))  /* interrupt table is in this section  */
          .           = ALIGN(4);
          *(.text*)       /* instructions follows */
          .           = ALIGN(4);
          *(.rodata*)     /* constants follows */
          .           = ALIGN(4);

    } > FLASH              /* All in flash */
  _text_end   = .; /* end of flash used */
  _etext      = .;
    /**
     * Inicialized data are in RAM but a copy of initial values must be
     *   in flash and copied to RAM after reset
     *
     * Start of this area in flash : _etext
     * Start of this area in RAM   : _data
     */

    .data : /*AT(_etext) */
    {
          .           = ALIGN(4);
          _data       = .;
          _data_start = .;              /* start address */
          *(.data*);                    /* data section stored here */
          .           = ALIGN(4);
          *(vtable)                     /* vtables (C++) */
          _data_end   = .;              /* end of initialized data */

          _edata      = .;

    } > SRAM  AT>FLASH      /* A copy on flash */

    /**
     * Inicialized data stored in RAM.
     * Must be zeroed after reset
     */
    .bss :
    {
          .           = ALIGN(4);
        _bss          = .;
        _bss_start    = .; /* start of non initialized area */
        *(.bss.*)          /* non initilized data store here */
        *(COMMON)          /* Fortran? */
        _bss_end =      .; /* end address */

        _ebss         = .;
        HEAP_START    = .;
        _end          = .;
    } > SRAM

    /*
     * Stack area: just initilize pointer
     */
    .stack :
    {

    } > SRAM

}

