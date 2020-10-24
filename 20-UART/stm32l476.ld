/*
 * Definicao de memoria para STM32L476VG
 * FLASH: start address 0x00000000 length 0x100000 ( 1024 KB)
 * SRAM:  start address 0x20000000 length 0x18000  (   96 KB)
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
_stack_end   = STACK_END; /* Valor inicial */
_stack_init  = STACK_END;

/*
 * Definição das secoes para C (para C++ existem secoes adicionais)
 * .text  - instrucoes
 * .data  - dados inicializados
 * .bss   - dados não inicializados
 * .stack - ponteiro para inicio da RAM (endereco mais alto da RAM)
 */

SECTIONS
{
    /*
     * Na Flash
     * 1) Tabela de excecoes/interrupcoes
     *      Armazenada na Flash a partir do endereco 0
     *      Endereco 0: valor inicial do SP
     *      Endereco 1: valor inicial do PC
     * 2) Codigo (Instrucoes)
     * 3) Constantes
     * 4) Valores iniciais das variaveis globais
     */
  _text       = .;
  _text_start = .; /* endereco do inicio (=0) */
    .text :
    {

     KEEP(*(.isr_vector))  /* tabela sera configurada para ficar nesta secao */
          .           = ALIGN(4);
          *(.text*)       /* instrucoes logo a seguir */
          .           = ALIGN(4);
          *(.rodata*)     /* constantes logo a seguir */
          .           = ALIGN(4);

    } > FLASH              /* tudo armazenado na memoria flash */
  _text_end   = .; /* o endereco deste simbolo e o final do codigo */
  _etext      = .;
    /*
     * Dados inicializados devem ser ficar na RAM, mas o conteudo inicial
     * deve ficar na memoria flash
     * AT(ADDR(.text) + SIZEOF(.text))
     */

    .data : /*AT(_etext) */
    {
          .           = ALIGN(4);
          _data       = .;
          _data_start = .; /* endereco do inicio */
          *(.data*)      /* dados */
          .           = ALIGN(4);
          *(vtable)        /* vtables gerados pelo compilador C++ */
          _data_end   = .; /* endereco do final */
          _edata      = .;

    } > SRAM  AT>FLASH      /* configurado para RAM mas armazenado na flash */

    /*
     * Dados não inicializados ficam na RAM.
     * Conteudo inicial deve ser zerado na inicializacao
     */


    .bss :
    {
          .           = ALIGN(4);
        _bss          = .;
        _bss_start    = .; /* endereco do inicio */
        *(.bss.*)          /* dados nao inicializados */
        *(COMMON)          /* etc */
        _bss_end =      .; /* endereco do final */
        _ebss         = .;
        HEAP_START = .;
    } > SRAM

    /*
     * Definicao da pilha consiste do valor inicial do SP
     */
    .stack :
    {

    } > SRAM

}
