/* H723XG: keep CPU/RTOS/DMA objects in DMA1/2-accessible AXI SRAM.
 * D-cache remains disabled in this integration board. No DTCM DMA buffers.
 * MSP, newlib heap and FreeRTOS heap all have explicit RAM accounting. */
ENTRY(Reset_Handler)
MEMORY {
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM (xrw) : ORIGIN = 0x24000000, LENGTH = 320K
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Stack_Size = 16K;
_Min_Heap_Size = 8K;
PHDRS {
  flash PT_LOAD FLAGS(5);
  ram PT_LOAD FLAGS(6);
}
SECTIONS {
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) } > FLASH :flash
  .text : {
    . = ALIGN(4); *(.text .text.*) *(.rodata .rodata.*)
    KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); _etext = .;
  } > FLASH :flash
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH :flash
  .ARM.exidx : {
    __exidx_start = .; *(.ARM.exidx*) __exidx_end = .;
  } > FLASH :flash
  .preinit_array : {
    PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH :flash
  .init_array : {
    PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH :flash
  .fini_array : {
    PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > FLASH :flash
  _sidata = LOADADDR(.data);
  .data : {
    . = ALIGN(32); _sdata = .; *(.data .data.*) . = ALIGN(32); _edata = .;
  } > RAM AT> FLASH :ram
  .bss (NOLOAD) : {
    . = ALIGN(32); _sbss = .; *(.bss .bss.*) *(COMMON) . = ALIGN(32); _ebss = .;
    /* Reserve newlib heap/MSP capacity after the startup zeroing range. */
    . = ALIGN(8); _end = .; PROVIDE(end = .);
    . += _Min_Heap_Size; . += _Min_Stack_Size;
  } > RAM :ram
  .ARM.attributes 0 : { *(.ARM.attributes) }
}
ASSERT(_ebss + _Min_Heap_Size + _Min_Stack_Size <= _estack, "AXI SRAM overflow")
