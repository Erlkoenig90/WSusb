MEMORY {
    FLASH    : ORIGIN = 0x8000000,    LENGTH = 128K
    SRAM    : ORIGIN = 0x20000000,    LENGTH =  20K
}

ENTRY(Reset_Handler)

SECTIONS {
    .VectorTable : {
        KEEP(*(.VectorTable))
    } >FLASH
    
	.text : {
		*(.text)
		. = ALIGN(4);

		_InitArrayStart = .;
		*(SORT(.preinit_array*))
		*(SORT(.init_array*))
		_InitArrayEnd = .;
	} >FLASH

    
    .stack (NOLOAD) : {
        . = . + 0x400;
        _StackEnd = .;
    } >SRAM
    
    .data : {
        _DataStart = .;
        *(.data);
        . = ALIGN(4);
        _DataEnd = .;
    } >SRAM AT >FLASH
    
    _DataLoad = LOADADDR(.data);
    
    .bss : {
        _BssStart = .;
        *(.bss);
        . = ALIGN(4);
        _BssEnd = .;
    } >SRAM
}

