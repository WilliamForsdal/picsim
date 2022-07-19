processor 12F508

#include <xc.inc>

CONFIG  OSC = IntRC        
CONFIG  WDT = OFF
CONFIG  CP = OFF  
CONFIG  MCLRE = OFF

psect code
reset_vector:

main:

main_loop:
; 0
    movlw       0xff
    movwf       0x10        ; 0xff
    addwf       0x10, f     ; 0xfe
    andwf       0x10, w     ; 0xfe
    clrf        0x10        ; 0
    comf        0x10, f     ; 0xff
    decf        0x10, f     ; 0xfe
    decfsz      0x10, f     ; 0xfc
; 8
    incf        0x10, f     ; 0xfe
    incfsz      0x10, f     ; 0xff
    iorwf       0x10, f     ; 0xff
    movf        0x11, f     ; 0xff
    rlf         0x10, f     ; 0x7f
    rrf         0x10, f     ; 0x7f probably
    subwf       0x10, f     ; 0x?
    swapf       0x10, f     ; 
; 16
    xorwf       0x10, f
    bsf         0x10, 7
    bcf         0x10, 2
    btfsc       0x10, 2
    btfss       0x10, 7
    andlw       0xaa
    call        func
    clrwdt
; 24
    iorlw       0x55
    option 
    sleep
    tris        6
    xorlw       0x55
    goto        main_loop

; 30
func:
    retlw       0

noop:
    nop         

end reset_vector
