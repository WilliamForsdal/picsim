processor 12F508

; inc files:
; C:\Program Files\Microchip\xc8\v2.32\pic\include

#include <xc.inc>

CONFIG  OSC = IntRC           ; Oscillator Selection bits (internal RC oscillator)
CONFIG  WDT = OFF             ; Watchdog Timer Enable bit (WDT disabled)
CONFIG  CP = OFF              ; Code Protection bit (Code protection off)
CONFIG  MCLRE = OFF           ; GP3/MCLR Pin Function Select bit (GP3/MCLR pin function is MCLR)

; Not sure how exactly to specify program memory etc.
psect code
reset_vector:
    goto main

main:

main_loop:
    movlw       0xaa
    movwf       0x10
    addwf       0x12, f
    andwf       0x13, w
    clrf        0x13
    clrw
    comf        0x14, f
    decf        0x10, f
    decfsz      0x10, f
    incf        0x10, f
    incfsz      0x10, f
    iorwf       0x10, f
    movf        0x10, w
    nop
    rlf         0x10, f
    rrf         0x10, f
    subwf       0x10, f
    swapf       0x10, f
    xorwf       0x10, f

    bsf         0x10, 2
    bcf         0x10, 2
    btfsc       0x10, 2
    bcf         0x10, 2
    btfss       0x10, 2

    andlw       0xff
    call        func
    ;clrwdt ; TODO
    movlw       0xaa
    iorlw       0x55
    ; option TODO
    xorlw       0x55
    ; sleep TODO

    tris        GPIO
    goto        main_loop

func:
    retlw       5

end reset_vector
