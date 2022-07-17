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
    movlw       10
    call        func
    goto        main_loop

func:
    retlw       5

end reset_vector
