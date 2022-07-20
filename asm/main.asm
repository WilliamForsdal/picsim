processor 12F508

#include <xc.inc>

CONFIG  OSC = IntRC        
CONFIG  WDT = OFF
CONFIG  CP = OFF  
CONFIG  MCLRE = OFF

psect code
reset_vector:
    goto main

main:
    movlw   0b11010111
    option
    movlw   0b111110
    tris    GPIO

    movlw   1
    movwf   TMR0
    
    goto    main_loop

main_loop:
    movf    TMR0, W
    btfsc   STATUS, 2 ; if overflowed this cycle
    goto    led_on
    goto    led_off

led_on:
    bsf     GPIO, 0
    goto    loop_end

led_off:
    bcf     GPIO, 0
    goto    loop_end
    
loop_end:
    goto    main_loop


end reset_vector
