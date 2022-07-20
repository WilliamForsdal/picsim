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
    movlw   0b11011000
    option
    goto    main_loop

main_loop:
    call    m
    retlw   0xaa
    
loop_end:
    goto    main_loop

m:
    call n
    retlw   1

n:
    retlw   2


end reset_vector