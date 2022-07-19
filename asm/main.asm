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
    nop         
    goto        main_loop

end reset_vector
