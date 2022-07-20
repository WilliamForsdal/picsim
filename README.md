# picsim
## PIC12F508 Simulator in rust

Implements a PIC12F508 microcontroller. Programs can be loaded from 
hex-files (generated by pic-as.exe) or written directly in rust. Instructions
can be encoded to 12-bit words, and decoded back to opcodes.

## Not implemented:
* WD timer with timeouts
* SLEEP and wake from sleep
* TMR0 in counter mode
* config word settings