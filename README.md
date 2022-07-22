# picsim
## PIC12F508 Simulator in rust

`cargo run --example from_hex_file`

Implements an 8-bit PIC12F508 microcontroller. Programs can be loaded from 
hex-files (compiled by the mplab pic-as.exe compiler) or written directly as opcodes in rust. 
Instructions in the 12F508 are 12 bits wide. These can be encoded and decoded by the `OpCode` enum
associated functions.

## Implemented:
* All instructions (except `SLEEP` and `CLRWDT`) of the pic baseline instruction set (33 instructions)
* Instruction encoding (to 12-bit words) and decoding (to structs in rust)
* Program branches and regular linear execution
* Branches 2-cycle delay
* 2 levels deep call stack for `CALL` and `RETLW` instructions
* GPIO input and output
* TMR0 with prescale, and 2-cycle delay after TMR0 reg write
* Indirect adressing via INDF and FSR registers
* STATUS bits affected by arithmetic operations

## Not implemented:
* WD timer with timeouts
* `SLEEP` and wake from sleep
* TMR0 in counter mode
* config word settings

## Fun TODOs
* Small assembly compiler
* GUI