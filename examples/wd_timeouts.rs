
use picsim::{pic12f508::*, opcode::OpCode};

use std::time::{Instant};


fn main() {
    let mut cpu: Pic12F508 = Pic12F508::from_ops_and_cfg(vec![
        OpCode::NOP,
    ],
    ConfigWord { mclre: true, cp_disable: true, wdte: true, fosc: ClockSource::INTRC });

    let mut ticks: u128 = 0;
    let start = Instant::now();
    loop {
        let now = Instant::now();
        if now < start {
            continue;
        }

        let elapsed = now - start;
        let ticks_this_iteration = elapsed.as_micros() - ticks; 
        ticks += ticks_this_iteration;
        cpu.run(ticks_this_iteration as u32);
    }
}