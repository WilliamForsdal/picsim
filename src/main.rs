
#![feature(test)]

pub mod cpu;
pub mod opcode;

use crate::cpu::CPU;
use std::fs;

fn main() {
    let contents = fs::read_to_string("obj/main2.hex").unwrap();
    let mut cpu: CPU = CPU::from_hex(&contents);
    println!("w:{}, pc:{:3}, z:{}, c:{}", cpu.w, cpu.pc, cpu.status_z(), cpu.status_c());
    for _ in 0..10000 {
        cpu.tick();
        println!("pc:{:3} prev: {:?}, next: {:?}", cpu.pc, cpu.last_executed_op, cpu.next_op);
    }
    cpu.run(2); // timer starts now
    for t in 0..1000000 {
        let prev_gpio = cpu.gpio;
        cpu.tick();
        if cpu.gpio != prev_gpio {
            println!("{t:07}: gpio 0b{:08b}", cpu.gpio);
        }
    }
}