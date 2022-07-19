
#![feature(test)]

pub mod cpu;
pub mod opcode;

use crate::cpu::CPU;
use crate::opcode::OpCode;
use std::fs;

fn main() {

    for i in 0..u16::MAX {
        let code = OpCode::decode(i);
        println!("{code:?}");
    }

    let contents = fs::read_to_string("obj/main.hex").unwrap();
    let mut cpu: CPU = CPU::from_hex(&contents);
    println!("w:{}, pc:{:3}, z:{}, c:{}", cpu.w, cpu.pc, cpu.status_z(), cpu.status_c());
    for _ in 0..100000 {
        cpu.tick();
    }
}