mod cpu;
use crate::cpu::CPU;
use std::fs;

fn main() {
    let contents = fs::read_to_string("obj/main.hex").unwrap();
    let mut cpu: CPU = CPU::new(&contents);
    println!("w:{}, pc:{:3}, z:{}, c:{}", cpu.w, cpu.pc, cpu.status_z(), cpu.status_c());
    for i in 0..10000 {
        cpu.tick();
        println!("{i:03} pc: {:03} W: 0x{:02X} executed: {:?} ", cpu.pc, cpu.w, cpu.prev_op);
    }

}
