use picsim::{pic12f508::Pic12F508, opcode::OpCode};


fn main() {
    let mut cpu: Pic12F508 = Pic12F508::from_ops(
        vec![
            OpCode::GOTO { k: 10 },
            OpCode::NOP,
            OpCode::NOP,
            OpCode::NOP,
            OpCode::NOP,
            OpCode::NOP,
            OpCode::NOP,
            OpCode::NOP,
            OpCode::NOP,
            OpCode::MOVLW { k: 0xaa },
            OpCode::MOVWF { f: 0x10 },
            
            
        ]
    );
    println!("w:{}, pc:{:3}, z:{}, c:{}", cpu.w, cpu.pc, cpu.status_z(), cpu.status_c());
    for _ in 0..100 {
        cpu.tick();
        println!("pc:{:3} prev: {:?}, next: {:?}", cpu.pc, cpu.last_executed_op, cpu.next_op);
    }
}