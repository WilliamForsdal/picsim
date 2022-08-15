
use picsim::{pic12f508::*, opcode::OpCode};

fn main() {
    let mut cpu: Pic12F508 = Pic12F508::from_ops(vec![OpCode::GOTO { k: 0 }, ]);
    cpu.tick();
}