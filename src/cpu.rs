use crate::opcode::*;

pub struct CPU {
    // Some instructions take 2 cycles and updates pc on the next cycle
    //pub nop_next: bool,
    pub last_executed_op: Option<OpCode>,
    pub next_op: Option<OpCode>,
    pc_written: bool,

    pub cfg_word: u16,
    pub flash: [u16; 512], // actually 12 bit words

    pub sram: [u8; 32], // lower 6 not used, SRFs

    pub stack: [u16; 2],

    pub w: u8,

    // Remember to set pc_written when writing to PC
    pub pc: u16, // actually 10 bits for real
    pub option: u8,
    pub trisgpio: u8, // not a memory mapped register

    pub input_buffer: u8, // store input data
    pub output_buffer: u8,

    pub indf: u8,
    pub tmr0: u8,
    pub pcl: u8,
    pub status: u8,
    pub fsr: u8,
    pub osccal: u8,
    pub gpio: u8,

    tmr0_state: TMR0,
}

// Store state for tmr0
// When a write occurs to tmr0 reg, the increment
// is inhibited for 2 cycles. Keep track of that here
#[derive(Debug, Default)]
struct TMR0 {
    pub tmr0: u8,
    tmr_prev: u8,
    inc_inhibit: u8,
    prescale_counter: u16,
}

impl TMR0 {
    pub fn update(&mut self, option: u8) {
        let enabled = (option & 0b100000) == 0;
        if !enabled {
            return;
        }

        // writes to TMR0 pause timer for 2 cycles
        if self.tmr_prev != self.tmr0 {
            self.tmr_prev = self.tmr0;
            self.inc_inhibit = 2;
            self.prescale_counter = 0; // prescaler counter is reset on write
            return;
        }
        if self.inc_inhibit > 0 {
            self.inc_inhibit -= 1;
            return;
        }

        let prescale = option & 0b111;
        let prescaler = 1 << (prescale + 1);
        let prescale_enabled = (option & 0b1000) == 0;

        if prescale_enabled {
            self.prescale_counter += 1;
            if self.prescale_counter >= prescaler {
                self.prescale_counter = 0;
                let (t, _overflow) = self.tmr0.overflowing_add(1);
                self.tmr0 = t;
            }
        } else {
            let (t, _overflow) = self.tmr0.overflowing_add(1);
            self.tmr0 = t;
        }
        self.tmr_prev = self.tmr0;
    }
}

const INDF: usize = 0;
const TMR0: usize = 1;
const PCL: usize = 2;
const STATUS: usize = 3;
const FSR: usize = 4;
const OSCCAL: usize = 5;
const GPIO: usize = 6;
const OSC_CALIB_VAL: u8 = 0xb0;

impl CPU {
    pub fn from_hex(program: &str) -> CPU {
        let (flash, cfg_word) = OpCode::from_hex(program);
        CPU::new(flash, cfg_word)
    }

    pub fn from_asm(asm: &str) -> CPU {
        let ops = OpCode::compile(asm);
        CPU::from_ops(ops)
    }

    pub fn from_ops(ops: Vec<OpCode>) -> CPU {
        let mut flash: [u16; 512] = [0; 512];
        for (i, op) in ops.iter().enumerate() {
            flash[i] = op.to_owned().encode();
        }
        CPU::new(flash, 4074)
    }

    fn new(flash: [u16; 512], cfg_word: u16) -> CPU {
        let mut cpu = CPU {
            last_executed_op: None, // for debugging
            next_op: None,          // next fetched instruction
            pc_written: false,

            cfg_word,
            flash,
            sram: [0; 32],
            stack: [0; 2],

            w: 0, // will be overwritten with calibration value at reset
            pc: 511,
            trisgpio: 0b11_1111,
            option: 0xff,

            input_buffer: 0,
            output_buffer: 0,

            indf: 0,             // INDF   xxxx xxxx
            tmr0: 0,             // tmr0   xxxx xxxx
            pcl: 0b11111111,     // PC     1111 1111
            status: 0b0001_1000, // STATUS 1111 1xxx
            fsr: 0b1110_0000,    // FSR    111x xxxx
            osccal: 0b1111_1111, // OSCCAL 111x xxxx
            gpio: 0b0000_0000,   // GPIO   111x xxxx

            tmr0_state: TMR0::default(),
        };
        cpu.flash[511] = (OpCode::MOVLW { k: OSC_CALIB_VAL }).encode(); // calibration value stored in W

        // update sram and sfrs
        cpu.update_regs_and_ram();

        // Tick past the first instruction so the first tick after
        // creating a CPU will be the first instruction, ie reset vector
        while cpu.pc > 0 {
            // PC rolls over after 2 cycles
            cpu.tick();
        }

        return cpu;
    }

    pub fn set_input(&mut self, gpio_bit: u8) {
        if gpio_bit > 5 {
            return;
        }
        self.input_buffer |= 1 << gpio_bit;
    }

    pub fn clr_input(&mut self, gpio_bit: u8) {
        if gpio_bit > 5 {
            return;
        }
        self.input_buffer &= !(1 << gpio_bit);
    }

    pub fn status_z(&self) -> bool {
        (self.status & 0b100) > 0
    }

    pub fn status_c(&self) -> bool {
        (self.status & 0b1) > 0
    }

    fn affect_zero(&mut self, val: bool) {
        if val {
            self.status |= 0b100;
        } else {
            self.status &= !0b100;
        }
    }

    fn affect_dc(&mut self, val: bool) {
        if val {
            self.status |= 0b10;
        } else {
            self.status &= !0b10;
        }
    }

    fn affect_carry(&mut self, val: bool) {
        if val {
            self.status |= 0b1;
        } else {
            self.status &= !0b1;
        }
    }

    fn fetch(&self, pc: u16) -> u16 {
        self.flash[(pc & 0x1ff) as usize]
    }

    fn fetch_next(&mut self) {
        let instruction = self.fetch(self.pc + 1);
        let op: OpCode = OpCode::decode(instruction);
        self.next_op = Some(op);
    }

    pub fn run(&mut self, ticks: u32) {
        for _ in 0..ticks {
            self.tick();
        }
    }

    // Ticks 4 clock cycles
    pub fn tick(&mut self) {
        match self.next_op {
            Some(op) => {
                self.fetch_next(); // updates next_op
                self.execute_op_code(op); // runs current op

                // save the executed op-code for debugging
                self.last_executed_op = Some(op);

                // If PC was changed, throw away the next instruction
                if self.pc_written {
                    //if self.pc != pc_prev {
                    self.pc_written = false;
                    self.next_op = None; // clear next instruction
                } else {
                    self.pc += 1; // increase PC as usual
                }

                // Wrap around if we index outside
                self.pc &= 0x1ff;

                self.pcl = (self.pc & 0xff) as u8;
                self.update_regs_and_ram();
            }
            None => {
                // next_op is None first cycle after start,
                // or after a branching instruction
                // PC is not increased in this case
                let current_pc_instruction: u16 = self.fetch(self.pc);
                let next_op_code: OpCode = OpCode::decode(current_pc_instruction);
                self.next_op = Some(next_op_code);
                self.last_executed_op = None;
                self.update_regs_and_ram();
            }
        };
    }

    // Updates registers and TMR0, and then updates
    // sram to reflect the current registers.
    fn update_regs_and_ram(&mut self) {
        // Cannot read INDF indirectly
        if self.fsr == 0 {
            self.indf = 0;
        } else {
            self.indf = self.sram[(self.fsr & 0b11111) as usize];
        }
        self.tmr0_state.tmr0 = self.tmr0; // if it was written to this tick
        self.tmr0_state.update(self.option);
        self.tmr0 = self.tmr0_state.tmr0;

        // update sram to reflect SFRs
        // This is so instructions can read sram directly
        // to access SFRs
        // We could also make a special "read()" function
        // but this is easier.
        self.sram[INDF] = self.indf;
        self.sram[TMR0] = self.tmr0;
        self.sram[PCL] = self.pcl;
        self.sram[STATUS] = self.status;
        self.sram[FSR] = self.fsr;
        self.sram[OSCCAL] = self.osccal;
        self.sram[GPIO] = self.gpio;
    }

    pub fn execute_op_code(&mut self, code: OpCode) {
        match code { // It's like rust enums were made for this
            OpCode::ADDWF { f, d } => self.addwf(f, d),
            OpCode::ANDWF { f, d } => self.andwf(f, d),
            OpCode::CLRF { f } => self.clrf(f),
            OpCode::CLRW => self.clrw(),
            OpCode::COMF { f, d } => self.comf(f, d),
            OpCode::DECF { f, d } => self.decf(f, d),
            OpCode::DECFSZ { f, d } => self.decfsz(f, d),
            OpCode::INCF { f, d } => self.incf(f, d),
            OpCode::INCFSZ { f, d } => self.incfsz(f, d),
            OpCode::IORWF { f, d } => self.iorwf(f, d),
            OpCode::MOVF { f, d } => self.movf(f, d),
            OpCode::MOVWF { f } => self.movwf(f),
            OpCode::NOP => self.nop(),
            OpCode::RLF { f, d } => self.rlf(f, d),
            OpCode::RRF { f, d } => self.rrf(f, d),
            OpCode::SUBWF { f, d } => self.subwf(f, d),
            OpCode::SWAPF { f, d } => self.swapf(f, d),
            OpCode::XORWF { f, d } => self.xorwf(f, d),

            // bit oriented operations
            OpCode::BCF { f, b } => self.bcf(f, b),
            OpCode::BSF { f, b } => self.bsf(f, b),
            OpCode::BTFSC { f, b } => self.btfsc(f, b),
            OpCode::BTFSS { f, b } => self.btfss(f, b),

            // branching and literal operations
            OpCode::ANDLW { k } => self.andlw(k),
            OpCode::CALL { k } => self.call(k),
            OpCode::CLRWDT => self.clrwdt(),
            OpCode::GOTO { k } => self.goto(k),
            OpCode::IORLW { k } => self.iorlw(k),
            OpCode::MOVLW { k } => self.movlw(k),
            OpCode::OPTION => self.option(),
            OpCode::RETLW { k } => self.retlw(k),
            OpCode::SLEEP => self.sleep(),
            OpCode::TRIS { f } => self.tris(f),
            OpCode::XORLW { k } => self.xorlw(k),
            OpCode::INVALID { code } => self.invalid(code),
        }
    }

    // Writing to SFRs causes special things to happen
    fn write(&mut self, f: u8, value: u8) {
        if f < 7 {
            match f as usize {
                INDF => self.write_indf(value),
                TMR0 => self.write_tmr0(value),
                PCL => self.write_pcl(value),
                STATUS => self.write_status(value),
                FSR => self.write_fsr(value),
                OSCCAL => self.write_osccal(value),
                GPIO => self.write_gpio(value),
                _ => panic!("Should not happen"),
            }
        } else {
            self.sram[f as usize] = value;
        }
    }

    fn write_indf(&mut self, value: u8) {
        let fsr = self.fsr;
        if fsr as usize == INDF {
            return; // cant write to INDF indirectly
        }
        self.write(fsr, value);
    }

    fn write_tmr0(&mut self, value: u8) {
        self.tmr0 = value;
    }

    fn write_pcl(&mut self, value: u8) {
        // Writing to PCL updates the program counter
        // Changing the PC flushes the next instruction
        // making this a 2 cycle instruction
        self.pc = value as u16;
        self.pc_written = true;
    }

    fn write_status(&mut self, value: u8) {
        self.status = value & 0b11100111; // some bits can't be written
    }

    fn write_fsr(&mut self, value: u8) {
        self.fsr = value & 0b11111; // 5 bit wide
    }

    fn write_osccal(&mut self, value: u8) {
        self.osccal = value & !1; // bit0 unimplemented
    }

    fn write_gpio(&mut self, value: u8) {
        // Writing to GPIO sets outputs
        self.output_buffer = value & 0b11_1111; // only 6 bits
        self.gpio = ((self.input_buffer & self.trisgpio) | (self.output_buffer & !self.trisgpio))
            & 0b11_1111; // 6 bits
    }
}

// Instructions
impl CPU {
    fn addwf(&mut self, f: u8, d: bool) {
        let a = self.sram[f as usize];
        let b = self.w;
        let (result, overflow) = a.overflowing_add(b);
        self.affect_carry(overflow);
        self.affect_zero(result == 0);
        self.affect_dc((a & 0xf + b & 0xf) > 15);

        if d {
            self.write(f, result as u8);
        } else {
            self.w = result as u8;
        }
    }

    fn andwf(&mut self, f: u8, d: bool) {
        let result = self.w & self.sram[f as usize];
        if d {
            self.write(f, result as u8);
        } else {
            self.w = result;
        }
        self.affect_zero(result == 0);
    }

    fn clrf(&mut self, f: u8) {
        self.write(f, 0);
        self.affect_zero(true);
    }

    fn clrw(&mut self) {
        self.w = 0;
        self.affect_zero(true);
    }

    fn comf(&mut self, f: u8, d: bool) {
        let result = !self.sram[f as usize];
        if d {
            self.write(f, result);
        } else {
            self.w = result;
        }
        self.affect_zero(result == 0);
    }

    fn decf(&mut self, f: u8, d: bool) {
        let (result, _) = self.sram[f as usize].overflowing_sub(1);
        if d {
            self.write(f, result);
        } else {
            self.w = result;
        }
        self.affect_zero(result == 0);
    }

    fn decfsz(&mut self, f: u8, d: bool) {
        let (result, _) = self.sram[f as usize].overflowing_sub(1);
        if d {
            self.write(f, result);
        } else {
            self.w = result;
        }
        // If result is 0, execute a nop instead of next instruction
        if result == 0 {
            // Replace next op with a NOP
            self.next_op = Some(OpCode::NOP);
        }
    }

    fn incf(&mut self, f: u8, d: bool) {
        let (result, _) = self.sram[f as usize].overflowing_add(1);
        if d {
            self.write(f, result);
        } else {
            self.w = result;
        }
        self.affect_zero(result == 0);
    }

    fn incfsz(&mut self, f: u8, d: bool) {
        let (result, overflow) = self.sram[f as usize].overflowing_add(1);
        if d {
            self.write(f, result);
        } else {
            self.w = result;
        }

        // If result is 0, execute a nop instead of next instruction
        if overflow {
            // Replace next op with a NOP
            self.next_op = Some(OpCode::NOP);
        }
    }

    fn iorwf(&mut self, f: u8, d: bool) {
        let result = self.w | self.sram[f as usize];
        if d {
            self.write(f, result);
        } else {
            self.w = result;
        }
        self.affect_zero(result == 0);
    }

    fn movf(&mut self, f: u8, d: bool) {
        let result = self.sram[f as usize];
        if d {
            //
        } else {
            self.w = result;
        }
        self.affect_zero(result == 0);
    }

    fn movwf(&mut self, f: u8) {
        self.write(f, self.w);
    }

    fn nop(&mut self) {
        // nop :)
    }

    fn rlf(&mut self, f: u8, d: bool) {
        let mut result = self.sram[f as usize];
        let msb = result & 0x80 > 0;
        result <<= 1;
        if self.status_c() {
            // carry bit was set
            result |= 1;
        }
        self.affect_carry(msb);
        if d {
            self.write(f, result);
        } else {
            self.w = result;
        }
    }

    fn rrf(&mut self, f: u8, d: bool) {
        let mut result = self.sram[f as usize];
        let lsb = result & 0x1 > 0;
        result >>= 1;
        if self.status_c() {
            // carry bit was set
            result |= 0x80;
        }
        self.affect_carry(lsb);
        if d {
            self.write(f, result);
        } else {
            self.w = result;
        }
    }

    fn subwf(&mut self, f: u8, d: bool) {
        let a = self.sram[f as usize];
        let b = self.w;
        let (result, overflow) = a.overflowing_sub(b);
        self.affect_carry(overflow);
        self.affect_zero(result == 0);
        self.affect_dc(a & 0xf < b & 0xf); // not sure about this one
        if d {
            self.write(f, result);
        } else {
            self.w = result;
        }
    }

    fn swapf(&mut self, f: u8, d: bool) {
        let value = self.sram[f as usize];
        let result = (value & 0xf) << 4 | (value >> 4);
        if d {
            self.write(f, result);
        } else {
            self.w = result;
        }
    }

    fn xorwf(&mut self, f: u8, d: bool) {
        let result = self.sram[f as usize] ^ self.w;
        self.affect_zero(result == 0);
        if d {
            self.write(f, result);
        } else {
            self.w = result;
        }
    }

    fn bcf(&mut self, f: u8, b: u8) {
        let result = self.sram[f as usize] & !(1 << b);
        self.write(f, result);
    }

    fn bsf(&mut self, f: u8, b: u8) {
        let result = self.sram[f as usize] | (1 << b);
        self.write(f, result);
    }

    fn btfsc(&mut self, f: u8, b: u8) {
        if self.sram[f as usize] & (1 << b) == 0 {
            // Replace next op with a NOP
            self.next_op = Some(OpCode::NOP);
        }
    }

    fn btfss(&mut self, f: u8, b: u8) {
        if self.sram[f as usize] & (1 << b) != 0 {
            // Replace next op with a NOP
            self.next_op = Some(OpCode::NOP);
        }
    }

    fn andlw(&mut self, k: u8) {
        self.w &= k;
        self.affect_zero(self.w == 0);
    }

    fn call(&mut self, k: u8) {
        self.stack[1] = self.stack[0];
        self.stack[0] = self.pc + 1;
        self.pc = k as u16; // can only call first 256 indexes
        self.pc_written = true;
    }

    fn clrwdt(&mut self) {
        // he CLRWDT instruction resets the WDT.
        // It resets the prescaler, if it's assigned to the WDT
        // Status bits TO and PD are set.
        todo!()
    }

    fn goto(&mut self, k: u16) {
        self.pc = k;
        self.pc_written = true;
    }

    fn iorlw(&mut self, k: u8) {
        self.w |= k;
        self.affect_zero(self.w == 0);
    }

    fn movlw(&mut self, k: u8) {
        self.w = k;
    }

    fn option(&mut self) {
        self.option = self.w;
        // TODO update stuff depending on OPTION reg bits
    }

    fn retlw(&mut self, k: u8) {
        // Pop stack and update pc
        self.w = k;
        self.pc = self.stack[0];
        self.stack[0] = self.stack[1];
        self.pc_written = true;
    }

    fn sleep(&mut self) {
        todo!()
    }

    fn tris(&mut self, f: u8) {
        if f == 6 {
            self.trisgpio = self.w & 0b11_1111; // 6 bits
        } else {
            println!("ERROR: Executed TRIS with invalid argument: {f:03x} (NOP)");
        }
    }

    fn xorlw(&mut self, k: u8) {
        self.w ^= k;
        self.affect_zero(self.w == 0);
    }

    fn invalid(&self, code: u16) {
        println!("ERROR: Executed invalid instruction {code:03x} as NOP");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate test;
    use std::fs;
    use test::Bencher;

    #[test]
    fn test_run_cpu() {
        let ops = vec![OpCode::MOVLW { k: 0xff }, OpCode::MOVWF { f: 0x10 }];
        let mut cpu = CPU::from_ops(ops);
        assert_eq!(cpu.next_op.unwrap(), OpCode::MOVLW { k: 0xff });
        assert_eq!(cpu.pc, 0); // has executed first MOVF op at PC=0x1ff
        assert_eq!(cpu.w, OSC_CALIB_VAL); // contains osccal at start
        cpu.tick();
        assert_eq!(cpu.pc, 1);
        assert_eq!(cpu.next_op.unwrap(), OpCode::MOVWF { f: 0x10 });
    }

    #[test]
    fn calculated_jump_sets_pc() {
        let mut cpu = CPU::from_ops(vec![
            OpCode::MOVLW { k: 0x55 },
            OpCode::MOVWF { f: PCL as u8 },
        ]);
        cpu.tick(); // MOVLW
        cpu.tick(); // jump
        assert_eq!(cpu.pc, 0x55);
        assert_eq!(cpu.next_op, None);
        cpu.tick();
        assert_eq!(cpu.pc, 0x55);
    }

    #[test]
    fn jump_to_same_instruction_takes_2_cycles() {
        // If jumping to the same location, PC will not change
        // so we have to track every "write" to PC, regardless
        // of the value written. This is done in any instruction
        // that modifies the PC
        let mut cpu = CPU::from_ops(vec![OpCode::GOTO { k: 0 }]);
        cpu.tick();
        assert_eq!(cpu.pc, 0);
        assert_eq!(cpu.next_op, None);
        cpu.tick();
        assert_eq!(cpu.pc, 0);
        cpu.tick(); // execute GOTO 0 again here
        assert_eq!(cpu.pc, 0);
    }

    #[bench]
    fn bench_run_cpu(b: &mut Bencher) {
        let contents = fs::read_to_string("obj/main.hex").unwrap();
        let cpu: CPU = CPU::from_hex(&contents);
        let mut cpu = test::black_box(cpu);
        b.iter(|| cpu.tick());
    }

    mod tmr0_tests {
        use super::*;

        #[test]
        fn clock_is_off_normally() {
            let mut cpu = CPU::from_ops(vec![OpCode::GOTO { k: 0 }]);
            assert_eq!(cpu.tmr0, 0);
            cpu.run(100);
            assert_eq!(cpu.tmr0, 0);
        }

        #[test]
        fn clock_inc_one_per_cycle() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0b11011000 }, // start timer
                OpCode::OPTION,
            ]);
            assert_eq!(cpu.tmr0, 0);
            cpu.run(2);
            // after the OPTION instruction, the timer will start
            // Since the timer increases after op execute, tmr0 will be 1 here
            assert_eq!(cpu.tmr0, 1);
            cpu.run(9);
            assert_eq!(cpu.tmr0, 10);
            cpu.run(245);
            assert_eq!(cpu.tmr0, 255);
            cpu.run(1);
            assert_eq!(cpu.tmr0, 0); // wrap
        }

        #[test]
        fn tmr_inhibit_2cycles_after_write() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0b11011000 }, // start timer
                OpCode::OPTION,
                OpCode::NOP,
                OpCode::MOVLW { k: 100 },
                OpCode::MOVWF { f: TMR0 as u8 }, // write
                OpCode::MOVF {
                    f: TMR0 as u8,
                    d: false,
                }, // read 100
                OpCode::MOVF {
                    f: TMR0 as u8,
                    d: false,
                }, // read 100
                OpCode::MOVF {
                    f: TMR0 as u8,
                    d: false,
                }, // read 100?
                OpCode::MOVF {
                    f: TMR0 as u8,
                    d: false,
                }, // read 101?
            ]);
            assert_eq!(cpu.tmr0, 0);
            cpu.run(2);
            // after the OPTION instruction, the timer will start
            // Since the timer increases after op execute, tmr0 will be 1 here
            assert_eq!(cpu.tmr0, 1);
            cpu.tick(); // nop
            cpu.tick(); // movlw
            assert_eq!(cpu.tmr0, 3);
            cpu.tick(); // movwf tmr0
            assert_eq!(cpu.tmr0, 100);
            cpu.tick(); // movf
            assert_eq!(cpu.w, 100);
            assert_eq!(cpu.tmr0, 100);
            cpu.tick();
            assert_eq!(cpu.w, 100);
            assert_eq!(cpu.tmr0, 100);
            cpu.tick(); // tmr starts again this tick
            assert_eq!(cpu.w, 100);
            assert_eq!(cpu.tmr0, 101);
            cpu.tick();
            assert_eq!(cpu.w, 101);
            assert_eq!(cpu.tmr0, 102);
        }

        #[test]
        fn tmr_1to2_prescaler() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0b11010000 }, // start timer with 1:2 prescale
                OpCode::OPTION,
                OpCode::MOVLW { k: 100 },
                OpCode::MOVWF { f: TMR0 as u8 },
                OpCode::MOVF {
                    f: TMR0 as u8,
                    d: false,
                },
                OpCode::MOVF {
                    f: TMR0 as u8,
                    d: false,
                },
                OpCode::MOVF {
                    f: TMR0 as u8,
                    d: false,
                },
                OpCode::MOVF {
                    f: TMR0 as u8,
                    d: false,
                },
                OpCode::MOVF {
                    f: TMR0 as u8,
                    d: false,
                },
            ]);
            assert_eq!(cpu.tmr0, 0);
            cpu.run(2); // OPTION
            assert_eq!(cpu.tmr0, 0);

            cpu.run(2); // MOVLW, MOVWF
            assert_eq!(cpu.tmr0, 100);

            cpu.tick(); // MOVF
            assert_eq!(cpu.tmr0, 100);

            cpu.tick(); // MOVF
            assert_eq!(cpu.tmr0, 100);

            cpu.tick(); // MOVF
            assert_eq!(cpu.tmr0, 100);

            cpu.tick(); // MOVF
            assert_eq!(cpu.tmr0, 101);
            assert_eq!(cpu.w, 100);

            cpu.tick();
            assert_eq!(cpu.tmr0, 101);
            assert_eq!(cpu.w, 101);
        }

        #[test]
        fn tmr_1to4_prescaler() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0b11010001 }, // start timer with 1:4 prescale
                OpCode::OPTION,
                OpCode::NOP,
                OpCode::NOP,
                OpCode::NOP,
                OpCode::NOP,
                OpCode::NOP,
                OpCode::MOVLW { k: 100 },
                OpCode::MOVWF { f: TMR0 as u8 },
                OpCode::MOVF {
                    f: TMR0 as u8,
                    d: false,
                },
                OpCode::MOVF {
                    f: TMR0 as u8,
                    d: false,
                },
                OpCode::MOVF {
                    f: TMR0 as u8,
                    d: false,
                },
                OpCode::MOVF {
                    f: TMR0 as u8,
                    d: false,
                },
                OpCode::MOVF {
                    f: TMR0 as u8,
                    d: false,
                },
                OpCode::MOVF {
                    f: TMR0 as u8,
                    d: false,
                },
            ]);
            assert_eq!(cpu.tmr0, 0);
            cpu.run(2); // OPTION
            assert_eq!(cpu.tmr0, 0);
            cpu.run(2);
            assert_eq!(cpu.tmr0, 0);
            cpu.tick();
            assert_eq!(cpu.tmr0, 1);
            cpu.run(4); // movlw 100, movwf tmr0
            assert_eq!(cpu.tmr0, 100);
            cpu.run(5);
            assert_eq!(cpu.tmr0, 100);
            cpu.tick();
            assert_eq!(cpu.tmr0, 101);
            assert_eq!(cpu.w, 100);
        }
        #[test]
        fn tmr_1to256_prescaler() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0b11010111 }, // start timer with 1:4 prescale
                OpCode::OPTION,
                OpCode::MOVF {
                    f: TMR0 as u8,
                    d: false,
                },
                OpCode::GOTO { k: 2 },
            ]);
            assert_eq!(cpu.tmr0, 0);
            cpu.run(2); // OPTION
            assert_eq!(cpu.tmr0, 0);
            cpu.run(254);
            assert_eq!(cpu.tmr0, 0);
            cpu.run(1);
            assert_eq!(cpu.tmr0, 1);
            cpu.run(255);
            assert_eq!(cpu.tmr0, 1);
            cpu.run(1);
            assert_eq!(cpu.tmr0, 2);
        }
    }

    mod indirect_addressing {
        use super::*;

        #[test]
        fn read_indf() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0x55 },
                OpCode::MOVWF { f: 0x10 },
                OpCode::MOVLW { k: 0x10 },
                OpCode::MOVWF { f: FSR as u8 },
                OpCode::CLRW,
                OpCode::MOVF {
                    f: INDF as u8,
                    d: false,
                },
            ]);
            cpu.run(6);
            assert_eq!(cpu.w, 0x55);
        }
        #[test]
        fn read_status_indf() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: STATUS as u8 },
                OpCode::MOVWF { f: FSR as u8 },
                OpCode::CLRW,
                OpCode::MOVF {
                    f: INDF as u8,
                    d: false,
                },
            ]);
            cpu.run(4);
            assert_eq!(cpu.w, cpu.status);
        }

        #[test]
        fn write_gpio_indf() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: GPIO as u8 },
                OpCode::MOVWF { f: FSR as u8 },
                OpCode::CLRW,
                OpCode::TRIS { f: GPIO as u8 },
                OpCode::MOVLW { k: 0b1111_1111 },
                OpCode::MOVWF { f: INDF as u8 },
                OpCode::CLRW,
                OpCode::MOVF {
                    f: INDF as u8,
                    d: false,
                },
            ]);
            cpu.run(8);
            assert_eq!(cpu.gpio, 0b11_1111);
            assert_eq!(cpu.w, 0b11_1111);
        }
    }

    mod instructions {
        use super::*;

        #[test]
        fn addwf() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0xaa },
                OpCode::MOVWF { f: 0x10 },
                OpCode::MOVLW { k: 0x60 },
                OpCode::ADDWF { f: 0x10, d: true },  // store in F
                OpCode::ADDWF { f: 0x10, d: false }, // store in w
            ]);
            cpu.tick();
            cpu.tick();
            cpu.tick();
            cpu.tick();
            assert!(cpu.status_c());
            assert_eq!(cpu.sram[0x10], 0x0a);
            cpu.tick();
            assert_eq!(cpu.w, 0x6A);
            assert!(!cpu.status_c());
        }

        #[test]
        fn andwf() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0xaf },
                OpCode::MOVWF { f: 0x10 },
                OpCode::MOVLW { k: 0x55 },
                OpCode::ANDWF { f: 0x10, d: false }, // store in w
            ]);
            cpu.tick();
            cpu.tick();
            cpu.tick();
            cpu.tick();
            assert!(!cpu.status_z());
            assert_eq!(cpu.sram[0x10], 0xaf);
            assert_eq!(cpu.w, 0x05); // result stored in w
        }

        #[test]
        fn clrf() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0xaa },
                OpCode::MOVWF { f: 0x10 },
                OpCode::CLRF { f: 0x10 },
            ]);
            cpu.tick(); // MOVLW
            cpu.tick(); // MOVWF
            cpu.tick(); // CLRF
            assert_eq!(cpu.sram[0x10], 0);
            assert!(cpu.status_z());
        }

        #[test]
        fn clrw() {
            let mut cpu = CPU::from_ops(vec![OpCode::MOVLW { k: 0xa5 }, OpCode::CLRW]);
            cpu.tick();
            cpu.tick();
            assert_eq!(cpu.w, 0);
            assert!(cpu.status_z());
        }

        #[test]
        fn comf() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0xa5 },
                OpCode::MOVWF { f: 0x10 },
                OpCode::COMF { f: 0x10, d: true },
                OpCode::CLRW,
                OpCode::COMF { f: 0x10, d: false },
                OpCode::MOVLW { k: 0xff },
                OpCode::MOVWF { f: 0x10 },
                OpCode::COMF { f: 0x10, d: true },
            ]);
            cpu.tick();
            cpu.tick();
            cpu.tick();
            assert_eq!(cpu.sram[0x10], !0xa5);
            cpu.tick();
            cpu.tick();
            assert_eq!(cpu.w, 0xa5);
            cpu.tick();
            cpu.tick();
            cpu.tick();
            assert_eq!(cpu.sram[0x10], 0);
            assert!(cpu.status_z());
        }

        #[test]
        fn decf() {
            let mut ops = vec![
                OpCode::MOVLW { k: 0xff },
                OpCode::MOVWF { f: 0x10 }, // store 0xff in F=0x10
                OpCode::CLRW,              // clear W
            ];

            // DECF 255 times to make it 0
            for _ in 0..255 {
                ops.push(OpCode::DECF { f: 0x10, d: true })
            }

            // DECF one more time to wrap to 0xff, and store in W
            ops.push(OpCode::DECF { f: 0x10, d: false });

            let mut cpu = CPU::from_ops(ops);
            cpu.run(3);
            for n in 0..255 {
                cpu.tick();
                assert_eq!(254 - n, cpu.sram[0x10]);
            }
            assert!(cpu.status_z());
            cpu.tick();
            assert_eq!(cpu.w, 0xff);
            assert!(!cpu.status_z());
        }

        #[test]
        fn decfsz() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 2 },
                OpCode::MOVWF { f: 0x10 },
                OpCode::DECFSZ { f: 0x10, d: true },
                OpCode::MOVLW { k: 0xaa },
                OpCode::DECFSZ { f: 0x10, d: true },
                OpCode::MOVLW { k: 0x55 },
            ]);
            cpu.tick();
            cpu.tick();
            cpu.tick();
            assert_eq!(cpu.next_op.unwrap(), OpCode::MOVLW { k: 0xaa });
            cpu.tick();
            cpu.tick();
            assert_eq!(cpu.next_op.unwrap(), OpCode::NOP);
            cpu.tick();
            assert_eq!(cpu.sram[0x10], 0x0);
            assert_eq!(cpu.w, 0xaa);
        }

        #[test]
        fn incf() {
            let mut ops = vec![
                OpCode::MOVLW { k: 0x0 },
                OpCode::MOVWF { f: 0x10 }, // store 0xff in F=0x10
                OpCode::CLRW,              // clear W
            ];

            // INCF 255 times to make it 255
            for _ in 0..255 {
                ops.push(OpCode::INCF { f: 0x10, d: true })
            }

            // INCF one more time to wrap to 0x0, and store in W
            ops.push(OpCode::INCF { f: 0x10, d: false });

            let mut cpu = CPU::from_ops(ops);
            cpu.run(3);
            for n in 0..255 {
                assert_eq!(n, cpu.sram[0x10]);
                cpu.tick();
                assert_eq!(n + 1, cpu.sram[0x10]);
            }
            assert!(!cpu.status_z());
            cpu.tick();
            assert_eq!(cpu.w, 0);
            assert!(cpu.status_z());
        }

        #[test]
        fn incfsz() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0xfe },
                OpCode::MOVWF { f: 0x10 },
                OpCode::INCFSZ { f: 0x10, d: true },
                OpCode::MOVLW { k: 0xaa },
                OpCode::INCFSZ { f: 0x10, d: true },
                OpCode::MOVLW { k: 0x55 },
            ]);
            cpu.tick();
            cpu.tick();
            cpu.tick();
            assert_eq!(cpu.next_op.unwrap(), OpCode::MOVLW { k: 0xaa });
            cpu.tick();
            cpu.tick();
            assert_eq!(cpu.next_op.unwrap(), OpCode::NOP);
            cpu.tick();
            assert_eq!(cpu.sram[0x10], 0x0);
            assert_eq!(cpu.w, 0xaa);
        }

        #[test]
        fn iorwf() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0xaa },
                OpCode::MOVWF { f: 0x10 },
                OpCode::MOVLW { k: 0x55 },
                OpCode::IORWF { f: 0x10, d: true },
                OpCode::IORWF { f: 0x10, d: false },
                OpCode::CLRF { f: 0x10 },
                OpCode::CLRW,
                OpCode::INCF { f: 0x11, d: true }, // clear status flag
                OpCode::IORWF { f: 0x10, d: false },
            ]);
            cpu.run(4);
            assert_eq!(cpu.sram[0x10], 0xff);
            assert!(!cpu.status_z());
            cpu.tick();
            assert_eq!(cpu.w, 0xff);
            cpu.run(4);
            assert!(cpu.status_z());
        }

        #[test]
        fn movf() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVF { f: 0x10, d: true }, // affect Z flag
                OpCode::MOVLW { k: 0xaa },
                OpCode::MOVWF { f: 0x10 },
                OpCode::CLRW,
                OpCode::MOVF { f: 0x10, d: false },
            ]);
            cpu.tick();
            assert!(cpu.status_z());
            cpu.run(4);
            assert!(!cpu.status_z());
            assert_eq!(cpu.w, 0xaa);
        }

        #[test]
        fn movwf() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0xaa },
                OpCode::MOVWF { f: 0x10 },
                OpCode::MOVLW { k: 0x55 },
                OpCode::MOVWF { f: 0x10 },
                OpCode::MOVLW { k: 0x00 },
                OpCode::MOVWF { f: 0x10 },
                OpCode::MOVLW { k: 0xff },
                OpCode::MOVWF { f: 0x10 },
            ]);
            cpu.run(2);
            assert_eq!(cpu.sram[0x10], 0xaa);
            cpu.run(2);
            assert_eq!(cpu.sram[0x10], 0x55);
            cpu.run(2);
            assert_eq!(cpu.sram[0x10], 0);
            cpu.run(2);
            assert_eq!(cpu.sram[0x10], 0xff);
        }

        #[test]
        fn nop() {
            // Test that nothing happens
            let mut cpu = CPU::from_ops(vec![OpCode::NOP]);
            let status = cpu.status;
            cpu.tick();
            assert_eq!(cpu.status, status);
        }

        #[test]
        fn rlf() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0x55 },
                OpCode::MOVWF { f: 0x10 },
                OpCode::RLF { f: 0x10, d: true },
                OpCode::RLF { f: 0x10, d: true },
                OpCode::RLF { f: 0x10, d: true },
            ]);
            cpu.run(3);
            assert_eq!(cpu.sram[0x10], 0xaa);
            assert!(!cpu.status_c());
            cpu.tick();
            assert_eq!(cpu.sram[0x10], 0x54);
            assert!(cpu.status_c());
            cpu.tick();
            assert_eq!(cpu.sram[0x10], 0xA9);
            assert!(!cpu.status_c());
        }

        #[test]
        fn rrf() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0xaa },
                OpCode::MOVWF { f: 0x10 },
                OpCode::RRF { f: 0x10, d: true },
                OpCode::RRF { f: 0x10, d: true },
                OpCode::RRF { f: 0x10, d: true },
            ]);
            cpu.run(3);
            assert_eq!(cpu.sram[0x10], 0x55);
            assert!(!cpu.status_c());
            cpu.tick();
            assert_eq!(cpu.sram[0x10], 0x2a);
            assert!(cpu.status_c());
            cpu.tick();
            assert_eq!(cpu.sram[0x10], 0x95);
            assert!(!cpu.status_c());
        }

        #[test]
        fn subwf() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0xaa },
                OpCode::MOVWF { f: 0x10 },
                OpCode::SUBWF { f: 0x10, d: true },
                OpCode::SUBWF { f: 0x10, d: false },
            ]);
            cpu.run(3);
            assert!(cpu.status_z());
            assert_eq!(cpu.w, 0xaa);
            assert_eq!(cpu.sram[0x10], 0x00);
            cpu.tick();
            assert_eq!(cpu.w, 0x56);
            assert!(!cpu.status_z());
        }

        #[test]
        fn swapf() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0xa5 },
                OpCode::MOVWF { f: 0x10 },
                OpCode::SWAPF { f: 0x10, d: true },
                OpCode::SWAPF { f: 0x10, d: true },
                OpCode::SWAPF { f: 0x10, d: false },
            ]);
            cpu.run(3);
            assert_eq!(cpu.sram[0x10], 0x5a);
            assert!(!cpu.status_z());
            cpu.tick();
            assert_eq!(cpu.sram[0x10], 0xa5);
            cpu.tick();
            assert_eq!(cpu.w, 0x5a);
        }

        #[test]
        fn xorwf() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0xa5 },
                OpCode::MOVWF { f: 0x10 },
                OpCode::MOVLW { k: 0xa0 },
                OpCode::XORWF { f: 0x10, d: true },
                OpCode::MOVLW { k: 0x05 },
                OpCode::XORWF { f: 0x10, d: true },
            ]);
            cpu.run(4);
            assert_eq!(cpu.sram[16], 0x05);
            cpu.run(2);
            assert_eq!(cpu.sram[16], 0x0);
            assert!(cpu.status_z());
        }

        #[test]
        fn bcf() {
            let mut ops = vec![OpCode::MOVLW { k: 0xff }, OpCode::MOVWF { f: 0x10 }];
            let mut ops2: Vec<OpCode> = (0..8).map(|i| OpCode::BCF { f: 0x10, b: i }).collect();
            ops.append(&mut ops2);
            let mut cpu = CPU::from_ops(ops);
            cpu.run(2);
            let mut expected = 0xff;
            for i in 0..8 {
                cpu.tick();
                expected &= !(1 << i);
                assert_eq!(cpu.sram[16], expected);
            }
        }

        #[test]
        fn bsf() {
            let ops: Vec<OpCode> = (0..8).map(|i| OpCode::BSF { f: 0x10, b: i }).collect();
            let mut cpu = CPU::from_ops(ops);
            let mut expected = 0;
            for i in 0..8 {
                cpu.tick();
                expected |= 1 << i;
                assert_eq!(cpu.sram[16], expected);
            }
        }

        #[test]
        fn btfsc() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::BTFSC { f: 0x10, b: 0 },
                OpCode::MOVLW { k: 0xaa }, // should be skipped
                OpCode::MOVLW { k: 0x01 },
                OpCode::MOVWF { f: 0x10 },
                OpCode::BTFSC { f: 0x10, b: 0 },
                OpCode::MOVLW { k: 0xaa }, // should not be skipped
            ]);
            cpu.tick();
            assert_eq!(cpu.next_op.unwrap(), OpCode::NOP);
            cpu.run(4);
            assert_eq!(cpu.next_op.unwrap(), OpCode::MOVLW { k: 0xaa });
        }

        #[test]
        fn btfss() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::BTFSS { f: 0x10, b: 0 },
                OpCode::MOVLW { k: 0xaa }, // should not be skipped
                OpCode::MOVLW { k: 0x01 },
                OpCode::MOVWF { f: 0x10 },
                OpCode::BTFSS { f: 0x10, b: 0 },
                OpCode::MOVLW { k: 0xaa }, // should be skipped
            ]);
            cpu.tick();
            assert_eq!(cpu.next_op.unwrap(), OpCode::MOVLW { k: 0xaa });
            cpu.run(4);
            assert_eq!(cpu.next_op.unwrap(), OpCode::NOP);
        }

        #[test]
        fn andlw() {
            let mut cpu = CPU::from_ops(vec![OpCode::MOVLW { k: 0xaa }, OpCode::ANDLW { k: 0x55 }]);
            cpu.run(2);
            assert_eq!(cpu.w, 0x00);
        }

        #[test]
        fn call() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::CALL { k: 3 },
                OpCode::SWAPF { f: 0x10, d: true },
                OpCode::SWAPF { f: 0x10, d: true },
                OpCode::CALL { k: 6 },
                OpCode::SWAPF { f: 0x10, d: true },
                OpCode::SWAPF { f: 0x10, d: true },
            ]);
            cpu.tick();
            assert_eq!(cpu.pc, 3);
            assert_eq!(cpu.stack[0], 1);
            cpu.tick();
            cpu.tick();
            assert_eq!(cpu.pc, 6);
            assert_eq!(cpu.stack[0], 4);
            assert_eq!(cpu.stack[1], 1);
        }

        #[test]
        fn clrwdt() {
            // he CLRWDT instruction resets the WDT.
            // It resets the prescaler, if it's assigned to the WDT
            // Status bits TO and PD are set.
            let mut cpu = CPU::from_ops(vec![]);
            cpu.tick();
            // todo!();
        }

        #[test]
        fn goto() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::GOTO { k: 4 },
                OpCode::NOP,
                OpCode::NOP,
                OpCode::NOP,
                OpCode::MOVLW { k: 0xff },
                OpCode::GOTO { k: 500 },
            ]);
            cpu.tick();
            assert_eq!(cpu.pc, 4);
            cpu.tick(); // new fetch
            assert_eq!(cpu.pc, 4); // dont increase PC
            assert_eq!(cpu.next_op.unwrap(), OpCode::MOVLW { k: 0xff });
            cpu.tick(); // new fetch
            cpu.tick(); // new fetch
            assert_eq!(cpu.pc, 500);
        }

        #[test]
        fn iorlw() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0x00 },
                OpCode::IORLW { k: 0x00 },
                OpCode::IORLW { k: 0xaa },
                OpCode::IORLW { k: 0x55 },
            ]);
            cpu.tick();
            cpu.tick();
            assert_eq!(cpu.w, 0);
            assert!(cpu.status_z());
            cpu.tick();
            assert_eq!(cpu.w, 0xaa);
            cpu.tick();
            assert_eq!(cpu.w, 0xff);
        }

        #[test]
        fn movlw() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0xff },
                OpCode::MOVLW { k: 0xaa },
                OpCode::MOVLW { k: 0x00 },
            ]);
            cpu.tick();
            assert_eq!(cpu.w, 0xff);
            cpu.tick();
            assert_eq!(cpu.w, 0xaa);
            cpu.tick();
            assert_eq!(cpu.w, 0x00);
            assert!(!cpu.status_z()) // does not set flag
        }

        #[test]
        fn option() {
            let mut cpu = CPU::from_ops(vec![OpCode::MOVLW { k: 0xaa }, OpCode::OPTION]);
            assert_eq!(cpu.option, 0xff);
            cpu.tick();
            cpu.tick();
            assert_eq!(cpu.option, 0xaa);
        }

        #[test]
        fn retlw() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::CALL { k: 4 },
                OpCode::NOP,
                OpCode::NOP,
                OpCode::NOP,
                OpCode::RETLW { k: 0x55 },
            ]);
            cpu.run(2); // two ticks for CALL
            cpu.run(2); // two ticks for RETLW
            assert_eq!(cpu.pc, 1);
            assert_eq!(cpu.w, 0x55);
        }

        #[test]
        fn sleep() {
            // let mut cpu = CPU::from_ops(vec![]);
            // cpu.tick();
            // todo!();
        }

        #[test]
        fn tris() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0b010101 },
                OpCode::TRIS { f: GPIO as u8 },
            ]);
            cpu.tick();
        }

        #[test]
        fn xorlw() {
            let mut cpu = CPU::from_ops(vec![
                OpCode::MOVLW { k: 0x00 },
                OpCode::XORLW { k: 0x00 },
                OpCode::XORLW { k: 0xaa },
                OpCode::XORLW { k: 0x55 },
                OpCode::XORLW { k: 0x55 },
                OpCode::XORLW { k: 0xaa },
            ]);
            cpu.tick();
            cpu.tick();
            assert_eq!(cpu.w, 0);
            assert!(cpu.status_z());
            cpu.tick();
            assert_eq!(cpu.w, 0xaa);
            cpu.tick();
            assert_eq!(cpu.w, 0xff);
            cpu.tick();
            assert_eq!(cpu.w, 0xaa);
            cpu.tick();
            assert_eq!(cpu.w, 0x00);
            assert!(cpu.status_z());
        }
    }
}
