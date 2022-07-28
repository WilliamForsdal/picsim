use core::panic;

use crate::opcode::*;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CfgFosc {
    EXTRC,
    INTRC,
    XT,
    LP,
}

pub struct ConfigWord {
    pub mclre: bool,
    pub cp_disable: bool,
    pub wdte: bool,
    pub fosc: CfgFosc,
}

impl ConfigWord {
    pub fn is_wd_enabled(&self) -> bool {
        self.wdte
    }
    pub fn is_mclr_enabled(&self) -> bool {
        self.mclre
    }

    pub fn default() -> ConfigWord {
        ConfigWord {
            mclre: false,
            cp_disable: true,
            wdte: false,
            fosc: CfgFosc::INTRC,
        }
    }
    pub fn from_value(value: u16) -> ConfigWord {
        const MCLRE: u16 = 4;
        const CP: u16 = 3;
        const WDTE: u16 = 2;
        const FOCS_MASK: u16 = 0b11;
        ConfigWord {
            mclre: (value >> MCLRE) & 1 > 0,
            cp_disable: (value >> CP) & 1 > 0,
            wdte: (value >> WDTE) & 1 > 0,
            fosc: match value & FOCS_MASK {
                0b00 => CfgFosc::LP,
                0b01 => CfgFosc::XT,
                0b10 => CfgFosc::INTRC,
                0b11 => CfgFosc::EXTRC,
                _ => panic!("can't happen"),
            },
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CpuState {
    Normal,
    Sleep,
    MclrLow, // When MCLR is enabled, and MCLR pin is pulled low
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Input {
    Floating,
    Low,
    High,
}

pub fn bit_is_set(bits: u8, bit: u8) -> bool {
    (bits & (1 << bit)) > 0
}

pub fn bit_is_clear(bits: u8, bit: u8) -> bool {
    (bits & (1 << bit)) == 0
}

pub struct OptionReg {
    pub bits: u8,
}
const GPWU: u8 = 7;
const GPPU: u8 = 6;
const T0CS: u8 = 5;
// const T0SE: u8 = 4;
const PSA: u8 = 3;
// const PS2: u8 = 2;
// const PS1: u8 = 1;
// const PS0: u8 = 0;

impl OptionReg {
    pub fn wake_on_pin_change_enabled(&self) -> bool {
        bit_is_clear(self.bits, GPWU)
    }
    pub fn weak_pull_ups_enabled(&self) -> bool {
        bit_is_clear(self.bits, GPPU)
    }
    pub fn tmr0_enabled(&self) -> bool {
        bit_is_clear(self.bits, T0CS)
    }
    pub fn prescaler_assigned_to_wdt(&self) -> bool {
        bit_is_set(self.bits, PSA)
    }
    pub fn prescaler_assigned_to_tmr0(&self) -> bool {
        bit_is_clear(self.bits, PSA)
    }
    pub fn prescaler_rate_wdt(&self) -> u16 {
        1 << (self.bits & 7)
    }
    pub fn prescaler_rate_tmr0(&self) -> u16 {
        1 << ((self.bits & 7) + 1)
    }
}

pub const PIN_MCLR: u8 = 3;

pub struct Pic12F508 {
    pub ticks: u64,
    pub last_executed_op: Option<OpCode>,

    // the decoded next op
    // None at first tick, or after branch
    pub next_op: Option<OpCode>,

    pc_written: bool,

    pub config: ConfigWord,
    pub flash: [u16; 512], // actually 12 bit words
    pub sram: [u8; 32],    // lower 6 not used, SRFs
    pub stack: [u16; 2],

    pub w: u8,

    // Remember to set pc_written when writing to PC
    pub pc: u16, // actually 10 bits for real
    pub option: OptionReg,
    pub trisgpio: u8, // not a memory mapped register

    pub input_buffer: [Input; 6], // store input data
    pub output_buffer: u8,

    pub indf: u8,
    pub pcl: u8,
    pub status: u8,
    pub fsr: u8,
    pub osccal: u8,
    pub gpio: u8,

    pub state: CpuState,
    pub last_reset_reason: ResetReason,

    pub tmr0: u8,
    tmr0_inhibit: u8,

    // Actual register
    // make it u16 so it fits 256 for TMR0
    prescaler: u16,

    // we need to count 18k instructions = 18ms.
    // The WD oscillator is independent from the
    // main oscillator, but simulating that would
    // be more complicated. Just count to 18k here
    wd_timer: u32,
}

// SFR addresses
const INDF: usize = 0;
const TMR0: usize = 1;
const PCL: usize = 2;
const STATUS: usize = 3;
const FSR: usize = 4;
const OSCCAL: usize = 5;
const GPIO: usize = 6;
const OSC_CALIB_VALUE: u8 = 0xb0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetReason {
    POR,
    MCLR,
    MCLRSleep,
    WDT,
    WDTSleep,
    PinChangeSleep,
}

enum StatusBit {
    C,
    DC,
    Z,
    PD,
    TO,
    PA0,
    GPWUF,
}

impl Pic12F508 {
    pub fn from_hex(program: &str) -> Pic12F508 {
        let (flash, cfg_word) = OpCode::from_hex(program);
        Pic12F508::new_from_flash(flash, ConfigWord::default())
    }

    pub fn from_asm(asm: &str) -> Pic12F508 {
        let ops = OpCode::compile(asm);
        Pic12F508::from_ops(ops)
    }

    pub fn from_ops(ops: Vec<OpCode>) -> Pic12F508 {
        let mut flash: [u16; 512] = [0; 512];
        for (i, op) in ops.iter().enumerate() {
            flash[i] = op.to_owned().encode();
        }
        Pic12F508::new_from_flash(flash, ConfigWord::default())
    }

    pub fn from_ops_and_cfg(ops: Vec<OpCode>, cfg: ConfigWord) -> Pic12F508 {
        let mut flash: [u16; 512] = [0; 512];
        for (i, op) in ops.iter().enumerate() {
            flash[i] = op.to_owned().encode();
        }
        Pic12F508::new_from_flash(flash, cfg)
    }

    pub fn new() -> Pic12F508 {
        Pic12F508::new_from_flash([0; 512], ConfigWord::default())
    }

    fn new_from_flash(flash: [u16; 512], cfg_word: ConfigWord) -> Pic12F508 {
        if cfg_word.fosc != CfgFosc::INTRC {
            panic!("Only INTRC implemented.");
        }
        let mut cpu = Pic12F508 {
            ticks: 0,
            last_executed_op: None, // for debugging
            next_op: None,          // next fetched instruction
            pc_written: false,

            config: cfg_word,
            flash,
            sram: [0; 32],
            stack: [0; 2],

            w: 0, // will be overwritten with calibration value at reset
            pc: 511,
            trisgpio: 0b11_1111,
            option: OptionReg { bits: 0xff },

            input_buffer: [Input::Floating; 6],
            output_buffer: 0,

            indf: 0,             // INDF   xxxx xxxx
            tmr0: 0,             // tmr0   xxxx xxxx
            pcl: 0b11111111,     // PC     1111 1111
            status: 0b0001_1000, // STATUS 1111 1xxx
            fsr: 0b1110_0000,    // FSR    111x xxxx
            osccal: 0b1111_1110, // OSCCAL 1111 111x
            gpio: 0b00_0000,     // GPIO   111x xxxx

            prescaler: 0,
            tmr0_inhibit: 0,
            wd_timer: 0,
            state: CpuState::Normal,
            last_reset_reason: ResetReason::POR,
        };
        cpu.flash[511] = (OpCode::MOVLW { k: OSC_CALIB_VALUE }).encode(); // calibration value stored in W
        cpu.reset(ResetReason::POR);

        return cpu;
    }

    pub fn set_pin(&mut self, gpio_bit: u8, state: Input) {
        if gpio_bit > 5 {
            return;
        }
        let pin_mask = 1 << gpio_bit;
        let inputs_before = self.gpio & self.trisgpio;
        self.input_buffer[gpio_bit as usize] = state;
        self.set_gpio_inputs();
        let inputs_after = self.gpio & self.trisgpio;

        // Handle MCLRE
        if self.config.is_mclr_enabled() && gpio_bit == 3 {
            if inputs_after & pin_mask == 0 {
                match self.state {
                    CpuState::Sleep => self.reset(ResetReason::MCLRSleep),
                    CpuState::Normal => self.reset(ResetReason::MCLR),
                    CpuState::MclrLow => (),
                }
                // wait for mclr to turn on
                self.state = CpuState::MclrLow;
            } else if inputs_before & pin_mask == 0 && inputs_after & pin_mask > 0 {
                self.state = CpuState::Normal;
            }
        }

        // Handle Wake on pin change
        if !self.option.wake_on_pin_change_enabled() || !self.sleeping() {
            return;
        }
        let wake_on_change_mask = 0b1011 & pin_mask;
        println!("{inputs_before:08b}, {inputs_after:08b}, {wake_on_change_mask:08b}");
        if inputs_before & wake_on_change_mask != inputs_after & wake_on_change_mask {
            self.reset(ResetReason::PinChangeSleep);
        }
    }

    // Status flags
    pub fn status_c(&self) -> bool {
        (self.status & (1 << 0)) > 0
    }
    pub fn status_dc(&self) -> bool {
        (self.status & (1 << 1)) > 0
    }
    pub fn status_z(&self) -> bool {
        (self.status & (1 << 2)) > 0
    }
    pub fn status_pd(&self) -> bool {
        (self.status & (1 << 3)) > 0
    }
    pub fn status_to(&self) -> bool {
        (self.status & (1 << 4)) > 0
    }
    pub fn status_pa0(&self) -> bool {
        (self.status & (1 << 5)) > 0
    }
    pub fn status_gpwuf(&self) -> bool {
        (self.status & (1 << 7)) > 0
    }

    pub fn sleeping(&self) -> bool {
        self.state == CpuState::Sleep
    }

    pub fn in_normal_mode(&self) -> bool {
        if let CpuState::Normal = self.state {
            true
        } else {
            false
        }
    }

    fn affect_status_flag(&mut self, bit: StatusBit, val: bool) {
        let bit_mask = 1
            << match bit {
                StatusBit::C => 0,
                StatusBit::DC => 1,
                StatusBit::Z => 2,
                StatusBit::PD => 3,
                StatusBit::TO => 4,
                StatusBit::PA0 => 5,
                StatusBit::GPWUF => 7,
            };
        if val {
            self.status |= bit_mask;
        } else {
            self.status &= !bit_mask;
        }
    }

    fn affect_carry(&mut self, val: bool) {
        self.affect_status_flag(StatusBit::C, val);
    }

    fn affect_dc(&mut self, val: bool) {
        self.affect_status_flag(StatusBit::DC, val);
    }
    fn affect_zero(&mut self, val: bool) {
        self.affect_status_flag(StatusBit::Z, val);
    }
    fn affect_pd(&mut self, val: bool) {
        self.affect_status_flag(StatusBit::PD, val);
    }
    fn affect_to(&mut self, val: bool) {
        self.affect_status_flag(StatusBit::TO, val);
    }
    fn affect_pa0(&mut self, val: bool) {
        self.affect_status_flag(StatusBit::PA0, val);
    }
    fn affect_gpwuf(&mut self, val: bool) {
        self.affect_status_flag(StatusBit::GPWUF, val);
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
        self.ticks += 1;

        if let CpuState::Sleep = self.state {
            // TODO wake on pin change
            self.wd_tick(); // wd keeps running
            return;
        }

        if let CpuState::MclrLow = self.state {
            return;
        }

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
                self.tmr0_tick();
                self.wd_tick();
                self.update_regs_and_ram();
            }
            None => {
                // next_op is None first cycle after power on,
                // or after a branching instruction
                // PC is not increased in this case
                let current_pc_instruction: u16 = self.fetch(self.pc);
                let next_op_code: OpCode = OpCode::decode(current_pc_instruction);
                self.next_op = Some(next_op_code);
                self.last_executed_op = None;
                self.tmr0_tick();
                self.wd_tick();
                self.update_regs_and_ram();
            }
        };
    }

    // Updates registers and TMR0, and then updates
    // sram to reflect the current registers.
    fn update_regs_and_ram(&mut self) {
        // update sram to reflect SFRs
        // This is so instructions can read sram directly
        // to access SFRs
        // We could also make a special "read()" function
        // but this is easier.
        self.sram[TMR0] = self.tmr0;
        self.sram[PCL] = self.pcl;
        self.sram[STATUS] = self.status;
        self.sram[FSR] = self.fsr;
        self.sram[OSCCAL] = self.osccal;
        self.sram[GPIO] = self.gpio;

        // Cannot read INDF indirectly
        if self.fsr == 0 {
            self.indf = 0;
        } else {
            self.indf = self.sram[(self.fsr & 0b11111) as usize];
        }
        self.sram[INDF] = self.indf; // update indf last
    }

    pub fn tmr0_tick(&mut self) {
        if !self.option.tmr0_enabled() {
            return;
        }
        // Update timer0 only if cpu is in normal operation mode
        if !self.in_normal_mode() {
            return; // TMR0 stopped.
        }

        if self.tmr0_inhibit > 0 {
            if self.option.prescaler_assigned_to_tmr0() {
                self.prescaler = 0;
            }
            self.tmr0_inhibit -= 1;
            return;
        }

        // u16 to not overflow

        if self.option.prescaler_assigned_to_tmr0() {
            self.prescaler += 1;
            if self.prescaler >= self.option.prescaler_rate_tmr0() {
                self.prescaler = 0;
                let (t, _overflow) = self.tmr0.overflowing_add(1);
                self.tmr0 = t;
            }
        } else {
            let (t, _overflow) = self.tmr0.overflowing_add(1);
            self.tmr0 = t;
        }
    }

    fn wd_tick(&mut self) {
        if !self.config.is_wd_enabled() {
            return;
        }
        self.wd_timer += 1;
        let wd_wrap = 18000; // 18 ms nominal

        if self.wd_timer > wd_wrap {
            self.wd_timer = 0;
            if self.option.prescaler_assigned_to_wdt() {
                self.prescaler += 1;
                if self.prescaler >= self.option.prescaler_rate_wdt() {
                    // Watchdog timeout
                    self.prescaler = 0;
                    if let CpuState::Sleep = self.state {
                        self.reset(ResetReason::WDTSleep);
                    } else {
                        self.reset(ResetReason::WDT);
                    }
                }
            } else {
                // Watchdog timeout
                if let CpuState::Sleep = self.state {
                    self.reset(ResetReason::WDTSleep);
                } else {
                    self.reset(ResetReason::WDT);
                }
            }
        }
    }

    fn reset(&mut self, reason: ResetReason) {
        println!("Reset occurred. Reason: {reason:?} at ticks={}", self.ticks);
        self.state = CpuState::Normal;
        self.last_reset_reason = reason;
        self.next_op = None;
        // W gets loaded with oscillator calibration value by first instruction
        // indf unchanged
        // TMR0 unchanged
        self.pc = 511;

        let status = self.status;
        // Only status is affected by reason
        self.status = match reason {
            ResetReason::POR => 0b0001_1000,
            ResetReason::MCLR => status & 0b0001_1111,
            ResetReason::MCLRSleep => 0b0001_0000 | (status & 0b111),
            ResetReason::WDT => (status & 0b1111),
            ResetReason::WDTSleep => (status & 0b111),
            ResetReason::PinChangeSleep => 0b1001_0000 | (status & 0b111),
        };

        if reason == ResetReason::POR {
            self.fsr = 0b1110_0000;
            self.osccal = 0b1111_1110;
            self.tmr0 = 0;
            self.prescaler = 0;
        } 
        else {
            self.fsr |= 0b1110_0000;
        }

        self.option.bits = 0xff;
        self.trisgpio = 0b0011_1111;
        self.set_gpio_inputs();

        // update sram and sfrs
        self.update_regs_and_ram();

        // Tick past the first instruction so the first tick after
        // creating a CPU will be the first instruction, ie reset vector
        while self.pc > 0 {
            // PC rolls over after 2 cycles
            self.tick();
        }
    }

    fn set_gpio_inputs(&mut self) {
        for input in 0..6 {
            let mask = 1 << input;
            if (self.trisgpio & mask) != 0 || (input == PIN_MCLR && self.config.is_mclr_enabled()) {
                match self.input_buffer[input as usize] {
                    Input::Floating => {
                        if self.is_pin_pulled_up(input) {
                            self.gpio |= mask;
                        } else {
                            self.gpio &= !mask;
                        }
                    }
                    Input::Low => {
                        self.gpio &= !mask;
                    }
                    Input::High => {
                        self.gpio |= mask;
                    }
                }
            }
        }
    }

    pub fn execute_op_code(&mut self, code: OpCode) {
        match code {
            // It's like rust enums were made for this
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
        self.tmr0_inhibit = 3; // inhibit timer for 2 cycles + this cycle!
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
        // Output buffer is saved regardless of trisgpio
        self.output_buffer = value & 0b11_0111; // only 6 bits, GP3 is input only
        let outputs = self.output_buffer & !self.trisgpio;
        self.gpio = (self.gpio & self.trisgpio) | outputs;
    }

    fn is_pin_pulled_up(&self, gpio_bit: u8) -> bool {
        match gpio_bit {
            0 => self.option.weak_pull_ups_enabled(),
            1 => self.option.weak_pull_ups_enabled(),
            3 => self.option.weak_pull_ups_enabled() || self.config.is_mclr_enabled(),
            _ => false,
        }
    }
}

// Instructions
impl Pic12F508 {
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
        self.affect_pd(true);
        self.affect_to(true);
        self.wd_timer = 0;
        if self.option.prescaler_assigned_to_wdt() {
            self.prescaler = 0;
        }
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
        self.option.bits = self.w;
        // GPIO depends on option, refresh the pins here.
        self.set_gpio_inputs();
    }

    fn retlw(&mut self, k: u8) {
        // Pop stack and update pc
        self.w = k;
        self.pc = self.stack[0];
        self.stack[0] = self.stack[1];
        self.pc_written = true;
    }

    fn sleep(&mut self) {
        /*  00h → WDT;
        0 → WDT prescaler;
        1 → TO;
        0 → PD */
        self.wd_timer = 0;
        self.affect_to(true); // set TO
        self.affect_pd(false);
        if self.option.prescaler_assigned_to_wdt() {
            self.prescaler = 0;
        }
        self.state = CpuState::Sleep;
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
    use test::Bencher;

    #[test]
    fn test_run_cpu() {
        let ops = vec![OpCode::MOVLW { k: 0xff }, OpCode::MOVWF { f: 0x10 }];
        let mut cpu = Pic12F508::from_ops(ops);
        assert_eq!(cpu.next_op.unwrap(), OpCode::MOVLW { k: 0xff });
        assert_eq!(cpu.pc, 0); // has executed first MOVF op at PC=0x1ff
        assert_eq!(cpu.w, OSC_CALIB_VALUE); // contains osccal at start
        cpu.tick();
        assert_eq!(cpu.pc, 1);
        assert_eq!(cpu.next_op.unwrap(), OpCode::MOVWF { f: 0x10 });
    }

    #[test]
    fn calculated_jump_sets_pc() {
        let mut cpu = Pic12F508::from_ops(vec![
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
        // of the value written. This is done manually in any instruction
        // that modifies the PC
        let mut cpu = Pic12F508::from_ops(vec![OpCode::GOTO { k: 0 }]);
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
        const HEX: &str = ":10000000010AD70C02003E0C0600010C2100080A70
:10001000010243060C0A0E0A0605100A0604100A1D
:02002000080ACC
:021FFE00EA0FE8
:00000001FF
";
        let contents = HEX;
        let cpu: Pic12F508 = Pic12F508::from_hex(&contents);
        let mut cpu = test::black_box(cpu);
        b.iter(|| cpu.tick());
    }

    mod gpio {
        use super::*;

        #[test]
        fn gpio_writes_pin_values_not_latch() {
            todo!("When an I/O register is modified as a function of itself (e.g. MOVF PORTB, 1), the value used will be that
            value present on the pins themselves. For example, if the data latch is ‘1’ for a pin configured as input and
            is driven low by an external device, the data will be written back with a ‘0’.");
        }

        #[test]
        fn weak_pull_ups_set_input() {
            let mut cpu = Pic12F508::from_ops(vec![
                OpCode::MOVLW { k: 0b00111111 },
                OpCode::OPTION,
                OpCode::MOVLW { k: 0b01111111 },
                OpCode::OPTION,
            ]);
            assert_eq!(cpu.gpio, 0x0);
            cpu.run(2);
            assert_eq!(cpu.gpio, 0x0b);
            cpu.run(2);
            assert_eq!(cpu.gpio, 0x0);
        }

        #[test]
        fn weak_pull_ups_set_low_is_low() {
            let mut cpu = Pic12F508::from_ops(vec![OpCode::MOVLW { k: 0b00111111 }, OpCode::OPTION]);
            assert_eq!(cpu.gpio, 0x0);
            cpu.run(2);
            assert_eq!(cpu.gpio, 0x0b);
            cpu.set_pin(0, Input::Low);
            assert_eq!(cpu.gpio, 0x0a);
            cpu.set_pin(1, Input::Low);
            assert_eq!(cpu.gpio, 0x08);
            cpu.set_pin(3, Input::Low);
            assert_eq!(cpu.gpio, 0);
            cpu.set_pin(0, Input::Floating);
            cpu.set_pin(1, Input::Floating);
            cpu.set_pin(3, Input::Floating);
            assert_eq!(cpu.gpio, 0x0b);
        }

        #[test]
        fn write_gpio_sets_outputs() {
            let mut cpu = Pic12F508::from_ops(vec![
                OpCode::CLRW,
                OpCode::TRIS { f: GPIO as u8 },
                OpCode::BSF {
                    f: GPIO as u8,
                    b: 0,
                },
                OpCode::BSF {
                    f: GPIO as u8,
                    b: 1,
                },
                OpCode::BSF {
                    f: GPIO as u8,
                    b: 2,
                },
                OpCode::BSF {
                    f: GPIO as u8,
                    b: 3,
                }, // 3 is input only. nothing happens
                OpCode::BSF {
                    f: GPIO as u8,
                    b: 4,
                },
                OpCode::BSF {
                    f: GPIO as u8,
                    b: 5,
                },
            ]);
            cpu.run(2);
            assert_eq!(cpu.gpio, 0x00);
            cpu.tick();
            assert_eq!(cpu.gpio, 0x01);
            cpu.tick();
            assert_eq!(cpu.gpio, 0x03);
            cpu.tick();
            assert_eq!(cpu.gpio, 0x07);
            cpu.tick();
            assert_eq!(cpu.gpio, 0x07); // 3 is input only. nothing happens
            cpu.tick();
            assert_eq!(cpu.gpio, 0x17);
            cpu.tick();
            assert_eq!(cpu.gpio, 0x37);
        }
    }

    mod tmr0 {
        use super::*;

        #[test]
        fn clock_is_off_normally() {
            let mut cpu = Pic12F508::from_ops(vec![OpCode::GOTO { k: 0 }]);
            assert_eq!(cpu.tmr0, 0);
            cpu.run(100);
            assert_eq!(cpu.tmr0, 0);
        }


        #[test]
        fn write_tmr0_clears_prescaler() {
            let mut cpu = Pic12F508::from_ops(vec![
                OpCode::MOVLW { k: 0b11010111 }, // start timer with 1:256 prescale
                OpCode::OPTION,
                OpCode::MOVLW { k: 0xf0 },
                OpCode::MOVWF { f: 0x10 },
                OpCode::DECFSZ { f: 0x10, d: true },
                OpCode::GOTO { k: 4 },
                OpCode::CLRF { f: TMR0 as u8 },
            ]);
            assert_eq!(cpu.tmr0, 0);
            while cpu.pc < 7 {
                cpu.tick();
            }
            assert_eq!(cpu.prescaler, 0);
            
        }

        #[test]
        fn write_tmr0_when_disabled() {
            let mut cpu = Pic12F508::from_ops(vec![
                OpCode::MOVLW { k: 100 },
                OpCode::MOVWF { f: TMR0 as u8 },
            ]);
            cpu.run(2);
            assert_eq!(cpu.tmr0, 100);
        }

        #[test]
        fn clock_inc_one_per_cycle() {
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
                OpCode::MOVLW { k: STATUS as u8 },
                OpCode::MOVWF { f: FSR as u8 },
                OpCode::CLRW,
                OpCode::MOVF {
                    f: INDF as u8,
                    d: false,
                },
            ]);
            cpu.run(3);
            let status = cpu.status;
            cpu.tick();
            assert_eq!(cpu.w, status);
        }

        #[test]
        fn write_gpio_indf() {
            let mut cpu = Pic12F508::from_ops(vec![
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
            assert_eq!(cpu.gpio, 0b11_0111);
            assert_eq!(cpu.w, 0b11_0111);
        }
    }

    mod states {
        use super::*;

        #[test]
        fn mclr_holds_in_reset() {
            let cfg = ConfigWord {
                mclre: true,
                cp_disable: true,
                wdte: false,
                fosc: CfgFosc::INTRC,
            };
            let mut cpu = Pic12F508::from_ops_and_cfg(
                vec![
                    OpCode::MOVLW { k: 0b1100_1000 },
                    OpCode::OPTION,
                    OpCode::MOVLW { k: 0 },
                    OpCode::TRIS { f: GPIO as u8 },
                    OpCode::GOTO { k: 2 },
                ],
                cfg,
            );

            assert!(cpu.config.is_mclr_enabled());
            cpu.run(100);
            cpu.set_pin(3, Input::Low);

            // Actual reset happens on pin low transition
            assert_eq!(cpu.state, CpuState::MclrLow);
            assert!(!cpu.status_gpwuf()); // 0
            assert!(cpu.status_to()); // u
            assert!(cpu.status_pd()); // u
            assert_eq!(cpu.trisgpio, 0b11_1111);

            for _ in 0..1_000_000 {
                cpu.tick();
                assert_eq!(cpu.pc, 0); // waiting to run because mclr still low
            }
            cpu.set_pin(3, Input::High);
            assert_eq!(cpu.state, CpuState::Normal);
        }

        #[test]
        fn sleep_wake_by_mclr() {
            let cfg = ConfigWord {
                mclre: true,
                cp_disable: true,
                wdte: false,
                fosc: CfgFosc::INTRC,
            };
            let mut cpu = Pic12F508::from_ops_and_cfg(vec![OpCode::NOP, OpCode::SLEEP], cfg);
            assert!(cpu.config.is_mclr_enabled());
            cpu.run(2);
            assert_eq!(cpu.state, CpuState::Sleep);
            cpu.set_pin(3, Input::Low);
            assert_eq!(cpu.state, CpuState::MclrLow);
            for _ in 0..1_000 {
                cpu.tick();
                assert_eq!(cpu.pc, 0);
            }
            cpu.set_pin(3, Input::Floating);
            assert!(cpu.gpio & 0b1000 > 0); // Weakly pulled up
            assert_eq!(cpu.state, CpuState::Normal);
            assert!(!cpu.status_gpwuf()); // 0
            assert!(cpu.status_to()); //1
            assert!(!cpu.status_pd()); // 0
        }

        #[test]
        fn sleep_wake_by_wdt() {
            // todo!("WDT timeout during sleep should wake cpu and set correct bits.");
            let cfg = ConfigWord {
                mclre: false,
                cp_disable: true,
                wdte: true,
                fosc: CfgFosc::INTRC,
            };
            let mut cpu = Pic12F508::from_ops_and_cfg(
                vec![
                    OpCode::MOVLW { k: 0b1100_1000 },
                    OpCode::OPTION,
                    OpCode::SLEEP,
                ],
                cfg,
            );
            cpu.run(3);
            while cpu.pc > 0 {
                cpu.tick();
            }
            assert!(!cpu.status_to());
            assert!(!cpu.status_pd());
        }

        #[test]
        fn sleep_wdt_disabled_does_not_wake() {
            let cfg = ConfigWord {
                mclre: false,
                cp_disable: true,
                wdte: false,
                fosc: CfgFosc::INTRC,
            };
            let mut cpu = Pic12F508::from_ops_and_cfg(
                vec![
                    OpCode::MOVLW { k: 0b1100_1000 },
                    OpCode::OPTION,
                    OpCode::SLEEP,
                ],
                cfg,
            );
            cpu.run(1_000_000);
            assert!(cpu.status_to());
            assert!(!cpu.status_pd());
            assert_eq!(cpu.state, CpuState::Sleep);
        }

        #[test]
        fn por_registers_are_correct() {
            let mut cpu = Pic12F508::new();
            cpu.reset(ResetReason::POR);
            assert_eq!(cpu.w, OSC_CALIB_VALUE);
            // indf is undefined
            // tmr0 is undefined?
            assert_eq!(cpu.pc, 0); // 1111_1111 at por but executes one op to wrap to 0.

            assert_eq!(cpu.status & 0b1111_100, 0b0001_1000); // 0001 1xxx
            assert_eq!(cpu.fsr & 0b1110_0000, 0b1110_0000); // 111x xxxx
            assert_eq!(cpu.osccal, 0b1111_1110);
            assert_eq!(cpu.option.bits, 0xff);
            assert_eq!(cpu.trisgpio, 0x3f);
        }

        #[test]
        fn mclr_status_correct() {
            let mut cpu = Pic12F508::from_ops(vec![OpCode::CLRW]); // sets Z-flag
            cpu.run(1);
            let bits_before = cpu.status & 0b0001_1111;
            cpu.reset(ResetReason::MCLR);
            assert_eq!(cpu.status, bits_before);
        }

        #[test]
        fn mclr_sleep_status_correct() {
            let mut cpu = Pic12F508::from_ops(vec![OpCode::CLRW, OpCode::SLEEP]); // sets Z-flag
            cpu.run(2);
            let bits_before = cpu.status & 0b0001_1111;
            cpu.reset(ResetReason::MCLRSleep);
            assert_eq!(cpu.status, bits_before);
        }

        #[test]
        fn wdt_reset_resets_prescaler() {
            let mut cpu: Pic12F508 = Pic12F508::from_ops_and_cfg(
                vec![OpCode::GOTO { k: 0 }],
                ConfigWord {
                    mclre: true,
                    cp_disable: true,
                    wdte: true,
                    fosc: CfgFosc::INTRC,
                },
            );
            cpu.run(2304126);
            assert_eq!(cpu.last_reset_reason, ResetReason::WDT);
            cpu.reset(ResetReason::POR);
            cpu.run(2304126);
        }

        #[test]
        fn wdt_status_correct() {
            let mut cpu = Pic12F508::from_ops(vec![OpCode::CLRW]); // sets Z-flag
            cpu.run(1);
            let bits_before = cpu.status & 0b0000_1111;
            cpu.reset(ResetReason::WDT);
            assert_eq!(cpu.status, bits_before);
            assert!(!cpu.status_to()); // TO flag is inverted. 0 -> timeout
        }

        #[test]
        fn wdt_sleep_status_correct() {
            let mut cpu = Pic12F508::from_ops(vec![OpCode::CLRW]); // sets Z-flag
            cpu.run(1);
            let bits_before = cpu.status & 0b0000_0111;
            cpu.reset(ResetReason::WDTSleep);
            assert_eq!(cpu.status, bits_before);
            assert!(!cpu.status_to()); // TO flag is inverted. 0 -> timeout
            assert!(!cpu.status_pd()); // TO flag is inverted. 0 -> timeout
        }

        #[test]
        fn pin_change_wake_sleep_status_correct() {
            let cfg = ConfigWord {
                mclre: true,
                cp_disable: true,
                wdte: false,
                fosc: CfgFosc::INTRC,
            };
            let mut cpu = Pic12F508::from_ops_and_cfg(
                vec![OpCode::MOVLW { k: 0 }, OpCode::OPTION, OpCode::SLEEP],
                cfg,
            );
            cpu.run(3);
            assert_eq!(cpu.state, CpuState::Sleep);
            cpu.set_pin(4, Input::High);
            assert_eq!(cpu.state, CpuState::Sleep); // GP4 does not wake
            cpu.set_pin(1, Input::High);
            assert_eq!(cpu.state, CpuState::Sleep); // GP1 is pulled high
            cpu.set_pin(1, Input::Low);
            assert_eq!(cpu.last_reset_reason, ResetReason::PinChangeSleep);
            assert!(cpu.status_gpwuf()); // 1
            assert!(cpu.status_to()); // 1
            assert!(!cpu.status_pd()); // 0
        }
    }

    mod instructions {
        use super::*;

        #[test]
        fn addwf() {
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![OpCode::MOVLW { k: 0xa5 }, OpCode::CLRW]);
            cpu.tick();
            cpu.tick();
            assert_eq!(cpu.w, 0);
            assert!(cpu.status_z());
        }

        #[test]
        fn comf() {
            let mut cpu = Pic12F508::from_ops(vec![
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

            let mut cpu = Pic12F508::from_ops(ops);
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
            let mut cpu = Pic12F508::from_ops(vec![
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

            let mut cpu = Pic12F508::from_ops(ops);
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![OpCode::NOP]);
            let status = cpu.status;
            cpu.tick();
            assert_eq!(cpu.status, status);
        }

        #[test]
        fn rlf() {
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(ops);
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
            let mut cpu = Pic12F508::from_ops(ops);
            let mut expected = 0;
            for i in 0..8 {
                cpu.tick();
                expected |= 1 << i;
                assert_eq!(cpu.sram[16], expected);
            }
        }

        #[test]
        fn btfsc() {
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![OpCode::MOVLW { k: 0xaa }, OpCode::ANDLW { k: 0x55 }]);
            cpu.run(2);
            assert_eq!(cpu.w, 0x00);
        }

        #[test]
        fn call() {
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
                OpCode::MOVLW { k: 0b11001000 },
                OpCode::OPTION,
                OpCode::CLRWDT,
            ]);
            cpu.run(2);
            cpu.status &= 0b11100111; // manually clear these bits
            cpu.prescaler = 0xaa;
            assert!(cpu.option.prescaler_assigned_to_wdt());
            assert!(!cpu.status_pd());
            assert!(!cpu.status_to());
            cpu.tick();
            assert!(cpu.status_pd());
            assert!(cpu.status_to());
            assert_eq!(cpu.prescaler, 0);
        }

        #[test]
        fn goto() {
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![
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
            let mut cpu = Pic12F508::from_ops(vec![OpCode::MOVLW { k: 0xaa }, OpCode::OPTION]);
            assert_eq!(cpu.option.bits, 0xff);
            cpu.tick();
            cpu.tick();
            assert_eq!(cpu.option.bits, 0xaa);
        }

        #[test]
        fn retlw() {
            let mut cpu = Pic12F508::from_ops(vec![
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
            /* 00h → WDT;
            0 → WDT prescaler;
            1 → TO;
            0 → PD */
            let mut cpu = Pic12F508::from_ops(vec![
                OpCode::MOVLW { k: 0b11001111 }, // assign prescaler to WDT
                OpCode::OPTION,
                OpCode::SLEEP,
            ]);
            cpu.run(2);
            cpu.prescaler = 0xaa;
            cpu.tick();
            assert_eq!(cpu.wd_timer, 0);
            assert!(cpu.status_to());
            assert!(!cpu.status_pd());
            assert_eq!(cpu.prescaler, 0);
            assert_eq!(cpu.state, CpuState::Sleep);

            /* 00h → WDT;
            0 → WDT prescaler;
            1 → TO;
            0 → PD */
            let mut cpu = Pic12F508::from_ops(vec![
                OpCode::MOVLW { k: 0b11100111 }, // assign prescaler to tmr0
                OpCode::OPTION,
                OpCode::SLEEP,
            ]);
            cpu.run(2);
            cpu.prescaler = 0xaa;
            cpu.tick();
            assert_eq!(cpu.wd_timer, 0);
            assert!(cpu.status_to());
            assert!(!cpu.status_pd());
            assert_eq!(cpu.prescaler, 0xaa); // not affected
        }

        #[test]
        fn tris() {
            let mut cpu = Pic12F508::from_ops(vec![
                OpCode::MOVLW { k: 0b010101 },
                OpCode::TRIS { f: GPIO as u8 },
            ]);
            cpu.tick();
        }

        #[test]
        fn xorlw() {
            let mut cpu = Pic12F508::from_ops(vec![
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
