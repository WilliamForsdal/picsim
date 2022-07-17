
use ihex::{Reader, Record};

pub struct CPU {
    
    // Some instructions take 2 cycles and updates pc on the next cycle
    pub next_cycle_pc: Option<u16>, 
    pub nop_next: bool,

    pub last_executed_op: Option<OpCode>,

    //pub fetched: Option<OpCode>,

    pub cfg_word: u16,
    pub flash: [u16;512], // actually 12 bit words
    

    pub sram: [u8;32], // lower 6 not used, SRFs

    pub stack: [u16;2],

    pub w: u8,
    pub pc: u16, // actually 12 bits? or 10?
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
}

const INDF:    usize = 0;
const TMR0:    usize = 1;
const PCL:     usize = 2;
const STATUS:  usize = 3;
const FSR:     usize = 4;
const OSCCAL:  usize = 5;
const GPIO:    usize = 6;

impl CPU {

    pub fn new(program: &str) -> CPU {

        // parse hex file into flash etc.
        let mut cfg_word: u16  = 0;
        let mut flash: [u16;512] = [0;512];
        
        let reader = Reader::new(program);

        for a in reader {
            let record = a.unwrap();
            match record {
                Record::Data { offset, value } => {
                    if offset == 0x1ffe {
                        // configuration word
                        cfg_word = value[0] as u16 | ((value[1] as u16) << 8);
                    }
                    else {
                        let mut ptr = 0; // offset is in bytes, we want words
                        let mut flash_idx = (offset / 2) as usize;
                        while ptr < value.len() {
                            let val: u16 = value[ptr] as u16 | ((value[ptr+1] as u16) << 8);
                            flash[flash_idx] = val;
                            let op = CPU::decode_instruction(flash[flash_idx]);
                            println!("op {flash_idx:03}  {:?}", op);
                            ptr += 2;
                            flash_idx += 1;
                        }
                    }
                }
                Record::EndOfFile => break,
                Record::ExtendedSegmentAddress(_) => todo!(),
                Record::StartSegmentAddress { cs: _, ip: _ } => todo!(),
                Record::ExtendedLinearAddress(_) => todo!(),
                Record::StartLinearAddress(_) => todo!(),
            }
        }

        flash[511] = 0x0205; // MOVF OSCCAL, w


        let cpu = CPU {
            next_cycle_pc: None,
            nop_next: false,
            last_executed_op: None,

            cfg_word,
            flash,
            sram: [0;32],
            stack: [0;2],
            
            w: 0b11111110, // contains "calibration" value
            pc: 511,
            trisgpio: 0b11_1111,
            option: 0xff,

            input_buffer: 0,
            output_buffer: 0,

            indf: 0,               // INDF   xxxx xxxx
            tmr0: 0,               // tmr0   xxxx xxxx
            pcl: 0b11111111,       // PC     1111 1111
            status: 0b0001_1000,   // STATUS 1111 1xxx
            fsr: 0b1110_0000,      // FSR    111x xxxx
            osccal: 0b1111_1111,   // OSCCAL 111x xxxx
            gpio: 0b0000_0000,     // GPIO   111x xxxx

        };

        return cpu;
    }

    pub fn status_z(&self) -> bool {
        (self.status & 0b100)  > 0
    }

    pub fn status_c(&self) -> bool {
        (self.status & 0b10)  > 0
    }

    fn affect_zero(&mut self, val: bool) {
        if val {
            self.status |= 0b100;
       }
       else {
            self.status &= !0b100;
       }
   }
   
   fn affect_dc(&mut self, val: bool) {
       if val {
            self.status |= 0b10;
      }
      else {
            self.status &= !0b10;
      }
  }

   fn affect_carry(&mut self, val: bool) {
        if val {
            self.status |= 0b10;
       }
       else {
        self.status &= !0b10;
       }
   }


    // Ticks 4 clock cycles
    pub fn tick(&mut self) {

        // Some instructions "skip if.." execute
        // a NOP after a negative test
        if self.nop_next {
            self.nop_next = false;
            self.nop();
            self.last_executed_op = Some(OpCode::NOP);
            self.pc += 1;
            return;
        }

        // Handle second part of 2 cycle instruction
        if let Some(new_pc) = self.next_cycle_pc {
            self.next_cycle_pc = None;
            self.last_executed_op = None;
            self.pc = new_pc;
            return;
        }

        // update sram to reflect SFRs
        // This is so instructions can read sram directly
        // to access SFRs
        self.sram[INDF] = self.indf;
        self.sram[TMR0] = self.tmr0;
        self.sram[PCL] = self.pcl;
        self.sram[STATUS] = self.status;
        self.sram[FSR] = self.fsr;
        self.sram[OSCCAL] = self.osccal;
        self.sram[GPIO] = self.gpio;

        // fetch instruction
        let instruction: u16 = self.flash[self.pc as usize];

        // decode
        let op_code: OpCode = CPU::decode_instruction(instruction);
        
        let pcl_before = self.pcl; // save pcl if it is changed by an instruction

        // execute
        self.execute_op_code(op_code);
        // save last op-code for debugging
        self.last_executed_op = Some(op_code);


        // check writes to PCL. Should update PC
        if self.next_cycle_pc.is_some() {
            // Don't update PC. Next cycle should do that.
        }
        else if self.pcl != pcl_before {
            // wrote to pcl. should update pc
            // PIC12F508 only allows computed jumps to pc=0-255: bit8 is always cleared
            self.pc = self.pcl as u16;

            // Any program branch flushes the currently fetched instruction
            // So even a movf -> pcl will take 2 cycles.
            self.nop_next = true; 
        }
        else {
            self.pc += 1; // increase PC as usual
        }

        // Wrap around if we index outside
        self.pc &= 0x1ff;
        
        // update sfr
        self.pcl = (self.pc & 0xff) as u8;
        
    }



    fn decode_instruction(instruction: u16) -> OpCode {
        let k8bit = (instruction & 0xff) as u8;
        let k9bit = instruction & 0x1ff;
        let b = ((instruction >> 5) & 7) as u8;
        let d = instruction & 0b100000 > 0;
        let f5bit = (instruction & 0b11111) as u8;

        // match with longest first, then shorter
        match instruction {
            0b0000_0100_0000 => return OpCode::CLRW,
            0b0000_0000_0000 => return OpCode::NOP,
            0b0000_0000_0100 => return OpCode::CLRWDT,
            0b0000_0000_0010 => return OpCode::OPTION,
            0b0000_0000_0011 => return OpCode::SLEEP,
            _ => ()
        };

        match instruction >> 3 {
            0 => return OpCode::TRIS { f: (instruction as u8) & 0b111 },
            _ => (),            
        }
    
        match instruction >> 6 {
            0b0001_11 => return OpCode::ADDWF { f: f5bit, d: d },
            0b0001_01 => return OpCode::ANDWF {f: f5bit, d: d},
            0b0000_01 => return OpCode::CLRF { f: f5bit }, // this could also match CLRW, but that matched earlier
            0b0010_01 => return OpCode::COMF { f: f5bit, d: d },
            0b0000_11 => return OpCode::DECF { f: f5bit, d: d },
            0b0010_11 => return OpCode::DECFSZ { f: f5bit, d: d },
            0b0010_10 => return OpCode::INCF { f: f5bit, d: d },
            0b0011_11 => return OpCode::INCFSZ { f: f5bit, d: d },
            0b0001_00 => return OpCode::IORWF { f: f5bit, d: d },
            0b0010_00 => return OpCode::MOVF { f: f5bit, d: d },
            0b0000_00 => return OpCode::MOVWF { f: f5bit },
            0b0011_01 => return OpCode::RLF { f: f5bit, d: d },
            0b0011_00 => return OpCode::RRF { f: f5bit, d: d },
            0b0000_10 => return OpCode::SUBWF { f: f5bit, d: d },
            0b0011_10 => return OpCode::SWAPF { f: f5bit, d: d },
            0b0001_10 => return OpCode::XORWF { f: f5bit, d: d },

            _ => ()
        };

        // match shorter                 
        // match first group
        match instruction >> 8 {
    
            // bit oriented instructions
            0b0100 => return OpCode::BCF { f: f5bit, b: b },
            0b0101 => return OpCode::BSF { f: f5bit, b: b },
            0b0110 => return OpCode::BTFSC { f: f5bit, b: b },
            0b0111 => return OpCode::BTFSS { f: f5bit, b: b },
    
            // literal and ctrl
            0b1110 => return OpCode::ANDLW { k: k8bit },
            0b1001 => return OpCode::CALL { k: k8bit },
            0b1101 => return OpCode::IORLW { k: k8bit },
            0b1100 => return OpCode::MOVLW { k: k8bit },
            0b1000 => return OpCode::RETLW { k: k8bit },
            0b1111 => return OpCode::XORLW { k: k8bit },
            _ => (),
        };
    
        match instruction >> 9 {
            0b101 => return OpCode::GOTO { k: k9bit },
            _ => (),
        };
        panic!("instruction {instruction} failed to  match!");
    }
    
    pub fn execute_op_code(&mut self, code: OpCode) {
        match code {
            OpCode::ADDWF { f, d } => self.addwf(f, d),
            OpCode::ANDWF { f, d } => self.andwf(f, d),
            OpCode::CLRF { f } => self.clrf(f),
            OpCode::CLRW => self.clrw(),
            OpCode::COMF { f, d } => self.comf(f,d),
            OpCode::DECF { f, d } => self.decf(f,d),
            OpCode::DECFSZ { f, d } => self.decfsz(f,d),
            OpCode::INCF { f, d }   => self.incf(f,d),
            OpCode::INCFSZ { f, d } => self.incfsz(f,d),
            OpCode::IORWF { f, d }  => self.iorwf(f,d),
            OpCode::MOVF { f, d }   => self.movf(f,d),
            OpCode::MOVWF { f } => self.movwf(f),
            OpCode::NOP => self.nop(),
            OpCode::RLF { f, d } => self.rlf(f,d),
            OpCode::RRF { f, d } => self.rrf(f,d),
            OpCode::SUBWF { f, d } => self.subwf(f,d),
            OpCode::SWAPF { f, d } => self.swapf(f,d),
            OpCode::XORWF { f, d } => self.xorwf(f,d),

            // bit oriented operations
            OpCode::BCF { f, b } => self.bcf(f,b),
            OpCode::BSF { f, b } => self.bsf(f,b),
            OpCode::BTFSC { f, b } => self.btfsc(f,b),
            OpCode::BTFSS { f, b } => self.btfss(f,b),

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
        }
    }

    fn addwf(&mut self, f: u8, d: bool) {
        let a = self.sram[f as usize];
        let b = self.w;
        let result: u32 = a as u32 + b as u32;
        self.affect_carry(result > 255);
        self.affect_zero(result % 256 == 0);
        self.affect_dc(a & 0xf + b & 0xf > 15);

        if d {
            self.write(f, result as u8);
        }
        else {
            self.w = result as u8;
        }
    }

    fn andwf(&mut self, f: u8, d: bool) {
        let result = self.w & self.sram[f as usize];
        if d {
            self.write(f, result as u8);
        }
        else {
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
        }
        else {
            self.w = result;
        }
        self.affect_zero(result == 0);
    }

    fn decf(&mut self, f: u8, d: bool) {
        let result = self.sram[f as usize] - 1;
        if d {
            self.write(f, result);
        }
        else {
            self.w = result;
        }
        self.affect_zero(result == 0);
    }

    fn decfsz(&mut self, f: u8, d: bool) {
        let result = self.sram[f as usize] - 1;
        if d {
            self.write(f, result);
        }
        else {
            self.w = result;
        }
        // If result is 0, execute a nop instead of next instruction
        if result == 0 {
            self.nop_next = true;
        }
    }

    fn incf(&mut self, f: u8, d: bool) {
        let result = self.sram[f as usize] + 1;
        if d {
            self.write(f, result);
        }
        else {
            self.w = result;
        }
        self.affect_zero(result == 0);
    }

    fn incfsz(&mut self, f: u8, d: bool) {
        let result = self.sram[f as usize] + 1;
        if d {
            self.write(f, result);
        }
        else {
            self.w = result;
        }
        // If result is 0, execute a nop instead of next instruction
        if result == 0 {
            self.nop_next = true;
        }
    }

    fn iorwf(&mut self, f: u8, d: bool) {
        let result = self.w | self.sram[f as usize];
        if d {
            self.write(f, result);
        }
        else {
            self.w = result;
        }
        self.affect_zero(result == 0);
    }

    fn movf(&mut self, f: u8, d: bool) {
        let result = self.sram[f as usize];
        if d {
            // 
        }
        else {
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
        }
        else {
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
        }
        else {
            self.w = result;
        }
    }

    fn subwf(&mut self, f: u8, d: bool) {
        let a = self.sram[f as usize];
        let b = self.w;
        let result = a - b;
        self.affect_carry(b > a);
        self.affect_zero(b == a);
        self.affect_dc(a & 0xf < b & 0xf); // not sure about this one
        if d {
            self.write(f, result);
        }
        else {
            self.w = result;
        }
    }

    fn swapf(&mut self, f: u8, d: bool) {
        let value = self.sram[f as usize];
        let result = (value & 0xf) << 4 | (value >> 4);
        if d {
            self.write(f, result);
        }
        else {
            self.w = result;
        }
    }

    fn xorwf(&mut self, f: u8, d: bool) {
        let result = self.sram[f as usize] ^ self.w;
        self.affect_zero(result == 0);
        if d {
            self.write(f, result);
        }
        else {
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
            // skip next instruction
            self.nop_next = true;
        }
    }

    fn btfss(&mut self, f: u8, b: u8) {
        if self.sram[f as usize] & (1 << b) != 0 {
            // skip next instruction
            self.nop_next = true;
        }
    }

    fn andlw(&mut self, k: u8) {
        self.w = self.w & k;
        self.affect_zero(self.w == 0);
    }

    fn call(&mut self, k: u8) {
        self.stack[1] = self.stack[0];
        self.stack[0] = self.pc + 1;
        let new_pc = k as u16 | ((self.status & 0b0110_0000) as u16) << 4;
        self.next_cycle_pc = Some(new_pc);
    }

    fn clrwdt(&mut self) {
        todo!()
    }

    fn goto(&mut self, k: u16) {
        let new_pc = k;
        // We don't use paging on PIC12F508
        //new_pc |= (self.status() as u16 & 0b0011_0000) << 4;
        self.next_cycle_pc = Some(new_pc);
    }

    fn iorlw(&mut self, k: u8) {
        self.w = self.w | k;
    }

    fn movlw(&mut self, k: u8) {
        self.w = k;
    }

    fn option(&mut self) {
        todo!()
    }

    fn retlw(&mut self, k: u8) {
        // Pop stack and set pc next cycle
        self.w = k;
        self.next_cycle_pc = Some(self.stack[0]);
        self.stack[0] = self.stack[1];
    }

    fn sleep(&mut self) {
        todo!()
    }

    fn tris(&mut self, f: u8) {
        if f == 6 {
            self.trisgpio = self.w;
        }
    }

    fn xorlw(&mut self, k: u8) {
        self.w = self.w ^ k;
        self.affect_zero(self.w == 0);
    }

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
        }
        else {
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

    fn write_tmr0(&mut self, _value: u8) {
        todo!()
    }

    fn write_pcl(&mut self, value: u8) {
        self.pcl = value;
        // Writing to PCL updates the program counter and
        // results in a 2 cycle instruction
    }

    fn write_status(&mut self, value: u8) {
        self.status = value & 0b11100111;
    }

    fn write_fsr(&mut self, value: u8) {
        self.fsr = value & 0b11111; // 5 bit wide
    }

    fn write_osccal(&mut self, value: u8) {
        self.osccal = value & !1; // bit0 unimplemented
    }

    fn write_gpio(&mut self, value: u8) {
        // Writing to GPIO sets outputs
        self.output_buffer = value & !self.trisgpio;
    }




}

#[derive(Debug, Clone, Copy)]
pub enum OpCode {
    ADDWF {
        f: u8,
        d: bool
    },
    ANDWF{
        f: u8,
        d: bool
    },
    CLRF {
        f: u8,
    },
    CLRW,
    COMF {
        f: u8,
        d: bool
    },
    DECF {
        f: u8,
        d: bool
    },
    DECFSZ {
        f: u8,
        d: bool
    },
    INCF {
        f: u8,
        d: bool
    },
    INCFSZ {
        f: u8,
        d: bool
    },
    IORWF {
        f: u8,
        d: bool
    },
    MOVF {
        f: u8,
        d: bool
    },
    MOVWF {
        f: u8,
    },
    NOP,
    RLF {
        f: u8,
        d: bool
    },
    RRF {
        f: u8,
        d: bool
    },
    SUBWF {
        f: u8,
        d: bool
    },
    SWAPF {
        f: u8,
        d: bool
    },
    XORWF {
        f: u8,
        d: bool
    },

    BCF {
        f: u8,
        b: u8
    },

    BSF {
        f: u8,
        b: u8
    },
    BTFSC {
        f: u8,
        b: u8
    },
    BTFSS {
        f: u8,
        b: u8
    },
    
    ANDLW {
        k: u8,
    },
    CALL {
        k: u8,
    },
    CLRWDT,
    GOTO {
        k: u16 // 9 bit value!
    },
    IORLW {
        k: u8,
    },
    MOVLW {
        k: u8,

    },
    OPTION,
    RETLW {
        k: u8,
    },
    SLEEP,
    TRIS {
        f: u8,
    },
    XORLW {
        k: u8,
    },
}
