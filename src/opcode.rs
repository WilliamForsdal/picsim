use ihex::{Reader, Record};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OpCode {
    ADDWF { f: u8, d: bool },
    ANDWF { f: u8, d: bool },
    CLRF { f: u8 },
    CLRW,
    COMF { f: u8, d: bool },
    DECF { f: u8, d: bool },
    DECFSZ { f: u8, d: bool },
    INCF { f: u8, d: bool },
    INCFSZ { f: u8, d: bool },
    IORWF { f: u8, d: bool },
    MOVF { f: u8, d: bool },
    MOVWF { f: u8 },
    NOP,
    RLF { f: u8, d: bool },
    RRF { f: u8, d: bool },
    SUBWF { f: u8, d: bool },
    SWAPF { f: u8, d: bool },
    XORWF { f: u8, d: bool },

    BCF { f: u8, b: u8 },
    BSF { f: u8, b: u8 },
    BTFSC { f: u8, b: u8 },
    BTFSS { f: u8, b: u8 },

    ANDLW { k: u8 },
    CALL { k: u8 },
    CLRWDT,
    GOTO { k: u16 }, // 9 bit value for GOTO
    IORLW { k: u8 },
    MOVLW { k: u8 },
    OPTION,
    RETLW { k: u8 },
    SLEEP,
    TRIS { f: u8 },
    XORLW { k: u8 },

    INVALID { code: u16 },
}

impl OpCode {
    pub fn decode(instruction: u16) -> OpCode {
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
            _ => (),
        };

        match instruction >> 3 {
            0 => {
                return OpCode::TRIS {
                    f: (instruction as u8) & 0b111,
                }
            }
            _ => (),
        }

        match instruction >> 6 {
            0b0001_11 => return OpCode::ADDWF { f: f5bit, d: d },
            0b0001_01 => return OpCode::ANDWF { f: f5bit, d: d },
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

            _ => (),
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
        panic!("Failed to decode instruction {instruction:03x}.");
    }

    pub fn encode(self) -> u16 {
        match self {
            OpCode::ADDWF { f, d } => 0b0001_1100_0000 | f as u16 | if d { 0b100000 } else { 0 },
            OpCode::ANDWF { f, d } => 0b0001_01 << 6 | f as u16 | if d { 0b100000 } else { 0 },
            OpCode::CLRF { f } => 0b0000_011 << 5 | f as u16,
            OpCode::CLRW => 0b0000_0100_0000,
            OpCode::COMF { f, d } => 0b0010_01 << 6 | f as u16 | if d { 0b100000 } else { 0 },
            OpCode::DECF { f, d } => 0b0000_11 << 6 | f as u16 | if d { 0b100000 } else { 0 },
            OpCode::DECFSZ { f, d } => 0b0010_11 << 6 | f as u16 | if d { 0b100000 } else { 0 },
            OpCode::INCF { f, d } => 0b0010_10 << 6 | f as u16 | if d { 0b100000 } else { 0 },
            OpCode::INCFSZ { f, d } => 0b0011_11 << 6 | f as u16 | if d { 0b100000 } else { 0 },
            OpCode::IORWF { f, d } => 0b0001_00 << 6 | f as u16 | if d { 0b100000 } else { 0 },
            OpCode::MOVF { f, d } => 0b0010_00 << 6 | f as u16 | if d { 0b100000 } else { 0 },
            OpCode::MOVWF { f } => 0b0000_001 << 5 | f as u16,
            OpCode::NOP => 0b0000_0000_0000,
            OpCode::RLF { f, d } => 0b0011_01 << 6 | f as u16 | if d { 0b100000 } else { 0 },
            OpCode::RRF { f, d } => 0b0011_00 << 6 | f as u16 | if d { 0b100000 } else { 0 },
            OpCode::SUBWF { f, d } => 0b0000_10 << 6 | f as u16 | if d { 0b100000 } else { 0 },
            OpCode::SWAPF { f, d } => 0b0011_10 << 6 | f as u16 | if d { 0b100000 } else { 0 },
            OpCode::XORWF { f, d } => 0b0001_10 << 6 | f as u16 | if d { 0b100000 } else { 0 },

            OpCode::BCF { f, b } => 0b0100 << 8 | f as u16 | (b as u16) << 5,
            OpCode::BSF { f, b } => 0b0101 << 8 | f as u16 | (b as u16) << 5,
            OpCode::BTFSC { f, b } => 0b0110 << 8 | f as u16 | (b as u16) << 5,
            OpCode::BTFSS { f, b } => 0b0111 << 8 | f as u16 | (b as u16) << 5,
            OpCode::ANDLW { k } => 0b1110 << 8 | k as u16,
            OpCode::CALL { k } => 0b1001 << 8 | k as u16,
            OpCode::CLRWDT => 0b0000_0000_0100,
            OpCode::GOTO { k } => (0b101 << 9) | k,
            OpCode::IORLW { k } => 0b1101 << 8 | k as u16,
            OpCode::MOVLW { k } => 0b1100 << 8 | k as u16,
            OpCode::OPTION => 0b0000_0000_0010,
            OpCode::RETLW { k } => 0b1000 << 8 | k as u16,
            OpCode::SLEEP => 0b0000_0000_0011,
            OpCode::TRIS { f } => 0b0 | f as u16,
            OpCode::XORLW { k } => 0b1111 << 8 | k as u16,

            OpCode::INVALID { code } => code, // stop encoding invalid codes!!
        }
    }

    pub fn compile(_asm: &str) -> Vec<OpCode> {
        // split into lines
        // find index of labels
        // parse instructions
        // encode into hex
        todo!();
    }

    pub fn from_hex(hex: &str) -> ([u16; 512], u16) {
        // parse hex file into flash etc.
        let mut cfg_word: u16 = 4074;
        let mut flash: [u16; 512] = [0; 512];

        let reader = Reader::new(hex);

        for a in reader {
            let record = a.unwrap();
            match record {
                Record::Data { offset, value } => {
                    if offset == 0x1ffe {
                        // configuration word
                        cfg_word = value[0] as u16 | ((value[1] as u16) << 8);
                    } else {
                        let mut ptr = 0; // offset is in bytes, we want words
                        let mut flash_idx = (offset / 2) as usize;
                        while ptr < value.len() {
                            let val: u16 = value[ptr] as u16 | ((value[ptr + 1] as u16) << 8);
                            flash[flash_idx] = val;
                            let op = OpCode::decode(flash[flash_idx]);
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

        (flash, cfg_word)
    }
}

#[cfg(test)]
mod tests {
    use super::OpCode;

    #[test]
    fn test_decode_all() {
        // This hex string contains all 33 instructions
        let hex = ":10000000FF0C3000F001500170007002F000F002AF
:10001000B002F0033001310270033003B000B003CE
:10002000B001F00550045006F007AA0E1E090400A6
:10003000550D020003000600550F000A00080000DD
:021FFE00EA0FE8
:00000001FF
";
        let (flash, cfg) = OpCode::from_hex(&hex);
        assert_eq!(cfg, 0b1111_1110_1010);

        let codes = flash.map(|f| OpCode::decode(f));
        assert_eq!(codes[0], OpCode::MOVLW { k: 0xff });
        assert_eq!(codes[1], OpCode::MOVWF { f: 0x10 });
        assert_eq!(codes[2], OpCode::ADDWF { f: 0x10, d: true });
        assert_eq!(codes[3], OpCode::ANDWF { f: 0x10, d: false });
        assert_eq!(codes[4], OpCode::CLRF { f: 0x10 });
        assert_eq!(codes[5], OpCode::COMF { f: 0x10, d: true });
        assert_eq!(codes[6], OpCode::DECF { f: 0x10, d: true });
        assert_eq!(codes[7], OpCode::DECFSZ { f: 0x10, d: true });
        assert_eq!(codes[8], OpCode::INCF { f: 0x10, d: true });
        assert_eq!(codes[9], OpCode::INCFSZ { f: 0x10, d: true });
        assert_eq!(codes[10], OpCode::IORWF { f: 0x10, d: true });
        assert_eq!(codes[11], OpCode::MOVF { f: 0x11, d: true });
        assert_eq!(codes[12], OpCode::RLF { f: 0x10, d: true });
        assert_eq!(codes[13], OpCode::RRF { f: 0x10, d: true });
        assert_eq!(codes[14], OpCode::SUBWF { f: 0x10, d: true });
        assert_eq!(codes[15], OpCode::SWAPF { f: 0x10, d: true });
        assert_eq!(codes[16], OpCode::XORWF { f: 0x10, d: true });
        assert_eq!(codes[17], OpCode::BSF { f: 0x10, b: 7 });
        assert_eq!(codes[18], OpCode::BCF { f: 0x10, b: 2 });
        assert_eq!(codes[19], OpCode::BTFSC { f: 0x10, b: 2 });
        assert_eq!(codes[20], OpCode::BTFSS { f: 0x10, b: 7 });
        assert_eq!(codes[21], OpCode::ANDLW { k: 0xaa });
        assert_eq!(codes[22], OpCode::CALL { k: 30 });
        assert_eq!(codes[23], OpCode::CLRWDT);
        assert_eq!(codes[24], OpCode::IORLW { k: 0x55 });
        assert_eq!(codes[25], OpCode::OPTION);
        assert_eq!(codes[26], OpCode::SLEEP);
        assert_eq!(codes[27], OpCode::TRIS { f: 6 });
        assert_eq!(codes[28], OpCode::XORLW { k: 0x55 });
        assert_eq!(codes[29], OpCode::GOTO { k: 0 });
        assert_eq!(codes[30], OpCode::RETLW { k: 0 });
    }

    #[test]
    fn encode_all() {
        let codes = vec![
            OpCode::MOVLW { k: 0xff },
            OpCode::MOVWF { f: 0x10 },
            OpCode::ADDWF { f: 0x10, d: true },
            OpCode::ANDWF { f: 0x10, d: false },
            OpCode::CLRF { f: 0x10 },
            OpCode::COMF { f: 0x10, d: true },
            OpCode::DECF { f: 0x10, d: true },
            OpCode::DECFSZ { f: 0x10, d: true },
            OpCode::INCF { f: 0x10, d: true },
            OpCode::INCFSZ { f: 0x10, d: true },
            OpCode::IORWF { f: 0x10, d: true },
            OpCode::MOVF { f: 0x11, d: true },
            OpCode::RLF { f: 0x10, d: true },
            OpCode::RRF { f: 0x10, d: true },
            OpCode::SUBWF { f: 0x10, d: true },
            OpCode::SWAPF { f: 0x10, d: true },
            OpCode::XORWF { f: 0x10, d: true },
            OpCode::BSF { f: 0x10, b: 7 },
            OpCode::BCF { f: 0x10, b: 2 },
            OpCode::BTFSC { f: 0x10, b: 2 },
            OpCode::BTFSS { f: 0x10, b: 7 },
            OpCode::ANDLW { k: 0xaa },
            OpCode::CALL { k: 30 },
            OpCode::CLRWDT,
            OpCode::IORLW { k: 0x55 },
            OpCode::OPTION,
            OpCode::SLEEP,
            OpCode::TRIS { f: 6 },
            OpCode::XORLW { k: 0x55 },
            OpCode::GOTO { k: 0 },
            OpCode::RETLW { k: 0 },
            OpCode::NOP,
        ];
        // This hex string contains all 33 instructions
        let hex = ":10000000FF0C3000F001500170007002F000F002AF
:10001000B002F0033001310270033003B000B003CE
:10002000B001F00550045006F007AA0E1E090400A6
:10003000550D020003000600550F000A00080000DD
:021FFE00EA0FE8
:00000001FF
";
        let (flash, _) = OpCode::from_hex(&hex);

        for (i, code) in codes.iter().enumerate() {
            let encoded = code.encode();
            println!("{i}");
            assert_eq!(flash[i], encoded);
        }

    }
}
