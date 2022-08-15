pub trait Baseline {
    // byte oriented file register operations
    fn addwf(&mut self, f: u8, d: bool);
    fn andwf(&mut self, f: u8, d: bool);
    fn clrf(&mut self, f: u8);
    fn clrw(&mut self);
    fn comf(&mut self, f: u8, d: bool);
    fn decf(&mut self, f: u8, d: bool);
    fn decfsz(&mut self, f: u8, d: bool);
    fn incf(&mut self, f: u8, d: bool);
    fn incfsz(&mut self, f: u8, d: bool);
    fn iorwf(&mut self, f: u8, d: bool);
    fn movf(&mut self, f: u8, d: bool);
    fn movwf(&mut self, f: u8);
    fn nop(&mut self);
    fn rlf(&mut self, f: u8, d: bool);
    fn rrf(&mut self, f: u8, d: bool);
    fn subwf(&mut self, f: u8, d: bool);
    fn swapf(&mut self, f: u8, d: bool);
    fn xorwf(&mut self, f: u8, d: bool);

    // bit oriented file register operations
    fn bcf(&mut self, f: u8, b: u8);
    fn bsf(&mut self, f: u8, b: u8);
    fn btfsc(&mut self, f: u8, b: u8);
    fn btfss(&mut self, f: u8, b: u8);

    // literan and control operations
    fn andlw(&mut self, k: u8);
    fn call(&mut self, k: u8);
    fn clrwdt(&mut self);
    fn goto(&mut self, k: u16);
    fn iorlw(&mut self, k: u8);
    fn movlw(&mut self, k: u8);
    fn option(&mut self);
    fn retlw(&mut self, k: u8);
    fn sleep(&mut self);
    fn tris(&mut self, f: u8);
    fn xorlw(&mut self, k: u8);
}
