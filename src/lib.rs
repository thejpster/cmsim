//! An ARMv6-M processor simulator.
//!
//! Designed to run on `no_std` systems (although at the moment it's full of
//! println! calls).

use core::num::Wrapping;

/// All the ways we can fail to execute code
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    InvalidAddress,
    InvalidInstruction,
    UnalignedAccess,
    NonThumbPc
}

/// Describes a block of 32-bit memory.
///
/// 16-bit and 8-bit operations are done with a read, modify, write on the
/// appropriate 32-bit word. Unaligned accesses are rejected.
pub trait Memory {
    /// Fetch a 32-bit value from memory at the given address.
    ///
    /// Gives an error if the address is out-of-bounds or not aligned.
    fn load_u32(&self, addr: u32) -> Result<u32, Error>;

    /// Store a 32-bit value into memory at the given address.
    ///
    /// Gives an error if the address is out-of-bounds or not aligned.
    fn store_u32(&mut self, addr: u32, value: u32) -> Result<(), Error>;

    /// Fetch a 16-bit value from memory at the given address.
    ///
    /// Gives an error if the address is out-of-bounds or not aligned.
    fn load_u16(&self, addr: u32) -> Result<u16, Error> {
        if addr & 0b1 != 0 {
            return Err(Error::UnalignedAccess);
        }
        let word = self.load_u32(addr & !0b11)?;
        // Get the appropriate half of the 32-bit word
        if addr & 0b10 != 0 {
            Ok((word >> 16) as u16)
        } else {
            Ok(word as u16)
        }
    }

    /// Fetch an 8-bit value from memory at the given address.
    ///
    /// Gives an error if the address is out-of-bounds.
    fn load_u8(&self, addr: u32) -> Result<u8, Error> {
        let word = self.load_u16(addr & !0b1)?;
        // Get the appropriate half of the 16-bit word
        if addr & 0b1 != 0 {
            Ok((word >> 8) as u8)
        } else {
            Ok(word as u8)
        }
    }

    /// Store a 16-bit value into memory at the given address.
    ///
    /// Gives an error if the address is out-of-bounds or not aligned.
    fn store_u16(&mut self, addr: u32, value: u16) -> Result<(), Error> {
        if addr & 0b1 != 0 {
            return Err(Error::UnalignedAccess);
        }
        // Read, Modify, Write the 32-bit word
        let mut word = self.load_u32(addr & !0b11)?;
        if addr & 0b10 != 0 {
            word &= 0x0000_FFFF;
            word |= u32::from(value) << 16;
        } else {
            word &= 0xFFFF_0000;
            word |= u32::from(value);
        }
        self.store_u32(addr & !0b11, word)
    }

    /// Store an 8-bit value into memory at the given address.
    ///
    /// Gives an error if the address is out-of-bounds.
    fn store_u8(&mut self, addr: u32, value: u8) -> Result<(), Error> {
        // Read, Modify, Write a 16-bit word
        let mut word = self.load_u16(addr & !0b1)?;
        if addr & 0b1 != 0 {
            word &= 0x00FF;
            word |= u16::from(value) << 8;
        } else {
            word &= 0xFF00;
            word |= u16::from(value);
        }
        self.store_u16(addr & !0b1, word)
    }
}

impl Memory for [u32] {
    fn load_u32(&self, addr: u32) -> Result<u32, Error> {
        self.get(addr as usize)
            .copied()
            .ok_or(Error::InvalidAddress)
    }

    fn store_u32(&mut self, addr: u32, value: u32) -> Result<(), Error> {
        *self.get_mut(addr as usize).ok_or(Error::InvalidAddress)? = value;
        Ok(())
    }
}

impl<const N: usize> Memory for [u32; N] {
    fn load_u32(&self, addr: u32) -> Result<u32, Error> {
        self.get(addr as usize)
            .copied()
            .ok_or(Error::InvalidAddress)
    }

    fn store_u32(&mut self, addr: u32, value: u32) -> Result<(), Error> {
        *self.get_mut(addr as usize).ok_or(Error::InvalidAddress)? = value;
        Ok(())
    }
}

/// All the ARMv6-M supported instructions
///
/// TODO:
///
/// DC, ADD, ADR, AND, ASR, BIC, BKPT, BLX, BX, CMN, CMP, CPS, EOR, LDM, LDR,
/// LDRB, LDRH, LDRSB, LDRSH, LSL, LSR, MUL, MVN, NOP, ORR, POP, PUSH, REV,
/// REV16, REVSH, ROR, RSB, SBC, SEV, STM, STR, STRB, STRH, SUB, SVC, SXTB,
/// SXTH, TST, UXTB, UXTH, WFE, WFI, YIELD
///
/// BL, DMB, DSB, ISB, MRS, MSR
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Instruction {
    Branch { imm11: i16 },
    Mov { rd: u8, imm8: u8 },
}

/// Represents the core of an ARMv6-M compatible processor, like a Cortex-M0.
///
/// Contains the registers, stack pointer, program counter, and flags.
#[derive(Debug, Clone, Default)]
pub struct Armv6M {
    regs: [Wrapping<u32>; 13],
    sp: Wrapping<u32>,
    lr: Wrapping<u32>,
    pc: Wrapping<u32>,
}

impl Armv6M {
    /// Create a new CPU.
    ///
    /// All registers default to 0, except the Stack Pointer and Program
    /// Counter, which you must supply.
    pub fn new(sp: u32, pc: u32) -> Armv6M {
        Armv6M {
            sp: Wrapping(sp),
            pc: Wrapping(pc),
            ..Default::default()
        }
    }

    /// Fetch, Decode and Execute one instruction.
    pub fn step(&mut self, memory: &mut dyn Memory) -> Result<(), Error> {
        let instruction = self.fetch(memory)?;
        println!("Got {:?}", instruction);
        self.execute(instruction, memory);
        Ok(())
    }

    /// Fetch an instruction from memory and decode it.
    pub fn fetch(&self, memory: &dyn Memory) -> Result<Instruction, Error> {
        if self.pc.0 & 1 != 1 {
            return Err(Error::NonThumbPc);
        }
        let word = memory.load_u16(self.pc.0 & !1)?;
        println!("Loaded 0x{:04x} from 0x{:08x}", word, self.pc);
        Self::decode(word)
    }

    /// Decode a 16-bit word into an instruction.
    ///
    /// Some instructions are made up of two 16-bit values - TODO how we handle
    /// that. Probably with some `decode32` function.
    pub fn decode(word: u16) -> Result<Instruction, Error> {
        let opcode = word >> 11;
        println!("Opcode is 0b{:05b}", opcode);
        match opcode {
            0b00100 => {
                // MOV
                let imm8 = (word & 0xFF) as u8;
                let rd = ((word >> 8) & 0b111) as u8;
                Ok(Instruction::Mov { rd, imm8 })
            }
            0b11100 => {
                // B
                let imm11 = Self::sign_extend_imm11((word & 0x3FF) << 1);
                Ok(Instruction::Branch { imm11 })
            }
            _ => Err(Error::InvalidInstruction),
        }
    }

    /// Execute an instruction.
    pub fn execute(&mut self, instruction: Instruction, _memory: &mut dyn Memory) {
        match instruction {
            Instruction::Mov { rd, imm8 } => {
                self.regs[rd as usize] = Wrapping(u32::from(imm8));
                self.pc += 2;
            }
            Instruction::Branch { imm11 } => {
                self.pc += i32::from(imm11) as u32;
            }
        }
    }

    /// Given a signed 11-bit value, sign-extend it up to an i16.
    fn sign_extend_imm11(imm11: u16) -> i16 {
        if imm11 & 0x200 != 0 {
            // Negative, so set upper bits
            (imm11 | 0xFC00) as i16
        } else {
            imm11 as i16
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn basic_memory() {
        use Memory;

        let mut r = [0u32; 8];

        assert_eq!(Ok(0), r.load_u32(0));
        assert_eq!(Ok(0), r.load_u16(0));
        assert_eq!(Ok(0), r.load_u16(2));
        assert_eq!(Ok(0), r.load_u8(0));
        assert_eq!(Ok(0), r.load_u8(1));
        assert_eq!(Ok(0), r.load_u8(2));
        assert_eq!(Ok(0), r.load_u8(3));

        r.store_u32(0, 0xAABBCCDD).unwrap();

        // RAM contains DD CC BB AA

        assert_eq!(Ok(0xAABBCCDD), r.load_u32(0));
        assert_eq!(Ok(0xCCDD), r.load_u16(0));
        assert_eq!(Ok(0xAABB), r.load_u16(2));
        assert_eq!(Ok(0xDD), r.load_u8(0));
        assert_eq!(Ok(0xCC), r.load_u8(1));
        assert_eq!(Ok(0xBB), r.load_u8(2));
        assert_eq!(Ok(0xAA), r.load_u8(3));

        r.store_u16(2, 0xEEFF).unwrap();

        // RAM contains DD CC FF EE

        assert_eq!(Ok(0xEEFFCCDD), r.load_u32(0));
        assert_eq!(Ok(0xCCDD), r.load_u16(0));
        assert_eq!(Ok(0xEEFF), r.load_u16(2));
        assert_eq!(Ok(0xDD), r.load_u8(0));
        assert_eq!(Ok(0xCC), r.load_u8(1));
        assert_eq!(Ok(0xFF), r.load_u8(2));
        assert_eq!(Ok(0xEE), r.load_u8(3));

        r.store_u8(1, 0x00).unwrap();

        // RAM contains DD CC FF EE

        assert_eq!(Ok(0xEEFF00DD), r.load_u32(0));
        assert_eq!(Ok(0x00DD), r.load_u16(0));
        assert_eq!(Ok(0xEEFF), r.load_u16(2));
        assert_eq!(Ok(0xDD), r.load_u8(0));
        assert_eq!(Ok(0x00), r.load_u8(1));
        assert_eq!(Ok(0xFF), r.load_u8(2));
        assert_eq!(Ok(0xEE), r.load_u8(3));
    }

    #[test]
    fn mov_instruction() {
        assert_eq!(Ok(Instruction::Mov { rd: 0, imm8: 64 }), Armv6M::decode(0x2040));
    }

    #[test]
    fn branch_instruction() {
        assert_eq!(Ok(Instruction::Branch { imm11: -2 }), Armv6M::decode(0xe7fe));
        assert_eq!(Ok(Instruction::Branch { imm11: -4 }), Armv6M::decode(0xe7fc));
        assert_eq!(Ok(Instruction::Branch { imm11: 4 }), Armv6M::decode(0xe001));
    }
}

// End of file
