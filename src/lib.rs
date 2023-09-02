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
    NonThumbPc,
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
    Branch { imm11: u16 },
    Movs { rd: u8, imm8: u8 },
    Breakpoint { imm8: u8 },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Register {
    R0,
    R1,
    R2,
    R3,
    R4,
    R5,
    R6,
    R7,
    R8,
    R9,
    R10,
    R11,
    R12,
    Lr,
    Sp,
    Pc,
}

/// Represents the core of an ARMv6-M compatible processor, like a Cortex-M0.
///
/// Contains the registers, stack pointer, program counter, and flags.
#[derive(Debug, Default, Clone)]
pub struct Armv6M {
    regs: [Wrapping<u32>; 13],
    sp: Wrapping<u32>,
    lr: Wrapping<u32>,
    pc: Wrapping<u32>,
    breakpoint: Option<u8>,
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

    /// Get the contents of a register
    pub fn register(&self, reg: Register) -> u32 {
        match reg {
            Register::R0 => self.regs[0].0,
            Register::R1 => self.regs[1].0,
            Register::R2 => self.regs[2].0,
            Register::R3 => self.regs[3].0,
            Register::R4 => self.regs[4].0,
            Register::R5 => self.regs[5].0,
            Register::R6 => self.regs[6].0,
            Register::R7 => self.regs[7].0,
            Register::R8 => self.regs[8].0,
            Register::R9 => self.regs[9].0,
            Register::R10 => self.regs[10].0,
            Register::R11 => self.regs[11].0,
            Register::R12 => self.regs[12].0,
            Register::Lr => self.lr.0,
            Register::Sp => self.sp.0,
            Register::Pc => self.pc.0,
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
        println!("Loaded 0x{:04x} from 0x{:08x}", word, self.pc.0 & !1);
        Self::decode(word)
    }

    /// Decode a 16-bit word into an instruction.
    ///
    /// Some instructions are made up of two 16-bit values - TODO how we handle
    /// that. Probably with some `decode32` function.
    pub fn decode(word: u16) -> Result<Instruction, Error> {
        if (word >> 11) == 0b00100 {
            // MOV <rd>, #imm8
            let imm8 = (word & 0xFF) as u8;
            let rd = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::Movs { rd, imm8 })
        } else if (word >> 11) == 0b11100 {
            // B <location>
            Ok(Instruction::Branch {
                imm11: word & 0x7FF,
            })
        } else if (word >> 8) == 0b10111110 {
            Ok(Instruction::Breakpoint { imm8: word as u8 })
        } else {
            Err(Error::InvalidInstruction)
        }
    }

    /// Execute an instruction.
    pub fn execute(&mut self, instruction: Instruction, _memory: &mut dyn Memory) {
        self.breakpoint = None;
        match instruction {
            Instruction::Movs { rd, imm8 } => {
                self.pc += 2;
                self.regs[rd as usize] = Wrapping(u32::from(imm8));
            }
            Instruction::Branch { imm11 } => {
                let imm32 = Self::sign_extend_imm11(imm11) << 1;
                // Branch assumes pre-fetch has occurred, putting PC 4 ahead.
                self.pc += 4;
                self.pc += imm32 as u32;
            }
            Instruction::Breakpoint { imm8 } => {
                self.breakpoint = Some(imm8);
            }
        }
    }

    pub fn breakpoint(&self) -> Option<u8> {
        self.breakpoint
    }

    /// Given a signed 11-bit word offset, sign-extend it up to an i32 byte offset.
    fn sign_extend_imm11(imm11: u16) -> i32 {
        let imm11 = u32::from(imm11);
        if imm11 & 0x200 != 0 {
            // Top bit set, so negative, so set upper bits
            (imm11 | 0xFFFF_FC00) as i32
        } else {
            imm11 as i32
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn sign_extend_imm11() {
        assert_eq!(-2, Armv6M::sign_extend_imm11(0x7FE));
        assert_eq!(-1, Armv6M::sign_extend_imm11(0x7FF));
        assert_eq!(0, Armv6M::sign_extend_imm11(0));
        assert_eq!(1, Armv6M::sign_extend_imm11(1));
        assert_eq!(2, Armv6M::sign_extend_imm11(2));
    }

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
        assert_eq!(
            Ok(Instruction::Movs { rd: 0, imm8: 64 }),
            Armv6M::decode(0x2040)
        );
    }

    #[test]
    fn branch_instruction() {
        assert_eq!(
            Ok(Instruction::Branch { imm11: 0x7fe }),
            Armv6M::decode(0xe7fe)
        );
        assert_eq!(
            Ok(Instruction::Branch { imm11: 0x7fc }),
            Armv6M::decode(0xe7fc)
        );
        assert_eq!(Ok(Instruction::Branch { imm11: 1 }), Armv6M::decode(0xe001));

        let mut cpu = Armv6M::new(0, 8);
        let mut ram = [0u32; 6];
        cpu.execute(Instruction::Branch { imm11: 0x7fe }, &mut ram);
        // PC was 8, PC is still 8, because `B 0x7FE` means branch to yourself
        assert_eq!(cpu.pc.0, 8);
    }

    #[test]
    fn bkpt_instruction() {
        assert_eq!(
            Ok(Instruction::Breakpoint { imm8: 0xCC }),
            Armv6M::decode(0xbecc)
        );
    }
}

// End of file
