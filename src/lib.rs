//! An ARMv6-M processor simulator.
//!
//! Designed to run on `no_std` systems (although at the moment it's full of
//! println! calls).

/// All the ways we can fail to execute code
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    InvalidAddress(u32),
    InvalidInstruction(u16),
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
        self.get(addr as usize >> 2)
            .copied()
            .ok_or(Error::InvalidAddress(addr))
    }

    fn store_u32(&mut self, addr: u32, value: u32) -> Result<(), Error> {
        *self
            .get_mut(addr as usize >> 2)
            .ok_or(Error::InvalidAddress(addr))? = value;
        Ok(())
    }
}

impl<const N: usize> Memory for [u32; N] {
    fn load_u32(&self, addr: u32) -> Result<u32, Error> {
        self.get(addr as usize >> 2)
            .copied()
            .ok_or(Error::InvalidAddress(addr))
    }

    fn store_u32(&mut self, addr: u32, value: u32) -> Result<(), Error> {
        *self
            .get_mut(addr as usize >> 2)
            .ok_or(Error::InvalidAddress(addr))? = value;
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
    /// B <label>
    Branch { imm11: u16 },
    /// MOV <Rd>,#<imm8>
    MovImm { rd: Register, imm8: u8 },
    /// MOV <Rd>,<Rm>
    MovRegT1 { rm: Register, rd: Register },
    /// BKPT <imm8>
    Breakpoint { imm8: u8 },
    /// LDR <Rt>,#<imm8>
    LdrLiteral { rt: Register, imm8: u8 },
    /// PUSH {<register list>}
    Push { register_list: u8, m: bool },
    /// ADD <Rd>,SP,#<imm8>
    AddSpT1 { rd: Register, imm8: u8 },
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

impl From<u8> for Register {
    fn from(value: u8) -> Self {
        match value & 0x0F {
            0 => Register::R0,
            1 => Register::R1,
            2 => Register::R2,
            3 => Register::R3,
            4 => Register::R4,
            5 => Register::R5,
            6 => Register::R6,
            7 => Register::R7,
            8 => Register::R8,
            9 => Register::R9,
            10 => Register::R10,
            11 => Register::R11,
            12 => Register::R12,
            13 => Register::Sp,
            14 => Register::Lr,
            _ => Register::Pc,
        }
    }
}

/// Represents the core of an ARMv6-M compatible processor, like a Cortex-M0.
///
/// Contains the registers, stack pointer, program counter, and flags.
#[derive(Debug, Default, Clone)]
pub struct Armv6M {
    regs: [u32; 13],
    sp: u32,
    lr: u32,
    pc: u32,
    breakpoint: Option<u8>,
    flag_zero: bool,
    flag_negative: bool,
}

impl Armv6M {
    /// Create a new CPU.
    ///
    /// All registers default to 0, except the Stack Pointer and Program
    /// Counter, which you must supply.
    pub fn new(sp: u32, pc: u32) -> Armv6M {
        Armv6M {
            sp,
            pc,
            ..Default::default()
        }
    }

    /// Get the contents of a register
    pub fn register(&self, reg: Register) -> u32 {
        match reg {
            Register::R0 => self.regs[0],
            Register::R1 => self.regs[1],
            Register::R2 => self.regs[2],
            Register::R3 => self.regs[3],
            Register::R4 => self.regs[4],
            Register::R5 => self.regs[5],
            Register::R6 => self.regs[6],
            Register::R7 => self.regs[7],
            Register::R8 => self.regs[8],
            Register::R9 => self.regs[9],
            Register::R10 => self.regs[10],
            Register::R11 => self.regs[11],
            Register::R12 => self.regs[12],
            Register::Lr => self.lr,
            Register::Sp => self.sp,
            Register::Pc => self.pc,
        }
    }

    /// Fetch, Decode and Execute one instruction.
    pub fn step(&mut self, memory: &mut dyn Memory) -> Result<(), Error> {
        let instruction = self.fetch(memory)?;
        println!("Got {:?}", instruction);
        self.execute(instruction, memory)?;
        Ok(())
    }

    /// Fetch an instruction from memory and decode it.
    pub fn fetch(&self, memory: &dyn Memory) -> Result<Instruction, Error> {
        if self.pc & 1 != 1 {
            return Err(Error::NonThumbPc);
        }
        let word = memory.load_u16(self.pc & !1)?;
        println!("Loaded 0x{:04x} from 0x{:08x}", word, self.pc & !1);
        Self::decode(word)
    }

    /// Decode a 16-bit word into an instruction.
    ///
    /// Some instructions are made up of two 16-bit values - TODO how we handle
    /// that. Probably with some `decode32` function.
    pub fn decode(word: u16) -> Result<Instruction, Error> {
        if (word >> 11) == 0b00100 {
            let imm8 = (word & 0xFF) as u8;
            let rd = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::MovImm {
                rd: Register::from(rd),
                imm8,
            })
        } else if (word >> 11) == 0b11100 {
            Ok(Instruction::Branch {
                imm11: word & 0x7FF,
            })
        } else if (word >> 11) == 0b10101 {
            let imm8 = (word & 0xFF) as u8;
            let rd = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::AddSpT1 {
                rd: Register::from(rd),
                imm8,
            })
        } else if (word >> 11) == 0b01001 {
            let imm8 = (word & 0xFF) as u8;
            let rt = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::LdrLiteral {
                rt: Register::from(rt),
                imm8,
            })
        } else if (word >> 9) == 0b1011010 {
            let m = ((word >> 8) & 1) == 1;
            let register_list = word as u8;
            Ok(Instruction::Push { register_list, m })
        } else if (word >> 8) == 0b10111110 {
            let imm8 = (word & 0xFF) as u8;
            Ok(Instruction::Breakpoint { imm8 })
        } else if (word >> 8) == 0b01000110 {
            let d: u8 = ((word >> 7) & 0b1) as u8;
            let rd = (d << 3) | (word & 0b111) as u8;
            let rm: u8 = ((word >> 3) & 0x0F) as u8;
            Ok(Instruction::MovRegT1 {
                rm: Register::from(rm),
                rd: Register::from(rd),
            })
        } else {
            Err(Error::InvalidInstruction(word))
        }
    }

    /// Execute an instruction.
    pub fn execute(
        &mut self,
        instruction: Instruction,
        memory: &mut dyn Memory,
    ) -> Result<(), Error> {
        self.breakpoint = None;
        self.pc = self.pc.wrapping_add(2);
        match instruction {
            Instruction::MovImm { rd, imm8 } => {
                // MOVS <Rd>,#<imm8>
                let imm32 = u32::from(imm8);
                self.store_reg(rd, imm32);
                self.flag_zero = imm32 == 0;
                self.flag_negative = imm32 & 0x8000_0000 != 0;
            }
            Instruction::MovRegT1 { rm, rd } => {
                // MOV <Rd>,<Rm>
                let value = self.fetch_reg(rm);
                if rd == Register::Pc {
                    self.store_reg(rd, value * 2);
                } else {
                    self.store_reg(rd, value);
                }
            }
            Instruction::Branch { imm11 } => {
                let imm32 = Self::sign_extend_imm11(imm11) << 1;
                // Assume's PC next increment has happened already
                self.pc = self.pc.wrapping_add(2 + (imm32 as u32));
            }
            Instruction::Breakpoint { imm8 } => {
                self.breakpoint = Some(imm8);
            }
            Instruction::LdrLiteral { rt, imm8 } => {
                let imm32 = u32::from(imm8) << 2;
                // Assume's PC next increment has happened already
                // Also align to 4 bytes
                let base = (self.pc + 2) & !0b11;
                let addr = base + imm32;
                let value = memory.load_u32(addr)?;
                self.store_reg(rt, value);
            }
            Instruction::Push { register_list, m } => {
                if m {
                    let lr = self.fetch_reg(Register::Lr);
                    self.push_stack(lr, memory)?;
                }
                for register in [7, 6, 5, 4, 3, 2, 1, 0] {
                    if (register_list & 1 << register) != 0 {
                        let value = self.fetch_reg(Register::from(register));
                        self.push_stack(value, memory)?;
                    }
                }
            }
            Instruction::AddSpT1 { rd, imm8 } => {
                let sp = self.fetch_reg(Register::Sp);
                let imm32 = u32::from(imm8) << 2;
                let value = sp.wrapping_add(imm32);
                self.store_reg(rd, value);
            }
        }
        Ok(())
    }

    fn push_stack(&mut self, value: u32, memory: &mut dyn Memory) -> Result<(), Error> {
        let sp = self.fetch_reg(Register::Sp) - 4;
        println!("Pushing {:08x} to {:08x}", value, sp);
        memory.store_u32(sp, value)?;
        self.store_reg(Register::Sp, sp);
        Ok(())
    }

    fn store_reg(&mut self, register: Register, value: u32) {
        match register {
            Register::R0 => self.regs[0] = value,
            Register::R1 => self.regs[1] = value,
            Register::R2 => self.regs[2] = value,
            Register::R3 => self.regs[3] = value,
            Register::R4 => self.regs[4] = value,
            Register::R5 => self.regs[5] = value,
            Register::R6 => self.regs[6] = value,
            Register::R7 => self.regs[7] = value,
            Register::R8 => self.regs[8] = value,
            Register::R9 => self.regs[9] = value,
            Register::R10 => self.regs[10] = value,
            Register::R11 => self.regs[11] = value,
            Register::R12 => self.regs[12] = value,
            Register::Lr => self.lr = value,
            Register::Sp => self.sp = value,
            Register::Pc => self.pc = value,
        }
    }

    fn fetch_reg(&mut self, register: Register) -> u32 {
        match register {
            Register::R0 => self.regs[0],
            Register::R1 => self.regs[1],
            Register::R2 => self.regs[2],
            Register::R3 => self.regs[3],
            Register::R4 => self.regs[4],
            Register::R5 => self.regs[5],
            Register::R6 => self.regs[6],
            Register::R7 => self.regs[7],
            Register::R8 => self.regs[8],
            Register::R9 => self.regs[9],
            Register::R10 => self.regs[10],
            Register::R11 => self.regs[11],
            Register::R12 => self.regs[12],
            Register::Lr => self.lr,
            Register::Sp => self.sp,
            Register::Pc => self.pc,
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
            Ok(Instruction::MovImm {
                rd: Register::R0,
                imm8: 64
            }),
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
        cpu.execute(Instruction::Branch { imm11: 0x7fe }, &mut ram)
            .unwrap();
        // PC was 8, PC is still 8, because `B 0x7FE` means branch to yourself
        assert_eq!(cpu.pc, 8);
    }

    #[test]
    fn bkpt_instruction() {
        assert_eq!(
            Ok(Instruction::Breakpoint { imm8: 0xCC }),
            Armv6M::decode(0xbecc)
        );
    }

    #[test]
    fn ldr_instruction() {
        assert_eq!(
            Ok(Instruction::LdrLiteral {
                rt: Register::R0,
                imm8: 1
            }),
            Armv6M::decode(0x4801)
        )
    }

    #[test]
    fn movregt1_instruction() {
        assert_eq!(
            Ok(Instruction::MovRegT1 {
                rm: Register::R0,
                rd: Register::Lr
            }),
            Armv6M::decode(0x4686)
        )
    }

    #[test]
    fn push_instruction() {
        assert_eq!(
            // Push {LR, R7}
            Ok(Instruction::Push {
                register_list: 0x80,
                m: true
            }),
            Armv6M::decode(0xb580)
        )
    }

    #[test]
    fn push_operation() {
        let mut cpu = Armv6M::new(32, 0x0);
        let mut ram = [15; 8];
        cpu.store_reg(Register::R0, 0);
        cpu.store_reg(Register::R1, 1);
        cpu.store_reg(Register::R2, 2);
        cpu.store_reg(Register::R3, 3);
        cpu.store_reg(Register::R4, 4);
        cpu.store_reg(Register::R5, 5);
        cpu.store_reg(Register::R6, 6);
        cpu.store_reg(Register::R7, 7);
        cpu.store_reg(Register::Lr, 8);
        cpu.execute(
            Instruction::Push {
                register_list: 0x55,
                m: false,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!([15, 15, 15, 15, 0, 2, 4, 6], ram);
    }

    #[test]
    fn add_instruction() {
        assert_eq!(
            // add r7, sp, #0
            Ok(Instruction::AddSpT1 {
                rd: Register::R7,
                imm8: 0
            }),
            Armv6M::decode(0xaf00)
        )
    }

    #[test]
    fn add_operation() {
        let sp = 32;
        let mut cpu = Armv6M::new(sp, 0x0);
        let mut ram = [15; 8];
        cpu.execute(
            Instruction::AddSpT1 {
                rd: Register::R0,
                imm8: 4,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(32 + 4 * 4, cpu.regs[0]);
    }
}

// End of file
