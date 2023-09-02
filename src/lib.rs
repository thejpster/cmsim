//! An ARMv6-M processor simulator.
//!
//! Designed to run on `no_std` systems (although at the moment it's full of
//! println! calls).

#![deny(missing_docs)]
#![deny(missing_debug_implementations)]

/// All the ways we can fail to execute code
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Did not understand this memory address
    InvalidAddress(u32),
    /// Did not understand this instruction
    InvalidInstruction(u16),
    /// Did not understand this 32-bit instruction
    InvalidInstruction32(u16, u16),
    /// Attempted to read a 32-bit value with an address that did not end in `0b00`
    UnalignedAccess,
    /// Decoded a 32-bit instruction as a 16-bit instruction
    WideInstruction,
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

/// Conditions that can be applied to an operation like a Branch
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Condition {
    /// Equal (Z == 0)
    Eq = 0,
    /// Not Equal (Z == 0)
    Ne = 1,
    /// Carry Set (C == 1)
    Cs = 2,
    /// Carry Clear (C == 0)
    Cc = 3,
    /// Minus, negative (N == 1)
    Mi = 4,
    /// Plus, positive (N == 0)
    Pl = 5,
    /// Overflow (V == 1)
    Vs = 6,
    /// No overflow (V == 0)
    Vc = 7,
    /// Unsigned higher (C == 1 and Z == 0)
    Hi = 8,
    /// Unsigned lower or same (C == 0 or Z == 1)
    Ls = 9,
    /// Signed greater than or equal (N == N)
    Ge = 10,
    /// Signed less than (N != N)
    Lt = 11,
    /// Signed greater than (Z == 0 and N == V)
    Gt = 12,
    /// Signed less than or equal (Z == 1 or N != V)
    Le = 13,
    /// Always executes
    Always = 15,
}

impl From<u8> for Condition {
    fn from(value: u8) -> Condition {
        match value & 0x0F {
            0 => Condition::Eq,
            1 => Condition::Ne,
            2 => Condition::Cs,
            3 => Condition::Cc,
            4 => Condition::Mi,
            5 => Condition::Pl,
            6 => Condition::Vs,
            7 => Condition::Vc,
            8 => Condition::Hi,
            9 => Condition::Ls,
            10 => Condition::Ge,
            11 => Condition::Lt,
            12 => Condition::Gt,
            13 => Condition::Le,
            _ => Condition::Always,
        }
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
    Branch {
        /// The 11-bit signed immediate value
        imm11: u16,
    },
    /// B.xx <label>
    BranchConditional {
        /// The condition code (e.g. only branch if Zero)
        cond: Condition,
        /// The 8-bit signed immediate value
        imm8: u8,
    },
    /// MOV <Rd>,#<imm8>
    MovImm {
        /// Which register to move the value into
        rd: Register,
        /// The 8-bit signed immediate value
        imm8: u8,
    },
    /// MOV <Rd>,<Rm>
    MovRegT1 {
        /// Which register to move into
        rd: Register,
        /// Which register to move from
        rm: Register,
    },
    /// BKPT <imm8>
    Breakpoint {
        /// The 8-bit signed immediate value
        imm8: u8,
    },
    /// LDR <Rt>,#<imm8>
    LdrLiteral {
        /// Which register to move into
        rt: Register,
        /// The 8-bit signed immediate value
        imm8: u8,
    },
    /// PUSH {<register list>}
    Push {
        /// A bitmask of registers (R7 to R0) to push
        register_list: u8,
        /// Also push LR
        m: bool,
    },
    /// ADD <Rd>,SP,#<imm8>
    AddSpT1 {
        /// Which register to store the result in
        rd: Register,
        /// The 8-bit signed immediate value
        imm8: u8,
    },
    /// BL <label>
    BranchLink {
        /// Branch argument from the first word
        s_imm10: u16,
        /// Branch argument from the second word
        j1_1_j2_imm11: u16,
    },
    /// BX <Rm>
    BranchExchange {
        /// Which register contains the new PC
        rm: Register,
    },
    /// SUB SP, #<imm7>
    SubSpImm {
        /// The 7-bit signed immediate value
        imm7: u8,
    },
    /// STR <Rt>,[SP,#<imm8>]
    StrImmT2 {
        /// Which register to fetch the value from
        rt: Register,
        /// The 8-bit signed immediate value       
        imm8: u8,
    },
    /// LDR <Rt>,[SP,#<imm8>]
    LdrImmT2 {
        /// Which register to store the value in
        rt: Register,
        /// The 8-bit signed immediate value       
        imm8: u8,
    },
    /// CMP <Rn>,<Rm>
    CmpReg {
        /// The left hand register to compare
        rn: Register,
        /// The right hand register to compare
        rm: Register,
    },
}

/// Identifies a register in our CPU
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Register {
    /// Register R0
    R0,
    /// Register R1
    R1,
    /// Register R2
    R2,
    /// Register R3
    R3,
    /// Register R4
    R4,
    /// Register R5
    R5,
    /// Register R6
    R6,
    /// Register R7
    R7,
    /// Register R8
    R8,
    /// Register R9
    R9,
    /// Register R10
    R10,
    /// Register R11
    R11,
    /// Register R12
    R12,
    /// Link Register
    Lr,
    /// Stack Pointer
    Sp,
    /// Program Counter
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

/// CPU execution modes
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Executing an exception or interrupt handler
    #[default]
    Handler,
    /// Not executing an exception or interrupt handler
    Thread,
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
    aspr: u32,
    mode: Mode,
}

impl Armv6M {
    const FLAG_N: u32 = 1 << 31;
    const FLAG_Z: u32 = 1 << 30;
    const FLAG_C: u32 = 1 << 29;
    const FLAG_V: u32 = 1 << 28;

    /// Create a new CPU.
    ///
    /// All registers default to 0, except the Stack Pointer and Program
    /// Counter, which you must supply.
    pub fn new(sp: u32, pc: u32) -> Armv6M {
        Armv6M {
            sp,
            // Trim off the thumb bit - we don't care
            pc: pc & !1,
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
        let word = memory.load_u16(self.pc)?;
        println!("Loaded 0x{:04x} from 0x{:08x}", word, self.pc);
        match Self::decode(word) {
            Ok(i) => Ok(i),
            Err(Error::WideInstruction) => {
                let word2 = memory.load_u16(self.pc + 2)?;
                Self::decode32(word, word2)
            }
            Err(e) => Err(e),
        }
    }

    /// Decode a 16-bit word into an instruction.
    ///
    /// Some instructions are made up of two 16-bit values - you will get
    /// `Error::WideInstruction` if you try and decode half of one.
    pub fn decode(word: u16) -> Result<Instruction, Error> {
        if (word >> 12) == 0b1101 {
            let cond = ((word >> 8) & 0x0F) as u8;
            let imm8 = word as u8;
            Ok(Instruction::BranchConditional {
                cond: Condition::from(cond),
                imm8,
            })
        } else if (word >> 11) == 0b11110 {
            // This is a 32-bit BL <label>
            Err(Error::WideInstruction)
        } else if (word >> 11) == 0b10010 {
            let imm8 = (word & 0xFF) as u8;
            let rt = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::StrImmT2 {
                rt: Register::from(rt),
                imm8,
            })
        } else if (word >> 11) == 0b10011 {
            let imm8 = (word & 0xFF) as u8;
            let rt = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::LdrImmT2 {
                rt: Register::from(rt),
                imm8,
            })
        } else if (word >> 11) == 0b00100 {
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
        } else if (word >> 7) == 0b010001110 {
            let rm = ((word >> 3) & 0x0F) as u8;
            Ok(Instruction::BranchExchange {
                rm: Register::from(rm),
            })
        } else if (word >> 7) == 0b101100001 {
            let imm7 = (word & 0x7F) as u8;
            Ok(Instruction::SubSpImm { imm7 })
        } else if (word >> 6) == 0b0100001010 {
            let rm = ((word >> 3) & 0b111) as u8;
            let rn = (word & 0b111) as u8;
            Ok(Instruction::CmpReg {
                rm: Register::from(rm),
                rn: Register::from(rn),
            })
        } else {
            Err(Error::InvalidInstruction(word))
        }
    }

    /// Decode a 32-bit T2 instruction from two 16-bit words.
    pub fn decode32(word1: u16, word2: u16) -> Result<Instruction, Error> {
        if (word1 >> 11) == 0b11110 && (word2 >> 14) == 0b11 {
            // Branch with Link
            let s_imm10 = word1 & 0x7FF;
            let j1_1_j2_imm11 = word2 & 0x3FFF;
            Ok(Instruction::BranchLink {
                s_imm10,
                j1_1_j2_imm11,
            })
        } else {
            Err(Error::InvalidInstruction32(word1, word2))
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
                let imm32 = u32::from(imm8);
                self.store_reg(rd, imm32);
                self.set_z(imm32 == 0);
                self.set_n(imm32 >= 0x80000000);
                // c is unchanged
                // v is unchanged
            }
            Instruction::MovRegT1 { rm, rd } => {
                let value = self.fetch_reg(rm);
                if rd == Register::Pc {
                    self.store_reg(rd, value * 2);
                } else {
                    self.store_reg(rd, value);
                }
            }
            Instruction::BranchConditional { cond, imm8 } => {
                let imm32 = Self::sign_extend_imm8(imm8) << 1;
                if self.check_condition(cond) {
                    self.pc = self.pc.wrapping_add(2);
                    self.pc = self.pc.wrapping_add(imm32);
                }
            }
            Instruction::Branch { imm11 } => {
                let imm32 = Self::sign_extend_imm11(imm11) << 1;
                // Assume's PC next increment has happened already
                self.pc = self.pc.wrapping_add(2);
                self.pc = self.pc.wrapping_add(imm32);
            }
            Instruction::Breakpoint { imm8 } => {
                self.breakpoint = Some(imm8);
            }
            Instruction::LdrLiteral { rt, imm8 } => {
                let imm32 = u32::from(imm8) << 2;
                // Assume's PC next increment has happened already
                // Also align to 4 bytes
                let base = (self.pc.wrapping_add(2)) & !0b11;
                let addr = base.wrapping_add(imm32);
                let value = memory.load_u32(addr)?;
                self.store_reg(rt, value);
            }
            Instruction::Push { register_list, m } => {
                if m {
                    let lr = self.lr;
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
                let sp = self.sp;
                let imm32 = u32::from(imm8) << 2;
                let value = sp.wrapping_add(imm32);
                self.store_reg(rd, value);
            }
            Instruction::BranchLink {
                s_imm10,
                j1_1_j2_imm11,
            } => {
                // See ARMv6-M Architecture Rference Manual A6.7.13
                let s_imm10 = u32::from(s_imm10);
                let j1_1_j2_imm11 = u32::from(j1_1_j2_imm11);
                let s = (s_imm10 >> 10) & 1;
                let s_prefix = if s == 1 { 0xFFu32 << 24 } else { 0 };
                let j1 = (j1_1_j2_imm11 >> 13) & 1;
                let j2 = (j1_1_j2_imm11 >> 11) & 1;
                let imm10 = s_imm10 & 0x3FF;
                let imm11 = j1_1_j2_imm11 & 0x7FF;
                let i1 = if j1 == s { 1 << 23 } else { 0 };
                let i2 = if j2 == s { 1 << 22 } else { 0 };
                let imm32 = s_prefix | i1 | i2 | (imm10 << 12) | (imm11 << 1);
                // Assume the prefetch has occurred
                let old_pc = self.pc.wrapping_add(2);
                self.lr = (old_pc & !1) | 1;
                self.pc = old_pc.wrapping_add(imm32);
            }
            Instruction::BranchExchange { rm } => {
                let addr = self.fetch_reg(rm);
                if self.mode == Mode::Handler && (addr >> 28) == 0b1111 {
                    // Return from Exception
                    todo!()
                } else if addr & 1 == 0 {
                    // Generate a hardfault on the next instruction
                    todo!()
                } else {
                    self.pc = addr & !1;
                }
            }
            Instruction::SubSpImm { imm7 } => {
                let imm32 = u32::from(imm7) << 2;
                // This does a subtraction
                let value = self.sp.wrapping_add(!imm32).wrapping_add(1);
                self.sp = value;
            }
            Instruction::StrImmT2 { rt, imm8 } => {
                let imm32 = u32::from(imm8) << 2;
                let offset_addr = self.sp.wrapping_add(imm32);
                let value = self.fetch_reg(rt);
                memory.store_u32(offset_addr, value)?;
            }
            Instruction::LdrImmT2 { rt, imm8 } => {
                let imm32 = u32::from(imm8) << 2;
                let offset_addr = self.sp.wrapping_add(imm32);
                let value = memory.load_u32(offset_addr)?;
                self.store_reg(rt, value);
            }
            Instruction::CmpReg { rm, rn } => {
                let value_m = self.fetch_reg(rm);
                let value_n = self.fetch_reg(rn);
                let (result, carry, overflow) = Self::add_with_carry(value_n, !value_m, true);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry);
                self.set_v(overflow);
            }
        }
        Ok(())
    }

    /// Adds two 32-bit numbers, with carry in, producing a result, carry out, and overflow.
    fn add_with_carry(value1: u32, value2: u32, carry_in: bool) -> (u32, bool, bool) {
        let (result, carry_out1) = value1.overflowing_add(value2);
        let (result, carry_out2) = result.overflowing_add(if carry_in { 1 } else { 0 });
        // Did we carry into the 32nd bit?
        let carry_out = carry_out1 | carry_out2;
        // You added two positive numbers but got a negative number
        let overflow_out =
            (value1 <= 0x7FFF_FFFF) && (value2 <= 0x7FFF_FFFF) && (result > 0x7FFF_FFFF);
        println!(
            "add_with_carry {:#x} + {:#x} + {} -> {:#x} {} {}",
            value1, value2, carry_in, result, carry_out, overflow_out
        );
        (result, carry_out, overflow_out)
    }

    /// Check whether the CPU's ASPR flags match the given condition.
    fn check_condition(&self, cond: Condition) -> bool {
        match cond {
            Condition::Eq => self.is_z(),
            Condition::Ne => !self.is_z(),
            Condition::Cs => self.is_c(),
            Condition::Cc => !self.is_c(),
            Condition::Mi => self.is_n(),
            Condition::Pl => !self.is_n(),
            Condition::Vs => self.is_v(),
            Condition::Vc => !self.is_v(),
            Condition::Hi => self.is_c() && !self.is_z(),
            Condition::Ls => !self.is_c() || self.is_z(),
            Condition::Ge => self.is_n() == self.is_v(),
            Condition::Lt => self.is_n() != self.is_v(),
            Condition::Gt => !self.is_z() && (self.is_n() == self.is_v()),
            Condition::Le => self.is_z() || (self.is_n() != self.is_v()),
            Condition::Always => true,
        }
    }

    /// Push one value onto the stack
    fn push_stack(&mut self, value: u32, memory: &mut dyn Memory) -> Result<(), Error> {
        let sp = self.sp - 4;
        println!("Pushing {:08x} to {:08x}", value, sp);
        memory.store_u32(sp, value)?;
        self.store_reg(Register::Sp, sp);
        Ok(())
    }

    /// Store a value into the given register
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

    /// Get a value from the given register
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

    /// Get whether the last instruction was a BKPT instruction
    pub fn breakpoint(&self) -> Option<u8> {
        self.breakpoint
    }

    /// Get the N flag
    fn is_n(&self) -> bool {
        (self.aspr & Self::FLAG_N) != 0
    }

    /// Get the Z flag
    fn is_z(&self) -> bool {
        (self.aspr & Self::FLAG_Z) != 0
    }

    /// Get the C flag
    fn is_c(&self) -> bool {
        (self.aspr & Self::FLAG_C) != 0
    }

    /// Get the V flag
    fn is_v(&self) -> bool {
        (self.aspr & Self::FLAG_V) != 0
    }

    /// Set the N flag
    fn set_n(&mut self, flag: bool) {
        if flag {
            self.aspr |= Self::FLAG_N;
        } else {
            self.aspr &= !Self::FLAG_N;
        }
    }

    /// Set the Z flag
    fn set_z(&mut self, flag: bool) {
        if flag {
            self.aspr |= Self::FLAG_Z;
        } else {
            self.aspr &= !Self::FLAG_Z;
        }
    }

    /// Set the C flag
    fn set_c(&mut self, flag: bool) {
        if flag {
            self.aspr |= Self::FLAG_C;
        } else {
            self.aspr &= !Self::FLAG_C;
        }
    }

    /// Set the V flag
    fn set_v(&mut self, flag: bool) {
        if flag {
            self.aspr |= Self::FLAG_V;
        } else {
            self.aspr &= !Self::FLAG_V;
        }
    }

    /// Given a signed 11-bit word offset, sign-extend it up to an i32 byte offset.
    fn sign_extend_imm11(imm11: u16) -> u32 {
        let imm11 = u32::from(imm11);
        if imm11 & 0x200 != 0 {
            // Top bit set, so negative, so set upper bits
            imm11 | 0xFFFF_FC00
        } else {
            imm11
        }
    }

    /// Given a signed 8-bit word offset, sign-extend it up to an i32 byte offset.
    fn sign_extend_imm8(imm8: u8) -> u32 {
        let imm8 = u32::from(imm8);
        if imm8 & 0x80 != 0 {
            // Top bit set, so negative, so set upper bits
            imm8 | 0xFFFF_FF00
        } else {
            imm8
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn add_with_carry() {
        static TEST_CASES: &[(u32, u32, bool, u32, bool, bool)] = &[
            (1, 1, false, 2, false, false),
            (1, 0xFFFF_FFFF, false, 0, true, false),
            (1, 0xFFFF_FFFF, true, 1, true, false),
            (0xFFFF_FFFF, 0xFFFF_FFFF, false, 0xFFFF_FFFE, true, false),
            (0xFFFF_FFFF, 0xFFFF_FFFF, true, 0xFFFF_FFFF, true, false),
            (0x4FFF_FFFF, 0x4FFF_FFFF, false, 0x9FFF_FFFE, false, true),
        ];
        for case in TEST_CASES {
            let (result, carry, overflow) = Armv6M::add_with_carry(case.0, case.1, case.2);
            assert_eq!(
                result, case.3,
                "result for {:#x} + {:#x} + {}",
                case.0, case.1, case.2
            );
            assert_eq!(
                carry, case.4,
                "carry for {:#x} + {:#x} + {}",
                case.0, case.1, case.2
            );
            assert_eq!(
                overflow, case.5,
                "overflow for {:#x} + {:#x} + {}",
                case.0, case.1, case.2
            );
        }
    }

    #[test]
    fn sign_extend_imm11() {
        assert_eq!(0xFFFF_FFFE, Armv6M::sign_extend_imm11(0x7FE));
        assert_eq!(0xFFFF_FFFF, Armv6M::sign_extend_imm11(0x7FF));
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

    #[test]
    fn branch_link_instruction() {
        assert_eq!(Err(Error::WideInstruction), Armv6M::decode(0xF04A));
        assert_eq!(
            Ok(Instruction::BranchLink {
                s_imm10: 0x004A,
                j1_1_j2_imm11: 0x3E41,
            }),
            Armv6M::decode32(0xF04A, 0xFE41)
        );
    }

    #[test]
    fn branch_link_operation() {
        // d0:	f04a fe41 	bl	4ad56
        let sp = 32;
        let start_pc = 0xd0;
        let mut cpu = Armv6M::new(sp, start_pc);
        let mut ram = [15; 8];
        cpu.execute(
            Instruction::BranchLink {
                s_imm10: 0x004A,
                j1_1_j2_imm11: 0x3E41,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(start_pc + 5, cpu.lr);
        assert_eq!(0x4ad56, cpu.pc);
    }

    #[test]
    fn branch_exchange_instruction() {
        assert_eq!(
            Ok(Instruction::BranchExchange { rm: Register::Lr }),
            Armv6M::decode(0x4770)
        );
    }

    #[test]
    fn branch_exchange_operation() {
        let sp = 32;
        let start_pc = 0xd0;
        let mut cpu = Armv6M::new(sp, start_pc);
        cpu.lr = 0x0000_1235;
        let mut ram = [15; 8];
        cpu.execute(Instruction::BranchExchange { rm: Register::Lr }, &mut ram)
            .unwrap();
        assert_eq!(0x0000_1234, cpu.pc);
    }

    #[test]
    fn sub_sp_imm_instruction() {
        assert_eq!(
            Ok(Instruction::SubSpImm { imm7: 16 }),
            Armv6M::decode(0xb090)
        );
    }

    #[test]
    fn sub_sp_imm_operation() {
        let sp = 0x100;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [15; 8];
        cpu.execute(Instruction::SubSpImm { imm7: 16 }, &mut ram)
            .unwrap();
        assert_eq!(0x100 - (4 * 16), cpu.sp);
    }

    #[test]
    fn str_imm_t2_instruction() {
        assert_eq!(
            Ok(Instruction::StrImmT2 {
                rt: Register::R1,
                imm8: 2
            }),
            Armv6M::decode(0x9102)
        );
    }

    #[test]
    fn str_imm_t2_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [15; 8];
        cpu.regs[1] = 0x12345678;
        cpu.execute(
            Instruction::StrImmT2 {
                rt: Register::R1,
                imm8: 2,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(ram[((sp as usize) / 4) + 2], 0x12345678);
    }

    #[test]
    fn ldr_imm_t2_instruction() {
        assert_eq!(
            Ok(Instruction::LdrImmT2 {
                rt: Register::R1,
                imm8: 2
            }),
            Armv6M::decode(0x9902)
        );
    }

    #[test]
    fn ldr_imm_t2_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0, 0, 0, 0, 0, 0, 0x12345678, 0];
        cpu.execute(
            Instruction::LdrImmT2 {
                rt: Register::R1,
                imm8: 2,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x12345678, cpu.regs[1]);
    }

    #[test]
    fn compare_instruction() {
        assert_eq!(
            Ok(Instruction::CmpReg {
                rm: Register::R1,
                rn: Register::R0,
            }),
            Armv6M::decode(0x4288)
        );
    }

    #[test]
    fn compare_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 0x2007815c;
        cpu.regs[1] = 0x2007a0ac;
        cpu.execute(
            Instruction::CmpReg {
                rn: Register::R0,
                rm: Register::R1,
            },
            &mut ram,
        )
        .unwrap();
        assert!(cpu.is_n());
        assert!(!cpu.is_z());
        assert!(!cpu.is_c());
        assert!(!cpu.is_v());
    }
}

// End of file
