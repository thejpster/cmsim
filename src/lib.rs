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

impl<const N: usize> Memory for [u32; N] {
    fn load_u32(&self, addr: u32) -> Result<u32, Error> {
        if addr & 0b11 != 0 {
            return Err(Error::UnalignedAccess);
        }
        self.get(addr as usize >> 2)
            .copied()
            .ok_or(Error::InvalidAddress(addr))
    }

    fn store_u32(&mut self, addr: u32, value: u32) -> Result<(), Error> {
        if addr & 0b11 != 0 {
            return Err(Error::UnalignedAccess);
        }
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

impl std::fmt::Display for Condition {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Condition::Eq => "EQ",
                Condition::Ne => "NE",
                Condition::Cs => "CS",
                Condition::Cc => "CC",
                Condition::Mi => "MI",
                Condition::Pl => "PL",
                Condition::Vs => "VS",
                Condition::Vc => "VC",
                Condition::Hi => "HI",
                Condition::Ls => "LS",
                Condition::Ge => "GE",
                Condition::Lt => "LT",
                Condition::Gt => "GT",
                Condition::Le => "LE",
                Condition::Always => "",
            }
        )
    }
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
    // =======================================================================
    // Branch Instructions
    // =======================================================================
    /// `B <label>`
    Branch {
        /// The PC-relative signed offset.
        ///
        /// A value of -4 means jump to same location, because PC has been
        /// incremented twice before this is executed.
        imm32: i32,
    },
    /// `B.xx <label>`
    BranchConditional {
        /// The condition code (e.g. only branch if Zero)
        cond: Condition,
        /// The PC-relative signed offset.
        ///
        /// A value of -4 means jump to same location, because PC has been
        /// incremented twice before this is executed.
        imm32: i32,
    },
    /// `BL <label>`
    BranchLink {
        /// The PC-relative signed offset.
        ///
        /// A value of -4 means jump to same location, because PC has been
        /// incremented twice before this is executed.
        imm32: i32,
    },
    /// `BX <Rm>`
    BranchExchange {
        /// Which register contains the new PC
        rm: Register,
    },
    // =======================================================================
    // Move Instructions
    // =======================================================================
    /// `MOV <Rd>,#<imm8>`
    MovImm {
        /// Which register to move the value into
        rd: Register,
        /// The 8-bit signed immediate value
        imm8: u8,
    },
    /// `MOV <Rd>,<Rm>`
    MovRegT1 {
        /// Which register to move into
        rd: Register,
        /// Which register to move from
        rm: Register,
    },
    // =======================================================================
    // Store Instructions
    // =======================================================================
    /// `STR <Rt>,[<Rn>{,#<imm5>}]`
    StrImm {
        /// Which register contains the value to store
        rt: Register,
        /// Which register contains the base address to store at
        rn: Register,
        /// What offset to apply to the base address
        imm32: u32,
    },
    /// `STRB <Rt>,[<Rn>{,#<imm5>}]`
    StrbImm {
        /// Which register contains the value to store
        rt: Register,
        /// Which register contains the address to store at
        rn: Register,
        /// What offset to apply to the storage address
        imm5: u8,
    },
    /// `STR <Rt>,[SP,#<imm8>]`
    StrImmSp {
        /// Which register to contains the value to store
        rt: Register,
        /// The offset to apply to the stack pointer
        imm32: u32,
    },
    // =======================================================================
    // Load Instructions
    // =======================================================================
    /// `LDR <Rt>,#<imm8>`
    LdrLiteral {
        /// Which register to move into
        rt: Register,
        /// The value to load
        imm32: u32,
    },
    /// `LDR <Rt>,[SP,#<imm8>]`
    LdrImmSp {
        /// Which register to store the loaded value in
        rt: Register,
        /// The offset to apply to the stack pointer
        imm32: u32,
    },
    // =======================================================================
    // Stack Instructions
    // =`======================================================================`
    /// `PUSH {<register list>}`
    Push {
        /// A bitmask of registers (R7 to R0) to push
        register_list: u8,
        /// Also push LR
        m: bool,
    },
    /// `ADD <Rd>,SP,#<imm8>`
    AddSpT1 {
        /// Which register to store the result in
        rd: Register,
        /// The increment for the stack pointer
        imm32: u32,
    },
    /// `ADD SP,#<imm11>`
    AddSpImm {
        /// How much to add to the stack pointer
        imm32: u32,
    },
    /// `SUB SP,#<imm7>`
    SubSpImm {
        /// How much to subtract from the stack pointer
        imm32: u32,
    },
    // =======================================================================
    // Arithmetic Instructions
    // =======================================================================
    /// `ADDS <Rd>,<Rn>,#<imm3>`
    Adds {
        /// Which register to store the result in
        rd: Register,
        /// Which register to get the value from
        rn: Register,
        /// How much to add to <Rn>
        imm3: u8,
    },
    // =======================================================================
    // Logical Instructions
    // =======================================================================
    /// `CMP <Rn>,<Rm>`
    CmpReg {
        /// The left hand register to compare
        rn: Register,
        /// The right hand register to compare
        rm: Register,
    },
    // =======================================================================
    // Other Instructions
    // =======================================================================
    /// `BKPT <imm8>`
    Breakpoint {
        /// The 8-bit signed immediate value
        imm8: u8,
    },
}

impl std::fmt::Display for Instruction {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Instruction::Branch { imm32 } => write!(f, "B {:+#}", *imm32 + 4),
            Instruction::BranchConditional { cond, imm32 } => {
                write!(f, "B{} {:+#}", cond, *imm32 + 4)
            }
            Instruction::BranchLink { imm32 } => write!(f, "BL {:+#}", *imm32 + 4),
            Instruction::BranchExchange { rm } => write!(f, "BX {}", *rm),
            Instruction::MovImm { rd, imm8 } => write!(f, "MOV {},#{}", rd, imm8),
            Instruction::MovRegT1 { rd, rm } => write!(f, "MOV {},{}", rd, rm),
            Instruction::StrImm { rt, rn, imm32 } => write!(f, "STR {},[{},#{}]", rt, rn, imm32),
            Instruction::StrbImm { rt, rn, imm5 } => write!(f, "STRB {},[{},#{}]", rt, rn, imm5),
            Instruction::StrImmSp { rt, imm32 } => write!(f, "STR {},[SP,#{}]", rt, imm32),
            Instruction::LdrLiteral { rt, imm32 } => write!(f, "LDR {},#{}", rt, imm32),
            Instruction::LdrImmSp { rt, imm32 } => write!(f, "LDR {},[SP,#{}]", rt, imm32),
            Instruction::Push { register_list, m } => {
                let mut first = true;
                write!(f, "PUSH {{")?;
                for register in [7, 6, 5, 4, 3, 2, 1, 0] {
                    if (*register_list & 1 << register) != 0 {
                        if !first {
                            write!(f, ",")?;
                        }
                        first = false;
                        write!(f, "R{}", register)?;
                    }
                }
                if *m {
                    if !first {
                        write!(f, ",")?;
                    }
                    write!(f, "LR")?;
                }
                write!(f, "}}")
            }
            Instruction::AddSpT1 { rd, imm32 } => write!(f, "ADD {},SP,#{}", rd, imm32),
            Instruction::AddSpImm { imm32 } => write!(f, "ADD SP,#{}", imm32),
            Instruction::SubSpImm { imm32 } => write!(f, "SUB SP,#{}", imm32),
            Instruction::CmpReg { rn, rm } => write!(f, "CMP {},{}", rn, rm),
            Instruction::Breakpoint { imm8 } => write!(f, "BKPT 0x{:02x}", imm8),
            Instruction::Adds { rd, rn, imm3 } => write!(f, "ADDS {},{},#{}", rd, rn, imm3),
        }
    }
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

impl std::fmt::Display for Register {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Register::R0 => "R0",
                Register::R1 => "R1",
                Register::R2 => "R2",
                Register::R3 => "R3",
                Register::R4 => "R4",
                Register::R5 => "R5",
                Register::R6 => "R6",
                Register::R7 => "R7",
                Register::R8 => "R8",
                Register::R9 => "R9",
                Register::R10 => "R1",
                Register::R11 => "R1",
                Register::R12 => "R1",
                Register::Lr => "LR",
                Register::Sp => "SP",
                Register::Pc => "PC",
            }
        )
    }
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
        println!("Got \"{}\" ({:?})", instruction, instruction);
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
                imm32: Self::sign_extend_imm8(imm8) << 1,
            })
        } else if (word >> 11) == 0b11110 {
            // This is a 32-bit BL <label>
            Err(Error::WideInstruction)
        } else if (word >> 11) == 0b10010 {
            let imm8 = (word & 0xFF) as u8;
            let rt = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::StrImmSp {
                rt: Register::from(rt),
                imm32: u32::from(imm8) << 2,
            })
        } else if (word >> 11) == 0b10011 {
            let imm8 = (word & 0xFF) as u8;
            let rt = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::LdrImmSp {
                rt: Register::from(rt),
                imm32: u32::from(imm8) << 2,
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
                imm32: Self::sign_extend_imm11(word & 0x7FF) << 1,
            })
        } else if (word >> 11) == 0b10101 {
            let imm8 = (word & 0xFF) as u8;
            let rd = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::AddSpT1 {
                rd: Register::from(rd),
                imm32: u32::from(imm8) << 2,
            })
        } else if (word >> 11) == 0b01001 {
            let imm8 = (word & 0xFF) as u8;
            let rt = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::LdrLiteral {
                rt: Register::from(rt),
                imm32: u32::from(imm8) << 2,
            })
        } else if (word >> 11) == 0b01110 {
            let imm5 = ((word >> 6) & 0x1F) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rt = (word & 0b111) as u8;
            Ok(Instruction::StrbImm {
                rt: Register::from(rt),
                rn: Register::from(rn),
                imm5,
            })
        } else if (word >> 11) == 0b01100 {
            let imm5 = ((word >> 6) & 0x1F) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rt = (word & 0b111) as u8;
            Ok(Instruction::StrImm {
                rt: Register::from(rt),
                rn: Register::from(rn),
                imm32: u32::from(imm5) << 2,
            })
        } else if (word >> 9) == 0b1011010 {
            let m = ((word >> 8) & 1) == 1;
            let register_list = word as u8;
            Ok(Instruction::Push { register_list, m })
        } else if (word >> 9) == 0b0001110 {
            let imm3 = ((word >> 6) & 0x07) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rd = (word & 0b111) as u8;
            Ok(Instruction::Adds {
                rd: Register::from(rd),
                rn: Register::from(rn),
                imm3,
            })
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
        } else if (word >> 7) == 0b101100000 {
            let imm7 = (word & 0x7F) as u8;
            Ok(Instruction::AddSpImm {
                imm32: u32::from(imm7) << 2,
            })
        } else if (word >> 7) == 0b101100001 {
            let imm7 = (word & 0x7F) as u8;
            Ok(Instruction::SubSpImm {
                imm32: u32::from(imm7) << 2,
            })
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
            // See ARMv6-M Architecture Rference Manual A6.7.13
            let s_imm10 = u32::from(word1 & 0x7FF);
            let j1_1_j2_imm11 = u32::from(word2 & 0x3FFF);
            let s = (s_imm10 >> 10) & 1;
            let s_prefix = if s == 1 { 0xFFu32 << 24 } else { 0 };
            let j1 = (j1_1_j2_imm11 >> 13) & 1;
            let j2 = (j1_1_j2_imm11 >> 11) & 1;
            let imm10 = s_imm10 & 0x3FF;
            let imm11 = j1_1_j2_imm11 & 0x7FF;
            let i1 = if j1 == s { 1 << 23 } else { 0 };
            let i2 = if j2 == s { 1 << 22 } else { 0 };
            let imm32 = s_prefix | i1 | i2 | (imm10 << 12) | (imm11 << 1);
            Ok(Instruction::BranchLink {
                imm32: imm32 as i32,
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
            Instruction::BranchConditional { cond, imm32 } => {
                if self.check_condition(cond) {
                    self.pc = self.pc.wrapping_add(2);
                    self.pc = self.pc.wrapping_add(imm32 as u32);
                }
            }
            Instruction::Branch { imm32 } => {
                // Assume's PC next increment has happened already
                self.pc = self.pc.wrapping_add(2);
                self.pc = self.pc.wrapping_add(imm32 as u32);
            }
            Instruction::Breakpoint { imm8 } => {
                self.breakpoint = Some(imm8);
            }
            Instruction::LdrLiteral { rt, imm32 } => {
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
            Instruction::AddSpT1 { rd, imm32 } => {
                let sp = self.sp;
                let value = sp.wrapping_add(imm32);
                self.store_reg(rd, value);
            }
            Instruction::BranchLink { imm32 } => {
                // Assume the prefetch has occurred
                let old_pc = self.pc.wrapping_add(2);
                self.lr = (old_pc & !1) | 1;
                self.pc = old_pc.wrapping_add(imm32 as u32);
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
            Instruction::AddSpImm { imm32 } => {
                self.sp = self.sp.wrapping_add(imm32);
            }
            Instruction::SubSpImm { imm32 } => {
                // This does a subtraction
                let value = self.sp.wrapping_add(!imm32).wrapping_add(1);
                self.sp = value;
            }
            Instruction::StrImmSp { rt, imm32 } => {
                let offset_addr = self.sp.wrapping_add(imm32);
                let value = self.fetch_reg(rt);
                memory.store_u32(offset_addr, value)?;
            }
            Instruction::LdrImmSp { rt, imm32 } => {
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
            Instruction::StrbImm { rt, rn, imm5 } => {
                let imm32 = u32::from(imm5);
                let addr = self.fetch_reg(rn);
                let offset_addr = addr.wrapping_add(imm32);
                let value = self.fetch_reg(rt);
                memory.store_u8(offset_addr, (value & 0xFF) as u8)?;
            }
            Instruction::StrImm { rt, rn, imm32 } => {
                let addr = self.fetch_reg(rn);
                let offset_addr = addr.wrapping_add(imm32);
                let value = self.fetch_reg(rt);
                memory.store_u32(offset_addr, value)?;
            }
            Instruction::Adds { rd, rn, imm3 } => {
                let value1 = self.fetch_reg(rn);
                let value2 = u32::from(imm3);
                let (result, carry, overflow) = Self::add_with_carry(value1, value2, false);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry);
                self.set_v(overflow);
                self.store_reg(rd, result);
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
    fn sign_extend_imm11(imm11: u16) -> i32 {
        let imm11 = u32::from(imm11);
        if imm11 & 0x200 != 0 {
            // Top bit set, so negative, so set upper bits
            (imm11 | 0xFFFF_FC00) as i32
        } else {
            imm11 as i32
        }
    }

    /// Given a signed 8-bit word offset, sign-extend it up to an i32 byte offset.
    fn sign_extend_imm8(imm8: u8) -> i32 {
        let imm8 = u32::from(imm8);
        if imm8 & 0x80 != 0 {
            // Top bit set, so negative, so set upper bits
            (imm8 | 0xFFFF_FF00) as i32
        } else {
            imm8 as i32
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
        r.store_u8(2, 0x11).unwrap();
        r.store_u8(3, 0x22).unwrap();

        // RAM contains DD 00 11 22

        assert_eq!(Ok(0x221100DD), r.load_u32(0));
        assert_eq!(Ok(0x00DD), r.load_u16(0));
        assert_eq!(Ok(0x2211), r.load_u16(2));
        assert_eq!(Ok(0xDD), r.load_u8(0));
        assert_eq!(Ok(0x00), r.load_u8(1));
        assert_eq!(Ok(0x11), r.load_u8(2));
        assert_eq!(Ok(0x22), r.load_u8(3));

        assert!(r.load_u16(1).is_err());
        assert!(r.load_u32(3).is_err());
        assert!(r.store_u16(1, 0).is_err());
        assert!(r.store_u32(3, 0).is_err());
    }

    #[test]
    fn mov_instruction() {
        let i = Armv6M::decode(0x2240);
        assert_eq!(
            Ok(Instruction::MovImm {
                rd: Register::R2,
                imm8: 64
            }),
            i
        );
        assert_eq!("MOV R2,#64", format!("{}", i.unwrap()));
    }

    #[test]
    fn branch_instruction() {
        let i = Armv6M::decode(0xe7fe);
        assert_eq!(Ok(Instruction::Branch { imm32: -4 }), i);
        assert_eq!("B +0", format!("{}", i.unwrap()));

        let i = Armv6M::decode(0xe7fc);
        assert_eq!(Ok(Instruction::Branch { imm32: -8 }), i);
        assert_eq!("B -4", format!("{}", i.unwrap()));

        let i = Armv6M::decode(0xe001);
        assert_eq!(Ok(Instruction::Branch { imm32: 2 }), i);
        assert_eq!("B +6", format!("{}", i.unwrap()));

        let mut cpu = Armv6M::new(0, 8);
        let mut ram = [0u32; 6];
        cpu.execute(Instruction::Branch { imm32: -4 }, &mut ram)
            .unwrap();
        // PC was 8, PC is still 8, because `0xe7fe` means spin in a loop
        assert_eq!(cpu.pc, 8);
    }

    #[test]
    fn bkpt_instruction() {
        let i = Armv6M::decode(0xbecc);
        assert_eq!(Ok(Instruction::Breakpoint { imm8: 0xCC }), i);
        assert_eq!("BKPT 0xcc", format!("{}", i.unwrap()));
    }

    #[test]
    fn ldr_instruction() {
        let i = Armv6M::decode(0x4801);
        assert_eq!(
            Ok(Instruction::LdrLiteral {
                rt: Register::R0,
                imm32: 4
            }),
            i
        );
        assert_eq!("LDR R0,#4", format!("{}", i.unwrap()));
    }

    #[test]
    fn movregt1_instruction() {
        let i = Armv6M::decode(0x46B6);
        assert_eq!(
            Ok(Instruction::MovRegT1 {
                rd: Register::Lr,
                rm: Register::R6,
            }),
            i
        );
        assert_eq!("MOV LR,R6", format!("{}", i.unwrap()));
    }

    #[test]
    fn push_instruction() {
        let i = Armv6M::decode(0xb580);
        assert_eq!(
            // Push {LR, R7}
            Ok(Instruction::Push {
                register_list: 0x80,
                m: true
            }),
            i
        );
        assert_eq!("PUSH {R7,LR}", format!("{}", i.unwrap()));

        let i = Armv6M::decode(0xb4FF);
        assert_eq!(
            Ok(Instruction::Push {
                register_list: 0xFF,
                m: false
            }),
            i
        );
        assert_eq!("PUSH {R7,R6,R5,R4,R3,R2,R1,R0}", format!("{}", i.unwrap()));
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
        let i = Armv6M::decode(0xaf00);
        assert_eq!(
            Ok(Instruction::AddSpT1 {
                rd: Register::R7,
                imm32: 0
            }),
            i
        );
        assert_eq!("ADD R7,SP,#0", format!("{}", i.unwrap()));
    }

    #[test]
    fn add_operation() {
        let sp = 32;
        let mut cpu = Armv6M::new(sp, 0x0);
        let mut ram = [15; 8];
        cpu.execute(
            Instruction::AddSpT1 {
                rd: Register::R0,
                imm32: 16,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(32 + 4 * 4, cpu.regs[0]);
    }

    #[test]
    fn branch_link_instruction() {
        assert_eq!(Err(Error::WideInstruction), Armv6M::decode(0xF04A));
        let i = Armv6M::decode32(0xF04A, 0xFE41);
        assert_eq!(Ok(Instruction::BranchLink { imm32: 0x4ac82 }), i);
        assert_eq!("BL +306310", format!("{}", i.unwrap()));
    }

    #[test]
    fn branch_link_operation() {
        // d0:	f04a fe41 	bl	4ad56
        let sp = 32;
        let start_pc = 0xd0;
        let mut cpu = Armv6M::new(sp, start_pc);
        let mut ram = [15; 8];
        cpu.execute(Instruction::BranchLink { imm32: 0x4ac82 }, &mut ram)
            .unwrap();
        assert_eq!(start_pc + 5, cpu.lr);
        assert_eq!(0x4ad56, cpu.pc);
    }

    #[test]
    fn branch_exchange_instruction() {
        let i = Armv6M::decode(0x4770);
        assert_eq!(Ok(Instruction::BranchExchange { rm: Register::Lr }), i);
        assert_eq!("BX LR", format!("{}", i.unwrap()));
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
    fn add_sp_imm_instruction() {
        let i = Armv6M::decode(0xb010);
        assert_eq!(Ok(Instruction::AddSpImm { imm32: 64 }), i);
        assert_eq!("ADD SP,#64", format!("{}", i.unwrap()));
    }

    #[test]
    fn add_sp_imm_operation() {
        let sp = 0x100;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [15; 8];
        cpu.execute(Instruction::AddSpImm { imm32: 64 }, &mut ram)
            .unwrap();
        assert_eq!(0x100 + 64, cpu.sp);
    }

    #[test]
    fn sub_sp_imm_instruction() {
        let i = Armv6M::decode(0xb090);
        assert_eq!(Ok(Instruction::SubSpImm { imm32: 64 }), i);
        assert_eq!("SUB SP,#64", format!("{}", i.unwrap()));
    }

    #[test]
    fn sub_sp_imm_operation() {
        let sp = 0x100;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [15; 8];
        cpu.execute(Instruction::SubSpImm { imm32: 64 }, &mut ram)
            .unwrap();
        assert_eq!(0x100 - 64, cpu.sp);
    }

    #[test]
    fn str_imm_t2_instruction() {
        let i = Armv6M::decode(0x9502);
        assert_eq!(
            Ok(Instruction::StrImmSp {
                rt: Register::R5,
                imm32: 8
            }),
            i
        );
        assert_eq!("STR R5,[SP,#8]", format!("{}", i.unwrap()));
    }

    #[test]
    fn str_imm_t2_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [15; 8];
        cpu.regs[1] = 0x12345678;
        cpu.execute(
            Instruction::StrImmSp {
                rt: Register::R1,
                imm32: 8,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(ram[((sp as usize) / 4) + 2], 0x12345678);
    }

    #[test]
    fn ldr_imm_t2_instruction() {
        let i = Armv6M::decode(0x9902);
        assert_eq!(
            Ok(Instruction::LdrImmSp {
                rt: Register::R1,
                imm32: 8
            }),
            i
        );
        assert_eq!("LDR R1,[SP,#8]", format!("{}", i.unwrap()));
    }

    #[test]
    fn ldr_imm_t2_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0, 0, 0, 0, 0, 0, 0x12345678, 0];
        cpu.execute(
            Instruction::LdrImmSp {
                rt: Register::R1,
                imm32: 8,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x12345678, cpu.regs[1]);
    }

    #[test]
    fn compare_instruction() {
        let i = Armv6M::decode(0x4288);
        assert_eq!(
            Ok(Instruction::CmpReg {
                rn: Register::R0,
                rm: Register::R1,
            }),
            i
        );
        assert_eq!("CMP R0,R1", format!("{}", i.unwrap()));
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

    #[test]
    fn strb_immediate_instruction() {
        let i = Armv6M::decode(0x7019);
        assert_eq!(
            Ok(Instruction::StrbImm {
                rt: Register::R1,
                rn: Register::R3,
                imm5: 0
            }),
            i
        );
        assert_eq!("STRB R1,[R3,#0]", format!("{}", i.unwrap()));
    }

    #[test]
    fn strb_immediate_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 4;
        cpu.regs[1] = 0x12345678;
        cpu.execute(
            Instruction::StrbImm {
                rt: Register::R1,
                rn: Register::R0,
                imm5: 1,
            },
            &mut ram,
        )
        .unwrap();
        // Lowest 8 bits of R1 written to address 4 + 1 = 5
        assert_eq!([0, 0x7800, 0, 0, 0, 0, 0, 0], ram);
    }

    #[test]
    fn str_immediate_instruction() {
        let i = Armv6M::decode(0x602C);
        assert_eq!(
            Ok(Instruction::StrImm {
                rt: Register::R4,
                rn: Register::R5,
                imm32: 0
            }),
            i
        );
        assert_eq!("STR R4,[R5,#0]", format!("{}", i.unwrap()));
    }

    #[test]
    fn str_immediate_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 4;
        cpu.regs[1] = 0x12345678;
        cpu.execute(
            Instruction::StrImm {
                rt: Register::R1,
                rn: Register::R0,
                imm32: 4,
            },
            &mut ram,
        )
        .unwrap();
        // R1 written to address 4 + (1 * 4) = 8
        assert_eq!([0, 0, 0x12345678, 0, 0, 0, 0, 0], ram);
    }

    #[test]
    fn adds_instruction() {
        let i = Armv6M::decode(0x1d00);
        assert_eq!(
            Ok(Instruction::Adds {
                rd: Register::R0,
                rn: Register::R0,
                imm3: 4
            }),
            i
        );
        assert_eq!("ADDS R0,R0,#4", format!("{}", i.unwrap()));
    }

    #[test]
    fn adds_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 4;
        cpu.regs[1] = 0x12345678;
        cpu.execute(
            Instruction::Adds {
                rd: Register::R0,
                rn: Register::R1,
                imm3: 1,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x12345679, cpu.regs[0]);
        assert!(!cpu.is_n());
        assert!(!cpu.is_z());
        assert!(!cpu.is_c());
        assert!(!cpu.is_v());
    }

    #[test]
    fn beq_instruction() {
        let i = Armv6M::decode(0xd003);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Eq,
                imm32: 6
            }),
            i
        );
        assert_eq!("BEQ +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bne_instruction() {
        let i = Armv6M::decode(0xd103);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Ne,
                imm32: 6
            }),
            i
        );
        assert_eq!("BNE +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bcs_instruction() {
        let i = Armv6M::decode(0xd203);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Cs,
                imm32: 6
            }),
            i
        );
        assert_eq!("BCS +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bcc_instruction() {
        let i = Armv6M::decode(0xd303);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Cc,
                imm32: 6
            }),
            i
        );
        assert_eq!("BCC +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bmi_instruction() {
        let i = Armv6M::decode(0xd403);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Mi,
                imm32: 6
            }),
            i
        );
        assert_eq!("BMI +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bpl_instruction() {
        let i = Armv6M::decode(0xd503);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Pl,
                imm32: 6
            }),
            i
        );
        assert_eq!("BPL +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bvs_instruction() {
        let i = Armv6M::decode(0xd603);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Vs,
                imm32: 6
            }),
            i
        );
        assert_eq!("BVS +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bvc_instruction() {
        let i = Armv6M::decode(0xd703);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Vc,
                imm32: 6
            }),
            i
        );
        assert_eq!("BVC +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bhi_instruction() {
        let i = Armv6M::decode(0xd803);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Hi,
                imm32: 6
            }),
            i
        );
        assert_eq!("BHI +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bls_instruction() {
        let i = Armv6M::decode(0xd903);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Ls,
                imm32: 6
            }),
            i
        );
        assert_eq!("BLS +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bge_instruction() {
        let i = Armv6M::decode(0xda03);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Ge,
                imm32: 6
            }),
            i
        );
        assert_eq!("BGE +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn blt_instruction() {
        let i = Armv6M::decode(0xdb03);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Lt,
                imm32: 6
            }),
            i
        );
        assert_eq!("BLT +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bgt_instruction() {
        let i = Armv6M::decode(0xdc03);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Gt,
                imm32: 6
            }),
            i
        );
        assert_eq!("BGT +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn ble_instruction() {
        let i = Armv6M::decode(0xdd03);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Le,
                imm32: 6
            }),
            i
        );
        assert_eq!("BLE +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn balways_instruction() {
        let i = Armv6M::decode(0xde03);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Always,
                imm32: 6
            }),
            i
        );
        assert_eq!("B +10", format!("{}", i.unwrap()));
    }
}

// End of file
