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
    /// We don't understand this special register
    UnknownSpecialRegister(u8),
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
    /// Signed greater than or equal (N == V)
    Ge = 10,
    /// Signed less than (N != V)
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
    // =======================================================================
    // Load Instructions
    // =======================================================================
    /// `LDR <Rt>,[PC,#<imm8>]`
    LdrLiteral {
        /// Which register to load into
        rt: Register,
        /// Offset to add to PC
        imm32: u32,
    },
    /// `LDR <Rt>,[<Rn>{,#<imm5>}]`
    LdrImm {
        /// Which register to load into
        rt: Register,
        /// Which register contains the base address to read from
        rn: Register,
        /// What offset to apply to the base address
        imm32: u32,
    },
    /// `LDRB <Rt>,[<Rn>{,#<imm5>}]`
    LdrbImm {
        /// Which register to load into
        rt: Register,
        /// Which register contains the base address to read from
        rn: Register,
        /// What offset to apply to the base address
        imm5: u8,
    },
    // =======================================================================
    // Stack Instructions
    // =======================================================================`
    /// `PUSH {<register list>}`
    Push {
        /// A bitmask of registers (R7 to R0) to push
        register_list: RegisterList,
        /// Also push LR
        m: bool,
    },
    /// `POP {<register list>}`
    Pop {
        /// A bitmask of registers (R7 to R0) to pop
        register_list: RegisterList,
        /// Also pop LR
        p: bool,
    },
    /// `LDMIA <Rn>!,{<register list>}`
    Ldmia {
        /// Base address to load from
        rn: Register,
        /// Register list
        register_list: RegisterList,
    },
    /// `STMIA <Rn>!,{<register list>}`
    Stmia {
        /// Base address to save to
        rn: Register,
        /// Register list
        register_list: RegisterList,
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
    /// `LDR <Rt>,[SP,#<imm8>]`
    LdrSpImm {
        /// Which register to store the loaded value in
        rt: Register,
        /// The offset to apply to the stack pointer
        imm32: u32,
    },
    /// `STR <Rt>,[SP,#<imm8>]`
    StrSpImm {
        /// Which register to contains the value to store
        rt: Register,
        /// The offset to apply to the stack pointer
        imm32: u32,
    },
    // =======================================================================
    // Arithmetic Instructions
    // =======================================================================
    /// `ADDS <Rd>,<Rn>,#<imm3>`
    AddsImm {
        /// Which register to store the result in
        rd: Register,
        /// Which register to get the value from
        rn: Register,
        /// How much to add to <Rn>
        imm3: u8,
    },
    /// `ADDS <Rd>,<Rn>,<Rm>`
    AddsReg {
        /// Which register to store the result in
        rd: Register,
        /// Which register to get the first value from
        rn: Register,
        /// Which register to get the second value from
        rm: Register,
    },
    /// `ADCS <Rdn>,<Rm>`
    AdcsReg {
        /// Which register to add to
        rdn: Register,
        /// Which register to get the value from
        rm: Register,
    },
    /// `SUBS <Rd>,<Rn>,#<imm3>`
    SubsImm {
        /// Which register to store the result in
        rd: Register,
        /// Which register to get the value from
        rn: Register,
        /// How much to subtract from <Rn>
        imm3: u8,
    },
    /// `SUBS <Rd>,<Rn>,<Rm>`
    SubsReg {
        /// Which register to store the result in
        rd: Register,
        /// Which register to get the first value from
        rn: Register,
        /// Which register to get the second value from
        rm: Register,
    },
    /// `RSBS <Rd>,<Rn>,#0`
    RsbImm {
        /// Which register to store the result in
        rd: Register,
        /// Which register to get the value from
        rn: Register,
    },
    /// `LSLS <Rd>,<Rm>,#<imm>`
    LslsImm {
        /// Which register to store the result in
        rd: Register,
        /// Which register to get the value from
        rm: Register,
        /// How many bits to shift
        imm5: u8,
    },
    /// `LSLS <Rdn>,<Rm>
    LslsReg {
        /// Which register to shift
        rdn: Register,
        /// How many bits to shift (read from bottom byte)
        rm: Register,
    },
    /// `LSRS <Rx>,<Rx>,#<imm>`
    LsrsImm {
        /// Which register to store the result in
        rd: Register,
        /// Which register to get the value from
        rm: Register,
        /// How many bits to shift
        imm5: u8,
    },
    /// `LSRS <Rdn>,<Rm>
    LsrsReg {
        /// Which register to shift
        rdn: Register,
        /// How many bits to shift (read from bottom byte)
        rm: Register,
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
    /// `CMP <Rn>,#<imm>`
    CmpImm {
        /// The left hand register to compare
        rn: Register,
        /// The right hand immediate value to compare
        imm32: u32,
    },
    /// `ANDS <Rdn>,<Rm>`
    AndsReg {
        /// The left hand register for the operation, and destination
        rdn: Register,
        /// The right hand register for the operation
        rm: Register,
    },
    /// `ORRS <Rdn>,<Rm>`
    OrrsReg {
        /// The left hand register for the operation, and destination
        rdn: Register,
        /// The right hand register for the operation
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
    /// `MRS <Rd>,<spec_reg>`
    Mrs {
        /// Which register to read from
        rd: Register,
        /// Which special register to write to
        sys_m: u8,
    },
    /// `MSR <Rn>,<spec_reg>`
    Msr {
        /// Which register to write to
        rn: Register,
        /// Which special register to read from
        sys_m: u8,
    },
    /// `CPSID i` or `CPSIE i`
    Cps {
        /// `true` to disable interrupts, `false` to enable
        im: bool,
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
            Instruction::LdrLiteral { rt, imm32 } => write!(f, "LDR {},[PC,#{}]", rt, imm32),
            Instruction::LdrImm { rt, rn, imm32 } => write!(f, "LDR {},[{},#{}]", rt, rn, imm32),
            Instruction::LdrbImm { rt, rn, imm5 } => write!(f, "LDRB {},[{},#{}]", rt, rn, imm5),
            Instruction::Push { register_list, m } => {
                write!(f, "PUSH {{{}", register_list)?;
                if *m {
                    if !register_list.is_empty() {
                        write!(f, ",")?;
                    }
                    write!(f, "LR")?;
                }
                write!(f, "}}")
            }
            Instruction::Pop { register_list, p } => {
                write!(f, "POP {{{}", register_list)?;
                if *p {
                    if !register_list.is_empty() {
                        write!(f, ",")?;
                    }
                    write!(f, "PC")?;
                }
                write!(f, "}}")
            }
            Instruction::Ldmia { rn, register_list } => {
                write!(f, "LDMIA {}!,{{{}}}", rn, register_list)
            }
            Instruction::Stmia { rn, register_list } => {
                write!(f, "STMIA {}!,{{{}}}", rn, register_list)
            }
            Instruction::AddSpT1 { rd, imm32 } => write!(f, "ADD {},SP,#{}", rd, imm32),
            Instruction::AddSpImm { imm32 } => write!(f, "ADD SP,#{}", imm32),
            Instruction::SubSpImm { imm32 } => write!(f, "SUB SP,#{}", imm32),
            Instruction::LdrSpImm { rt, imm32 } => write!(f, "LDR {},[SP,#{}]", rt, imm32),
            Instruction::StrSpImm { rt, imm32 } => write!(f, "STR {},[SP,#{}]", rt, imm32),
            Instruction::AddsImm { rd, rn, imm3 } => write!(f, "ADDS {},{},#{}", rd, rn, imm3),
            Instruction::AddsReg { rd, rn, rm } => write!(f, "ADDS {},{},{}", rd, rn, rm),
            Instruction::AdcsReg { rdn, rm } => write!(f, "ADCS {},{}", rdn, rm),
            Instruction::SubsImm { rd, rn, imm3 } => write!(f, "SUBS {},{},#{}", rd, rn, imm3),
            Instruction::SubsReg { rd, rn, rm } => write!(f, "SUBS {},{},{}", rd, rn, rm),
            Instruction::RsbImm { rd, rn } => write!(f, "RSBS {},{},#0", rd, rn),
            Instruction::LslsImm { rd, rm, imm5 } => write!(f, "LSLS {},{},#{}", rd, rm, imm5),
            Instruction::LslsReg { rdn, rm } => write!(f, "LSLS {},{}", rdn, rm),
            Instruction::LsrsImm { rd, rm, imm5 } => write!(f, "LSRS {},{},#{}", rd, rm, imm5),
            Instruction::LsrsReg { rdn, rm } => write!(f, "LSRS {},{}", rdn, rm),
            Instruction::CmpReg { rn, rm } => write!(f, "CMP {},{}", rn, rm),
            Instruction::CmpImm { rn, imm32 } => write!(f, "CMP {},#{}", rn, imm32),
            Instruction::AndsReg { rdn, rm } => write!(f, "ANDS {},{}", rdn, rm),
            Instruction::OrrsReg { rdn, rm } => write!(f, "ORRS {},{}", rdn, rm),
            Instruction::Breakpoint { imm8 } => write!(f, "BKPT 0x{:02x}", imm8),
            Instruction::Mrs { rd, sys_m } => write!(f, "MRS {},{}", rd, sys_m),
            Instruction::Msr { rn, sys_m } => write!(f, "MSR {},{}", rn, sys_m),
            Instruction::Cps { im } => write!(f, "CPSI{} i", if *im { "D" } else { "E" }),
        }
    }
}

/// Identifies a register in our CPU
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum Register {
    /// Register R0
    R0 = 0,
    /// Register R1
    R1 = 1,
    /// Register R2
    R2 = 2,
    /// Register R3
    R3 = 3,
    /// Register R4
    R4 = 4,
    /// Register R5
    R5 = 5,
    /// Register R6
    R6 = 6,
    /// Register R7
    R7 = 7,
    /// Register R8
    R8 = 8,
    /// Register R9
    R9 = 9,
    /// Register R10
    R10 = 10,
    /// Register R11
    R11 = 11,
    /// Register R12
    R12 = 12,
    /// Link Register
    Lr = 13,
    /// Stack Pointer
    Sp = 14,
    /// Program Counter
    Pc = 15,
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

/// A set of registers, from R0 to R7
#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
pub struct RegisterList(u8);

impl RegisterList {
    /// Create a new register list
    ///
    /// The R0 bit is the least significant bit and the R7 bit is the most
    /// significant bit.
    pub fn new(value: u8) -> RegisterList {
        RegisterList(value)
    }

    /// Does this list have any bits set?
    pub fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// Is this register in this list?
    pub fn is_set(self, register: Register) -> bool {
        let bit = register as u8;
        (self.0 & (1 << bit)) != 0
    }
}

impl std::fmt::Display for RegisterList {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut first = true;
        for reg in 0..=7 {
            if (self.0 & (1 << reg)) != 0 {
                if !first {
                    write!(f, ",")?;
                }
                first = false;
                write!(f, "R{}", reg)?;
            }
        }
        Ok(())
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
    primask: u32,
}

impl Armv6M {
    const FLAG_N: u32 = 1 << 31;
    const FLAG_Z: u32 = 1 << 30;
    const FLAG_C: u32 = 1 << 29;
    const FLAG_V: u32 = 1 << 28;

    const SYS_M_PRIMASK: u8 = 0b0001_0000;

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
            let cond = ((word >> 8) & 0b1111) as u8;
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
            Ok(Instruction::StrSpImm {
                rt: Register::from(rt),
                imm32: u32::from(imm8) << 2,
            })
        } else if (word >> 11) == 0b10011 {
            let imm8 = (word & 0xFF) as u8;
            let rt = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::LdrSpImm {
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
        } else if (word >> 11) == 0b01111 {
            let imm5 = ((word >> 6) & 0x1F) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rt = (word & 0b111) as u8;
            Ok(Instruction::LdrbImm {
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
        } else if (word >> 11) == 0b01101 {
            let imm5 = ((word >> 6) & 0x1F) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rt = (word & 0b111) as u8;
            Ok(Instruction::LdrImm {
                rt: Register::from(rt),
                rn: Register::from(rn),
                imm32: u32::from(imm5) << 2,
            })
        } else if (word >> 11) == 0b00000 {
            let imm5 = ((word >> 6) & 0x1F) as u8;
            let rm = ((word >> 3) & 0b111) as u8;
            let rd = (word & 0b111) as u8;
            Ok(Instruction::LslsImm {
                rd: Register::from(rd),
                rm: Register::from(rm),
                imm5,
            })
        } else if (word >> 11) == 0b00001 {
            let mut imm5 = ((word >> 6) & 0x1F) as u8;
            if imm5 == 0 {
                imm5 = 32;
            }
            let rm = ((word >> 3) & 0b111) as u8;
            let rd = (word & 0b111) as u8;
            Ok(Instruction::LsrsImm {
                rd: Register::from(rd),
                rm: Register::from(rm),
                imm5,
            })
        } else if (word >> 11) == 0b00101 {
            let imm8 = (word & 0xFF) as u8;
            let rn = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::CmpImm {
                rn: Register::from(rn),
                imm32: u32::from(imm8),
            })
        } else if (word >> 11) == 0b11000 {
            let register_list = (word & 0xFF) as u8;
            let rn = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::Stmia {
                rn: Register::from(rn),
                register_list: RegisterList::new(register_list),
            })
        } else if (word >> 11) == 0b11001 {
            let register_list = (word & 0xFF) as u8;
            let rn = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::Ldmia {
                rn: Register::from(rn),
                register_list: RegisterList::new(register_list),
            })
        } else if (word >> 9) == 0b1011010 {
            let m = ((word >> 8) & 1) == 1;
            let register_list = word as u8;
            Ok(Instruction::Push {
                register_list: RegisterList::new(register_list),
                m,
            })
        } else if (word >> 9) == 0b1011110 {
            let p = ((word >> 8) & 1) == 1;
            let register_list = word as u8;
            Ok(Instruction::Pop {
                register_list: RegisterList::new(register_list),
                p,
            })
        } else if (word >> 9) == 0b0001110 {
            let imm3 = ((word >> 6) & 0b111) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rd = (word & 0b111) as u8;
            Ok(Instruction::AddsImm {
                rd: Register::from(rd),
                rn: Register::from(rn),
                imm3,
            })
        } else if (word >> 9) == 0b0001100 {
            let rm = ((word >> 6) & 0b111) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rd = (word & 0b111) as u8;
            Ok(Instruction::AddsReg {
                rd: Register::from(rd),
                rn: Register::from(rn),
                rm: Register::from(rm),
            })
        } else if (word >> 9) == 0b0001111 {
            let imm3 = ((word >> 6) & 0b111) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rd = (word & 0b111) as u8;
            Ok(Instruction::SubsImm {
                rd: Register::from(rd),
                rn: Register::from(rn),
                imm3,
            })
        } else if (word >> 9) == 0b001101 {
            let rm = ((word >> 6) & 0b111) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rd = (word & 0b111) as u8;
            Ok(Instruction::SubsReg {
                rd: Register::from(rd),
                rn: Register::from(rn),
                rm: Register::from(rm),
            })
        } else if (word >> 8) == 0b10111110 {
            let imm8 = (word & 0xFF) as u8;
            Ok(Instruction::Breakpoint { imm8 })
        } else if (word >> 8) == 0b01000110 {
            let d: u8 = ((word >> 7) & 0b1) as u8;
            let rd = (d << 3) | (word & 0b111) as u8;
            let rm: u8 = ((word >> 3) & 0b1111) as u8;
            Ok(Instruction::MovRegT1 {
                rm: Register::from(rm),
                rd: Register::from(rd),
            })
        } else if (word >> 7) == 0b010001110 {
            let rm = ((word >> 3) & 0b1111) as u8;
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
        } else if (word >> 6) == 0b0100000010 {
            let rm = ((word >> 3) & 0b111) as u8;
            let rdn = (word & 0b111) as u8;
            Ok(Instruction::LslsReg {
                rdn: Register::from(rdn),
                rm: Register::from(rm),
            })
        } else if (word >> 6) == 0b0100000011 {
            let rm = ((word >> 3) & 0b111) as u8;
            let rdn = (word & 0b111) as u8;
            Ok(Instruction::LsrsReg {
                rdn: Register::from(rdn),
                rm: Register::from(rm),
            })
        } else if (word >> 6) == 0b0100001010 {
            let rm = ((word >> 3) & 0b111) as u8;
            let rn = (word & 0b111) as u8;
            Ok(Instruction::CmpReg {
                rn: Register::from(rn),
                rm: Register::from(rm),
            })
        } else if (word >> 6) == 0b0100000000 {
            let rm = ((word >> 3) & 0b111) as u8;
            let rdn = (word & 0b111) as u8;
            Ok(Instruction::AndsReg {
                rdn: Register::from(rdn),
                rm: Register::from(rm),
            })
        } else if (word >> 6) == 0b0100001100 {
            let rm = ((word >> 3) & 0b111) as u8;
            let rdn = (word & 0b111) as u8;
            Ok(Instruction::OrrsReg {
                rdn: Register::from(rdn),
                rm: Register::from(rm),
            })
        } else if (word >> 6) == 0b0100001001 {
            let rn = ((word >> 3) & 0b111) as u8;
            let rd = (word & 0b111) as u8;
            Ok(Instruction::RsbImm {
                rd: Register::from(rd),
                rn: Register::from(rn),
            })
        } else if (word >> 6) == 0b0100000101 {
            let rm = ((word >> 3) & 0b111) as u8;
            let rdn = (word & 0b111) as u8;
            Ok(Instruction::AdcsReg {
                rdn: Register::from(rdn),
                rm: Register::from(rm),
            })
        } else if (word >> 5) == 0b10110110011 {
            let im = ((word >> 4) & 1) != 0;
            Ok(Instruction::Cps { im })
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
        } else if (word1 == 0b1111001111101111) && (word2 >> 12) == 0b1000 {
            let sys_m = (word2 & 0xFF) as u8;
            let rd = ((word2 >> 8) & 0b1111) as u8;
            Ok(Instruction::Mrs {
                rd: Register::from(rd),
                sys_m,
            })
        } else if (word1 >> 4) == 0b111100111000 && (word2 >> 8) == 0b10001000 {
            let sys_m = (word2 & 0xFF) as u8;
            let rn = (word1 & 0b1111) as u8;
            Ok(Instruction::Msr {
                rn: Register::from(rn),
                sys_m,
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
            // =======================================================================
            // Branch Instructions
            // =======================================================================
            Instruction::Branch { imm32 } => {
                // Assume PC's next increment has happened already
                self.pc = self.pc.wrapping_add(2);
                self.pc = self.pc.wrapping_add(imm32 as u32);
            }
            Instruction::BranchConditional { cond, imm32 } => {
                if self.check_condition(cond) {
                    // Assume PC's next increment has happened already
                    self.pc = self.pc.wrapping_add(2);
                    self.pc = self.pc.wrapping_add(imm32 as u32);
                }
            }
            Instruction::BranchLink { imm32 } => {
                // Assume PC's next increment has happened already
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
            // =======================================================================
            // Move Instructions
            // =======================================================================
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
            // =======================================================================
            // Store Instructions
            // =======================================================================
            Instruction::StrImm { rt, rn, imm32 } => {
                let addr = self.fetch_reg(rn);
                let offset_addr = addr.wrapping_add(imm32);
                let value = self.fetch_reg(rt);
                memory.store_u32(offset_addr, value)?;
            }
            Instruction::StrbImm { rt, rn, imm5 } => {
                let imm32 = u32::from(imm5);
                let addr = self.fetch_reg(rn);
                let offset_addr = addr.wrapping_add(imm32);
                let value = self.fetch_reg(rt);
                memory.store_u8(offset_addr, (value & 0xFF) as u8)?;
            }
            // =======================================================================
            // Load Instructions
            // =======================================================================
            Instruction::LdrLiteral { rt, imm32 } => {
                // Assume's PC next increment has happened already
                // Also align to 4 bytes
                let base = (self.pc.wrapping_add(2)) & !0b11;
                let addr = base.wrapping_add(imm32);
                let value = memory.load_u32(addr)?;
                self.store_reg(rt, value);
            }
            Instruction::LdrImm { rt, rn, imm32 } => {
                let addr = self.fetch_reg(rn);
                let offset_addr = addr.wrapping_add(imm32);
                let value = memory.load_u32(offset_addr)?;
                self.store_reg(rt, value);
            }
            Instruction::LdrbImm { rt, rn, imm5 } => {
                let addr = self.fetch_reg(rn);
                let offset_addr = addr.wrapping_add(u32::from(imm5));
                let value = memory.load_u32(offset_addr)? & 0xFF;
                self.store_reg(rt, value);
            }
            // =======================================================================
            // Stack Instructions
            // =======================================================================`
            Instruction::Push { register_list, m } => {
                if m {
                    let lr = self.lr;
                    self.push_stack(lr, memory)?;
                }
                for register in (0..=7).rev().map(Register::from) {
                    if register_list.is_set(register) {
                        let value = self.fetch_reg(register);
                        self.push_stack(value, memory)?;
                    }
                }
            }
            Instruction::Pop { register_list, p } => {
                for register in (0..=7).map(Register::from) {
                    if register_list.is_set(register) {
                        let value = self.pop_stack(memory)?;
                        self.store_reg(register, value);
                    }
                }
                if p {
                    self.pc = self.pop_stack(memory)? & !1;
                }
            }
            Instruction::Ldmia { rn, register_list } => {
                let mut base = self.fetch_reg(rn);
                for register in (0..=7).map(Register::from) {
                    if register_list.is_set(register) {
                        let value = memory.load_u32(base)?;
                        base = base.wrapping_add(4);
                        self.store_reg(register, value);
                    }
                }
                if !register_list.is_set(rn) {
                    self.store_reg(rn, base);
                }
            }
            Instruction::Stmia { rn, register_list } => {
                let mut base = self.fetch_reg(rn);
                for register in (0..=7).map(Register::from) {
                    if register_list.is_set(register) {
                        let value = self.fetch_reg(register);
                        memory.store_u32(base, value)?;
                        base = base.wrapping_add(4);
                    }
                }
                if !register_list.is_set(rn) {
                    self.store_reg(rn, base);
                }
            }
            Instruction::AddSpT1 { rd, imm32 } => {
                let sp = self.sp;
                let value = sp.wrapping_add(imm32);
                self.store_reg(rd, value);
            }
            Instruction::AddSpImm { imm32 } => {
                self.sp = self.sp.wrapping_add(imm32);
            }
            Instruction::SubSpImm { imm32 } => {
                // This does a subtraction
                let value = self.sp.wrapping_add(!imm32).wrapping_add(1);
                self.sp = value;
            }
            Instruction::LdrSpImm { rt, imm32 } => {
                let offset_addr = self.sp.wrapping_add(imm32);
                let value = memory.load_u32(offset_addr)?;
                self.store_reg(rt, value);
            }
            Instruction::StrSpImm { rt, imm32 } => {
                let offset_addr = self.sp.wrapping_add(imm32);
                let value = self.fetch_reg(rt);
                memory.store_u32(offset_addr, value)?;
            }
            // =======================================================================
            // Arithmetic Instructions
            // =======================================================================
            Instruction::AddsImm { rd, rn, imm3 } => {
                let value1 = self.fetch_reg(rn);
                let value2 = u32::from(imm3);
                let (result, carry_out, overflow) = Self::add_with_carry(value1, value2, false);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry_out);
                self.set_v(overflow);
                self.store_reg(rd, result);
            }
            Instruction::AddsReg { rd, rn, rm } => {
                let value1 = self.fetch_reg(rn);
                let value2 = self.fetch_reg(rm);
                let (result, carry_out, overflow) = Self::add_with_carry(value1, value2, false);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry_out);
                self.set_v(overflow);
                self.store_reg(rd, result);
            }
            Instruction::AdcsReg { rdn, rm } => {
                let value1 = self.fetch_reg(rdn);
                let value2 = self.fetch_reg(rm);
                let (result, carry_out, overflow) =
                    Self::add_with_carry(value1, value2, self.is_c());
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry_out);
                self.set_v(overflow);
                self.store_reg(rdn, result);
            }
            Instruction::SubsImm { rd, rn, imm3 } => {
                let value1 = self.fetch_reg(rn);
                let value2 = u32::from(imm3);
                let (result, carry_out, overflow) = Self::add_with_carry(value1, !value2, true);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry_out);
                self.set_v(overflow);
                self.store_reg(rd, result);
            }
            Instruction::SubsReg { rd, rn, rm } => {
                let value1 = self.fetch_reg(rn);
                let value2 = self.fetch_reg(rm);
                let (result, carry_out, overflow) = Self::add_with_carry(value1, !value2, true);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry_out);
                self.set_v(overflow);
                self.store_reg(rd, result);
            }
            Instruction::RsbImm { rd, rn } => {
                let value1 = self.fetch_reg(rn);
                let (result, carry_out, overflow) = Self::add_with_carry(!value1, 0, true);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry_out);
                self.set_v(overflow);
                self.store_reg(rd, result);
            }
            Instruction::LslsImm { rd, rm, imm5 } => {
                let value = self.fetch_reg(rm);
                let shift_n = u32::from(imm5);
                let carry_in = self.is_c();
                let (result, carry_out) = if shift_n == 0 {
                    (value, carry_in)
                } else {
                    let wide_value = u64::from(value);
                    let shifted = wide_value << shift_n;
                    (shifted as u32, (shifted & (1 << 32)) != 0)
                };
                self.store_reg(rd, result);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry_out);
            }
            Instruction::LslsReg { rdn, rm } => {
                let value = self.fetch_reg(rdn);
                let shift_n = self.fetch_reg(rm) & 0xFF;
                let carry_in = self.is_c();
                let (result, carry_out) = if shift_n == 0 {
                    (value, carry_in)
                } else {
                    let wide_value = u64::from(value);
                    let shifted = wide_value << shift_n;
                    (shifted as u32, (shifted & (1 << 32)) != 0)
                };
                self.store_reg(rdn, result);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry_out);
            }
            Instruction::LsrsImm { rd, rm, imm5 } => {
                let value = self.fetch_reg(rm);
                let shift_n = u32::from(imm5);
                let result = value >> shift_n;
                let carry_out = (value & (1 << (shift_n - 1))) != 0;
                self.store_reg(rd, result);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry_out);
            }
            Instruction::LsrsReg { rdn, rm } => {
                let value = self.fetch_reg(rdn);
                let shift_n = self.fetch_reg(rm) & 0xFF;
                let result = value >> shift_n;
                let carry_out = (value & (1 << (shift_n - 1))) != 0;
                self.store_reg(rdn, result);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry_out);
            }
            // =======================================================================
            // Logical Instructions
            // =======================================================================
            Instruction::CmpReg { rm, rn } => {
                let value_m = self.fetch_reg(rm);
                let value_n = self.fetch_reg(rn);
                let (result, carry, overflow) = Self::add_with_carry(value_n, !value_m, true);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry);
                self.set_v(overflow);
            }
            Instruction::CmpImm { rn, imm32 } => {
                let value_n = self.fetch_reg(rn);
                let (result, carry, overflow) = Self::add_with_carry(value_n, !imm32, true);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry);
                self.set_v(overflow);
            }
            Instruction::AndsReg { rdn, rm } => {
                let value_m = self.fetch_reg(rm);
                let value_n = self.fetch_reg(rdn);
                let result = value_n & value_m;
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.store_reg(rdn, result);
            }
            Instruction::OrrsReg { rdn, rm } => {
                let value_m = self.fetch_reg(rm);
                let value_n = self.fetch_reg(rdn);
                let result = value_n | value_m;
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.store_reg(rdn, result);
            }
            // =======================================================================
            // Other Instructions
            // =======================================================================
            Instruction::Breakpoint { imm8 } => {
                // Wide instruction
                self.pc = self.pc.wrapping_add(2);
                self.breakpoint = Some(imm8);
            }
            Instruction::Mrs { rd, sys_m } => {
                // Wide instruction
                self.pc = self.pc.wrapping_add(2);

                match sys_m {
                    Self::SYS_M_PRIMASK => {
                        self.primask = self.fetch_reg(rd);
                    }
                    _ => {
                        return Err(Error::UnknownSpecialRegister(sys_m));
                    }
                }
            }
            Instruction::Msr { rn, sys_m } => {
                // Wide instruction
                self.pc = self.pc.wrapping_add(2);
                match sys_m {
                    Self::SYS_M_PRIMASK => {
                        self.store_reg(rn, self.primask);
                    }
                    _ => {
                        return Err(Error::UnknownSpecialRegister(sys_m));
                    }
                }
            }
            Instruction::Cps { im } => {
                if im {
                    // disable
                    self.primask |= 1;
                } else {
                    // enable
                    self.primask &= !1;
                }
            }
        }
        Ok(())
    }

    /// Adds two 32-bit numbers, with carry in, producing a result, carry out, and overflow.
    fn add_with_carry(value1: u32, value2: u32, carry_in: bool) -> (u32, bool, bool) {
        let uv1 = u64::from(value1);
        let uv2 = u64::from(value2);
        let uv3: u64 = u64::from(carry_in);
        let uresult = uv1.wrapping_add(uv2).wrapping_add(uv3);

        let sv1 = i64::from(value1 as i32);
        let sv2 = i64::from(value2 as i32);
        let sv3: i64 = i64::from(carry_in);
        let sresult = sv1.wrapping_add(sv2).wrapping_add(sv3);

        let result = uresult as u32;

        let carry_out = u64::from(result) != uresult;
        let overflow_out = i64::from(result as i32) != sresult;

        println!(
            "add_with_carry {:#x} + {:#x} + {} -> {:#x} c={} o={}",
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

    /// Pop one value off the stack
    fn pop_stack(&mut self, memory: &mut dyn Memory) -> Result<u32, Error> {
        let value = memory.load_u32(self.sp)?;
        println!("Popping {:08x} from {:08x}", value, self.sp);
        self.store_reg(Register::Sp, self.sp + 4);
        Ok(value)
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
        (((imm11 << 5) as i16) as i32) >> 5
    }

    /// Given a signed 8-bit word offset, sign-extend it up to an i32 byte offset.
    fn sign_extend_imm8(imm8: u8) -> i32 {
        (imm8 as i8) as i32
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

    // =======================================================================
    // Branch Instructions
    // =======================================================================
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
    // =======================================================================
    // Move Instructions
    // =======================================================================
    #[test]
    fn mov_imm_instruction() {
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
    fn mov_regt1_instruction() {
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
    // =======================================================================
    // Store Instructions
    // =======================================================================
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
    // =======================================================================
    // Load Instructions
    // =======================================================================
    #[test]
    fn ldr_literal_instruction() {
        let i = Armv6M::decode(0x4801);
        assert_eq!(
            Ok(Instruction::LdrLiteral {
                rt: Register::R0,
                imm32: 4
            }),
            i
        );
        assert_eq!("LDR R0,[PC,#4]", format!("{}", i.unwrap()));
    }

    #[test]
    fn ldr_immediate_instruction() {
        let i = Armv6M::decode(0x682C);
        assert_eq!(
            Ok(Instruction::LdrImm {
                rt: Register::R4,
                rn: Register::R5,
                imm32: 0
            }),
            i
        );
        assert_eq!("LDR R4,[R5,#0]", format!("{}", i.unwrap()));
    }

    #[test]
    fn ldr_immediate_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0, 0, 0x12345678, 0, 0, 0, 0, 0];
        cpu.regs[0] = 4;
        cpu.execute(
            Instruction::LdrImm {
                rt: Register::R1,
                rn: Register::R0,
                imm32: 4,
            },
            &mut ram,
        )
        .unwrap();
        // R1 loaded from address 4 + (1 * 4) = 8
        assert_eq!(0x12345678, cpu.regs[1]);
    }
    #[test]
    fn ldrb_immediate_instruction() {
        let i = Armv6M::decode(0x782C);
        assert_eq!(
            Ok(Instruction::LdrbImm {
                rt: Register::R4,
                rn: Register::R5,
                imm5: 0
            }),
            i
        );
        assert_eq!("LDRB R4,[R5,#0]", format!("{}", i.unwrap()));
    }
    #[test]
    fn ldrb_immediate_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0, 0, 0x12345678, 0, 0, 0, 0, 0];
        cpu.regs[0] = 4;
        cpu.execute(
            Instruction::LdrbImm {
                rt: Register::R1,
                rn: Register::R0,
                imm5: 4,
            },
            &mut ram,
        )
        .unwrap();
        // R1 loaded from address 4 + (1 * 4) = 8
        assert_eq!(0x78, cpu.regs[1]);
    }
    // =======================================================================
    // Stack Instructions
    // =======================================================================`
    #[test]
    fn push_instruction() {
        let i = Armv6M::decode(0xb580);
        assert_eq!(
            // Push {LR, R7}
            Ok(Instruction::Push {
                register_list: RegisterList::new(0x80),
                m: true
            }),
            i
        );
        assert_eq!("PUSH {R7,LR}", format!("{}", i.unwrap()));

        let i = Armv6M::decode(0xb4FF);
        assert_eq!(
            Ok(Instruction::Push {
                register_list: RegisterList::new(0xFF),
                m: false
            }),
            i
        );
        assert_eq!("PUSH {R0,R1,R2,R3,R4,R5,R6,R7}", format!("{}", i.unwrap()));
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
                register_list: RegisterList::new(0x55),
                m: false,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!([15, 15, 15, 15, 0, 2, 4, 6], ram);
    }

    #[test]
    fn pop_instruction() {
        let i = Armv6M::decode(0xbd80);
        assert_eq!(
            Ok(Instruction::Pop {
                register_list: RegisterList::new(0x80),
                p: true
            }),
            i
        );
        assert_eq!("POP {R7,PC}", format!("{}", i.unwrap()));

        let i = Armv6M::decode(0xBCFF);
        assert_eq!(
            Ok(Instruction::Pop {
                register_list: RegisterList::new(0xFF),
                p: false
            }),
            i
        );
        assert_eq!("POP {R0,R1,R2,R3,R4,R5,R6,R7}", format!("{}", i.unwrap()));
    }

    #[test]
    fn pop_operation() {
        let mut cpu = Armv6M::new(16, 0x0);
        let mut ram = [15, 15, 15, 15, 0, 2, 4, 6];
        cpu.execute(
            Instruction::Pop {
                register_list: RegisterList::new(0x55),
                p: false,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0, cpu.regs[0]);
        assert_eq!(2, cpu.regs[2]);
        assert_eq!(4, cpu.regs[4]);
        assert_eq!(6, cpu.regs[6]);
    }

    #[test]
    fn ldmia_instruction() {
        let i = Armv6M::decode(0xc938);
        assert_eq!(
            Ok(Instruction::Ldmia {
                rn: Register::R1,
                register_list: RegisterList::new(0x38),
            }),
            i
        );
        assert_eq!("LDMIA R1!,{R3,R4,R5}", format!("{}", i.unwrap()));
    }

    #[test]
    fn ldmia_operation() {
        let mut cpu = Armv6M::new(16, 0x0);
        let mut ram = [1, 2, 3, 4, 5, 6, 7, 8];
        cpu.regs[0] = 4;
        cpu.execute(
            Instruction::Ldmia {
                // R1, R3, R5, R7
                register_list: RegisterList::new(0xAA),
                rn: Register::R0,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(4 + 16, cpu.regs[0]);
        assert_eq!(2, cpu.regs[1]);
        assert_eq!(3, cpu.regs[3]);
        assert_eq!(4, cpu.regs[5]);
        assert_eq!(5, cpu.regs[7]);
    }

    #[test]
    fn stmia_instruction() {
        let i = Armv6M::decode(0xc038);
        assert_eq!(
            Ok(Instruction::Stmia {
                rn: Register::R0,
                register_list: RegisterList::new(0x38),
            }),
            i
        );
        assert_eq!("STMIA R0!,{R3,R4,R5}", format!("{}", i.unwrap()));
    }

    #[test]
    fn stmia_operation() {
        let mut cpu = Armv6M::new(16, 0x0);
        let mut ram = [0; 8];
        cpu.regs[0] = 4;
        cpu.regs[1] = 2;
        cpu.regs[3] = 3;
        cpu.regs[5] = 4;
        cpu.regs[7] = 5;
        cpu.execute(
            Instruction::Stmia {
                // R1, R3, R5, R7
                register_list: RegisterList::new(0xAA),
                rn: Register::R0,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(4 + 16, cpu.regs[0]);
        assert_eq!([0, 2, 3, 4, 5, 0, 0, 0], ram);
    }

    #[test]
    fn add_sp_t1_instruction() {
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
    fn add_sp_t1_operation() {
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
    fn ldr_sp_imm_instruction() {
        let i = Armv6M::decode(0x9902);
        assert_eq!(
            Ok(Instruction::LdrSpImm {
                rt: Register::R1,
                imm32: 8
            }),
            i
        );
        assert_eq!("LDR R1,[SP,#8]", format!("{}", i.unwrap()));
    }

    #[test]
    fn ldr_sp_imm_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0, 0, 0, 0, 0, 0, 0x12345678, 0];
        cpu.execute(
            Instruction::LdrSpImm {
                rt: Register::R1,
                imm32: 8,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x12345678, cpu.regs[1]);
    }

    #[test]
    fn str_sp_imm_instruction() {
        let i = Armv6M::decode(0x9502);
        assert_eq!(
            Ok(Instruction::StrSpImm {
                rt: Register::R5,
                imm32: 8
            }),
            i
        );
        assert_eq!("STR R5,[SP,#8]", format!("{}", i.unwrap()));
    }

    #[test]
    fn str_sp_imm_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [15; 8];
        cpu.regs[1] = 0x12345678;
        cpu.execute(
            Instruction::StrSpImm {
                rt: Register::R1,
                imm32: 8,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(ram[((sp as usize) / 4) + 2], 0x12345678);
    }
    // =======================================================================
    // Arithmetic Instructions
    // =======================================================================
    #[test]
    fn adds_imm_instruction() {
        let i = Armv6M::decode(0x1d00);
        assert_eq!(
            Ok(Instruction::AddsImm {
                rd: Register::R0,
                rn: Register::R0,
                imm3: 4
            }),
            i
        );
        assert_eq!("ADDS R0,R0,#4", format!("{}", i.unwrap()));
    }

    #[test]
    fn adds_imm_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 4;
        cpu.regs[1] = 0x12345678;
        cpu.execute(
            Instruction::AddsImm {
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
    fn adds_reg_instruction() {
        let i = Armv6M::decode(0x1840);
        assert_eq!(
            Ok(Instruction::AddsReg {
                rd: Register::R0,
                rn: Register::R0,
                rm: Register::R1,
            }),
            i
        );
        assert_eq!("ADDS R0,R0,R1", format!("{}", i.unwrap()));
    }
    #[test]
    fn adds_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 4;
        cpu.regs[1] = 0x12345678;
        cpu.regs[2] = 1;
        cpu.execute(
            Instruction::AddsReg {
                rd: Register::R0,
                rn: Register::R1,
                rm: Register::R2,
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
    fn adcs_reg_instruction() {
        let i = Armv6M::decode(0x4148);
        assert_eq!(
            Ok(Instruction::AdcsReg {
                rdn: Register::R0,
                rm: Register::R1,
            }),
            i
        );
        assert_eq!("ADCS R0,R1", format!("{}", i.unwrap()));
    }
    #[test]
    fn adcs_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 4;
        cpu.regs[2] = 1;
        cpu.set_c(true);
        cpu.execute(
            Instruction::AdcsReg {
                rdn: Register::R0,
                rm: Register::R2,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(6, cpu.regs[0]);
        assert!(!cpu.is_n());
        assert!(!cpu.is_z());
        assert!(!cpu.is_c());
        assert!(!cpu.is_v());
    }
    #[test]
    fn subs_imm_instruction() {
        let i = Armv6M::decode(0x1f00);
        assert_eq!(
            Ok(Instruction::SubsImm {
                rd: Register::R0,
                rn: Register::R0,
                imm3: 4
            }),
            i
        );
        assert_eq!("SUBS R0,R0,#4", format!("{}", i.unwrap()));
    }
    #[test]
    fn subs_imm_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 4;
        cpu.regs[1] = 0x12345678;
        cpu.execute(
            Instruction::SubsImm {
                rd: Register::R0,
                rn: Register::R1,
                imm3: 1,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x12345677, cpu.regs[0]);
        assert!(!cpu.is_n());
        assert!(!cpu.is_z());
        assert!(cpu.is_c());
        assert!(!cpu.is_v());
    }
    #[test]
    fn subs_reg_instruction() {
        let i = Armv6M::decode(0x1A40);
        assert_eq!(
            Ok(Instruction::SubsReg {
                rd: Register::R0,
                rn: Register::R0,
                rm: Register::R1,
            }),
            i
        );
        assert_eq!("SUBS R0,R0,R1", format!("{}", i.unwrap()));
    }

    #[test]
    fn subs_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 4;
        cpu.regs[1] = 0x12345678;
        cpu.regs[2] = 1;
        cpu.execute(
            Instruction::SubsReg {
                rd: Register::R0,
                rn: Register::R1,
                rm: Register::R2,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x12345677, cpu.regs[0]);
        assert!(!cpu.is_n());
        assert!(!cpu.is_z());
        assert!(cpu.is_c());
        assert!(!cpu.is_v());
    }
    #[test]
    fn rsb_imm_instruction() {
        let i = Armv6M::decode(0x4241);
        assert_eq!(
            Ok(Instruction::RsbImm {
                rd: Register::R1,
                rn: Register::R0,
            }),
            i
        );
        assert_eq!("RSBS R1,R0,#0", format!("{}", i.unwrap()));
    }
    #[test]
    fn rsb_imm_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 4;
        cpu.execute(
            Instruction::RsbImm {
                rd: Register::R1,
                rn: Register::R0,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0xFFFF_FFFC, cpu.regs[1]);
        assert!(cpu.is_n());
        assert!(!cpu.is_z());
        assert!(!cpu.is_c());
        assert!(!cpu.is_v());
    }
    #[test]
    fn lsls_imm_instruction() {
        let i = Armv6M::decode(0x0081);
        // lsls	r1, r0, #2
        assert_eq!(
            Ok(Instruction::LslsImm {
                rd: Register::R1,
                rm: Register::R0,
                imm5: 2
            }),
            i
        );
        assert_eq!("LSLS R1,R0,#2", format!("{}", i.unwrap()));
    }
    #[test]
    fn lsls_imm_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram: [u32; 8] = [0; 8];
        cpu.regs[1] = 0x12345678;
        cpu.execute(
            Instruction::LslsImm {
                rd: Register::R0,
                rm: Register::R1,
                imm5: 4,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x23456780, cpu.regs[0]);
        assert!(!cpu.is_n());
        assert!(!cpu.is_z());
        assert!(cpu.is_c());
        assert!(!cpu.is_v());
    }
    #[test]
    fn lsls_reg_instruction() {
        let i = Armv6M::decode(0x409c);
        // lsls	r1, r0, #2
        assert_eq!(
            Ok(Instruction::LslsReg {
                rdn: Register::R4,
                rm: Register::R3,
            }),
            i
        );
        assert_eq!("LSLS R4,R3", format!("{}", i.unwrap()));
    }
    #[test]
    fn lsls_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram: [u32; 8] = [0; 8];
        cpu.regs[4] = 0x12345678;
        cpu.regs[3] = 4;
        cpu.execute(
            Instruction::LslsReg {
                rdn: Register::R4,
                rm: Register::R3,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x23456780, cpu.regs[4]);
        assert!(!cpu.is_n());
        assert!(!cpu.is_z());
        assert!(cpu.is_c());
        assert!(!cpu.is_v());
    }
    #[test]
    fn lsrs_imm_instruction() {
        let i = Armv6M::decode(0x0881);
        assert_eq!(
            Ok(Instruction::LsrsImm {
                rd: Register::R1,
                rm: Register::R0,
                imm5: 2
            }),
            i
        );
        assert_eq!("LSRS R1,R0,#2", format!("{}", i.unwrap()));
    }

    #[test]
    fn lsrs_imm_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram: [u32; 8] = [0; 8];
        cpu.regs[1] = 0x12345678;
        cpu.execute(
            Instruction::LsrsImm {
                rd: Register::R0,
                rm: Register::R1,
                imm5: 4,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x01234567, cpu.regs[0]);
        assert!(!cpu.is_n());
        assert!(!cpu.is_z());
        assert!(cpu.is_c());
        assert!(!cpu.is_v());
    }
    #[test]
    fn lsrs_reg_instruction() {
        let i = Armv6M::decode(0x40dc);
        assert_eq!(
            Ok(Instruction::LsrsReg {
                rdn: Register::R4,
                rm: Register::R3,
            }),
            i
        );
        assert_eq!("LSRS R4,R3", format!("{}", i.unwrap()));
    }

    #[test]
    fn lsrs_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram: [u32; 8] = [0; 8];
        cpu.regs[4] = 0x12345678;
        cpu.regs[3] = 4;
        cpu.execute(
            Instruction::LsrsReg {
                rdn: Register::R4,
                rm: Register::R3,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x01234567, cpu.regs[4]);
        assert!(!cpu.is_n());
        assert!(!cpu.is_z());
        assert!(cpu.is_c());
        assert!(!cpu.is_v());
    }
    // =======================================================================
    // Logical Instructions
    // =======================================================================
    #[test]
    fn cmp_reg_instruction() {
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
    fn cmp_reg_operation() {
        static TEST_VALUES: &[i32] = &[
            i32::MIN,
            i32::MIN + 1,
            i32::MIN + 2,
            i32::MIN / 2,
            -2,
            -1,
            0,
            1,
            2,
            i32::MAX / 2,
            i32::MAX - 1,
            i32::MAX,
        ];
        for x in TEST_VALUES.iter().copied() {
            for y in TEST_VALUES.iter().copied() {
                let ux = x as u32;
                let uy = y as u32;
                let sp = 16;
                let mut cpu = Armv6M::new(sp, 0);
                let mut ram = [0; 8];
                cpu.regs[0] = ux;
                cpu.regs[1] = uy;
                cpu.execute(
                    Instruction::CmpReg {
                        rn: Register::R0,
                        rm: Register::R1,
                    },
                    &mut ram,
                )
                .unwrap();
                // C=1 & Z=0
                assert_eq!(
                    ux > uy,
                    cpu.check_condition(Condition::Hi),
                    "{:x} > {:x} (c={} n={} v={} z={})",
                    ux,
                    uy,
                    cpu.is_c(),
                    cpu.is_n(),
                    cpu.is_v(),
                    cpu.is_z()
                );
                // C=0 | Z=1
                assert_eq!(
                    ux <= uy,
                    cpu.check_condition(Condition::Ls),
                    "{:x} <= {:x} (c={} n={} v={} z={})",
                    ux,
                    uy,
                    cpu.is_c(),
                    cpu.is_n(),
                    cpu.is_v(),
                    cpu.is_z()
                );
                // Z=0 & N=V
                assert_eq!(
                    x > y as i32,
                    cpu.check_condition(Condition::Gt),
                    "{:x} > {:x} (c={} n={} v={} z={})",
                    x,
                    y,
                    cpu.is_c(),
                    cpu.is_n(),
                    cpu.is_v(),
                    cpu.is_z()
                );
                // Z=0 & N=V
                assert_eq!(
                    x >= y as i32,
                    cpu.check_condition(Condition::Ge),
                    "{:x} >= {:x} (c={} n={} v={} z={})",
                    x,
                    y,
                    cpu.is_c(),
                    cpu.is_n(),
                    cpu.is_v(),
                    cpu.is_z()
                );
                // N!=V
                assert_eq!(
                    x < y as i32,
                    cpu.check_condition(Condition::Lt),
                    "{:x} < {:x} (c={} n={} v={} z={})",
                    x,
                    y,
                    cpu.is_c(),
                    cpu.is_n(),
                    cpu.is_v(),
                    cpu.is_z()
                );
                // Z=1 | N!=V
                assert_eq!(
                    x <= y as i32,
                    cpu.check_condition(Condition::Le),
                    "{:x} <= {:x} (c={} n={} v={} z={})",
                    x,
                    y,
                    cpu.is_c(),
                    cpu.is_n(),
                    cpu.is_v(),
                    cpu.is_z()
                );
            }
        }
    }
    #[test]
    fn cmp_imm_instruction() {
        let i = Armv6M::decode(0x2805);
        assert_eq!(
            Ok(Instruction::CmpImm {
                rn: Register::R0,
                imm32: 5,
            }),
            i
        );
        assert_eq!("CMP R0,#5", format!("{}", i.unwrap()));
    }
    #[test]
    fn cmp_imm_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 100;
        cpu.execute(
            Instruction::CmpImm {
                rn: Register::R0,
                // Must be zero-extended, not sign-extended
                imm32: 50,
            },
            &mut ram,
        )
        .unwrap();
        // C=1 & Z=0
        assert_eq!(100 > 50, cpu.check_condition(Condition::Hi));
        // C=0 | Z=1
        assert_eq!(100 <= 50, cpu.check_condition(Condition::Ls));
        // Z=0 & N=V
        assert_eq!(100 > 50, cpu.check_condition(Condition::Gt));
        // Z=0 & N=V
        assert_eq!(100 >= 50, cpu.check_condition(Condition::Ge));
        // N!=V
        assert_eq!(100 < 50, cpu.check_condition(Condition::Lt));
        // Z=1 | N!=V
        assert_eq!(100 <= 50, cpu.check_condition(Condition::Le));
    }
    #[test]
    fn ands_reg_instruction() {
        let i = Armv6M::decode(0x401e);
        assert_eq!(
            Ok(Instruction::AndsReg {
                rdn: Register::R6,
                rm: Register::R3,
            }),
            i
        );
        assert_eq!("ANDS R6,R3", format!("{}", i.unwrap()));
    }
    #[test]
    fn ands_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[6] = 0x12345678;
        cpu.regs[3] = 0xFF0000FF;
        cpu.execute(
            Instruction::AndsReg {
                rdn: Register::R6,
                rm: Register::R3,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(cpu.regs[6], 0x12000078);
    }
    #[test]
    fn orrs_reg_instruction() {
        let i = Armv6M::decode(0x431e);
        assert_eq!(
            Ok(Instruction::OrrsReg {
                rdn: Register::R6,
                rm: Register::R3,
            }),
            i
        );
        assert_eq!("ORRS R6,R3", format!("{}", i.unwrap()));
    }
    #[test]
    fn orrs_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[6] = 0x12345678;
        cpu.regs[3] = 0xFF0000FF;
        cpu.execute(
            Instruction::OrrsReg {
                rdn: Register::R6,
                rm: Register::R3,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(cpu.regs[6], 0xFF3456FF);
    }
    // =======================================================================
    // Other Instructions
    // =======================================================================
    #[test]
    fn bkpt_instruction() {
        let i = Armv6M::decode(0xbecc);
        assert_eq!(Ok(Instruction::Breakpoint { imm8: 0xCC }), i);
        assert_eq!("BKPT 0xcc", format!("{}", i.unwrap()));
    }

    #[test]
    fn bkpt_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.execute(Instruction::Breakpoint { imm8: 0xCC }, &mut ram)
            .unwrap();
        assert_eq!(Some(0xCC), cpu.breakpoint());
    }

    #[test]
    fn mrs_instruction() {
        let i = Armv6M::decode32(0xf3ef, 0x8010);
        assert_eq!(
            Ok(Instruction::Mrs {
                rd: Register::R0,
                sys_m: Armv6M::SYS_M_PRIMASK
            }),
            i
        );
        assert_eq!("MRS R0,16", format!("{}", i.unwrap()));
    }

    #[test]
    fn mrs_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 0x12345678;
        cpu.execute(
            Instruction::Mrs {
                rd: Register::R0,
                sys_m: Armv6M::SYS_M_PRIMASK,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x12345678, cpu.primask);
    }

    #[test]
    fn msr_instruction() {
        let i = Armv6M::decode32(0xf380, 0x8810);
        assert_eq!(
            Ok(Instruction::Msr {
                rn: Register::R0,
                sys_m: Armv6M::SYS_M_PRIMASK
            }),
            i
        );
        assert_eq!("MSR R0,16", format!("{}", i.unwrap()));
    }

    #[test]
    fn msr_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.primask = 0x12345678;
        cpu.execute(
            Instruction::Msr {
                rn: Register::R0,
                sys_m: Armv6M::SYS_M_PRIMASK,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x12345678, cpu.regs[0]);
    }

    #[test]
    fn cpsid_instruction() {
        let i = Armv6M::decode(0xb672);
        assert_eq!(Ok(Instruction::Cps { im: true }), i);
        assert_eq!("CPSID i", format!("{}", i.unwrap()));
    }

    #[test]
    fn cpsie_instruction() {
        let i = Armv6M::decode(0xb662);
        assert_eq!(Ok(Instruction::Cps { im: false }), i);
        assert_eq!("CPSIE i", format!("{}", i.unwrap()));
    }
}

// End of file
