//! An ARMv6-M processor simulator.
//!
//! Designed to run on `no_std` systems (although at the moment it's full of
//! println! calls).

#![deny(missing_docs)]
#![deny(missing_debug_implementations)]

use tracing::debug;

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
///
/// TODO:
/// * nop
/// * sxtb
/// * tst
/// * udf
/// * wfi
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Instruction {
    // =======================================================================
    // Branch Instructions
    // =======================================================================
    /// Branch.
    ///
    /// Causes a branch to a target address. The instruction set cannot be
    /// changed by this function.
    ///
    /// `B <label>`, where `<label>` converts to a PC-relative offset.
    Branch {
        /// The PC-relative signed offset.
        ///
        /// A value of -4 means jump to same location, because PC has been
        /// incremented twice before this is executed.
        imm11x2: i32,
    },
    /// Conditional Branch.
    ///
    /// Causes a branch to a target address if a condition is met. The
    /// instruction set cannot be changed by this function.
    ///
    /// `B.xx <label>`, where `<label>` converts to a PC-relative offset.
    BranchConditional {
        /// The condition code (e.g. only branch if Zero)
        cond: Condition,
        /// The PC-relative signed offset.
        ///
        /// A value of -4 means jump to same location, because PC has been
        /// incremented twice before this is executed.
        imm8x2: i32,
    },
    /// Branch with Link (Immediate).
    ///
    /// Calls a subroutine at a PC-relative address. The instruction set cannot
    /// be changed by this function.
    ///
    /// `BL <label>`, where `<label>` converts to a PC-relative offset.
    BranchLink {
        /// The PC-relative signed offset.
        ///
        /// A value of -4 means jump to same location, because PC has been
        /// incremented twice before this is executed.
        imm24: i32,
    },
    /// Branch and Exchange.
    ///
    /// Causes a branch to an address and instruction set specified by a
    /// register.
    ///
    /// `BX <Rm>`
    BranchExchange {
        /// The register that contains the new PC
        rm: Register,
    },
    /// Branch with Link and Exchange.
    ///
    /// Calls a subroutine at an address and instruction set specified by a
    /// register.
    ///
    /// `BLX <Rm>`
    BranchLinkExchange {
        /// The register that contains the new PC
        rm: Register,
    },
    // =======================================================================
    // Move Instructions
    // =======================================================================
    /// Move and Set (Immediate).
    ///
    /// Writes an immediate value to the destination register. Condition flags
    /// are updated.
    ///
    /// `MOVS <Rd>,#<imm8>`
    MovsImm {
        /// The destination register
        rd: Register,
        /// The 8-bit signed immediate value
        imm8: u8,
    },
    /// Move (Register).
    ///
    /// Copies a value from a register to the destination register. Condition
    /// flags are not updated.
    ///
    /// `MOV <Rd>,<Rm>`
    MovReg {
        /// The destination register to write to
        rd: Register,
        /// The register to read from.
        rm: Register,
    },
    // =======================================================================
    // Store Instructions
    // =======================================================================
    /// Store Register (Immediate).
    ///
    /// Calculates an address from a base address register and an immediate
    /// offset, and stores a value from a register to that address.
    ///
    /// `STR <Rt>,[<Rn>{,#<imm5>}]`
    StrImm {
        /// The register that contains the value to store
        rt: Register,
        /// The register that contains the base address to store at
        rn: Register,
        /// What offset to apply to the base address
        imm5x4: u8,
    },
    /// Store Register (Register)
    ///
    /// Calculates an address from a base address register and a value in an
    /// offset register, and stores a word from a register to that address.
    ///
    /// `STR <Rt>,[<Rn>,<Rm>]`
    StrReg {
        /// The register that contains the value to store
        rt: Register,
        /// The register that contains the address to store at
        rn: Register,
        /// The register that contains the offset to apply
        rm: Register,
    },
    /// Store Register Halfword (Immediate).
    ///
    /// Calculates an address from a base address register and an immediate
    /// offset, and stores a halfword from a register to that address.
    ///
    /// `STRH <Rt>,[<Rn>{,#<imm5>}]`
    StrhImm {
        /// The register that contains the value to store
        rt: Register,
        /// The register that contains the address to store at
        rn: Register,
        /// What offset to apply to the storage address
        imm5x2: u8,
    },
    /// Store Register Halfword (Register)
    ///
    /// Calculates an address from a base address register and a value in an
    /// offset register, and stores a halfword from a register to that address.
    ///
    /// `STRH <Rt>,[<Rn>,<Rm>]`
    StrhReg {
        /// The register that contains the value to store
        rt: Register,
        /// The register that contains the address to store at
        rn: Register,
        /// The register that contains the offset to apply
        rm: Register,
    },
    /// Store Register Byte (Immediate).
    ///
    /// Calculates an address from a base address register and an immediate
    /// offset, and stores a byte from a register to that address.
    ///
    /// `STRB <Rt>,[<Rn>{,#<imm5>}]`
    StrbImm {
        /// The register that contains the value to store
        rt: Register,
        /// The register that contains the address to store at
        rn: Register,
        /// What offset to apply to the storage address
        imm5: u8,
    },
    /// Store Register Byte (Register)
    ///
    /// Calculates an address from a base address register and a value in an
    /// offset register, and stores a byte from a register to that address.
    ///
    /// `STRB <Rt>,[<Rn>,<Rm>]`
    StrbReg {
        /// The register that contains the value to store
        rt: Register,
        /// The register that contains the address to store at
        rn: Register,
        /// The register that contains the offset to apply
        rm: Register,
    },
    // =======================================================================
    // Load Instructions
    // =======================================================================
    /// Load Register (literal)
    ///
    /// Calculates a PC-relative address using an immediate offset, and loads
    /// a word from memory into a register.
    ///
    /// `LDR <Rt>,[PC,#<imm8>]`
    LdrLiteral {
        /// The register to load into
        rt: Register,
        /// Offset to add to PC. Must be a multiple of 4 in the range `0..=1020`.
        imm8x4: u16,
    },
    /// Load Register (Immediate).
    ///
    /// Calculates an address using a base register and an immediate offset, and
    /// loads a word from memory into a register.
    ///
    /// `LDR <Rt>,[<Rn>{,#<imm5>}]`
    LdrImm {
        /// The register to load into
        rt: Register,
        /// The register that contains the base address to read from
        rn: Register,
        /// What offset to apply to the base address
        imm5x4: u8,
    },
    /// Load Register (Register)
    ///
    /// Calculate an address from a base register value and an offset register
    /// value and load a word from memory at that address into a destination
    /// register.
    ///
    /// LDR <Rt>,[<Rn>, <Rm>]
    LdrReg {
        /// The register to load into
        rt: Register,
        /// The register that contains the base address to read from
        rn: Register,
        /// The register that contains the offset to apply
        rm: Register,
    },
    /// Load Register Halfword (Immediate).
    ///
    /// Calculates an address using a base register and an immediate offset, and
    /// loads a halfword from memory into a register.
    ///
    /// `LDRH <Rt>,[<Rn>{,#<imm5>}]`
    LdrhImm {
        /// The register to load into
        rt: Register,
        /// The register that contains the base address to read from
        rn: Register,
        /// What offset to apply to the base address
        imm5x2: u8,
    },
    /// Load Register Halfword (Register)
    ///
    /// Calculate an address from a base register value and an offset register
    /// value and load a halfword from memory at that address into a destination
    /// register.
    ///
    /// LDRH <Rt>,[<Rn>, <Rm}]
    LdrhReg {
        /// The register to load into
        rt: Register,
        /// The register that contains the base address to read from
        rn: Register,
        /// The register that contains the offset to apply
        rm: Register,
    },
    /// Load Register Byte (Immediate).
    ///
    /// Calculates an address using a base register and an immediate offset, and
    /// loads a byte from memory into a register.
    ///
    /// `LDRB <Rt>,[<Rn>{,#<imm5>}]`
    LdrbImm {
        /// The register to load into
        rt: Register,
        /// The register that contains the base address to read from
        rn: Register,
        /// What offset to apply to the base address
        imm5: u8,
    },
    /// Load Register Byte (Register)
    ///
    /// Calculate an address from a base register value and an offset register
    /// value and load a byte from memory at that address into a destination
    /// register.
    ///
    /// LDRB <Rt>,[<Rn>, <Rm}]
    LdrbReg {
        /// The register to load into
        rt: Register,
        /// The register that contains the base address to read from
        rn: Register,
        /// The register that contains the offset to apply
        rm: Register,
    },
    /// Load Register Signed Byte (Register)
    ///
    /// Calculate an address from a base register value and an offset register
    /// value and load a signed byte from memory at that address into a
    /// destination register.
    ///
    /// LDRSB <Rt>,[<Rn>, <Rm}]
    LdrsbReg {
        /// The register to load into
        rt: Register,
        /// The register that contains the base address to read from
        rn: Register,
        /// The register that contains the offset to apply
        rm: Register,
    },
    // =======================================================================
    // Stack Instructions
    // =======================================================================`
    /// Push Multiple Registers.
    ///
    /// Pushes multiple registers to the stack.
    ///
    /// `PUSH {<register list>}`
    Push {
        /// A bitmask of registers (R7 to R0) to push
        register_list: RegisterList,
        /// Also push LR
        m: bool,
    },
    /// Pop Multiple Registers
    ///
    /// Pops multiple registers from the stack.
    ///
    /// `POP {<register list>}`
    Pop {
        /// A bitmask of registers (R7 to R0) to pop
        register_list: RegisterList,
        /// Also pop LR
        p: bool,
    },
    /// Load Multiple, Increment After.
    ///
    /// Loads multiple registers from consecutive memory locations using an
    /// address from a base address register. The base address register is
    /// updated, if it is not part of the register list being written to.
    ///
    /// `LDMIA <Rn>!,{<register list>}`
    Ldmia {
        /// Base address to load from
        rn: Register,
        /// Register list
        register_list: RegisterList,
    },
    /// Store Multiple, Increment After.
    ///
    /// Stores multiple registers to consecutive memory locations using an
    /// address from a base register. The base address register is
    /// updated, if it is not part of the register list being written to.
    ///
    ///
    /// `STMIA <Rn>!,{<register list>}`
    Stmia {
        /// Base address to save to
        rn: Register,
        /// Register list
        register_list: RegisterList,
    },
    /// Add, Stack Pointer plus Immediate, to Register.
    ///
    /// Adds an immediate value to the value in the Stack Pointer and writes the
    /// result to the destination register.
    ///
    /// `ADD <Rd>,SP,#<imm8>`
    AddSpImmReg {
        /// The destination register
        rd: Register,
        /// How much to add to the stack pointer. Must be a multiple of 4 in
        /// the range `0..=1020`.
        imm8x4: u16,
    },
    /// Add, Stack Pointer plus Immediate.
    ///
    /// Increments the Stack Pointer by an immediate value.
    ///
    /// `ADD SP,SP,#<imm7>`
    AddSpImm {
        /// How much to add to the stack pointer. Must be a multiple of 4 in
        /// the range `0..=508`.
        imm7x4: u32,
    },
    /// Add, Stack Pointer plus Register.
    ///
    /// Increments the Stack Pointer by a register value.
    ///
    /// `ADD SP,<Rm>`
    AddSpReg {
        /// The register containing the value to add to SP
        rm: Register,
    },
    /// Add, Register plus Stack Pointer.
    ///
    /// Increments a register value by the Stack Pointer value.
    ///
    /// `ADD <Rdm>,SP`
    AddRegSp {
        /// The register to be incremented by the value in SP
        rdm: Register,
    },
    /// Subtract, Stack Pointer minus Immediate.
    ///
    /// Decrements the Stack Pointer by an immediate value.
    ///
    /// `SUB SP,SP,#<imm7>`
    SubSpImm {
        /// How much to subtract from the stack pointer. Must be a multiple of 4
        /// in the range `0..=508`.
        imm7x4: u32,
    },
    /// Load Register, Stack Pointer plus Immediate.
    ///
    /// Calculates an address from the Stack Pointer value plus an immediate
    /// value, and loads a word from memory at that address into a register.
    ///
    /// `LDR <Rt>,[SP,#<imm8>]`
    LdrSpImm {
        /// The register to store the loaded value in
        rt: Register,
        /// The offset to apply to the Stack Pointer value. Must be a multiple
        /// of 4 in the range `0..=1020`.
        imm8x4: u16,
    },
    /// Store Register, Stack Pointer plus Immediate.
    ///
    /// Calculates an address from the Stack Pointer value plus an immediate
    /// value, and stores a word from a register to memory at that address.
    ///
    /// `STR <Rt>,[SP,#<imm8>]`
    StrSpImm {
        /// The register to contains the value to store
        rt: Register,
        /// The offset to apply to the stack pointer. Must be a multiple of 4 in
        /// the range `0..=1020`.
        imm8x4: u16,
    },
    // =======================================================================
    // Arithmetic Instructions
    // =======================================================================
    /// Add and Set (Immediate)
    ///
    /// Add an immediate value to a register value and write the result to a
    /// destination register. Condition flags are updated.
    ///
    /// `ADDS <Rd>,<Rn>,#<imm3>`
    AddsImm3 {
        /// The destination register
        rd: Register,
        /// The register that contains the first operand.
        rn: Register,
        /// The second operand
        imm3: u8,
    },
    /// Add and Set (Immediate, same register)
    ///
    /// Add an immediate value to a register value and write the result back to
    /// the same register. One less register to specify means more bits for the
    /// immediate value. Condition flags are updated.
    ///
    /// `ADDS <Rdn>,#<imm8>`
    AddsImm8 {
        /// The register that contains the first operand, and the destination
        rdn: Register,
        /// How much to add to <Rdn>
        imm8: u8,
    },
    /// Add and Set (Register).
    ///
    /// Adds two register values together and write the result to a third
    /// register. Condition flags are updated.
    ///
    /// `ADDS <Rd>,<Rn>,<Rm>`
    AddsReg {
        /// The destination register
        rd: Register,
        /// The register that contains the first operand.
        rn: Register,
        /// The register that contains the second operand
        rm: Register,
    },
    /// Add with Carry, and Set (Register)
    ///
    /// Adds a register value, and the carry bit, to a register. Condition flags
    /// are updated.
    ///
    /// `ADCS <Rdn>,<Rm>`
    AdcsReg {
        /// The register that contains the first operand, and the destination
        rdn: Register,
        /// The register that contains the second operand
        rm: Register,
    },
    /// Address to Register
    ///
    /// Add an immediate value to the Program Counter value and write the result
    /// to the destination register.
    ///
    /// `ADR <Rd>,<label>`
    Adr {
        /// The destination register
        rd: Register,
        /// The offset to apply to the PC. Must be a multiple of 4 in the range
        /// `0..=1020`.
        imm8x4: u16,
    },
    /// Subtract and Set (Immediate)
    ///
    /// Subtract an immediate value from a register value and write the result to a
    /// destination register. Condition flags are updated.
    ///
    /// `SUBS <Rd>,<Rn>,#<imm3>`
    SubsImm3 {
        /// The destination register
        rd: Register,
        /// The register that contains the first operand.
        rn: Register,
        /// The immediate value for the second operand.
        imm3: u8,
    },
    /// Subtract and Set (Immediate, same register)
    ///
    /// Subtract an immediate value from a register value and write the result
    /// back to the same register. One less register to specify means more bits
    /// for the immediate value. Condition flags are updated.
    ///
    /// `SUBS <Rdn>,#<imm8>`
    SubsImm8 {
        /// The register that contains the first operand, and the destination
        rdn: Register,
        /// The immediate value for the second operand.
        imm8: u8,
    },
    /// Subtract and Set (Register).
    ///
    /// Subtract one register value from another, and write the result to a
    /// third register. Condition flags are updated.
    ///
    /// `SUBS <Rd>,<Rn>,<Rm>`
    SubsReg {
        /// The destination register
        rd: Register,
        /// The register that contains the first operand.
        rn: Register,
        /// The register that contains the second operand
        rm: Register,
    },
    /// Subtract with Carry, and Set (Register)
    ///
    /// Subtracts a register value and the value of `!CarryFlag` from a register
    /// value, and writes the result to the destination register. Condition
    /// flags are updated.
    ///
    /// `SBCS <Rdn>,<Rm>`
    SbcsReg {
        /// The register that contains the first operand, and the destination
        rdn: Register,
        /// The register that contains the second operand
        rm: Register,
    },
    /// Reverse Subtract (Immediate).
    ///
    /// Subtracts a register value from an immediate value and writes the result
    /// to a register.
    ///
    /// The immediate value can only be zero on ARMv6-M, so this ends up causing
    /// the value in the register to be negatated.
    ///
    /// `RSBS <Rd>,<Rn>,#0`
    RsbImm {
        /// The destination register
        rd: Register,
        /// The register that contains the value to be negated.
        rn: Register,
    },
    /// Logical Shift Left with Set (Immediate).
    ///
    /// Shift register left by a number of bits given in an immediate
    /// value, shifting in zeros. Condition flags are updated.
    ///
    /// `LSLS <Rd>,<Rm>,#<imm>`
    LslsImm {
        /// The destination register
        rd: Register,
        /// The register that contains the first operand.
        rm: Register,
        /// The second operand
        imm5: u8,
    },
    /// Logical Shift Left with Set (Register).
    ///
    /// Shift register left by a number of bits given in another register,
    /// shifting in zeros. Condition flags are updated.
    ///
    /// `LSLS <Rdn>,<Rm>
    LslsReg {
        /// The register that contains the first operand, and the destination
        rdn: Register,
        /// The register that contains the second operand
        rm: Register,
    },
    /// Logical Shift Right with Set (Immediate) (aka Move and Set (Register))
    ///
    /// Shift register right by a number of bits given in an immediate
    /// value, shifting in zeros. Condition flags are updated.
    ///
    /// If the immediate value is #0, this is a `MOVS` instruction.
    ///
    /// `LSRS <Rx>,<Rx>,#<imm>` or `MOVS <Rd>,<Rm>`
    LsrsImm {
        /// The destination register
        rd: Register,
        /// The register that contains the first operand.
        rm: Register,
        /// The second operand
        imm5: u8,
    },
    /// Logical Shift Right with Set (Register).
    ///
    /// Shift register right by a number of bits given in another register,
    /// shifting in zeros. Condition flags are updated.
    ///
    /// `LSRS <Rdn>,<Rm>
    LsrsReg {
        /// The register that contains the first operand, and the destination
        rdn: Register,
        /// The register that contains the second operand
        rm: Register,
    },
    /// Arithmetic Shift Right with Set (Immediate)
    ///
    /// Shift a register value right by some immediate number of bits, shifting
    /// in copies of the sign bit, and write the result to the destination
    /// register.
    ///
    /// `ASRS <Rd>,<Rm>,#imm5
    AsrsImm {
        /// The destination register
        rd: Register,
        /// The register that contains the first operand.
        rm: Register,
        /// The second operand
        imm5: u8,
    },
    /// Multiply with Set (Register).
    ///
    /// Multiply two register values. Condition flags are updated.
    ///
    /// `MULS <Rdn>,<Rm>
    Muls {
        /// The register that contains the first operand, and the destination
        rdn: Register,
        /// The register that contains the second operand
        rm: Register,
    },
    /// Unsigned Extend Halfword.
    ///
    /// Read 16 bits from one register, zero-extend to 32-bits, and write to the
    /// destination register.
    ///
    /// `UXTH <Rd>,<Rm>`
    Uxth {
        /// The destination register
        rd: Register,
        /// The register to read the 16-bit value from
        rm: Register,
    },
    /// Unsigned Extend Byte.
    ///
    /// Read 8 bits from one register, zero-extend to 32-bits, and write to the
    /// destination register.
    ///
    /// `UXTB <Rd>,<Rm>`
    Uxtb {
        /// The destination register
        rd: Register,
        /// The register to read the 8-bit value from
        rm: Register,
    },
    // =======================================================================
    // Logical Instructions
    // =======================================================================
    /// Compare (Register)
    ///
    /// Subtracts one register value from another, and updates the condition
    /// flags but discard the numeric result.
    ///
    /// `CMP <Rn>,<Rm>`
    CmpReg {
        /// The register that contains the first operand.
        rn: Register,
        /// The register that contains the second operand
        rm: Register,
    },
    /// Compare (Immediate)
    ///
    /// Subtracts an immediate value from a register value, and updates the
    /// condition flags but discard the numeric result.
    ///
    /// `CMP <Rn>,#<imm8>`
    CmpImm {
        /// The register that contains the first operand.
        rn: Register,
        /// The right hand immediate value to compare
        imm8: u8,
    },
    /// Bitwise AND, and Set (Register)
    ///
    /// Computes the bitwise AND of two register values. Condition flags are
    /// updated.
    ///
    /// `ANDS <Rdn>,<Rm>`
    AndsReg {
        /// The register that contains the first operand, and the destination
        rdn: Register,
        /// The register that contains the second operand
        rm: Register,
    },
    /// Bitwise OR, and Set (Register)
    ///
    /// Computes the bitwise OR of two register values. Condition flags are
    /// updated.
    ///
    /// `ORRS <Rdn>,<Rm>`
    OrrsReg {
        /// The register that contains the first operand, and the destination
        rdn: Register,
        /// The register that contains the second operand
        rm: Register,
    },
    /// Bitwise XOR, and Set (Register)
    ///
    /// Computes the bitwise OR of two register values. Condition flags are
    /// updated.
    ///
    /// `EORS <Rdn>,<Rm>`
    EorsReg {
        /// The register that contains the first operand, and the destination
        rdn: Register,
        /// The register that contains the second operand
        rm: Register,
    },
    /// Bit Clear, and Set (Register)
    ///
    /// Computes the bitwise AND of a register value, and the complement of a
    /// register value. This effectively clears any bits in the first value that
    /// are set in the second. Condition flags are updated.
    ///
    /// `BICS <Rdn>,<Rm>`
    BicsReg {
        /// The register that contains the first operand, and the destination
        rdn: Register,
        /// The register that contains the second operand
        rm: Register,
    },
    /// Bitwise Not (Register)
    ///
    /// Writes the bitwise inverse of a register value to the destination register.
    ///
    /// `MVNS <Rd>,<Rm>`
    MvnsReg {
        /// The destination register
        rd: Register,
        /// The register that contains the operand
        rm: Register,
    },
    // =======================================================================
    // Other Instructions
    // =======================================================================
    /// Breakpoint.
    ///
    /// Cause a HardFault, or a debug halt, depending on the presence of debug
    /// support.
    ///
    /// `BKPT <imm8>`
    Breakpoint {
        /// The 8-bit signed immediate value
        imm8: u8,
    },
    /// Move to Register from Special.
    ///
    /// Move a value from a Special Register to a Register.
    ///
    /// `MRS <Rd>,<spec_reg>`
    Mrs {
        /// The destination register
        rd: Register,
        /// Which special register to write to
        sys_m: u8,
    },
    /// Move to Special from Register.
    ///
    /// Move a value from a egister to a Special Register.
    ///
    /// `MSR <Rn>,<spec_reg>`
    Msr {
        /// The register to write to
        rn: Register,
        /// Which special register to read from
        sys_m: u8,
    },
    /// Change Processor State.
    ///
    /// Mask (disable) or Unmask (enable) Interrupts.
    ///
    /// `CPSID i` or `CPSIE i`
    Cps {
        /// `true` to mask (disable) interrupts, `false` to unmask (enable)
        im: bool,
    },
    /// Data Memory Barrier
    ///
    /// Acts as a memory barrier. All explicit memory accesses that appear in
    /// program order before the DMB instruction are observed before any
    /// explicit memory accesses that appear in program order after the DMB
    /// instruction.
    ///
    /// `DMB`
    Dmb,
}

impl std::fmt::Display for Instruction {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Instruction::Branch { imm11x2 } => {
                write!(f, "B {:+#}", *imm11x2 + 4)
            }
            Instruction::BranchConditional { cond, imm8x2 } => {
                write!(f, "B{} {:+#}", cond, *imm8x2 + 4)
            }
            Instruction::BranchLink { imm24 } => {
                write!(f, "BL {:+#}", *imm24 + 4)
            }
            Instruction::BranchExchange { rm } => {
                write!(f, "BX {}", *rm)
            }
            Instruction::BranchLinkExchange { rm } => {
                write!(f, "BLX {}", *rm)
            }
            Instruction::MovsImm { rd, imm8 } => {
                write!(f, "MOVS {rd},#{imm8}")
            }
            Instruction::MovReg { rd, rm } => {
                write!(f, "MOV {rd},{rm}")
            }
            Instruction::StrImm { rt, rn, imm5x4 } => {
                write!(f, "STR {rt},[{rn},#{imm5x4}]")
            }
            Instruction::StrReg { rt, rn, rm } => {
                write!(f, "STR {rt},[{rn},{rm}]")
            }
            Instruction::StrhImm { rt, rn, imm5x2 } => {
                write!(f, "STRH {rt},[{rn},#{imm5x2}]")
            }
            Instruction::StrhReg { rt, rn, rm } => {
                write!(f, "STRH {rt},[{rn},{rm}]")
            }
            Instruction::StrbImm { rt, rn, imm5 } => {
                write!(f, "STRB {rt},[{rn},#{imm5}]")
            }
            Instruction::StrbReg { rt, rn, rm } => {
                write!(f, "STRB {rt},[{rn},{rm}]")
            }
            Instruction::LdrLiteral { rt, imm8x4 } => {
                write!(f, "LDR {rt},[PC,#{imm8x4}]")
            }
            Instruction::LdrImm { rt, rn, imm5x4 } => {
                write!(f, "LDR {rt},[{rn},#{imm5x4}]")
            }
            Instruction::LdrReg { rt, rn, rm } => {
                write!(f, "LDR {rt},[{rn},{rm}]")
            }
            Instruction::LdrhImm { rt, rn, imm5x2 } => {
                write!(f, "LDRH {rt},[{rn},#{imm5x2}]")
            }
            Instruction::LdrhReg { rt, rn, rm } => {
                write!(f, "LDRH {rt},[{rn},{rm}]")
            }
            Instruction::LdrbImm { rt, rn, imm5 } => {
                write!(f, "LDRB {rt},[{rn},#{imm5}]")
            }
            Instruction::LdrbReg { rt, rn, rm } => {
                write!(f, "LDRB {rt},[{rn},{rm}]")
            }
            Instruction::LdrsbReg { rt, rn, rm } => {
                write!(f, "LDRSB {rt},[{rn},{rm}]")
            }
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
            Instruction::AddSpImmReg { rd, imm8x4 } => {
                write!(f, "ADD {rd},SP,#{imm8x4}")
            }
            Instruction::AddSpImm { imm7x4 } => {
                write!(f, "ADD SP,#{imm7x4}",)
            }
            Instruction::AddSpReg { rm } => {
                write!(f, "ADD SP,{rm}")
            }
            Instruction::AddRegSp { rdm } => {
                write!(f, "ADD {rdm},SP")
            }
            Instruction::SubSpImm { imm7x4 } => {
                write!(f, "SUB SP,#{imm7x4}",)
            }
            Instruction::LdrSpImm { rt, imm8x4 } => {
                write!(f, "LDR {rt},[SP,#{imm8x4}]")
            }
            Instruction::StrSpImm { rt, imm8x4 } => {
                write!(f, "STR {rt},[SP,#{imm8x4}]")
            }
            Instruction::AddsImm3 { rd, rn, imm3 } => {
                write!(f, "ADDS {rd},{rn},#{imm3}")
            }
            Instruction::AddsImm8 { rdn, imm8 } => {
                write!(f, "ADDS {rdn},#{imm8}")
            }
            Instruction::AddsReg { rd, rn, rm } => {
                write!(f, "ADDS {rd},{rn},{rm}")
            }
            Instruction::AdcsReg { rdn, rm } => {
                write!(f, "ADCS {rdn},{rm}")
            }
            Instruction::Adr { rd, imm8x4 } => {
                write!(f, "ADR {rd},#{imm8x4}")
            }
            Instruction::SubsImm3 { rd, rn, imm3 } => {
                write!(f, "SUBS {rd},{rn},#{imm3}")
            }
            Instruction::SubsImm8 { rdn, imm8 } => {
                write!(f, "SUBS {rdn},#{imm8}")
            }
            Instruction::SubsReg { rd, rn, rm } => {
                write!(f, "SUBS {rd},{rn},{rm}")
            }
            Instruction::SbcsReg { rdn, rm } => {
                write!(f, "SBCS {rdn},{rm}")
            }
            Instruction::RsbImm { rd, rn } => {
                write!(f, "RSBS {rd},{rn},#0")
            }
            Instruction::LslsImm { rd, rm, imm5 } => {
                if *imm5 == 0 {
                    write!(f, "MOVS {rd},{rm}")
                } else {
                    write!(f, "LSLS {rd},{rm},#{imm5}")
                }
            }
            Instruction::LslsReg { rdn, rm } => {
                write!(f, "LSLS {rdn},{rm}")
            }
            Instruction::LsrsImm { rd, rm, imm5 } => {
                write!(f, "LSRS {rd},{rm},#{imm5}")
            }
            Instruction::LsrsReg { rdn, rm } => {
                write!(f, "LSRS {rdn},{rm}")
            }
            Instruction::AsrsImm { rd, rm, imm5 } => {
                write!(f, "ASRS {rd},{rm},#{imm5}")
            }
            Instruction::Muls { rdn, rm } => {
                write!(f, "MULS {rdn},{rm},{rdn}")
            }
            Instruction::Uxth { rd, rm } => {
                write!(f, "UXTH {rd},{rm}")
            }
            Instruction::Uxtb { rd, rm } => {
                write!(f, "UXTB {rd},{rm}")
            }
            Instruction::CmpReg { rn, rm } => {
                write!(f, "CMP {rn},{rm}")
            }
            Instruction::CmpImm { rn, imm8 } => {
                write!(f, "CMP {rn},#{imm8}")
            }
            Instruction::AndsReg { rdn, rm } => {
                write!(f, "ANDS {rdn},{rm}")
            }
            Instruction::OrrsReg { rdn, rm } => {
                write!(f, "ORRS {rdn},{rm}")
            }
            Instruction::EorsReg { rdn, rm } => {
                write!(f, "EORS {rdn},{rm}")
            }
            Instruction::BicsReg { rdn, rm } => {
                write!(f, "BICS {rdn},{rm}")
            }
            Instruction::MvnsReg { rd, rm } => {
                write!(f, "MVNS {rd},{rm}")
            }
            Instruction::Breakpoint { imm8 } => {
                write!(f, "BKPT 0x{imm8:02x}",)
            }
            Instruction::Mrs { rd, sys_m } => {
                write!(f, "MRS {rd},{sys_m}")
            }
            Instruction::Msr { rn, sys_m } => {
                write!(f, "MSR {rn},{sys_m}")
            }
            Instruction::Cps { im } => {
                write!(f, "CPSI{} i", if *im { "D" } else { "E" })
            }
            Instruction::Dmb => {
                write!(f, "DMB")
            }
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
        self.execute(instruction, memory)?;
        Ok(())
    }

    /// Fetch an instruction from memory and decode it.
    pub fn fetch(&self, memory: &dyn Memory) -> Result<Instruction, Error> {
        let word = memory.load_u16(self.pc)?;
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
                imm8x2: Self::sign_extend_imm8(imm8) << 1,
            })
        } else if (word >> 11) == 0b11110 {
            // This is a 32-bit BL <label>
            Err(Error::WideInstruction)
        } else if (word >> 11) == 0b10010 {
            let imm8 = (word & 0xFF) as u8;
            let rt = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::StrSpImm {
                rt: Register::from(rt),
                imm8x4: u16::from(imm8) << 2,
            })
        } else if (word >> 11) == 0b10011 {
            let imm8 = (word & 0xFF) as u8;
            let rt = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::LdrSpImm {
                rt: Register::from(rt),
                imm8x4: u16::from(imm8) << 2,
            })
        } else if (word >> 11) == 0b00100 {
            let imm8 = (word & 0xFF) as u8;
            let rd = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::MovsImm {
                rd: Register::from(rd),
                imm8,
            })
        } else if (word >> 11) == 0b11100 {
            Ok(Instruction::Branch {
                imm11x2: Self::sign_extend_imm11(word & 0x7FF) << 1,
            })
        } else if (word >> 11) == 0b10101 {
            let imm8 = (word & 0xFF) as u8;
            let rd = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::AddSpImmReg {
                rd: Register::from(rd),
                imm8x4: u16::from(imm8) << 2,
            })
        } else if (word >> 11) == 0b00111 {
            let imm8 = (word & 0xFF) as u8;
            let rdn = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::SubsImm8 {
                rdn: Register::from(rdn),
                imm8,
            })
        } else if (word >> 11) == 0b01001 {
            let imm8 = (word & 0xFF) as u8;
            let rt = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::LdrLiteral {
                rt: Register::from(rt),
                imm8x4: u16::from(imm8) << 2,
            })
        } else if (word >> 11) == 0b01101 {
            let imm5 = ((word >> 6) & 0x1F) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rt = (word & 0b111) as u8;
            Ok(Instruction::LdrImm {
                rt: Register::from(rt),
                rn: Register::from(rn),
                imm5x4: imm5 << 2,
            })
        } else if (word >> 11) == 0b01100 {
            let imm5 = ((word >> 6) & 0x1F) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rt = (word & 0b111) as u8;
            Ok(Instruction::StrImm {
                rt: Register::from(rt),
                rn: Register::from(rn),
                imm5x4: imm5 << 2,
            })
        } else if (word >> 11) == 0b10001 {
            let imm5 = ((word >> 6) & 0x1F) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rt = (word & 0b111) as u8;
            Ok(Instruction::LdrhImm {
                rt: Register::from(rt),
                rn: Register::from(rn),
                imm5x2: imm5 << 1,
            })
        } else if (word >> 11) == 0b10000 {
            let imm5 = ((word >> 6) & 0x1F) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rt = (word & 0b111) as u8;
            Ok(Instruction::StrhImm {
                rt: Register::from(rt),
                rn: Register::from(rn),
                imm5x2: imm5 << 1,
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
        } else if (word >> 11) == 0b01110 {
            let imm5 = ((word >> 6) & 0x1F) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rt = (word & 0b111) as u8;
            Ok(Instruction::StrbImm {
                rt: Register::from(rt),
                rn: Register::from(rn),
                imm5,
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
        } else if (word >> 11) == 0b00010 {
            let mut imm5 = ((word >> 6) & 0x1F) as u8;
            if imm5 == 0 {
                imm5 = 32;
            }
            let rm = ((word >> 3) & 0b111) as u8;
            let rd = (word & 0b111) as u8;
            Ok(Instruction::AsrsImm {
                rd: Register::from(rd),
                rm: Register::from(rm),
                imm5,
            })
        } else if (word >> 11) == 0b00101 {
            let imm8 = (word & 0xFF) as u8;
            let rn = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::CmpImm {
                rn: Register::from(rn),
                imm8,
            })
        } else if (word >> 11) == 0b00110 {
            let imm8 = (word & 0xFF) as u8;
            let rdn = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::AddsImm8 {
                rdn: Register::from(rdn),
                imm8,
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
        } else if (word >> 11) == 0b10100 {
            let imm8 = (word & 0xFF) as u8;
            let rd = ((word >> 8) & 0b111) as u8;
            Ok(Instruction::Adr {
                rd: Register::from(rd),
                imm8x4: u16::from(imm8) << 2,
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
            Ok(Instruction::AddsImm3 {
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
            Ok(Instruction::SubsImm3 {
                rd: Register::from(rd),
                rn: Register::from(rn),
                imm3,
            })
        } else if (word >> 9) == 0b0001101 {
            let rm = ((word >> 6) & 0b111) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rd = (word & 0b111) as u8;
            Ok(Instruction::SubsReg {
                rd: Register::from(rd),
                rn: Register::from(rn),
                rm: Register::from(rm),
            })
        } else if (word >> 9) == 0b0101000 {
            let rm = ((word >> 6) & 0b111) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rt = (word & 0b111) as u8;
            Ok(Instruction::StrReg {
                rt: Register::from(rt),
                rn: Register::from(rn),
                rm: Register::from(rm),
            })
        } else if (word >> 9) == 0b0101001 {
            let rm = ((word >> 6) & 0b111) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rt = (word & 0b111) as u8;
            Ok(Instruction::StrhReg {
                rt: Register::from(rt),
                rn: Register::from(rn),
                rm: Register::from(rm),
            })
        } else if (word >> 9) == 0b0101010 {
            let rm = ((word >> 6) & 0b111) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rt = (word & 0b111) as u8;
            Ok(Instruction::StrbReg {
                rt: Register::from(rt),
                rn: Register::from(rn),
                rm: Register::from(rm),
            })
        } else if (word >> 9) == 0b0101100 {
            let rm = ((word >> 6) & 0b111) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rt = (word & 0b111) as u8;
            Ok(Instruction::LdrReg {
                rt: Register::from(rt),
                rn: Register::from(rn),
                rm: Register::from(rm),
            })
        } else if (word >> 9) == 0b0101101 {
            let rm = ((word >> 6) & 0b111) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rt = (word & 0b111) as u8;
            Ok(Instruction::LdrhReg {
                rt: Register::from(rt),
                rn: Register::from(rn),
                rm: Register::from(rm),
            })
        } else if (word >> 9) == 0b0101110 {
            let rm = ((word >> 6) & 0b111) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rt = (word & 0b111) as u8;
            Ok(Instruction::LdrbReg {
                rt: Register::from(rt),
                rn: Register::from(rn),
                rm: Register::from(rm),
            })
        } else if (word >> 9) == 0b0101011 {
            let rm = ((word >> 6) & 0b111) as u8;
            let rn = ((word >> 3) & 0b111) as u8;
            let rt = (word & 0b111) as u8;
            Ok(Instruction::LdrsbReg {
                rt: Register::from(rt),
                rn: Register::from(rn),
                rm: Register::from(rm),
            })
        } else if (word >> 8) == 0b10111110 {
            let imm8 = (word & 0xFF) as u8;
            Ok(Instruction::Breakpoint { imm8 })
        } else if (word >> 8) == 0b01000100 && ((word >> 3) & 0b1111) == 0b1101 {
            let dm = (word >> 7) & 1;
            let rdm = ((dm << 4) | (word & 0b111)) as u8;
            Ok(Instruction::AddRegSp {
                rdm: Register::from(rdm),
            })
        } else if (word >> 8) == 0b01000110 {
            let d: u8 = ((word >> 7) & 0b1) as u8;
            let rd = (d << 3) | (word & 0b111) as u8;
            let rm: u8 = ((word >> 3) & 0b1111) as u8;
            Ok(Instruction::MovReg {
                rm: Register::from(rm),
                rd: Register::from(rd),
            })
        } else if (word >> 7) == 0b010001110 {
            let rm = ((word >> 3) & 0b1111) as u8;
            Ok(Instruction::BranchExchange {
                rm: Register::from(rm),
            })
        } else if (word >> 7) == 0b010001111 {
            let rm = ((word >> 3) & 0b1111) as u8;
            Ok(Instruction::BranchLinkExchange {
                rm: Register::from(rm),
            })
        } else if (word >> 7) == 0b101100000 {
            let imm7 = (word & 0x7F) as u8;
            Ok(Instruction::AddSpImm {
                imm7x4: u32::from(imm7) << 2,
            })
        } else if (word >> 7) == 0b101100001 {
            let imm7 = (word & 0x7F) as u8;
            Ok(Instruction::SubSpImm {
                imm7x4: u32::from(imm7) << 2,
            })
        } else if (word >> 7) == 0b010001001 && (word & 0b111) == 0b101 {
            let rm: u8 = ((word >> 3) & 0b1111) as u8;
            Ok(Instruction::AddSpReg {
                rm: Register::from(rm),
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
        } else if (word >> 6) == 0b0100001101 {
            let rm = ((word >> 3) & 0b111) as u8;
            let rdn = (word & 0b111) as u8;
            Ok(Instruction::Muls {
                rdn: Register::from(rdn),
                rm: Register::from(rm),
            })
        } else if (word >> 6) == 0b1011001010 {
            let rm = ((word >> 3) & 0b111) as u8;
            let rd = (word & 0b111) as u8;
            Ok(Instruction::Uxth {
                rd: Register::from(rd),
                rm: Register::from(rm),
            })
        } else if (word >> 6) == 0b1011001011 {
            let rm = ((word >> 3) & 0b111) as u8;
            let rd = (word & 0b111) as u8;
            Ok(Instruction::Uxtb {
                rd: Register::from(rd),
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
        } else if (word >> 6) == 0b0100000001 {
            let rm = ((word >> 3) & 0b111) as u8;
            let rdn = (word & 0b111) as u8;
            Ok(Instruction::EorsReg {
                rdn: Register::from(rdn),
                rm: Register::from(rm),
            })
        } else if (word >> 6) == 0b0100001110 {
            let rm = ((word >> 3) & 0b111) as u8;
            let rdn = (word & 0b111) as u8;
            Ok(Instruction::BicsReg {
                rdn: Register::from(rdn),
                rm: Register::from(rm),
            })
        } else if (word >> 6) == 0b0100001111 {
            let rm = ((word >> 3) & 0b111) as u8;
            let rd = (word & 0b111) as u8;
            Ok(Instruction::MvnsReg {
                rd: Register::from(rd),
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
        } else if (word >> 6) == 0b0100000110 {
            let rm = ((word >> 3) & 0b111) as u8;
            let rdn = (word & 0b111) as u8;
            Ok(Instruction::SbcsReg {
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
            let imm24 = s_prefix | i1 | i2 | (imm10 << 12) | (imm11 << 1);
            Ok(Instruction::BranchLink {
                imm24: imm24 as i32,
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
        } else if (word1 >> 4) == 0b111100111011 && (word2 >> 4) == 0b100011110101 {
            Ok(Instruction::Dmb)
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
        debug!("Executing '{}' @ {:08x}", instruction, self.pc);
        self.breakpoint = None;
        self.pc = self.pc.wrapping_add(2);
        match instruction {
            // =======================================================================
            // Branch Instructions
            // =======================================================================
            Instruction::Branch { imm11x2 } => {
                // Assume PC's next increment has happened already
                self.pc = self.pc.wrapping_add(2);
                self.pc = self.pc.wrapping_add(imm11x2 as u32);
            }
            Instruction::BranchConditional { cond, imm8x2 } => {
                if self.check_condition(cond) {
                    // Assume PC's next increment has happened already
                    self.pc = self.pc.wrapping_add(2);
                    self.pc = self.pc.wrapping_add(imm8x2 as u32);
                }
            }
            Instruction::BranchLink { imm24 } => {
                // Assume PC's next increment has happened already
                let old_pc = self.pc.wrapping_add(2);
                self.lr = (old_pc & !1) | 1;
                self.pc = old_pc.wrapping_add(imm24 as u32);
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
            Instruction::BranchLinkExchange { rm } => {
                let addr = self.fetch_reg(rm);
                self.lr = (self.pc & !1) | 1;
                self.pc = addr & !1;
            }
            // =======================================================================
            // Move Instructions
            // =======================================================================
            Instruction::MovsImm { rd, imm8 } => {
                let imm32 = u32::from(imm8);
                self.store_reg(rd, imm32);
                self.set_z(imm32 == 0);
                self.set_n(imm32 >= 0x80000000);
                // c is unchanged
                // v is unchanged
            }
            Instruction::MovReg { rm, rd } => {
                let value = self.fetch_reg(rm);
                if rd == Register::Pc {
                    self.store_reg(rd, value & !1);
                } else {
                    self.store_reg(rd, value);
                }
            }
            // =======================================================================
            // Store Instructions
            // =======================================================================
            Instruction::StrImm { rt, rn, imm5x4 } => {
                let addr = self.fetch_reg(rn);
                let offset_addr = addr.wrapping_add(u32::from(imm5x4));
                let value = self.fetch_reg(rt);
                memory.store_u32(offset_addr, value)?;
            }
            Instruction::StrReg { rt, rn, rm } => {
                let offset = self.fetch_reg(rm);
                let addr = self.fetch_reg(rn);
                let offset_addr = addr.wrapping_add(offset);
                let value = self.fetch_reg(rt);
                memory.store_u32(offset_addr, value)?;
            }
            Instruction::StrhImm { rt, rn, imm5x2 } => {
                let addr = self.fetch_reg(rn);
                let offset_addr = addr.wrapping_add(u32::from(imm5x2));
                let value = self.fetch_reg(rt);
                memory.store_u16(offset_addr, value as u16)?;
            }
            Instruction::StrhReg { rt, rn, rm } => {
                let offset = self.fetch_reg(rm);
                let addr = self.fetch_reg(rn);
                let offset_addr = addr.wrapping_add(offset);
                let value = self.fetch_reg(rt);
                memory.store_u16(offset_addr, value as u16)?;
            }
            Instruction::StrbImm { rt, rn, imm5 } => {
                let addr = self.fetch_reg(rn);
                let offset_addr = addr.wrapping_add(u32::from(imm5));
                let value = self.fetch_reg(rt);
                memory.store_u8(offset_addr, (value & 0xFF) as u8)?;
            }
            Instruction::StrbReg { rt, rn, rm } => {
                let offset = self.fetch_reg(rm);
                let addr = self.fetch_reg(rn);
                let offset_addr = addr.wrapping_add(offset);
                let value = self.fetch_reg(rt);
                memory.store_u8(offset_addr, (value & 0xFF) as u8)?;
            }
            // =======================================================================
            // Load Instructions
            // =======================================================================
            Instruction::LdrLiteral { rt, imm8x4 } => {
                // Assume's PC next increment has happened already
                // Also align to 4 bytes
                let base = (self.pc.wrapping_add(2)) & !0b11;
                let addr = base.wrapping_add(u32::from(imm8x4));
                let value = memory.load_u32(addr)?;
                self.store_reg(rt, value);
            }
            Instruction::LdrImm { rt, rn, imm5x4 } => {
                let addr = self.fetch_reg(rn);
                let offset_addr = addr.wrapping_add(u32::from(imm5x4));
                let value = memory.load_u32(offset_addr)?;
                self.store_reg(rt, value);
            }
            Instruction::LdrReg { rt, rn, rm } => {
                let addr = self.fetch_reg(rn);
                let offset = self.fetch_reg(rm);
                let offset_addr = addr.wrapping_add(offset);
                let value = memory.load_u32(offset_addr)?;
                self.store_reg(rt, value);
            }
            Instruction::LdrhImm { rt, rn, imm5x2 } => {
                let addr = self.fetch_reg(rn);
                let offset_addr = addr.wrapping_add(u32::from(imm5x2));
                let value = memory.load_u16(offset_addr)?;
                self.store_reg(rt, u32::from(value));
            }
            Instruction::LdrhReg { rt, rn, rm } => {
                let addr = self.fetch_reg(rn);
                let offset = self.fetch_reg(rm);
                let offset_addr = addr.wrapping_add(offset);
                let value = memory.load_u16(offset_addr)?;
                self.store_reg(rt, u32::from(value));
            }
            Instruction::LdrbImm { rt, rn, imm5 } => {
                let addr = self.fetch_reg(rn);
                let offset_addr = addr.wrapping_add(u32::from(imm5));
                let value = memory.load_u8(offset_addr)?;
                self.store_reg(rt, u32::from(value));
            }
            Instruction::LdrbReg { rt, rn, rm } => {
                let addr = self.fetch_reg(rn);
                let offset = self.fetch_reg(rm);
                let offset_addr = addr.wrapping_add(offset);
                let value = memory.load_u8(offset_addr)?;
                self.store_reg(rt, u32::from(value));
            }
            Instruction::LdrsbReg { rt, rn, rm } => {
                let addr = self.fetch_reg(rn);
                let offset = self.fetch_reg(rm);
                let offset_addr = addr.wrapping_add(offset);
                let value = memory.load_u8(offset_addr)?;
                self.store_reg(rt, i32::from(value as i8) as u32);
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
            Instruction::AddSpImmReg { rd, imm8x4 } => {
                let sp = self.sp;
                let value = sp.wrapping_add(u32::from(imm8x4));
                self.store_reg(rd, value);
            }
            Instruction::AddSpImm { imm7x4 } => {
                self.sp = self.sp.wrapping_add(imm7x4);
            }
            Instruction::AddSpReg { rm } => {
                let offset = self.fetch_reg(rm);
                self.sp = self.sp.wrapping_add(offset);
            }
            Instruction::AddRegSp { rdm } => {
                let value = self.fetch_reg(rdm);
                self.store_reg(rdm, value.wrapping_add(self.sp));
            }
            Instruction::SubSpImm { imm7x4 } => {
                // This does a subtraction
                let value = self.sp.wrapping_add(!imm7x4).wrapping_add(1);
                self.sp = value;
            }
            Instruction::LdrSpImm { rt, imm8x4 } => {
                let offset_addr = self.sp.wrapping_add(u32::from(imm8x4));
                let value = memory.load_u32(offset_addr)?;
                self.store_reg(rt, value);
            }
            Instruction::StrSpImm { rt, imm8x4 } => {
                let offset_addr = self.sp.wrapping_add(u32::from(imm8x4));
                let value = self.fetch_reg(rt);
                memory.store_u32(offset_addr, value)?;
            }
            // =======================================================================
            // Arithmetic Instructions
            // =======================================================================
            Instruction::AddsImm3 { rd, rn, imm3 } => {
                let value1 = self.fetch_reg(rn);
                let value2 = u32::from(imm3);
                let (result, carry_out, overflow) = Self::add_with_carry(value1, value2, false);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry_out);
                self.set_v(overflow);
                self.store_reg(rd, result);
            }
            Instruction::AddsImm8 { rdn, imm8 } => {
                let value1 = self.fetch_reg(rdn);
                let value2 = u32::from(imm8);
                let (result, carry_out, overflow) = Self::add_with_carry(value1, value2, false);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry_out);
                self.set_v(overflow);
                self.store_reg(rdn, result);
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
            Instruction::Adr { rd, imm8x4 } => {
                let base = (self.pc.wrapping_add(2)) & !0b11;
                let addr = base.wrapping_add(u32::from(imm8x4));
                self.store_reg(rd, addr);
            }
            Instruction::SubsImm3 { rd, rn, imm3 } => {
                let value1 = self.fetch_reg(rn);
                let value2 = u32::from(imm3);
                let (result, carry_out, overflow) = Self::add_with_carry(value1, !value2, true);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry_out);
                self.set_v(overflow);
                self.store_reg(rd, result);
            }
            Instruction::SubsImm8 { rdn, imm8 } => {
                let value1 = self.fetch_reg(rdn);
                let value2 = u32::from(imm8);
                let (result, carry_out, overflow) = Self::add_with_carry(value1, !value2, true);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry_out);
                self.set_v(overflow);
                self.store_reg(rdn, result);
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
            Instruction::SbcsReg { rdn, rm } => {
                let value1 = self.fetch_reg(rdn);
                let value2 = self.fetch_reg(rm);
                let (result, carry_out, overflow) =
                    Self::add_with_carry(value1, !value2, self.is_c());
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.set_c(carry_out);
                self.set_v(overflow);
                self.store_reg(rdn, result);
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
            Instruction::AsrsImm { rd, rm, imm5 } => {
                let value = self.fetch_reg(rm) as i32;
                let shift_n = u32::from(imm5);
                let result = value >> shift_n;
                let carry_out = (value & (1 << (shift_n - 1))) != 0;
                self.store_reg(rd, result as u32);
                self.set_n(result < 0);
                self.set_z(result == 0);
                self.set_c(carry_out);
            }
            Instruction::Muls { rdn, rm } => {
                let value1 = self.fetch_reg(rdn);
                let value2 = self.fetch_reg(rm);
                let result = value1.wrapping_mul(value2);
                self.store_reg(rdn, result);
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
            }
            Instruction::Uxth { rd, rm } => {
                let value = self.fetch_reg(rm) & 0xFFFF;
                self.store_reg(rd, value);
            }
            Instruction::Uxtb { rd, rm } => {
                let value = self.fetch_reg(rm) & 0xFF;
                self.store_reg(rd, value);
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
            Instruction::CmpImm { rn, imm8 } => {
                let value_n = self.fetch_reg(rn);
                let (result, carry, overflow) =
                    Self::add_with_carry(value_n, !u32::from(imm8), true);
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
            Instruction::EorsReg { rdn, rm } => {
                let value_m = self.fetch_reg(rm);
                let value_n = self.fetch_reg(rdn);
                let result = value_n ^ value_m;
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.store_reg(rdn, result);
            }
            Instruction::BicsReg { rdn, rm } => {
                let value_m = self.fetch_reg(rm);
                let value_n = self.fetch_reg(rdn);
                let result = value_n & !value_m;
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.store_reg(rdn, result);
            }
            Instruction::MvnsReg { rd, rm } => {
                let value_m = self.fetch_reg(rm);
                let result = !value_m;
                self.set_n(result >= 0x8000_0000);
                self.set_z(result == 0);
                self.store_reg(rd, result);
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
            Instruction::Dmb => {
                // Wide instruction
                self.pc = self.pc.wrapping_add(2);
                // Let's do a fence - unsure if required?
                std::sync::atomic::fence(std::sync::atomic::Ordering::SeqCst);
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
        memory.store_u32(sp, value)?;
        self.store_reg(Register::Sp, sp);
        Ok(())
    }

    /// Pop one value off the stack
    fn pop_stack(&mut self, memory: &mut dyn Memory) -> Result<u32, Error> {
        let value = memory.load_u32(self.sp)?;
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
        let i = Armv6M::decode(0b1110011111111110);
        assert_eq!(Ok(Instruction::Branch { imm11x2: -4 }), i);
        assert_eq!("B +0", format!("{}", i.unwrap()));

        let i = Armv6M::decode(0b1110011111111100);
        assert_eq!(Ok(Instruction::Branch { imm11x2: -8 }), i);
        assert_eq!("B -4", format!("{}", i.unwrap()));

        let i = Armv6M::decode(0b1110000000000001);
        assert_eq!(Ok(Instruction::Branch { imm11x2: 2 }), i);
        assert_eq!("B +6", format!("{}", i.unwrap()));

        let mut cpu = Armv6M::new(0, 8);
        let mut ram = [0u32; 6];
        cpu.execute(Instruction::Branch { imm11x2: -4 }, &mut ram)
            .unwrap();
        // PC was 8, PC is still 8, because `0xe7fe` means spin in a loop
        assert_eq!(cpu.pc, 8);
    }

    #[test]
    fn beq_instruction() {
        let i = Armv6M::decode(0b1101000000000011);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Eq,
                imm8x2: 6
            }),
            i
        );
        assert_eq!("BEQ +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bne_instruction() {
        let i = Armv6M::decode(0b1101000100000011);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Ne,
                imm8x2: 6
            }),
            i
        );
        assert_eq!("BNE +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bcs_instruction() {
        let i = Armv6M::decode(0b1101001000000011);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Cs,
                imm8x2: 6
            }),
            i
        );
        assert_eq!("BCS +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bcc_instruction() {
        let i = Armv6M::decode(0b1101001100000011);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Cc,
                imm8x2: 6
            }),
            i
        );
        assert_eq!("BCC +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bmi_instruction() {
        let i = Armv6M::decode(0b1101010000000011);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Mi,
                imm8x2: 6
            }),
            i
        );
        assert_eq!("BMI +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bpl_instruction() {
        let i = Armv6M::decode(0b1101010100000011);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Pl,
                imm8x2: 6
            }),
            i
        );
        assert_eq!("BPL +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bvs_instruction() {
        let i = Armv6M::decode(0b1101011000000011);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Vs,
                imm8x2: 6
            }),
            i
        );
        assert_eq!("BVS +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bvc_instruction() {
        let i = Armv6M::decode(0b1101011100000011);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Vc,
                imm8x2: 6
            }),
            i
        );
        assert_eq!("BVC +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bhi_instruction() {
        let i = Armv6M::decode(0b1101100000000011);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Hi,
                imm8x2: 6
            }),
            i
        );
        assert_eq!("BHI +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bls_instruction() {
        let i = Armv6M::decode(0b1101100100000011);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Ls,
                imm8x2: 6
            }),
            i
        );
        assert_eq!("BLS +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bge_instruction() {
        let i = Armv6M::decode(0b1101101000000011);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Ge,
                imm8x2: 6
            }),
            i
        );
        assert_eq!("BGE +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn blt_instruction() {
        let i = Armv6M::decode(0b1101101100000011);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Lt,
                imm8x2: 6
            }),
            i
        );
        assert_eq!("BLT +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn bgt_instruction() {
        let i = Armv6M::decode(0b1101110000000011);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Gt,
                imm8x2: 6
            }),
            i
        );
        assert_eq!("BGT +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn ble_instruction() {
        let i = Armv6M::decode(0b1101110100000011);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Le,
                imm8x2: 6
            }),
            i
        );
        assert_eq!("BLE +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn balways_instruction() {
        let i = Armv6M::decode(0b1101111000000011);
        assert_eq!(
            Ok(Instruction::BranchConditional {
                cond: Condition::Always,
                imm8x2: 6
            }),
            i
        );
        assert_eq!("B +10", format!("{}", i.unwrap()));
    }

    #[test]
    fn branch_link_instruction() {
        assert_eq!(
            Err(Error::WideInstruction),
            Armv6M::decode(0b1111000001001010)
        );
        let i = Armv6M::decode32(0xF04A, 0xFE41);
        assert_eq!(Ok(Instruction::BranchLink { imm24: 0x4ac82 }), i);
        assert_eq!("BL +306310", format!("{}", i.unwrap()));
    }

    #[test]
    fn branch_link_operation() {
        // d0:	f04a fe41 	bl	4ad56
        let sp = 32;
        let start_pc = 0xd0;
        let mut cpu = Armv6M::new(sp, start_pc);
        let mut ram = [15; 8];
        cpu.execute(Instruction::BranchLink { imm24: 0x4ac82 }, &mut ram)
            .unwrap();
        assert_eq!(start_pc + 5, cpu.lr);
        assert_eq!(0x4ad56, cpu.pc);
    }

    #[test]
    fn branch_exchange_instruction() {
        let i = Armv6M::decode(0b0100011101110000);
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
    fn branch_link_exchange_instruction() {
        let i = Armv6M::decode(0b0100011110011000);
        assert_eq!(Ok(Instruction::BranchLinkExchange { rm: Register::R3 }), i);
        assert_eq!("BLX R3", format!("{}", i.unwrap()));
    }

    #[test]
    fn branch_link_exchange_operation() {
        let sp = 32;
        let start_pc = 0xd0;
        let mut cpu = Armv6M::new(sp, start_pc);
        cpu.lr = 0x0000_1235;
        cpu.regs[3] = 0x1000_0000;
        let mut ram = [15; 8];
        cpu.execute(
            Instruction::BranchLinkExchange { rm: Register::R3 },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x1000_0000, cpu.pc);
        assert_eq!(0xd0 + 2 + 1, cpu.lr);
    }
    // =======================================================================
    // Move Instructions
    // =======================================================================
    #[test]
    fn movs_imm_instruction() {
        let i = Armv6M::decode(0b0010001001000000);
        assert_eq!(
            Ok(Instruction::MovsImm {
                rd: Register::R2,
                imm8: 64
            }),
            i
        );
        assert_eq!("MOVS R2,#64", format!("{}", i.unwrap()));
    }
    #[test]
    fn movs_imm_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[1] = 0x12345678;
        cpu.execute(
            Instruction::MovsImm {
                rd: Register::R2,
                imm8: 64,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(64, cpu.regs[2]);
    }
    #[test]
    fn mov_reg_instruction() {
        let i = Armv6M::decode(0b0100011010110110);
        assert_eq!(
            Ok(Instruction::MovReg {
                rd: Register::Lr,
                rm: Register::R6,
            }),
            i
        );
        assert_eq!("MOV LR,R6", format!("{}", i.unwrap()));
    }
    #[test]
    fn mov_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[6] = 0x12345678;
        cpu.execute(
            Instruction::MovReg {
                rd: Register::Lr,
                rm: Register::R6,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x12345678, cpu.lr);
    }
    // =======================================================================
    // Store Instructions
    // =======================================================================
    #[test]
    fn str_immediate_instruction() {
        let i = Armv6M::decode(0b0110000000101100);
        assert_eq!(
            Ok(Instruction::StrImm {
                rt: Register::R4,
                rn: Register::R5,
                imm5x4: 0
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
                imm5x4: 4,
            },
            &mut ram,
        )
        .unwrap();
        // R1 written to address 4 + 4 = 8
        assert_eq!([0, 0, 0x12345678, 0, 0, 0, 0, 0], ram);
    }
    #[test]
    fn str_reg_instruction() {
        let i = Armv6M::decode(0b0101000100101011);
        assert_eq!(
            Ok(Instruction::StrReg {
                rt: Register::R3,
                rn: Register::R5,
                rm: Register::R4,
            }),
            i
        );
        assert_eq!("STR R3,[R5,R4]", format!("{}", i.unwrap()));
    }
    #[test]
    fn str_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 4;
        cpu.regs[1] = 4;
        cpu.regs[2] = 0x12345678;
        cpu.execute(
            Instruction::StrReg {
                rt: Register::R2,
                rn: Register::R1,
                rm: Register::R0,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!([0, 0, 0x12345678, 0, 0, 0, 0, 0], ram);
    }

    #[test]
    fn strh_immediate_instruction() {
        let i = Armv6M::decode(0b1000000000011001);
        assert_eq!(
            Ok(Instruction::StrhImm {
                rt: Register::R1,
                rn: Register::R3,
                imm5x2: 0
            }),
            i
        );
        assert_eq!("STRH R1,[R3,#0]", format!("{}", i.unwrap()));
    }

    #[test]
    fn strh_immediate_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 4;
        cpu.regs[1] = 0x12345678;
        cpu.execute(
            Instruction::StrhImm {
                rt: Register::R1,
                rn: Register::R0,
                imm5x2: 2,
            },
            &mut ram,
        )
        .unwrap();
        // Lowest 16 bits of R1 written to address 4 + 2 = 6
        assert_eq!([0, 0x56780000, 0, 0, 0, 0, 0, 0], ram);
    }
    #[test]
    fn strh_reg_instruction() {
        let i = Armv6M::decode(0b0101001100101011);
        assert_eq!(
            Ok(Instruction::StrhReg {
                rt: Register::R3,
                rn: Register::R5,
                rm: Register::R4,
            }),
            i
        );
        assert_eq!("STRH R3,[R5,R4]", format!("{}", i.unwrap()));
    }
    #[test]
    fn strh_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 2;
        cpu.regs[1] = 4;
        cpu.regs[2] = 0x12345678;
        cpu.execute(
            Instruction::StrhReg {
                rt: Register::R2,
                rn: Register::R1,
                rm: Register::R0,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!([0, 0x56780000, 0, 0, 0, 0, 0, 0], ram);
    }

    #[test]
    fn strb_immediate_instruction() {
        let i = Armv6M::decode(0b0111000000011001);
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
    fn strb_reg_instruction() {
        let i = Armv6M::decode(0b0101010100101011);
        assert_eq!(
            Ok(Instruction::StrbReg {
                rt: Register::R3,
                rn: Register::R5,
                rm: Register::R4,
            }),
            i
        );
        assert_eq!("STRB R3,[R5,R4]", format!("{}", i.unwrap()));
    }
    #[test]
    fn strb_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 1;
        cpu.regs[1] = 4;
        cpu.regs[2] = 0x12345678;
        cpu.execute(
            Instruction::StrbReg {
                rt: Register::R2,
                rn: Register::R1,
                rm: Register::R0,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!([0, 0x7800, 0, 0, 0, 0, 0, 0], ram);
    }
    // =======================================================================
    // Load Instructions
    // =======================================================================
    #[test]
    fn ldr_literal_instruction() {
        let i = Armv6M::decode(0b0100100000000001);
        assert_eq!(
            Ok(Instruction::LdrLiteral {
                rt: Register::R0,
                imm8x4: 4
            }),
            i
        );
        assert_eq!("LDR R0,[PC,#4]", format!("{}", i.unwrap()));
    }
    #[test]
    fn ldr_literal_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [1, 2, 0x12345678, 3, 4, 5, 6, 7];
        cpu.execute(
            Instruction::LdrLiteral {
                rt: Register::R0,
                imm8x4: 4,
            },
            &mut ram,
        )
        .unwrap();
        // R1 loaded from address PC + 4 + 4
        assert_eq!(0x12345678, cpu.regs[0]);
    }
    #[test]
    fn ldr_immediate_instruction() {
        let i = Armv6M::decode(0b0110100000101100);
        assert_eq!(
            Ok(Instruction::LdrImm {
                rt: Register::R4,
                rn: Register::R5,
                imm5x4: 0
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
                imm5x4: 4,
            },
            &mut ram,
        )
        .unwrap();
        // R1 loaded from address 4 + 4 = 8
        assert_eq!(0x12345678, cpu.regs[1]);
    }
    #[test]
    fn ldr_reg_instruction() {
        let i = Armv6M::decode(0b0101100010001000);
        assert_eq!(
            Ok(Instruction::LdrReg {
                rt: Register::R0,
                rn: Register::R1,
                rm: Register::R2,
            }),
            i
        );
        assert_eq!("LDR R0,[R1,R2]", format!("{}", i.unwrap()));
    }
    #[test]
    fn ldr_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0, 0, 0x12345678, 0, 0, 0, 0, 0];
        cpu.regs[1] = 4;
        cpu.regs[2] = 4;
        cpu.execute(
            Instruction::LdrReg {
                rt: Register::R0,
                rn: Register::R1,
                rm: Register::R2,
            },
            &mut ram,
        )
        .unwrap();
        // R1 loaded from address 4 + 4 = 8
        assert_eq!(0x12345678, cpu.regs[0]);
    }
    #[test]
    fn ldrh_immediate_instruction() {
        let i = Armv6M::decode(0b1000100001101100);
        assert_eq!(
            Ok(Instruction::LdrhImm {
                rt: Register::R4,
                rn: Register::R5,
                imm5x2: 2
            }),
            i
        );
        assert_eq!("LDRH R4,[R5,#2]", format!("{}", i.unwrap()));
    }
    #[test]
    fn ldrh_immediate_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0, 0, 0x12345678, 0, 0, 0, 0, 0];
        cpu.regs[0] = 4;
        cpu.execute(
            Instruction::LdrhImm {
                rt: Register::R1,
                rn: Register::R0,
                imm5x2: 4,
            },
            &mut ram,
        )
        .unwrap();
        // R1 loaded from address 4 + 4 = 8
        assert_eq!(0x5678, cpu.regs[1]);
    }
    #[test]
    fn ldrh_reg_instruction() {
        let i = Armv6M::decode(0b0101101010001001);
        assert_eq!(
            Ok(Instruction::LdrhReg {
                rt: Register::R1,
                rn: Register::R1,
                rm: Register::R2
            }),
            i
        );
        assert_eq!("LDRH R1,[R1,R2]", format!("{}", i.unwrap()));
    }
    #[test]
    fn ldrh_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0, 0, 0x12345678, 0, 0, 0, 0, 0];
        cpu.regs[0] = 4;
        cpu.regs[6] = 4;
        cpu.execute(
            Instruction::LdrhReg {
                rt: Register::R1,
                rn: Register::R0,
                rm: Register::R6,
            },
            &mut ram,
        )
        .unwrap();
        // R1 loaded from address 4 + 4 = 8
        assert_eq!(0x5678, cpu.regs[1]);
    }
    #[test]
    fn ldrb_immediate_instruction() {
        let i = Armv6M::decode(0b0111100000101100);
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
        // R1 loaded from address 4 + 4 = 8
        assert_eq!(0x78, cpu.regs[1]);
    }
    #[test]
    fn ldrb_reg_instruction() {
        let i = Armv6M::decode(0b0101110010101100);
        assert_eq!(
            Ok(Instruction::LdrbReg {
                rt: Register::R4,
                rn: Register::R5,
                rm: Register::R2,
            }),
            i
        );
        assert_eq!("LDRB R4,[R5,R2]", format!("{}", i.unwrap()));
    }
    #[test]
    fn ldrb_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0, 0, 0x12345678, 0, 0, 0, 0, 0];
        cpu.regs[0] = 4;
        cpu.regs[2] = 4;
        cpu.execute(
            Instruction::LdrbReg {
                rt: Register::R1,
                rn: Register::R0,
                rm: Register::R2,
            },
            &mut ram,
        )
        .unwrap();
        // R1 loaded from address 4 + 4 = 8
        assert_eq!(0x78, cpu.regs[1]);
    }
    #[test]
    fn ldrsb_reg_instruction() {
        let i = Armv6M::decode(0b0101011010101100);
        assert_eq!(
            Ok(Instruction::LdrsbReg {
                rt: Register::R4,
                rn: Register::R5,
                rm: Register::R2,
            }),
            i
        );
        assert_eq!("LDRSB R4,[R5,R2]", format!("{}", i.unwrap()));
    }
    #[test]
    fn ldrsb_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0, 0, 0x12345680, 0, 0, 0, 0, 0];
        cpu.regs[0] = 4;
        cpu.regs[2] = 4;
        cpu.execute(
            Instruction::LdrsbReg {
                rt: Register::R1,
                rn: Register::R0,
                rm: Register::R2,
            },
            &mut ram,
        )
        .unwrap();
        // R1 loaded from address 4 + 4 = 8
        assert_eq!(0xFFFFFF80, cpu.regs[1]);
    }
    // =======================================================================
    // Stack Instructions
    // =======================================================================`
    #[test]
    fn push_instruction() {
        let i = Armv6M::decode(0b1011010110000000);
        assert_eq!(
            // Push {LR, R7}
            Ok(Instruction::Push {
                register_list: RegisterList::new(0x80),
                m: true
            }),
            i
        );
        assert_eq!("PUSH {R7,LR}", format!("{}", i.unwrap()));

        let i = Armv6M::decode(0b1011010011111111);
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
        let i = Armv6M::decode(0b1011110110000000);
        assert_eq!(
            Ok(Instruction::Pop {
                register_list: RegisterList::new(0x80),
                p: true
            }),
            i
        );
        assert_eq!("POP {R7,PC}", format!("{}", i.unwrap()));

        let i = Armv6M::decode(0b1011110011111111);
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
        let i = Armv6M::decode(0b1100100100111000);
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
        let i = Armv6M::decode(0b1100000000111000);
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
    fn add_sp_imm_reg_instruction() {
        let i = Armv6M::decode(0b1010111100000000);
        assert_eq!(
            Ok(Instruction::AddSpImmReg {
                rd: Register::R7,
                imm8x4: 0
            }),
            i
        );
        assert_eq!("ADD R7,SP,#0", format!("{}", i.unwrap()));
    }

    #[test]
    fn add_sp_imm_reg_operation() {
        let sp = 32;
        let mut cpu = Armv6M::new(sp, 0x0);
        let mut ram = [15; 8];
        cpu.execute(
            Instruction::AddSpImmReg {
                rd: Register::R0,
                imm8x4: 16,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(32 + 4 * 4, cpu.regs[0]);
    }

    #[test]
    fn add_sp_imm_instruction() {
        let i = Armv6M::decode(0b1011000000010000);
        assert_eq!(Ok(Instruction::AddSpImm { imm7x4: 64 }), i);
        assert_eq!("ADD SP,#64", format!("{}", i.unwrap()));
    }

    #[test]
    fn add_sp_imm_operation() {
        let sp = 0x100;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [15; 8];
        cpu.execute(Instruction::AddSpImm { imm7x4: 64 }, &mut ram)
            .unwrap();
        assert_eq!(0x100 + 64, cpu.sp);
    }

    #[test]
    fn add_sp_reg_instruction() {
        let i = Armv6M::decode(0b0100010010001101);
        assert_eq!(Ok(Instruction::AddSpReg { rm: Register::R1 }), i);
        assert_eq!("ADD SP,R1", format!("{}", i.unwrap()));
    }

    #[test]
    fn add_sp_reg_operation() {
        let sp = 32;
        let mut cpu = Armv6M::new(sp, 0x0);
        let mut ram = [15; 8];
        cpu.regs[1] = 4;
        cpu.execute(Instruction::AddSpReg { rm: Register::R1 }, &mut ram)
            .unwrap();
        assert_eq!(36, cpu.sp);
    }

    #[test]
    fn add_reg_sp_instruction() {
        let i = Armv6M::decode(0b0100010001101001);
        assert_eq!(Ok(Instruction::AddRegSp { rdm: Register::R1 }), i);
        assert_eq!("ADD R1,SP", format!("{}", i.unwrap()));
    }

    #[test]
    fn add_reg_sp_operation() {
        let sp = 32;
        let mut cpu = Armv6M::new(sp, 0x0);
        let mut ram = [15; 8];
        cpu.regs[1] = 4;
        cpu.execute(Instruction::AddRegSp { rdm: Register::R1 }, &mut ram)
            .unwrap();
        assert_eq!(36, cpu.regs[1]);
    }

    #[test]
    fn sub_sp_imm_instruction() {
        let i = Armv6M::decode(0b1011000010010000);
        assert_eq!(Ok(Instruction::SubSpImm { imm7x4: 64 }), i);
        assert_eq!("SUB SP,#64", format!("{}", i.unwrap()));
    }

    #[test]
    fn sub_sp_imm_operation() {
        let sp = 0x100;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [15; 8];
        cpu.execute(Instruction::SubSpImm { imm7x4: 64 }, &mut ram)
            .unwrap();
        assert_eq!(0x100 - 64, cpu.sp);
    }

    #[test]
    fn ldr_sp_imm_instruction() {
        let i = Armv6M::decode(0b1001100100000010);
        assert_eq!(
            Ok(Instruction::LdrSpImm {
                rt: Register::R1,
                imm8x4: 8
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
                imm8x4: 8,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x12345678, cpu.regs[1]);
    }

    #[test]
    fn str_sp_imm_instruction() {
        let i = Armv6M::decode(0b1001010100000010);
        assert_eq!(
            Ok(Instruction::StrSpImm {
                rt: Register::R5,
                imm8x4: 8
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
                imm8x4: 8,
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
    fn adds_imm3_instruction() {
        let i = Armv6M::decode(0b0001110100000000);
        assert_eq!(
            Ok(Instruction::AddsImm3 {
                rd: Register::R0,
                rn: Register::R0,
                imm3: 4
            }),
            i
        );
        assert_eq!("ADDS R0,R0,#4", format!("{}", i.unwrap()));
    }
    #[test]
    fn adds_imm3_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 4;
        cpu.regs[1] = 0x12345678;
        cpu.execute(
            Instruction::AddsImm3 {
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
    fn adds_imm8_instruction() {
        let i = Armv6M::decode(0b0011000000000100);
        assert_eq!(
            Ok(Instruction::AddsImm8 {
                rdn: Register::R0,
                imm8: 4
            }),
            i
        );
        assert_eq!("ADDS R0,#4", format!("{}", i.unwrap()));
    }
    #[test]
    fn adds_imm8_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[1] = 0x12345678;
        cpu.execute(
            Instruction::AddsImm8 {
                rdn: Register::R1,
                imm8: 1,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x12345679, cpu.regs[1]);
        assert!(!cpu.is_n());
        assert!(!cpu.is_z());
        assert!(!cpu.is_c());
        assert!(!cpu.is_v());
    }
    #[test]
    fn adds_reg_instruction() {
        let i = Armv6M::decode(0b0001100001000000);
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
        let i = Armv6M::decode(0b0100000101001000);
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
    fn adr_instruction() {
        let i = Armv6M::decode(0b1010000000000001);
        assert_eq!(
            Ok(Instruction::Adr {
                rd: Register::R0,
                imm8x4: 4
            }),
            i
        );
        assert_eq!("ADR R0,#4", format!("{}", i.unwrap()));
    }
    #[test]
    fn adr_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 4);
        let mut ram = [0; 8];
        cpu.set_c(true);
        cpu.execute(
            Instruction::Adr {
                rd: Register::R0,
                imm8x4: 4,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(12, cpu.regs[0]);
    }
    #[test]
    fn subs_imm3_instruction() {
        let i = Armv6M::decode(0b0001111100000000);
        assert_eq!(
            Ok(Instruction::SubsImm3 {
                rd: Register::R0,
                rn: Register::R0,
                imm3: 4
            }),
            i
        );
        assert_eq!("SUBS R0,R0,#4", format!("{}", i.unwrap()));
    }
    #[test]
    fn subs_imm3_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[0] = 4;
        cpu.regs[1] = 0x12345678;
        cpu.execute(
            Instruction::SubsImm3 {
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
    fn subs_imm8_instruction() {
        let i = Armv6M::decode(0b0011110000001000);
        assert_eq!(
            Ok(Instruction::SubsImm8 {
                rdn: Register::R4,
                imm8: 8
            }),
            i
        );
        assert_eq!("SUBS R4,#8", format!("{}", i.unwrap()));
    }
    #[test]
    fn subs_imm8_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[1] = 0x12345678;
        cpu.execute(
            Instruction::SubsImm8 {
                rdn: Register::R1,
                imm8: 1,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x12345677, cpu.regs[1]);
        assert!(!cpu.is_n());
        assert!(!cpu.is_z());
        assert!(cpu.is_c());
        assert!(!cpu.is_v());
    }
    #[test]
    fn subs_reg_instruction() {
        let i = Armv6M::decode(0b0001101001000000);
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
    fn sbcs_reg_instruction() {
        let i = Armv6M::decode(0b0100000110001000);
        assert_eq!(
            Ok(Instruction::SbcsReg {
                rdn: Register::R0,
                rm: Register::R1,
            }),
            i
        );
        assert_eq!("SBCS R0,R1", format!("{}", i.unwrap()));
    }

    #[test]
    fn sbcs_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.set_c(false);
        cpu.regs[0] = 0x12345678;
        cpu.regs[2] = 1;
        cpu.execute(
            Instruction::SbcsReg {
                rdn: Register::R0,
                rm: Register::R2,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x12345676, cpu.regs[0]);
        assert!(!cpu.is_n());
        assert!(!cpu.is_z());
        assert!(cpu.is_c());
        assert!(!cpu.is_v());
    }
    #[test]
    fn rsb_imm_instruction() {
        let i = Armv6M::decode(0b0100001001000001);
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
        let i = Armv6M::decode(0b0000000010000001);
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
        let i = Armv6M::decode(0b0000000000000001);
        assert_eq!(
            Ok(Instruction::LslsImm {
                rd: Register::R1,
                rm: Register::R0,
                imm5: 0
            }),
            i
        );
        assert_eq!("MOVS R1,R0", format!("{}", i.unwrap()));
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
        let i = Armv6M::decode(0b0100000010011100);
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
        let i = Armv6M::decode(0b0000100010000001);
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
        let i = Armv6M::decode(0b0100000011011100);
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
    #[test]
    fn asrs_imm_instruction() {
        let i = Armv6M::decode(0b0001000010000001);
        assert_eq!(
            Ok(Instruction::AsrsImm {
                rd: Register::R1,
                rm: Register::R0,
                imm5: 2
            }),
            i
        );
        assert_eq!("ASRS R1,R0,#2", format!("{}", i.unwrap()));
    }

    #[test]
    fn asrs_imm_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram: [u32; 8] = [0; 8];
        cpu.regs[1] = 0xF1234567;
        cpu.execute(
            Instruction::AsrsImm {
                rd: Register::R0,
                rm: Register::R1,
                imm5: 4,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0xFF123456, cpu.regs[0]);
        assert!(cpu.is_n());
        assert!(!cpu.is_z());
        assert!(!cpu.is_c());
        assert!(!cpu.is_v());
    }

    #[test]
    fn muls_instruction() {
        let i = Armv6M::decode(0b0100001101011100);
        assert_eq!(
            Ok(Instruction::Muls {
                rdn: Register::R4,
                rm: Register::R3,
            }),
            i
        );
        assert_eq!("MULS R4,R3,R4", format!("{}", i.unwrap()));
    }
    #[test]
    fn muls_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram: [u32; 8] = [0; 8];
        cpu.regs[4] = 5;
        cpu.regs[3] = 4;
        cpu.execute(
            Instruction::Muls {
                rdn: Register::R4,
                rm: Register::R3,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(20, cpu.regs[4]);
        assert!(!cpu.is_n());
        assert!(!cpu.is_z());
    }
    #[test]
    fn uxth_instruction() {
        let i = Armv6M::decode(0b1011001010001001);
        assert_eq!(
            Ok(Instruction::Uxth {
                rd: Register::R1,
                rm: Register::R1,
            }),
            i
        );
        assert_eq!("UXTH R1,R1", format!("{}", i.unwrap()));
    }
    #[test]
    fn uxth_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram: [u32; 8] = [0; 8];
        cpu.regs[4] = 0x12345678;
        cpu.execute(
            Instruction::Uxth {
                rd: Register::R4,
                rm: Register::R4,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x5678, cpu.regs[4]);
    }
    #[test]
    fn uxtb_instruction() {
        let i = Armv6M::decode(0b1011001011001001);
        assert_eq!(
            Ok(Instruction::Uxtb {
                rd: Register::R1,
                rm: Register::R1,
            }),
            i
        );
        assert_eq!("UXTB R1,R1", format!("{}", i.unwrap()));
    }
    #[test]
    fn uxtb_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram: [u32; 8] = [0; 8];
        cpu.regs[4] = 0x12345678;
        cpu.execute(
            Instruction::Uxtb {
                rd: Register::R4,
                rm: Register::R4,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(0x78, cpu.regs[4]);
    }
    // =======================================================================
    // Logical Instructions
    // =======================================================================
    #[test]
    fn cmp_reg_instruction() {
        let i = Armv6M::decode(0b0100001010001000);
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
        let i = Armv6M::decode(0b0010100000000101);
        assert_eq!(
            Ok(Instruction::CmpImm {
                rn: Register::R0,
                imm8: 5,
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
                imm8: 50,
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
        let i = Armv6M::decode(0b0100000000011110);
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
        let i = Armv6M::decode(0b0100001100011110);
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

    #[test]
    fn eors_reg_instruction() {
        let i = Armv6M::decode(0b0100000001011110);
        assert_eq!(
            Ok(Instruction::EorsReg {
                rdn: Register::R6,
                rm: Register::R3,
            }),
            i
        );
        assert_eq!("EORS R6,R3", format!("{}", i.unwrap()));
    }
    #[test]
    fn eors_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[6] = 0x12345678;
        cpu.regs[3] = 0xFF0000FF;
        cpu.execute(
            Instruction::EorsReg {
                rdn: Register::R6,
                rm: Register::R3,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(cpu.regs[6], 0xED345687);
    }
    #[test]
    fn bics_reg_instruction() {
        let i = Armv6M::decode(0b0100001110101011);
        assert_eq!(
            Ok(Instruction::BicsReg {
                rdn: Register::R3,
                rm: Register::R5,
            }),
            i
        );
        assert_eq!("BICS R3,R5", format!("{}", i.unwrap()));
    }
    #[test]
    fn bics_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[6] = 0x12345678;
        cpu.regs[3] = 0xFF0000FF;
        cpu.execute(
            Instruction::BicsReg {
                rdn: Register::R6,
                rm: Register::R3,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(cpu.regs[6], 0x00345600);
    }
    #[test]
    fn mvns_reg_instruction() {
        let i = Armv6M::decode(0b0100001111101011);
        assert_eq!(
            Ok(Instruction::MvnsReg {
                rd: Register::R3,
                rm: Register::R5,
            }),
            i
        );
        assert_eq!("MVNS R3,R5", format!("{}", i.unwrap()));
    }
    #[test]
    fn mvns_reg_operation() {
        let sp = 16;
        let mut cpu = Armv6M::new(sp, 0);
        let mut ram = [0; 8];
        cpu.regs[6] = 0x12345678;
        cpu.execute(
            Instruction::MvnsReg {
                rd: Register::R3,
                rm: Register::R6,
            },
            &mut ram,
        )
        .unwrap();
        assert_eq!(cpu.regs[3], 0xedcba987);
    }
    // =======================================================================
    // Other Instructions
    // =======================================================================
    #[test]
    fn bkpt_instruction() {
        let i = Armv6M::decode(0b1011111011001100);
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
        let i = Armv6M::decode(0b1011011001110010);
        assert_eq!(Ok(Instruction::Cps { im: true }), i);
        assert_eq!("CPSID i", format!("{}", i.unwrap()));
    }

    #[test]
    fn cpsie_instruction() {
        let i = Armv6M::decode(0b1011011001100010);
        assert_eq!(Ok(Instruction::Cps { im: false }), i);
        assert_eq!("CPSIE i", format!("{}", i.unwrap()));
    }
}

// End of file
