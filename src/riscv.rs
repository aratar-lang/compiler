//! RISC-V 32-Bit Base Integer Instructions to compile into.

// NOTES
// - For speed: Make sure all data is aligned

use Reg::*;
use I::*;

/// A RISC-V Register
#[repr(u8)]
#[derive(Copy, Clone)]
pub(crate) enum Reg {
    /// Always 0
    ZERO = 0u8,
    /// Return address
    RA = 1,
    /// Stack pointer
    SP = 2,
    /// Global pointer
    GP = 3,
    /// Thread pointer
    TP = 4,
    /// Temporary
    T0 = 5,
    /// Temporary
    T1 = 6,
    /// Temporary
    T2 = 7,
    /// Saved (Frame Pointer)
    S0 = 8,
    /// Saved
    S1 = 9,
    /// Function arguments / Return values
    A0 = 10,
    /// Function arguments / Return values
    A1 = 11,
    /// Function arguments
    A2 = 12,
    /// Function arguments
    A3 = 13,
    /// Function arguments
    A4 = 14,
    /// Function arguments
    A5 = 15,
    /// Function arguments
    A6 = 16,
    /// Function arguments
    A7 = 17,
    /// Saved
    S2 = 18,
    /// Saved
    S3 = 19,
    /// Saved
    S4 = 20,
    /// Saved
    S5 = 21,
    /// Saved
    S6 = 22,
    /// Saved
    S7 = 23,
    /// Saved
    S8 = 24,
    /// Saved
    S9 = 25,
    /// Saved
    S10 = 26,
    /// Saved
    S11 = 27,
    /// Temporary
    T3 = 28,
    /// Temporary
    T4 = 29,
    /// Temporary
    T5 = 30,
    /// Temporary
    T6 = 31,
}

impl From<u32> for Reg {
    fn from(reg: u32) -> Self {
        match reg {
            0 => ZERO,
            1 => RA,
            2 => SP,
            3 => GP,
            4 => TP,
            5 => T0,
            6 => T1,
            7 => T2,
            8 => S0,
            9 => S1,
            10 => A0,
            11 => A1,
            12 => A2,
            13 => A3,
            14 => A4,
            15 => A5,
            16 => A6,
            17 => A7,
            18 => S2,
            19 => S3,
            20 => S4,
            21 => S5,
            22 => S6,
            23 => S7,
            24 => S8,
            25 => S9,
            26 => S10,
            27 => S11,
            28 => T3,
            29 => T4,
            30 => T5,
            31 => T6,
            _ => unreachable!(),
        }
    }
}

/// An assembly instruction (imm is limited to 12 bits)
pub(crate) enum I {
    /// Load Byte (R[d]: M[R[s] + imm])
    LB { d: Reg, s: Reg, imm: u16 },
    /// Load Half-Word (R[d]: M[R[s] + imm])
    LH { d: Reg, s: Reg, imm: u16 },
    /// Load Word (R[d]: M[R[s] + imm])
    LW { d: Reg, s: Reg, imm: u16 },
    /// Load Byte Unsigned (R[d]: M[R[s] + imm])
    LBU { d: Reg, s: Reg, imm: u16 },
    /// Load Half Unsigned (R[d]: M[R[s] + imm])
    LHU { d: Reg, s: Reg, imm: u16 },
    /// Add Immediate (R[d]: R[s] + imm)
    ADDI { d: Reg, s: Reg, imm: u16 },
    
    /// Add (R[d]: R[s1] + R[s2])
    ADD { d: Reg, s1: Reg, s2: Reg },
    /// Subtract (R[d]: R[s1] - R[s2])
    SUB { d: Reg, s1: Reg, s2: Reg },
}

impl I {
    /// - funct7: 7
    /// - src2:   5
    /// - src1:   5
    /// - funct3: 3
    /// - dst:    5
    /// - opcode: 7
    fn r(opcode: u32, d: Reg, funct3: u32, s1: Reg, s2: Reg, funct7: u32) -> u32
    {
        let dst: u32 = (d as u8).into();
        let src1: u32 = (s1 as u8).into();
        let src2: u32 = (s2 as u8).into();
        let mut out = opcode;
        out |= dst << 7;
        out |= funct3 << 12;
        out |= src1 << 15;
        out |= src2 << 20;
        out |= funct7 << 25;
        out
    }
    fn from_r(instruction: u32) -> (Reg, u32, Reg, Reg, u32) {
        let d = Reg::from((instruction & (0b11111 << 7)) >> 7);
        let funct3 = (instruction & (0b111 << 12)) >> 12;
        let s1 = Reg::from((instruction & (0b11111 << 15)) >> 15);
        let s2 = Reg::from((instruction & (0b11111 << 20)) >> 20);
        let funct7 = instruction >> 25;
        (d, funct3, s1, s2, funct7)
    }

    /// - imm:    12
    /// - src:    5
    /// - funct3: 3
    /// - dst:    5
    /// - opcode  7
    fn i(opcode: u32, d: Reg, funct3: u32, s: Reg, imm: u16) -> u32 {
        let imm: u32 = imm.into();
        let dst: u32 = (d as u8).into();
        let src: u32 = (s as u8).into();
        let mut out = opcode;
        out |= dst << 7;
        out |= funct3 << 12;
        out |= src << 15;
        out |= imm << 20;
        out
    }
    fn from_i(instruction: u32) -> (Reg, u32, Reg, u16) {
        let d = Reg::from((instruction & (0b11111 << 7)) >> 7);
        let funct3 = (instruction & (0b111 << 12)) >> 12;
        let s = Reg::from((instruction & (0b11111 << 15)) >> 15);
        let imm = (instruction >> 20) as u16;
        (d, funct3, s, imm)
    }
}

impl From<I> for u32 {
    fn from(with: I) -> Self {
        match with {
            LB { d, s, imm } => I::i(0b0000011, d, 0b000, s, imm),
            LH { d, s, imm } => I::i(0b0000011, d, 0b001, s, imm),
            LW { d, s, imm } => I::i(0b0000011, d, 0b010, s, imm),
            LBU { d, s, imm } => I::i(0b0000011, d, 0b100, s, imm),
            LHU { d, s, imm } => I::i(0b0000011, d, 0b101, s, imm),
            ADDI { d, s, imm } => I::i(0b0010011, d, 0b000, s, imm),
            ADD { d, s1, s2 } => I::r(0b0110011, d, 0b000, s1, s2, 0b0000000),
            SUB { d, s1, s2 } => I::r(0b0110011, d, 0b000, s1, s2, 0b0100000),
        }
    }
}

impl From<u32> for I {
    fn from(with: u32) -> Self {
        match with & 0b1111111 {
            // Load From RAM
            0b0000011 => match I::from_i(with) {
                (d, 0b000, s, imm) => LB { d, s, imm },
                (d, 0b001, s, imm) => LH { d, s, imm },
                (d, 0b010, s, imm) => LW { d, s, imm },
                (d, 0b100, s, imm) => LBU { d, s, imm },
                (d, 0b101, s, imm) => LHU { d, s, imm },
                (_, funct, _, _mm) => panic!("Unknown funct3: {}", funct),
            },
            // Immediate Arithmetic
            0b0010011 => match I::from_i(with) {
                (d, 0b000, s, imm) => ADDI { d, s, imm },
                (_, funct, _, _mm) => panic!("Unknown funct3: {}", funct),
            },
            // Register Arithmetic
            0b0110011 => match I::from_r(with) {
                (d, 0b000, s1, s2, 0b0000000) => ADD { d, s1, s2 },
                (d, 0b000, s1, s2, 0b0000001) => SUB { d, s1, s2 },
                (_, f3, _1, _2, f7) => panic!("Unknown F3:{} F7:{}", f3, f7),
            },
            o => panic!("Failed to parse RISC-V Assembly, Unknown Opcode {}", o),
        }
    }
}
