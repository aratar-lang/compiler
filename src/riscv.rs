//! RISC-V 32-Bit Base Integer Instructions to compile into, with optional
//! extensions that can be enabled.
//! - RV32M Multiply Extension
//! - RV32A Atomic Extension
//! - RV32F Single-Precision Floating Point Extension
//! - RV32D Double-Precision Floating Point Extension
//! -
//! # Ignored For Now
//! - RV32Q Quadruple-Precision Floating Point Extension
//! - RV32C Compression 16-bit Instructions
//! - All 64 bit extensions
//!
//! May be ported to other platforms with assembly translators.
//!
//! # Syscalls
//! Syscalls will need to be translated into other code in order to run outside
//! of the virtual machine.
//!
//! Registers Used:
//! - a0: (in: Syscall Code, out: evt EventType | buf Addr)
//! - a1: (in: len Offs, out: len Int32U | res (Int16U, Int16U))
//! - a2: (in: dat Addr, out: dur Nanos32U)
//!
//! ### Core syscalls
//! ```
//! a0: (
//!     syscall_id: Hex8 (behavior: Hex4, variant: Hex4),
//!     uses_a0: Bin1
//!     uses_a1: Bin1
//!     uses_a2: Bin1
//!     uses_a3: Bin1
//!     __reserved: Bin4
//!     value: Bin16 # limited to 12 bits for address offsets
//! )
//! ```
//! - x0: Blocking [Input] (only one)
//!   - x0: def wait() -> (a0: EventType)
//! - x1: Non-Blocking [Output]
//!   - x0: def info(a1: size, a2: text) -> ()
//! - x2: Async [Output]
//!   - x0: def rand() -> () # EventType.Random: x20
//!   - x1: def adc(a1: analog_storage_id) -> () # EventType.Adc: x21
//!   - x2: def task(a0~fn_ofs: Addr16S) -> () # EventType.Task: x22
//! - x3: Mapping [Input/Ouput]
//!   - x0: def pixarray() -> (
//!         a0~pixels: Addr, a1~res: (w: Int16U, h: Int16U), a2~dur: Nanos32U
//!     )
//!   - x1: def speaker() -> (
//!         a0~samples: Addr, a1~res: Int32U, a2~dur: Nanos32U
//!     )
//!   - x2: def digital_is_out() -> (a0~buf: Addr[Bit], a1~len: Int32U)
//!   - x3: def analog_is_out() -> (a0~buf: Addr[Bit], a1~len: Int32U)
//!   - x4: def camera() -> (
//!         a0~pixels: Addr, a1~res: (w: Int16U, h: Int16U), a2~dur: Nanos32U
//!     )
//!   - x5: def microphone() -> (a0: Addr, a1~len: Int32U, a2~dur: Nanos32U)
//!   - x8: def digital_gpio() -> (a0~buf: Addr[Bit], a1~len: Int32U)
//!   - x9: def analog_gpio() -> (a0~buf: Addr[Fix1_31], a1~len: Int32U)
//!
//! ## EventType: Bin32
//! - UserInput: x0
//!   - Keyboard: x0 (lang: [Ascii; 2])
//!     - Quit(Escape)/Tilde/NotSign/Power: 0x00
//!     - One/Bang/InvertedBang/LatinMode: 0x01
//!     - Two/At/Squared/ChineseCangjieMode: 0x02
//!     - Three/Pound/Pound/JapaneseOyayubiShifutoMode: 0x03
//!     - Four/Dollar/CurrencySign/KoreanDubeolsikMode: 0x04
//!     - Five/Percent/Euro/GeorgianPhoneticMode: 0x05
//!     - Six/Caret/Trademark/GreekCodepointMode: 0x06
//!     - Seven/Ampersand/Cent/HebrewArabicCyrillicMode: 0x07
//!     - Eight/Asterisk/Ruble/ArmenianPhoneticMode: 0x08
//!     - Nine/LParens/Yen/TamazightMode: 0x09
//!     - Zero/RParens/LaoMode: 0x0A
//!     - Minus/Underscore: 0x0B
//!     - Plus/Equal: 0x0C
//!     - Backspace/Delete: 0x0D
//!     - Tab/UnTab: 0x10
//!     - q/Q: 0x11
//!     - w/W: 0x12
//!     - e/E: 0x13
//!     - r/R: 0x14
//!     - t/T: 0x15
//!     - y/Y: 0x16
//!     - u/U: 0x17
//!     - i/I: 0x18
//!     - o/O: 0x19
//!     - p/P: 0x1A
//!     - LSquare/LBrace: 0x1B
//!     - LSquare/LBrace: 0x1C
//!     - Backslash/Bar: 0x1D
//!     - Compose/Search: 0x20
//!     - a/A: 0x21
//!     - s/S: 0x22
//!     - d/D: 0x23
//!     - f/F: 0x24
//!     - g/G: 0x25
//!     - h/H: 0x26
//!     - j/J: 0x27
//!     - k/K: 0x28
//!     - l/L: 0x29
//!     - Semicolon/Colon: 0x2A
//!     - Apostrophe/Quote: 0x2B
//!     - Enter/UnEnter: 0x2C
//!     - LShift: 0x30
//!     - z/Z: 0x32
//!     - x/X: 0x33
//!     - c/C: 0x34
//!     - v/V: 0x35
//!     - b/B: 0x36
//!     - n/N: 0x37
//!     - m/M: 0x38
//!     - Comma/LessThan: 0x39
//!     - Period/MoreThan: 0x3A
//!     - Slash/Question: 0x3B
//!     - ArrowUp: 0x3C
//!     - Shift: 0x3D
//!     - Control: 0x40
//!     - Clipboard: 0x41
//!     - System: 0x42
//!     - Alt: 0x43
//!     - Space: 0x45
//!     - AltGraph: 0x48
//!     - VolumeDown: 0x49
//!     - VolumeUp: 0x4A
//!     - ArrowLeft: 0x4B
//!     - ArrowDown: 0x4C
//!     - ArrowRight: 0x4D
//!     - TextInput: 0x70-0x7F (utf8: [Byte; 2])
//!     - \[ReleaseKey\]: 0x8_-0xB_
//!     - TextInputLong: 0xFF (pre_utf32: [Byte; 2])
//!   - Mouse: x1
//!     - ButtonLeft: 0x00 (Bool)
//!     - ButtonMiddle: 0x01 (Bool)
//!     - ButtonRight: 0x02 (Bool)
//!     - ButtonSide: 0x03 (Bool)
//!     - ButtonDpi: 0x04 (Bool)
//!     - MoveX: 0x80 (Int16U)
//!     - MoveY: 0x81 (Int16U)
//!     - ScrollX: 0x90 (Int16U)
//!     - ScrollY: 0x91 (Int16U)
//!   - Touchscreen: x2
//!     - TouchX: 0x00 (Int4U, Int16U)
//!     - TouchY: 0x01 (Int4U, Int16U)
//!     - Release: 0x0F ()
//!   - Touchpad: x3
//!     - ButtonLeft: 0x00 (Bool)
//!     - ButtonMiddle: 0x01 (Bool)
//!     - ButtonRight: 0x02 (Bool)
//!     - MoveX: 0x80 (Int16U)
//!     - MoveY: 0x81 (Int16U)
//!     - ScrollX: 0x90 (Int16U)
//!     - ScrollY: 0x91 (Int16U)
//!   - Gamepad: x4
//!     - ButtonLTrigger: 0x00 (Bool)
//!     - ButtonRTrigger 0x01 (Bool)
//!     - ButtonLBumper: 0x02 (Bool)
//!     - ButtonRBumper: 0x03 (Bool)
//!     - ButtonLStick: 0x04 (Bool)
//!     - ButtonRStick: 0x05 (Bool)
//!     - ButtonLMenu: 0x06 (Bool)
//!     - ButtonRMenu: 0x07 (Bool)
//!     - ButtonDpadUp: 0x08 (Bool)
//!     - ButtonDpadLeft: 0x09 (Bool)
//!     - ButtonDpadRight: 0x0A (Bool)
//!     - ButtonDpadDown: 0x0B (Bool)
//!     - ButtonV: 0x0C (Bool)
//!     - ButtonA: 0x0D (Bool)
//!     - ButtonB: 0x0E (Bool)
//!     - ButtonH: 0x0F (Bool)
//!     - ButtonPaddleL: 0x10 (Bool)
//!     - ButtonPaddleR: 0x11 (Bool)
//!     - AxisMainStickX: 0x80 (Int16U)
//!     - AxisMainStickY: 0x81 (Int16U)
//!     - AxisAltStickX: 0x82 (Int16U)
//!     - AxisAltStickY: 0x83 (Int16U)
//!     - AxisLTrigger: 0x84 (Int16U)
//!     - AxisRTrigger: 0x85 (Int16U)
//! - Custom Circuit Input: x1
//!   - Gpio: x0
//!     - Digital: x00
//!     - Analog: x01
//!     - Pwm: x02
//!   - Bus: x1
//!     - I2c: x00
//!     - Spi: x01
//!     - Can: x02
//!     - Pci: x03
//!     - Sada: xFF
//! - Async Ready: x2
//!   - RandomReady: x0
//!   - AdcReady: x1
//!   - TaskReady: x2
//! - Mapped (Media) Input: x3
//!   - Camera: x0
//!     - Capture: x00
//!   - Microphone: x1
//!     - Capture: x00
//! - Defined Ciruit Input: x4
//!   - TimerTick: x0
//!   - GPS: x1
//!   - Accelerometer: x2
//!   - Gyro: x3
//!   - StorageDrive: x4
//!   - BatterySensor: xF
//! - Network Input: x7
//!   - WifiEthernetData: x0
//!   - Bluetooth: x1
//!   - Cell: x2
//!     - Text: x00
//!     - Call: x01
//!     - TextMedia: x02
//!   - Ble: x3
//! - Media Output: xB
//!   - ScreenRefresh: x0
//!   - SpeakerRefresh: x1

// NOTES
// - For speed: Make sure all loads and stores are 32-bit aligned
// - For shifts: Shifting the width of the register is a no-op, to clear
//   register $R use ADDI $R, $ZERO, 0.
// Psuedo-Instructions
// - nop: addi $zero, $zero, 0
// - mv $d, $s: addi $d, $s, 0
// - not $d, $s: ori $d, $s, -1
// - neg $d, $s: sub $d, $zero, $s
// - j offset: jal $zero, offset (unconditional jump)
// - jal offset: jal $ra, offset (near function call)
// - call offset: (far function call)
//      auipc $ra, offset[31:12] + offset[11]
//      jalr $ra, offset[11:0]($ra)
// - ret: jalr $ra, 0($ra)
// - beqz $r, offset: beq $r, $zero, offset
// - bnez $r, offset: bne $r, $zero, offset
// - bgez $r, offset: bge $r, $zero, offset
// - bltz $r, offset: blt $r, $zero, offset
// - bgt $r1, $r2, offset: blt $r2, $r1, offset
// - ble $r1, $r2, offset: bge $r2, $r1, offset
// - fence: fence IORW, IORW
// - li $d, imm: addi $d, $zero, imm (set immediate)
// - li $d, imm:
//      lui $d, imm[31:12] + imm[11]
//      addi $d, $zero, imm[11:0]
// - la $d, symbol:
//      auipc $d, delta[31:12] + delta[11]
//      addi $d, $d, delta[11:0]
// - lw $d, symbol:
//      auipc $d, delta[31:12] + delta[11]
//      lw $d, $d, delta[11:0](rd)
// - sw $d, symbol, $t:
//      auipc $t, delta[31:12] + delta[11]
//      sw $d, $d, delta[11:0]($t)
// - seqz $d, $s: sltiu $d, $s, 0
// - snez $d, $s: sltu $d, $zero, $s
// Custom Pseudo-Instructions
// - zero $d: addi $r, $zero, 0 (set register to zero)
// - slt $d, $a, $b, $s: (with multiply cpu feature enabled)
//      # Option 1:
//      slt $d, $a, $b
//      mul $d, $d, $s
// - slt $d, $a, $b, $s: (without multiply cpu feature enabled)
//      # Option 1 (branching, ugh):
//      blt $a, $b, 12 # 12: ④
//      addi $d, $zero, 0
//      jal $zero 8 # Skip next instruction
//      add $d, $zero, $s # ④
//
//      # Option 2 (might be faster for without AND with multiply extension):
//      slt $d, $a, $b
//      slli $d, $d, 31
//      srai $d, $d, 31
//      and $d, $d, $s
// - slt $d, $a, $b, $s, $e: ($d: if $a < $b { $s } else { $e })
//      sub $s, $s, $e
//      slt $d, $a, $b, $s
//      add $d, $d, $e
//      add $s, $s, $e # can be elimated if s is dropped
// - slt $d, $a, $b, $s, $e, $t: ($d: if $a < $b { $s } else { $e })
//      sub $t, $s, $e
//      slt $d, $a, $b, $t
//      add $d, $d, $e

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
#[allow(clippy::enum_variant_names)]
pub(crate) enum I {
    //// One of 40 User mode instructions in the RV32I Base Instruction Set ////
    /// U: Set upper 20 bits to immediate value
    LUI { d: Reg, imm: u32 },
    /// U: Add upper 20 bits to immediate value in program counter
    AUIPC { d: Reg, imm: u32 },
    /// UJ: Jump and Link
    JAL { d: Reg, imm: u32 },
    /// I: Jump and Link, Register
    JALR { d: Reg, s: Reg, imm: u16 },
    /// SB: 12-bit immediate offset Branch on Equal
    BEQ { s1: Reg, s2: Reg, imm: u16 },
    /// SB: 12-bit immediate offset Branch on Not Equal
    BNE { s1: Reg, s2: Reg, imm: u16 },
    /// SB: 12-bit immediate offset Branch on Less Than
    BLT { s1: Reg, s2: Reg, imm: u16 },
    /// SB: 12-bit immediate offset Branch on Greater Than Or Equal To
    BGE { s1: Reg, s2: Reg, imm: u16 },
    /// SB: 12-bit immediate offset Branch on Less Than (Unsigned)
    BLTU { s1: Reg, s2: Reg, imm: u16 },
    /// SB: 12-bit immediate offset Branch on Greater Than Or Equal To (Unsigned)
    BGEU { s1: Reg, s2: Reg, imm: u16 },
    /// I: Load Byte (R[d]: M[R[s] + imm])
    LB { d: Reg, s: Reg, imm: u16 },
    /// I: Load Half-Word (R[d]: M[R[s] + imm])
    LH { d: Reg, s: Reg, imm: u16 },
    /// I: Load Word (R[d]: M[R[s] + imm])
    LW { d: Reg, s: Reg, imm: u16 },
    /// I: Load Byte Unsigned (R[d]: M[R[s] + imm])
    LBU { d: Reg, s: Reg, imm: u16 },
    /// I: Load Half Unsigned (R[d]: M[R[s] + imm])
    LHU { d: Reg, s: Reg, imm: u16 },
    /// S: Store Byte
    SB { s1: Reg, s2: Reg, imm: u16 },
    /// S: Store Half Word
    SH { s1: Reg, s2: Reg, imm: u16 },
    /// S: Store Word
    SW { s1: Reg, s2: Reg, imm: u16 },
    /// I: Add Immediate (R[d]: R[s] + imm)
    ADDI { d: Reg, s: Reg, imm: u16 },
    /// I: Set 1 on Less Than, 0 Otherwise Immediate
    SLTI { d: Reg, s: Reg, imm: u16 },
    /// I: Set 1 on Less Than, 0 Otherwise Immediate Unsigned
    SLTUI { d: Reg, s: Reg, imm: u16 },
    /// I: Xor Immediate
    XORI { d: Reg, s: Reg, imm: u16 },
    /// I: Or Immediate
    ORI { d: Reg, s: Reg, imm: u16 },
    /// I: And Immediate
    ANDI { d: Reg, s: Reg, imm: u16 },
    /// I: Logical Left Shift Immediate
    SLLI { d: Reg, s: Reg, imm: u8 },
    /// I: Logical Right Shift Immediate
    SRLI { d: Reg, s: Reg, imm: u8 },
    /// I: Arithmetic Shift Right Immediate (See SRA).
    SRAI { d: Reg, s: Reg, imm: u8 },
    /// R: Add (R[d]: R[s1] + R[s2])
    ADD { d: Reg, s1: Reg, s2: Reg },
    /// R: Subtract (R[d]: R[s1] - R[s2])
    SUB { d: Reg, s1: Reg, s2: Reg },
    /// R: Logical Left Shift
    SLL { d: Reg, s1: Reg, s2: Reg },
    /// R: Set 1 on Less Than, 0 Otherwise
    SLT { d: Reg, s1: Reg, s2: Reg },
    /// R: Set 1 on Less Than, 0 Otherwise Unsigned
    SLTU { d: Reg, s1: Reg, s2: Reg },
    /// R: Xor
    XOR { d: Reg, s1: Reg, s2: Reg },
    /// R: Logical Right Shift
    SRL { d: Reg, s1: Reg, s2: Reg },
    /// R: Arithmetic Shift Right (Sign Bit Copied Rather Than Filling In Zeros)
    SRA { d: Reg, s1: Reg, s2: Reg },
    /// R: Or
    OR { d: Reg, s1: Reg, s2: Reg },
    /// R: And
    AND { d: Reg, s1: Reg, s2: Reg },
    /// I: Invoke a system call (Registers defined by ABI, not hardware)
    ECALL {},
    /// I: Debugger Breakpoint
    EBREAK {},
    /// I: Fence (Immediate Is Made Up Of Ordered High Order To Low Order Bits:)
    /// - fm(4), PI(1), PO(1), PR(1), PW(1), SI(1), SO(1), SR(1), SW(1)
    FENCE { imm: u16 },
    //// Multiply Extension ////

    //// Atomic Extension ////

    //// Single-Precision Floating Point Extension ////

    //// Double-Precision Floating Point Extension ////

    //// Vector Extension ///

    //// SIMD Extension ////
}

impl I {
    /// - funct7: 7
    /// - src2:   5
    /// - src1:   5
    /// - funct3: 3
    /// - dst:    5
    /// - opcode: 7
    fn r(opcode: u32, d: Reg, funct3: u32, s1: Reg, s2: Reg, funct7: u32) -> u32 {
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

    /// - funct7: 7
    /// - imm:    5
    /// - src:    5
    /// - funct3: 3
    /// - dst:    5
    /// - opcode  7
    fn i7(opcode: u32, d: Reg, funct3: u32, s: Reg, imm: u8, funct7: u32) -> u32 {
        let imm: u32 = imm.into();
        let dst: u32 = (d as u8).into();
        let src: u32 = (s as u8).into();
        let mut out = opcode;
        out |= dst << 7;
        out |= funct3 << 12;
        out |= src << 15;
        out |= imm << 20;
        out |= funct7 << 25;
        out
    }
    fn from_i7(instruction: u32) -> (Reg, u32, Reg, u8, u32) {
        let d = Reg::from((instruction & (0b11111 << 7)) >> 7);
        let funct3 = (instruction & (0b111 << 12)) >> 12;
        let s = Reg::from((instruction & (0b11111 << 15)) >> 15);
        let imm = ((instruction & (0b11111 << 20)) >> 20) as u8;
        let funct7 = instruction >> 25;
        (d, funct3, s, imm, funct7)
    }

    /// - imm_h:  7
    /// - src2:   5
    /// - src1:   5
    /// - funct3: 3
    /// - imm_l:  5
    /// - opcode  7
    fn s(opcode: u32, funct3: u32, s1: Reg, s2: Reg, imm: u16) -> u32 {
        let imm: u32 = imm.into();
        let src1: u32 = (s1 as u8).into();
        let src2: u32 = (s2 as u8).into();
        let mut out = opcode;
        out |= (imm & 0b11111) << 7;
        out |= funct3 << 12;
        out |= src1 << 15;
        out |= src2 << 20;
        out |= (imm >> 5) << 25;
        out
    }
    fn from_s(instruction: u32) -> (u32, Reg, Reg, u16) {
        let mut imm = ((instruction & (0b11111 << 7)) >> 7) as u16;
        let funct3 = (instruction & (0b111 << 12)) >> 12;
        let s1 = Reg::from((instruction & (0b11111 << 15)) >> 15);
        let s2 = Reg::from((instruction & (0b11111 << 20)) >> 20);
        imm |= ((instruction >> 25) as u16) << 5;
        (funct3, s1, s2, imm)
    }

    /// - imm:    20
    /// - dst:    5
    /// - opcode  7
    fn u(opcode: u32, d: Reg, imm: u32) -> u32 {
        let dst: u32 = (d as u8).into();
        let mut out = opcode;
        out |= dst << 7;
        out |= imm << 12;
        out
    }
    fn from_u(instruction: u32) -> (Reg, u32) {
        let d = Reg::from((instruction & (0b11111 << 7)) >> 7);
        let imm = instruction >> 12;
        (d, imm)
    }
}

impl From<I> for u32 {
    fn from(with: I) -> Self {
        match with {
            LUI { d, imm } => I::u(0b0110111, d, imm),
            AUIPC { d, imm } => I::u(0b0010111, d, imm),
            JAL { d, imm } => I::u(0b1101111, d, imm),
            JALR { d, s, imm } => I::i(0b1100111, d, 0b000, s, imm),
            BEQ { s1, s2, imm } => I::s(0b1100011, 0b000, s1, s2, imm),
            BNE { s1, s2, imm } => I::s(0b1100011, 0b001, s1, s2, imm),
            BLT { s1, s2, imm } => I::s(0b1100011, 0b100, s1, s2, imm),
            BGE { s1, s2, imm } => I::s(0b1100011, 0b101, s1, s2, imm),
            BLTU { s1, s2, imm } => I::s(0b1100011, 0b110, s1, s2, imm),
            BGEU { s1, s2, imm } => I::s(0b1100011, 0b111, s1, s2, imm),
            LB { d, s, imm } => I::i(0b0000011, d, 0b000, s, imm),
            LH { d, s, imm } => I::i(0b0000011, d, 0b001, s, imm),
            LW { d, s, imm } => I::i(0b0000011, d, 0b010, s, imm),
            LBU { d, s, imm } => I::i(0b0000011, d, 0b100, s, imm),
            LHU { d, s, imm } => I::i(0b0000011, d, 0b101, s, imm),
            ADDI { d, s, imm } => I::i(0b0010011, d, 0b000, s, imm),
            SLTI { d, s, imm } => I::i(0b0010011, d, 0b010, s, imm),
            SLTUI { d, s, imm } => I::i(0b0010011, d, 0b011, s, imm),
            XORI { d, s, imm } => I::i(0b0010011, d, 0b100, s, imm),
            ORI { d, s, imm } => I::i(0b0010011, d, 0b110, s, imm),
            ANDI { d, s, imm } => I::i(0b0010011, d, 0b111, s, imm),
            SLLI { d, s, imm } => I::i7(0b0010011, d, 0b001, s, imm, 0b0000000),
            SRLI { d, s, imm } => I::i7(0b0010011, d, 0b101, s, imm, 0b0000000),
            SRAI { d, s, imm } => I::i7(0b0010011, d, 0b101, s, imm, 0b0100000),
            SB { s1, s2, imm } => I::s(0b0100011, 0b000, s1, s2, imm),
            SH { s1, s2, imm } => I::s(0b0100011, 0b001, s1, s2, imm),
            SW { s1, s2, imm } => I::s(0b0100011, 0b010, s1, s2, imm),
            ADD { d, s1, s2 } => I::r(0b0110011, d, 0b000, s1, s2, 0b0000000),
            SUB { d, s1, s2 } => I::r(0b0110011, d, 0b000, s1, s2, 0b0100000),
            SLL { d, s1, s2 } => I::r(0b0110011, d, 0b001, s1, s2, 0b0000000),
            SLT { d, s1, s2 } => I::r(0b0110011, d, 0b010, s1, s2, 0b0000000),
            SLTU { d, s1, s2 } => I::r(0b0110011, d, 0b011, s1, s2, 0b0000000),
            XOR { d, s1, s2 } => I::r(0b0110011, d, 0b100, s1, s2, 0b0000000),
            SRL { d, s1, s2 } => I::r(0b0110011, d, 0b101, s1, s2, 0b0000000),
            SRA { d, s1, s2 } => I::r(0b0110011, d, 0b101, s1, s2, 0b0100000),
            OR { d, s1, s2 } => I::r(0b0110011, d, 0b110, s1, s2, 0b0000000),
            AND { d, s1, s2 } => I::r(0b0110011, d, 0b111, s1, s2, 0b0000000),
            ECALL {} => I::i(0b1110011, ZERO, 0b000, ZERO, 0b000000000000),
            EBREAK {} => I::i(0b1110011, ZERO, 0b000, ZERO, 0b000000000001),
            FENCE { imm } => I::i(0b0001111, ZERO, 0b000, ZERO, imm),
        }
    }
}

impl From<u32> for I {
    // Using match makes it easier to extend code in the future.
    #[allow(clippy::match_single_binding)]
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
            // Misc. Memory Instructions
            0b0001111 => match I::from_i(with) {
                (_, 0b000, _, imm) => FENCE { imm },
                (_, funct, _, _mm) => panic!("Unknown funct3: {}", funct),
            },
            // Store To RAM
            0b0100011 => match I::from_s(with) {
                (0b000, s1, s2, imm) => SB { s1, s2, imm },
                (0b001, s1, s2, imm) => SH { s1, s2, imm },
                (0b010, s1, s2, imm) => SW { s1, s2, imm },
                (funct, _s, _z, _mm) => panic!("Unknown funct3: {}", funct),
            },
            // Immediate Arithmetic
            0b0010011 => match I::from_i(with) {
                (d, 0b000, s, imm) => ADDI { d, s, imm },
                (d, 0b010, s, imm) => SLTI { d, s, imm },
                (d, 0b011, s, imm) => SLTUI { d, s, imm },
                (d, 0b100, s, imm) => XORI { d, s, imm },
                (d, 0b110, s, imm) => ORI { d, s, imm },
                (d, 0b111, s, imm) => ANDI { d, s, imm },
                _ => match I::from_i7(with) {
                    (d, 0b001, s, imm, 0b0000000) => SLLI { d, s, imm },
                    (d, 0b101, s, imm, 0b0000000) => SRLI { d, s, imm },
                    (d, 0b101, s, imm, 0b0100000) => SRAI { d, s, imm },
                    (_, funct, _, _, _) => panic!("Unknown funct3: {}", funct),
                },
            },
            // Add Upper Immediate To Program Counter
            0b0010111 => match I::from_u(with) {
                (d, imm) => AUIPC { d, imm },
            },
            // Register Arithmetic
            0b0110011 => match I::from_r(with) {
                (d, 0b000, s1, s2, 0b0000000) => ADD { d, s1, s2 },
                (d, 0b000, s1, s2, 0b0100000) => SUB { d, s1, s2 },
                (d, 0b001, s1, s2, 0b0000000) => SLL { d, s1, s2 },
                (d, 0b010, s1, s2, 0b0000000) => SLT { d, s1, s2 },
                (d, 0b011, s1, s2, 0b0000000) => SLTU { d, s1, s2 },
                (d, 0b100, s1, s2, 0b0000000) => XOR { d, s1, s2 },
                (d, 0b101, s1, s2, 0b0000000) => SRL { d, s1, s2 },
                (d, 0b101, s1, s2, 0b0100000) => SRA { d, s1, s2 },
                (d, 0b110, s1, s2, 0b0000000) => OR { d, s1, s2 },
                (d, 0b111, s1, s2, 0b0000000) => AND { d, s1, s2 },
                (_, f3, _s, _z, f7) => panic!("Unknown F3:{} F7:{}", f3, f7),
            },
            // Load upper immediate
            0b0110111 => match I::from_u(with) {
                (d, imm) => LUI { d, imm },
            },
            // Branch on Condition
            0b1100011 => match I::from_s(with) {
                (0b000, s1, s2, imm) => BEQ { s1, s2, imm },
                (0b001, s1, s2, imm) => BNE { s1, s2, imm },
                (0b100, s1, s2, imm) => BLT { s1, s2, imm },
                (0b101, s1, s2, imm) => BGE { s1, s2, imm },
                (0b110, s1, s2, imm) => BLTU { s1, s2, imm },
                (0b111, s1, s2, imm) => BGEU { s1, s2, imm },
                (funct, _s, _z, _mm) => panic!("Unknown funct3: {}", funct),
            },
            // Jump and link register
            0b1100111 => match I::from_i(with) {
                (d, 0b000, s, imm) => JALR { d, s, imm },
                (_d, f3, _s, _imm) => panic!("Unknown F3:{}", f3),
            },
            // Jump and Link
            0b1101111 => match I::from_u(with) {
                (d, imm) => JAL { d, imm },
            },
            // Transfer Control
            0b1110011 => match I::from_i(with) {
                (ZERO, 0b000, ZERO, 0b000000000000) => ECALL {},
                (ZERO, 0b000, ZERO, 0b000000000001) => EBREAK {},
                _ => panic!("Unknown Environment Control Transfer"),
            },
            o => panic!("Failed to parse RISC-V Assembly, Unknown Opcode {}", o),
        }
    }
}
