use bitvec::{field::BitField, vec::BitVec};
use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
};

// 哥们给你一个大大的
const TRUE_BOOLS: [bool; 64] = [true; 64];

pub mod error {
    #[derive(Debug)]
    pub enum Error {
        Pin,
        State,
        IdCodeNotFound,
        TapNoValid,
        Other(String),
    }

    impl core::fmt::Display for Error {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            match self {
                Error::Pin => write!(f, "Pin Operation Error"),
                Error::State => write!(f, "Not Valid State"),
                Error::IdCodeNotFound => write!(f, "Not Valid IdCode"),
                Error::TapNoValid => write!(f, "Select Tap No Found"),
                Error::Other(s) => write!(f, "{}", s),
            }
        }
    }
    impl core::error::Error for Error {
        // TODO:
    }
}

pub struct JtagIo<I, O0, O1, O2, D> {
    tck: O0,
    tms: O1,
    tdi: O2,
    tdo: I,
    delay: D,
    speed: u32, // 为延时us
}

impl<I: InputPin, O0: OutputPin, O1: OutputPin, O2: OutputPin, D: DelayNs>
    JtagIo<I, O0, O1, O2, D>
{
    // 顺序为 TCK, TMS, TDI, TDO, Delay
    pub fn new(fileds: (O0, O1, O2, I, D)) -> Self {
        // 33 ns 为半个时钟, 66 ns 为一个时钟，大概是 15Mhz 的时钟频率
        Self {
            tck: fileds.0,
            tms: fileds.1,
            tdi: fileds.2,
            tdo: fileds.3,
            delay: fileds.4,
            speed: 33,
        }
    }

    /// 移位 1 比特的数据
    pub fn shift_bit(
        &mut self,
        tdi: bool,
        tms: bool,
        capture: Option<&mut bool>,
    ) -> Result<(), error::Error> {
        self.tck.set_low().map_err(|_| error::Error::Pin)?;
        self.tms
            .set_state(tms.into())
            .map_err(|_| error::Error::Pin)?;
        self.tdi
            .set_state(tdi.into())
            .map_err(|_| error::Error::Pin)?;
        self.delay.delay_ns(self.speed);
        self.tck.set_high().map_err(|_| error::Error::Pin)?;
        // 呃 需要测试
        self.delay.delay_ns(1);
        if let Some(cap) = capture {
            *cap = self.tdo.is_high().map_err(|_| error::Error::Pin)?;
        }
        self.delay.delay_ns(self.speed);
        Ok(())
    }
}

pub trait RawJtagIo {
    fn shift_bit(
        &mut self,
        tdi: bool,
        tms: bool,
        capture: Option<&mut bool>,
    ) -> Result<(), error::Error>;

    // 不会校验长度，调用者确保
    fn shift_bits(
        &mut self,
        tdi: &[bool],
        tms: &[bool],
        captures: Option<&mut [bool]>,
    ) -> Result<(), error::Error> {
        // 显然这会是双倍空间， 但是省去判断时间
        if let Some(values) = captures {
            for (i, (&tdi, &tms)) in tdi.iter().zip(tms).enumerate() {
                self.shift_bit(tdi, tms, Some(&mut values[i]))?;
            }
        } else {
            for (&tdi, &tms) in tdi.iter().zip(tms) {
                self.shift_bit(tdi, tms, None)?;
            }
        }
        Ok(())
    }
}

impl<I: InputPin, O0: OutputPin, O1: OutputPin, O2: OutputPin, D: DelayNs> RawJtagIo
    for JtagIo<I, O0, O1, O2, D>
{
    fn shift_bit(
        &mut self,
        tdi: bool,
        tms: bool,
        capture: Option<&mut bool>,
    ) -> Result<(), error::Error> {
        JtagIo::shift_bit(self, tdi, tms, capture)
    }
}

/// 这里只认 Idle, Shift 状态， 其他呃不想管
enum TapState {
    Unknown,
    Idle,
    Shift, // 不分 IR/DR
}

struct TapParams {
    pre: usize,
    pos: usize,
    select: usize,
    len: usize,
    taps: [u8; 32],
}

// 目前是没有命令缓存的，也就是及时行乐
pub struct JtagAdapter<J> {
    rawio: J,
    bits: BitVec,
    state: TapState,
    params: TapParams, // 你要同时控制 tap 吗
}

impl<J: RawJtagIo> JtagAdapter<J> {
    pub fn new(jtag_io: J) -> Self {
        Self {
            rawio: jtag_io,
            state: TapState::Unknown,
            bits: BitVec::new(),
            params: TapParams {
                pre: 0,
                pos: 0,
                select: 0,
                len: 0,
                taps: [0; 32],
            },
        }
    }

    fn reset_idle(&mut self) -> Result<(), error::Error> {
        let v = [true, true, true, true, true];
        // 此时进入 Reset
        self.rawio.shift_bits(&v, &v, None)?;
        // 进入 Idle
        self.rawio.shift_bit(true, false, None)?;
        self.state = TapState::Idle;
        Ok(())
    }

    fn idle_shift(&mut self, ir: bool) -> Result<(), error::Error> {
        if ir {
            self.rawio.shift_bit(true, true, None)?;
        }

        self.rawio
            .shift_bits(&[true, true, true], &[true, false, false], None)?;
        self.state = TapState::Shift;

        Ok(())
    }

    // 从 Exit1 状态返回 Idle
    fn exit1_idle(&mut self) -> Result<(), error::Error> {
        self.rawio.shift_bits(&[true, true], &[true, false], None)?;
        Ok(())
    }

    // 此方法，调用者确保在 Shift 状态使用
    fn sequence_bits(
        &mut self,
        tdi: &[bool],
        should_end: bool,
        captures: bool,
    ) -> Result<(), error::Error> {
        let mut value = false;
        let last = tdi.len() - 1;
        if captures {
            if !should_end {
                for &v in tdi {
                    self.rawio.shift_bit(v, false, Some(&mut value))?;
                    self.bits.push(value);
                }
            } else {
                for &v in tdi[..last].iter() {
                    self.rawio.shift_bit(v, false, Some(&mut value))?;
                    self.bits.push(value);
                }
                self.rawio.shift_bit(tdi[last], true, Some(&mut value))?;
                self.bits.push(value);
            }
        } else if !should_end {
            for &v in tdi {
                self.rawio.shift_bit(v, false, None)?;
            }
        } else {
            for &v in tdi[..last].iter() {
                self.rawio.shift_bit(v, false, None)?;
            }
            self.rawio.shift_bit(tdi[last], true, None)?;
        }
        Ok(())
    }

    fn read_capture_bits(&mut self) -> Result<BitVec, error::Error> {
        Ok(std::mem::take(&mut self.bits))
    }

    // 需要复位后直接进入 ShiftDR 操作，调用者确保在 ShidtDR 使用
    fn scan_idcode(&mut self) -> Result<u32, error::Error> {
        // 就是每次取一个 u32, 不是 -1 就行
        // 函数和代码需要重新设计一下才行，不然每次都需要在栈上压 [true; 32] ， 应该是 32 bytes,
        // 对于单片机还是吃力了, 能省则省
        self.sequence_bits(&[true; 32], false, true)?;
        let value = self.read_capture_bits()?;
        let value = value.load_le::<u32>();
        if value != 0xffff_ffff {
            Ok(value)
        } else {
            Err(error::Error::IdCodeNotFound)
        }
    }

    // 本质就是扫描 IR
    fn scan_tap(&mut self) -> Result<(), error::Error> {
        let mut pre = false;
        let mut pos = false;
        let mut index = 0;
        let mut status = 1;
        // 先获取第一个
        self.rawio.shift_bit(true, false, Some(&mut pre))?;

        while status > 0 {
            self.rawio.shift_bit(true, false, Some(&mut pos))?;
            if !pos {
                status += 1;
                pre = pos;
            } else if pre != pos {
                pre = pos;
                self.params.taps[index] = status;
                status = 1;
                index += 1;
            } else {
                status = 0
            }
        }
        self.params.len = index;
        Ok(())
    }

    // 选择 Tap
    pub fn select_tap(&mut self, index: usize) -> Result<(), error::Error> {
        if index >= self.params.len {
            Err(error::Error::TapNoValid)
        } else {
            self.params.select = index;
            let pre = self.params.taps[..index]
                .iter()
                .map(|x| *x as usize)
                .sum::<usize>();
            let pos = self.params.taps[index + 1..self.params.len]
                .iter()
                .map(|x| *x as usize)
                .sum::<usize>();
            self.params.pre = pre;
            self.params.pos = pos;
            Ok(())
        }
    }

    // 用于填充选定 tap 的 IR/DR 无用数据
    // 确保在 Shift 状态被调用
    // 如果是后填充，则默认会在最后退出 Shift
    fn shift_fill(&mut self, is_pre: bool, is_ir: bool) -> Result<(), error::Error> {
        // 提前退出
        let nums = if !is_ir {
            if is_pre {
                self.params.select
            } else {
                self.params.len - self.params.select - 1
            }
        } else if is_pre {
            self.params.pre
        } else {
            self.params.pos
        };

        // 没有就直接退出
        if nums != 0 {
            if !is_pre {
                // 后填充需要退出 Shift 状态
                self.sequence_bits(&TRUE_BOOLS[..nums], true, false)
            } else {
                self.sequence_bits(&TRUE_BOOLS[..nums], false, false)
            }
        } else {
            Ok(())
        }
    }

    // 向选中的 Tap 移入命令, 谁 IR 长度超过 8，IR你可以理解为 Op COde
    pub fn shift_ir(&mut self, value: u8, len: usize) -> Result<u8, error::Error> {
        // 有标准库就是好
        let mut tdi_bits = [false; 8];
        // 先来到 ShiftIR
        self.idle_shift(true)?;

        // 填充
        self.shift_fill(true, true)?;

        for (i, b) in tdi_bits.iter_mut().enumerate().take(len) {
            *b = value >> i & 1 == 1;
        }
        // 如果是最后一个，那么这边需要在移入最后一位后退出
        self.sequence_bits(&tdi_bits[..len], self.params.pos == 0, true)?;

        // 填充
        self.shift_fill(false, true)?;

        self.exit1_idle()?;

        let raw = self.read_capture_bits()?;
        let ir = raw.load_le::<u8>();

        Ok(ir)
    }

    pub fn shift_dr(&mut self, value: u64, len: usize) -> Result<u64, error::Error> {
        // 有标准库就是好
        let mut tdi_bits = [false; 64];
        // 先来到 ShiftDR
        self.idle_shift(false)?;

        // 填充
        self.shift_fill(true, false)?;

        for (i, b) in tdi_bits.iter_mut().enumerate().take(len) {
            *b = value >> i & 1 == 1;
        }

        // 如果是最后一个，那么这边需要在移入最后一位后退出
        self.sequence_bits(&tdi_bits[..len], self.params.pos == 0, true)?;

        // 填充
        self.shift_fill(false, false)?;

        self.exit1_idle()?;

        let raw = self.read_capture_bits()?;
        let dr = raw.load_le::<u64>();

        Ok(dr)
    }

    pub fn init(&mut self) -> Result<Vec<u32>, error::Error> {
        self.reset_idle()?;
        self.idle_shift(false)?;
        // 开始扫 IdCode, 确保是复位后的第一次操作 DR
        let mut idcodes = Vec::new();
        while let Ok(idcode) = self.scan_idcode() {
            idcodes.push(idcode);
        }

        // 退出 Shift, 返回 Idle
        self.rawio.shift_bit(true, true, None)?;
        self.exit1_idle()?;

        // 进入 IR
        self.idle_shift(true)?;

        // 扫 Tap 有多少个
        self.scan_tap()?;

        // 退出 Shift 并回到 Idle
        self.rawio.shift_bit(true, true, None)?;
        self.exit1_idle()?;

        // 这时候 IR 寄存器的值都为 1, 确保是复位后的第一次操作 IR

        Ok(idcodes)
    }
}

pub mod adi {
    pub mod v5 {
        pub const BYPASS: u8 = 0b1111;
        pub const IDCODE: u8 = 0b1110;
        pub const DPACC: u8 = 0b1010;
        pub const APACC: u8 = 0b1011;
        pub const ABORT: u8 = 0b1000;

        // Register
        pub const DP_CTRL_STAT: u8 = 0b0100;
        pub const DP_SELECT: u8 = 0b1000;
        pub const DP_RDBUFF: u8 = 0b1100;
    }
}
