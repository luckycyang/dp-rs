use bitvec::{field::BitField, vec::BitVec};
use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
};

pub mod error {
    pub enum Error {
        Pin,
        State,
        Other(String),
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

// 目前是没有命令缓存的，也就是及时行乐
pub struct JtagAdapter<J> {
    rawio: J,
    bits: BitVec,
    state: TapState,
}

impl<J: RawJtagIo> JtagAdapter<J> {
    pub fn new(jtag_io: J) -> Self {
        Self {
            rawio: jtag_io,
            state: TapState::Unknown,
            bits: BitVec::new(),
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
    fn sequence_bits(&mut self, tdi: &[bool], captures: bool) -> Result<(), error::Error> {
        let mut value = false;
        if captures {
            for &v in tdi {
                self.rawio.shift_bit(v, false, Some(&mut value))?;
                self.bits.push(value);
            }
        } else {
            for &v in tdi {
                self.rawio.shift_bit(v, false, None)?;
            }
        }
        Ok(())
    }

    fn read_capture_bits(&mut self) -> Result<BitVec, error::Error> {
        Ok(std::mem::take(&mut self.bits))
    }

    pub fn init(&mut self) -> Result<u64, error::Error> {
        // TODO: 作用是扫描 IDCODE 返回，然后保存 Tap 信息
        self.reset_idle()?;
        self.idle_shift(false)?;
        self.sequence_bits(&[true; 64], true)?;
        // 拿取读到的数据
        let bits = self.read_capture_bits()?;
        let value = bits.load_le::<u64>();

        // 退出 Shift, 返回 Idle
        self.rawio.shift_bit(true, true, None)?;
        self.exit1_idle()?;

        Ok(value)
    }
}
