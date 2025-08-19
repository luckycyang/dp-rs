use crate::error;



/// Swd 的位控制
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum IoSequenceItem {
    Output(bool),
    Input,
}

pub(crate) trait RawSwdIo {
    /// 用于向 SWDIO 输出输入
    fn swd_io<S>(&mut self, swdio: S) -> Result<Vec<bool>, error::Error>
    where
        S: IntoIterator<Item = IoSequenceItem>;

    /// 直接设置调试器的引脚.
    /// 个人建议不需要实现，正常我们都提供 TMS/SWDIO 输出达到复位操作
    /// 每个位表示的意义: 
    ///
    /// Bit 0: SWCLK/TCK
    /// Bit 1: SWDIO/TMS
    /// Bit 2: TDI
    /// Bit 3: TDO
    /// Bit 5: nTRST
    /// Bit 7: nRESET
    fn swj_pins(
        &mut self,
        pin_out: u32,
        pin_select: u32,
        pin_wait: u32,
    ) -> Result<u32, error::Error>;

    // 记录 Swd 需要的在读写后的空闲时钟配置
    // fn swd_settings(&self) -> &SwdSettings;

    // 传输性能记录， 还是去看源码
    // fn probe_statistics(&mut self) -> &mut ProbeStatistics;
}