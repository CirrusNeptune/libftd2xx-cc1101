//! I/O library for using [CC1101] RF transceiver via [libftd2xx-rs].
//!
//! # Usage
//! Simply add this crate as a dependency in your `Cargo.toml`.
//!
//! ```toml
//! [dependencies]
//! libftd2xx-cc1101 = "~0.1.0"
//! ```
//!
//! Also follow the README guidance in [libftd2xx-rs] to set up the connection with the FTDI device.
//!
//! [RX] and [TX] examples are good starting points.
//!
//! # References
//!
//! * [CC1101 Datasheet]
//!
//! [CC1101]: https://www.ti.com/product/CC1101
//! [libftd2xx-rs]: https://github.com/newAM/libftd2xx-rs
//! [RX]: https://github.com/CirrusNeptune/libftd2xx-cc1101/blob/main/examples/rx.rs
//! [TX]: https://github.com/CirrusNeptune/libftd2xx-cc1101/blob/main/examples/tx.rs
//! [CC1101 Datasheet]: https://www.ti.com/lit/gpn/cc1101

#![deny(missing_docs, unsafe_code, warnings)]

pub mod io;
pub mod regs;

use crate::regs::CcaMode;
use libftd2xx::{FtdiCommon, TimeoutError};
use libftd2xx_cc1101_derive::{CC1101Commands, CC1101ReadRegs, CC1101Regs};
use modular_bitfield_msb::prelude::*;
use regs::{
    FifoThreshold, FsAutocal, GDOCfg, LengthConfig, ModFormat, PoTimeout, Regs, SyncMode, WorRes,
};

/// Used in [chip status byte](https://www.ti.com/lit/ds/symlink/cc1101.pdf?ts=1623437777103&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FCC1101#%5B%7B%22num%22%3A146%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C69%2C753%2C0%5D)
#[repr(u8)]
#[derive(Debug, PartialEq, BitfieldSpecifier)]
#[bits = 3]
pub enum State {
    /// `IDLE` state
    Idle,
    /// Receive mode
    Rx,
    /// Transmit mode
    Tx,
    /// Fast TX ready
    FstxOn,
    /// Frequency synthesizer calibration is running
    Calibrate,
    /// PLL is settling
    Settling,
    /// RX FIFO has overflowed. Read out any useful data, then flush the FIFO with `SFRX`
    RxFifoOverflow,
    /// TX FIFO has underflowed. Acknowledge with SFTX
    TxFifoUnderflow,
}

/// [Chip status byte](https://www.ti.com/lit/ds/symlink/cc1101.pdf?ts=1623437777103&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FCC1101#%5B%7B%22num%22%3A146%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C69%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone)]
pub struct Status {
    pub chip_rdy_n: bool,
    pub state: State,
    pub fifo_bytes_available: B4,
}

mod mpsse_support {
    use crate::{Command, Regs, State, Status};
    use libftd2xx::{
        mpsse, ClockBits, ClockBitsOut, ClockDataIn, ClockDataOut, FtdiCommon, TimeoutError,
    };
    use log::trace;

    pub fn reset<Ft: FtdiCommon>(ftdi: &mut Ft) -> Result<(), TimeoutError> {
        trace!(">CC1101 SRES");
        mpsse! {
            const RESET = {
                set_gpio_lower(0x8, 0xb);
                set_gpio_lower(0x0, 0xb);
                clock_bits_out(ClockBitsOut::MsbNeg, Command::SRES as u8, 8);
                wait_on_io_low();
                set_gpio_lower(0x8, 0xb);
            };
        }
        ftdi.write_all(&RESET)
    }

    pub fn read_all_regs<Ft: FtdiCommon>(ftdi: &mut Ft) -> Result<Regs, TimeoutError> {
        trace!(">CC1101 READ_ALL_REGS");
        mpsse! {
            const (READ_ALL_REGS, READ_LEN) = {
                set_gpio_lower(0x0, 0xb);
                clock_bits_out(ClockBitsOut::MsbNeg, 0xC0, 8);
                clock_data_in(ClockDataIn::MsbNeg, 47);
                set_gpio_lower(0x8, 0xb);
            };
        }
        ftdi.write_all(&READ_ALL_REGS)?;
        let mut buf = [0_u8; READ_LEN];
        ftdi.read_all(&mut buf)?;
        let regs = Regs::from_bytes(buf);
        trace!("<CC1101 READ_ALL_REGS {:#x?}", regs);
        Ok(regs)
    }

    pub fn write_all_regs<Ft: FtdiCommon>(ftdi: &mut Ft, regs: Regs) -> Result<(), TimeoutError> {
        trace!(">CC1101 WRITE_ALL_REGS {:#x?}", regs);
        let regs_bytes = regs.into_bytes();
        mpsse! {
            let write_all_regs = {
                set_gpio_lower(0x0, 0xb);
                clock_data_out(ClockDataOut::MsbNeg, [0x40,
                    regs_bytes[0],
                    regs_bytes[1],
                    regs_bytes[2],
                    regs_bytes[3],
                    regs_bytes[4],
                    regs_bytes[5],
                    regs_bytes[6],
                    regs_bytes[7],
                    regs_bytes[8],
                    regs_bytes[9],
                    regs_bytes[10],
                    regs_bytes[11],
                    regs_bytes[12],
                    regs_bytes[13],
                    regs_bytes[14],
                    regs_bytes[15],
                    regs_bytes[16],
                    regs_bytes[17],
                    regs_bytes[18],
                    regs_bytes[19],
                    regs_bytes[20],
                    regs_bytes[21],
                    regs_bytes[22],
                    regs_bytes[23],
                    regs_bytes[24],
                    regs_bytes[25],
                    regs_bytes[26],
                    regs_bytes[27],
                    regs_bytes[28],
                    regs_bytes[29],
                    regs_bytes[30],
                    regs_bytes[31],
                    regs_bytes[32],
                    regs_bytes[33],
                    regs_bytes[34],
                    regs_bytes[35],
                    regs_bytes[36],
                    regs_bytes[37],
                    regs_bytes[38],
                    regs_bytes[39],
                    regs_bytes[40],
                    regs_bytes[41],
                    regs_bytes[42],
                    regs_bytes[43],
                    regs_bytes[44],
                    regs_bytes[45],
                    regs_bytes[46],
                ]);
                set_gpio_lower(0x8, 0xb);
            };
        }
        ftdi.write_all(&write_all_regs)?;
        Ok(())
    }

    pub fn read_pa_table<Ft: FtdiCommon>(ftdi: &mut Ft) -> Result<[u8; 8], TimeoutError> {
        trace!(">CC1101 READ_PA_TABLE");
        mpsse! {
            const (READ_ALL_REGS, READ_LEN) = {
                set_gpio_lower(0x0, 0xb);
                clock_bits_out(ClockBitsOut::MsbNeg, 0xFE, 8);
                clock_data_in(ClockDataIn::MsbNeg, 8);
                set_gpio_lower(0x8, 0xb);
            };
        }
        ftdi.write_all(&READ_ALL_REGS)?;
        let mut buf = [0_u8; READ_LEN];
        ftdi.read_all(&mut buf)?;
        trace!("<CC1101 READ_PA_TABLE {:x?}", buf);
        Ok(buf)
    }

    pub fn write_pa_table<Ft: FtdiCommon>(
        ftdi: &mut Ft,
        pa_table: [u8; 8],
    ) -> Result<(), TimeoutError> {
        trace!(">CC1101 WRITE_PA_TABLE {:x?}", pa_table);
        mpsse! {
            let write_pa_table = {
                set_gpio_lower(0x0, 0xb);
                clock_data_out(ClockDataOut::MsbNeg, [0x7E,
                    pa_table[0],
                    pa_table[1],
                    pa_table[2],
                    pa_table[3],
                    pa_table[4],
                    pa_table[5],
                    pa_table[6],
                    pa_table[7],
                ]);
                set_gpio_lower(0x8, 0xb);
            };
        }
        ftdi.write_all(&write_pa_table)?;
        Ok(())
    }

    pub fn get_state<Ft: FtdiCommon>(ftdi: &mut Ft) -> Result<State, TimeoutError> {
        trace!(">CC1101 SNOP");
        mpsse! {
            const (SNOP, READ_LEN) = {
                set_gpio_lower(0x0, 0xb);
                clock_bits(ClockBits::MsbPosIn, Command::SNOP as u8, 8);
                set_gpio_lower(0x8, 0xb);
                send_immediate();
            };
        }
        ftdi.write_all(&SNOP)?;
        let mut buf = [0_u8; READ_LEN];
        ftdi.read_all(&mut buf)?;
        let status = Status::from_bytes([buf[0]]);
        trace!("<CC1101 SNOP {:x?}", status);
        Ok(status.state())
    }

    pub fn sftx<Ft: FtdiCommon>(ftdi: &mut Ft) -> Result<(), TimeoutError> {
        trace!(">CC1101 SFTX");
        mpsse! {
            const SFTX = {
                set_gpio_lower(0x0, 0xb);
                clock_bits_out(ClockBitsOut::MsbNeg, Command::SFTX as u8, 8);
                set_gpio_lower(0x8, 0xb);
            };
        }
        ftdi.write_all(&SFTX)
    }

    pub fn sfrx<Ft: FtdiCommon>(ftdi: &mut Ft) -> Result<(), TimeoutError> {
        trace!(">CC1101 SFRX");
        mpsse! {
            const SFRX = {
                set_gpio_lower(0x0, 0xb);
                clock_bits_out(ClockBitsOut::MsbNeg, Command::SFRX as u8, 8);
                set_gpio_lower(0x8, 0xb);
            };
        }
        ftdi.write_all(&SFRX)
    }
}

/// Interface endpoint for CC1101.
///
/// References open ftdi handle and keeps cached [CC1101 registers][`Regs`].
pub struct CC1101<'f, Ft: FtdiCommon> {
    pub(crate) ftdi: &'f mut Ft,
    pub(crate) regs: Regs,
    pub(crate) pa_table: [u8; 8],
}

impl<'f, Ft: FtdiCommon> CC1101<'f, Ft> {
    /// Constructs a CC1101 interface from an open Ftdi device.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use libftd2xx::{Ft232h, Ftdi, FtdiMpsse, MpsseSettings};
    /// use libftd2xx_cc1101::CC1101;
    /// use std::convert::TryInto;
    /// use std::time::Duration;
    ///
    /// let ft = Ftdi::new().expect("unable to Ftdi::new");
    /// let mut ftdi: Ft232h = ft.try_into().expect("not a Ft232h");
    /// const SETTINGS: MpsseSettings = MpsseSettings {
    ///     reset: true,
    ///     in_transfer_size: 4096,
    ///     read_timeout: Duration::from_secs(1),
    ///     write_timeout: Duration::from_secs(1),
    ///     latency_timer: Duration::from_millis(16),
    ///     mask: 0xb,
    ///     clock_frequency: Some(1_000_000),
    /// };
    /// ftdi.initialize_mpsse(&SETTINGS)
    ///     .expect("unable to initialize mpsse");
    ///
    /// let mut cc1101 = CC1101::new(&mut ftdi);
    /// ```
    pub fn new(ftdi: &'f mut Ft) -> Self {
        Self {
            ftdi,
            regs: Regs::new(),
            pa_table: [0; 8],
        }
    }

    /// Initialize the CC1101 with default register values.
    pub fn initialize_default(&mut self) -> Result<(), TimeoutError> {
        self.initialize(|_, _| {})
    }

    /// Initialize the CC1101 with user-specified register values.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use libftd2xx::{Ft232h, Ftdi};
    /// # use libftd2xx_cc1101::CC1101;
    /// use libftd2xx_cc1101::regs::{ModFormat, SyncMode};
    /// # use std::convert::TryInto;
    /// # use std::io::Read;
    /// # let ft = Ftdi::new().expect("unable to Ftdi::new");
    /// # let mut ftdi: Ft232h = ft.try_into().expect("not a Ft232h");
    /// # let mut cc1101 = CC1101::new(&mut ftdi);
    ///
    /// cc1101
    ///     .initialize(|regs, pa_table| {
    ///         regs.set_freq(1093805); // 433.943634 MHz
    ///         regs.set_mdmcfg3(regs.mdmcfg3().with_drate_m(0x67)); // 2.2254 kBaud
    ///         regs.set_mdmcfg2(
    ///             regs.mdmcfg2()
    ///                 .with_mod_format(ModFormat::AskOok)
    ///                 .with_sync_mode(SyncMode::None),
    ///         );
    ///         *pa_table = [0, 0x60, 0, 0, 0, 0, 0, 0]; // 0dBm OOK
    ///     })
    ///     .expect("unable to initialize cc1101");
    /// ```
    pub fn initialize<F: Fn(&mut Regs, &mut [u8; 8])>(
        &mut self,
        regs_setup: F,
    ) -> Result<(), TimeoutError> {
        mpsse_support::reset(self.ftdi)?;
        self.regs = mpsse_support::read_all_regs(self.ftdi)?;

        self.regs
            .set_iocfg2(self.regs.iocfg2().with_gdo2_cfg(GDOCfg::TxFifoThreshold));
        self.regs
            .set_iocfg0(self.regs.iocfg0().with_gdo0_cfg(GDOCfg::ChipRdyN));
        self.regs.set_fifothr(
            self.regs
                .fifothr()
                .with_fifo_thr(FifoThreshold::Tx53Rx12)
                .with_adc_retention(true),
        );
        self.regs.set_sync(0xe8e8);
        self.regs.set_pktlen(255);
        self.regs.set_pktctrl0(
            self.regs
                .pktctrl0()
                .with_white_data(false)
                .with_crc_en(false)
                .with_length_config(LengthConfig::Infinite),
        );
        self.regs.set_fsctrl1(self.regs.fsctrl1().with_freq_if(6));
        self.regs.set_freq(1093805); // 433.943634 MHz
        self.regs.set_mdmcfg4(
            self.regs
                .mdmcfg4()
                .with_chanbw_e(3)
                .with_chanbw_m(3)
                .with_drate_e(6),
        );
        self.regs
            .set_mdmcfg3(self.regs.mdmcfg3().with_drate_m(0x67)); // 2.2254 kBaud
        self.regs.set_mdmcfg2(
            self.regs
                .mdmcfg2()
                .with_mod_format(ModFormat::AskOok)
                .with_sync_mode(SyncMode::None),
        );
        self.regs
            .set_deviatn(self.regs.deviatn().with_deviation_e(1).with_deviation_m(5));
        self.regs
            .set_mcsm1(self.regs.mcsm1().with_cca_mode(CcaMode::Always));
        self.regs.set_mcsm0(
            self.regs
                .mcsm0()
                .with_fs_autocal(FsAutocal::FromIdleToRxOrTxOrFstxOn)
                .with_po_timeout(PoTimeout::Count64),
        );
        self.regs
            .set_foccfg(self.regs.foccfg().with_foc_bs_cs_gate(false));
        self.regs
            .set_worctrl(self.regs.worctrl().with_wor_res(WorRes::Period2_15));
        self.regs.set_frend0(self.regs.frend0().with_pa_power(1));
        self.regs.set_fscal3(self.regs.fscal3().with_fscal3(3));
        self.regs
            .set_fscal2(self.regs.fscal2().with_vco_core_h_en(true));
        self.regs.set_fscal1(self.regs.fscal1().with_fscal1_res(0));
        self.regs
            .set_fscal0(self.regs.fscal0().with_fscal0_res(0x1f));
        self.regs.set_test2(0x81);
        self.regs.set_test1(0x35);
        self.regs
            .set_test0(self.regs.test0().with_vco_sel_cal_en(false));

        self.pa_table = mpsse_support::read_pa_table(self.ftdi)?;

        regs_setup(&mut self.regs, &mut self.pa_table);

        mpsse_support::write_all_regs(self.ftdi, self.regs)?;
        mpsse_support::write_pa_table(self.ftdi, self.pa_table)?;

        Ok(())
    }

    /// Create a new RXFIFO reader.
    ///
    /// The `BUF_CAP` const generic parameter is used to balance between access latency and
    /// frequency of underlying I/O operations depending on application. It must be in 1..=64.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use libftd2xx::{Ft232h, Ftdi};
    /// # use libftd2xx_cc1101::CC1101;
    /// # use std::convert::TryInto;
    /// # use std::io::Read;
    /// # let ft = Ftdi::new().expect("unable to Ftdi::new");
    /// # let mut ftdi: Ft232h = ft.try_into().expect("not a Ft232h");
    /// # let mut cc1101 = CC1101::new(&mut ftdi);
    /// let mut cc1101_reader = cc1101.reader::<32>();
    /// let mut buf = [0_u8; 32];
    /// cc1101_reader
    ///     .read_exact(&mut buf)
    ///     .expect("unable to read_exact");
    /// println!("{:?}", buf);
    /// ```
    pub fn reader<'c, const BUF_CAP: usize>(&'c mut self) -> io::FifoReader<'f, 'c, Ft, BUF_CAP> {
        io::FifoReader::<'f, 'c, Ft, BUF_CAP>::new(self)
    }

    /// Create a new TXFIFO writer.
    ///
    /// The `BUF_CAP` const generic parameter is used to balance between access latency and
    /// frequency of underlying I/O operations depending on application. It must be in 1..=64.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use libftd2xx::{Ft232h, Ftdi};
    /// # use libftd2xx_cc1101::CC1101;
    /// # use std::convert::TryInto;
    /// # use std::io::Write;
    /// # let ft = Ftdi::new().expect("unable to Ftdi::new");
    /// # let mut ftdi: Ft232h = ft.try_into().expect("not a Ft232h");
    /// # let mut cc1101 = CC1101::new(&mut ftdi);
    /// let mut cc1101_writer = cc1101.writer::<32>();
    /// cc1101_writer
    ///     .write_all(&[11, 22, 33, 44])
    ///     .expect("unable to write_all");
    /// cc1101_writer.flush().expect("unable to flush");
    /// ```
    pub fn writer<'c, const BUF_CAP: usize>(&'c mut self) -> io::FifoWriter<'f, 'c, Ft, BUF_CAP> {
        io::FifoWriter::<'f, 'c, Ft, BUF_CAP>::new(self)
    }

    pub(crate) fn get_state(&mut self) -> Result<State, TimeoutError> {
        mpsse_support::get_state(self.ftdi)
    }

    pub(crate) fn flush_fifos(&mut self, cur_state: State) -> Result<(), TimeoutError> {
        match cur_state {
            State::TxFifoUnderflow => mpsse_support::sftx(self.ftdi),
            State::RxFifoOverflow => mpsse_support::sfrx(self.ftdi),
            _ => Ok(()),
        }
    }
}

pub(crate) struct MpsseCmdBuilder(pub ::libftd2xx::MpsseCmdBuilder);

impl MpsseCmdBuilder {
    pub fn new() -> Self {
        Self(::libftd2xx::MpsseCmdBuilder::new())
    }

    pub fn as_slice(&self) -> &[u8] {
        self.0.as_slice()
    }

    pub fn cs_low(self) -> Self {
        Self(self.0.set_gpio_lower(0x0, 0xb))
    }

    pub fn cs_high(self) -> Self {
        Self(self.0.set_gpio_lower(0x8, 0xb))
    }

    pub fn clock_data_out(self, data: &[u8]) -> Self {
        Self(
            self.0
                .clock_data_out(::libftd2xx::ClockDataOut::MsbNeg, &data),
        )
    }

    pub fn clock_bits_out(self, data: u8, len: u8) -> Self {
        Self(
            self.0
                .clock_bits_out(::libftd2xx::ClockBitsOut::MsbNeg, data, len),
        )
    }
}

/// Internally-shadowed macro to support generated [`mpsse`] macro.
#[macro_export]
macro_rules! __mpsse_base {
    // ftdi <=> { command1(); command2(); ... commandN(); };
    ($ft:ident <=> {$($commands:tt)*}; $($tail:tt)*) => {
        // Assume CS is initially high
        mpsse!(((let, (@cc1101 ($ft, __buf, (), {}), cs_1)), 0) {$($commands)* cs_high();} -> []);
        mpsse!($($tail)*);
    };

    // CS-low transitions
    ((($const_let:tt, (@cc1101 $passthru:tt, cs_0)), $read_len:tt) {cs_low(); $($tail:tt)*} -> [$($out:tt)*]) => {
        mpsse!((($const_let, (@cc1101 $passthru, cs_0)), $read_len) {$($tail)*} -> [$($out)*]);
    };
    ((($const_let:tt, (@cc1101 $passthru:tt, $cs:tt)), $read_len:tt) {cs_low(); $($tail:tt)*} -> [$($out:tt)*]) => {
        mpsse!((($const_let, (@cc1101 $passthru, cs_0)), $read_len) {set_gpio_lower(0x0, 0xb); $($tail)*} -> [$($out)*]);
    };
    ((($const_let:tt, (@cc1101 $passthru:tt, $cs:tt)), $read_len:tt) {force_cs_low(); $($tail:tt)*} -> [$($out:tt)*]) => {
        mpsse!((($const_let, (@cc1101 $passthru, cs_0)), $read_len) {set_gpio_lower(0x0, 0xb); $($tail)*} -> [$($out)*]);
    };

    // CS-high transitions
    ((($const_let:tt, (@cc1101 $passthru:tt, cs_1)), $read_len:tt) {cs_high(); $($tail:tt)*} -> [$($out:tt)*]) => {
        mpsse!((($const_let, (@cc1101 $passthru, cs_1)), $read_len) {$($tail)*} -> [$($out)*]);
    };
    ((($const_let:tt, (@cc1101 $passthru:tt, $cs:tt)), $read_len:tt) {cs_high(); $($tail:tt)*} -> [$($out:tt)*]) => {
        mpsse!((($const_let, (@cc1101 $passthru, cs_1)), $read_len) {set_gpio_lower(0x8, 0xb); $($tail)*} -> [$($out)*]);
    };
    ((($const_let:tt, (@cc1101 $passthru:tt, $cs:tt)), $read_len:tt) {force_cs_high(); $($tail:tt)*} -> [$($out:tt)*]) => {
        mpsse!((($const_let, (@cc1101 $passthru, cs_1)), $read_len) {set_gpio_lower(0x8, 0xb); $($tail)*} -> [$($out)*]);
    };

    // Send command
    ($passthru:tt {command_strobe($cmd:expr); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(">CC1101 {:?}", $cmd as ::libftd2xx_cc1101::Command);
        mpsse!($passthru {
            cs_low();
            clock_bits_out(::libftd2xx::ClockBitsOut::MsbNeg, $cmd, 8);
            $($tail)*
        } -> [$($out)*]);
    };

    // Send command and read status
    ((($const_let:tt, (@cc1101 ($ft:tt, $buf:ident, ($($ret_ids:ident,)*), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     {let $stat_id:ident = command_strobe($cmd:expr); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(">CC1101 {:?}", $cmd as ::libftd2xx_cc1101::Command);
        mpsse!((($const_let, (@cc1101 ($ft, $buf, ($($ret_ids,)* $stat_id,), {
            $($post_reads)*
            let $stat_id = ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len]]);
            ::log::trace!("<CC1101 {:?} {:?}", $cmd as ::libftd2xx_cc1101::Command, $stat_id);
        }), $cs)), $read_len) {
            cs_low();
            clock_bits(::libftd2xx::ClockBits::MsbPosIn, $cmd, 8);
            $($tail)*
        } -> [$($out)*]);
    };

    // Generate array of Statuses from buffer
    (@stat_seq $buf:ident, $read_len:expr, [], ($($ret_ids:expr,)*)) => {
        [$(::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + $ret_ids]]),)*]
    };
    (@stat_seq $buf:ident, $read_len:expr, [$_d:expr, $($data:expr,)*], ($($ret_ids:expr,)*)) => {
        ::libftd2xx_cc1101::__mpsse_base!(@stat_seq $buf, $read_len, [$($data,)*], (0, $($ret_ids + 1,)*));
    };

    // Generic burst write
    ($passthru:tt @{write($msg:tt, $opcode:expr, [$($data:expr),* $(,)*]); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(concat!(">CC1101 ", stringify!($msg), " {:x?}"), [$($data,)*]);
        mpsse!($passthru {
            cs_low();
            clock_data_out(::libftd2xx::ClockDataOut::MsbNeg, [$opcode, $($data,)*]);
            cs_high();
            $($tail)*
        } -> [$($out)*]);
    };

    // Write TXFIFO
    ($passthru:tt {write_tx_fifo([$($data:expr),* $(,)*]); $($tail:tt)*} -> [$($out:tt)*]) => {
        mpsse!($passthru @{write(TXFIFO, 0x7F, [$($data,)*]); $($tail)*} -> [$($out)*]);
    };

    // Write PATABLE
    ($passthru:tt {write_pa_table([$($data:expr),* $(,)*]); $($tail:tt)*} -> [$($out:tt)*]) => {
        mpsse!($passthru @{write(PATABLE, 0x7E, [$($data,)*]); $($tail)*} -> [$($out)*]);
    };

    // Generic burst write with read status sequence
    ((($const_let:tt, (@cc1101 ($ft:tt, $buf:ident, ($($ret_ids:ident,)*), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     @{let $stat_id:ident = write($msg:tt, $opcode:expr, [$($data:expr),* $(,)*]); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(concat!(">CC1101 ", stringify!($msg), " {:x?}"), [$($data,)*]);
        mpsse!((($const_let, (@cc1101 ($ft, $buf, ($($ret_ids,)* $stat_id,), {
            $($post_reads)*
            let $stat_id = ::libftd2xx_cc1101::__mpsse_base!(@stat_seq $buf, $read_len, [$opcode, $($data,)*], ());
            ::log::trace!(concat!("<CC1101 ", stringify!($msg), " {:#?}"), $stat_id);
        }), $cs)), $read_len) {
            cs_low();
            clock_data(::libftd2xx::ClockData::MsbPosIn, [$opcode, $($data,)*]);
            cs_high();
            $($tail)*
        } -> [$($out)*]);
    };

    // Write TXFIFO and read status sequence
    ($passthru:tt {let $stat_id:ident = write_tx_fifo([$($data:expr),* $(,)*]); $($tail:tt)*} -> [$($out:tt)*]) => {
        mpsse!($passthru @{let $stat_id = write(TXFIFO, 0x7F, [$($data,)*]); $($tail)*} -> [$($out)*]);
    };

    // Write PATABLE and read status sequence
    ($passthru:tt {let $stat_id:ident = write_pa_table([$($data:expr),* $(,)*]); $($tail:tt)*} -> [$($out:tt)*]) => {
        mpsse!($passthru @{let $stat_id = write(PATABLE, 0x7E, [$($data,)*]); $($tail)*} -> [$($out)*]);
    };

    // Write all regs
    ($passthru:tt {write_all_regs($data:expr); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(">CC1101 ALL REGS {:#?}", $data);
        mpsse!($passthru {
            cs_low();
            clock_data_out(::libftd2xx::ClockDataOut::MsbNeg, [0x40,
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[0],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[1],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[2],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[3],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[4],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[5],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[6],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[7],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[8],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[9],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[10],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[11],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[12],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[13],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[14],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[15],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[16],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[17],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[18],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[19],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[20],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[21],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[22],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[23],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[24],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[25],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[26],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[27],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[28],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[29],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[30],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[31],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[32],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[33],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[34],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[35],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[36],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[37],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[38],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[39],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[40],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[41],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[42],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[43],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[44],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[45],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[46],
            ]);
            cs_high();
            $($tail)*
        } -> [$($out)*]);
    };

    // Write all regs with read status sequence
    ((($const_let:tt, (@cc1101 ($ft:tt, $buf:ident, ($($ret_ids:ident,)*), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     {let $stat_id:ident = write_all_regs($data:expr); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(">CC1101 ALL REGS {:#?}", $data);
        mpsse!((($const_let, (@cc1101 ($ft, $buf, ($($ret_ids,)* $stat_id,), {
            $($post_reads)*
            let $stat_id = [
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 1]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 2]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 3]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 4]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 5]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 6]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 7]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 8]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 9]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 10]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 11]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 12]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 13]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 14]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 15]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 16]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 17]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 18]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 19]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 20]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 21]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 22]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 23]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 24]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 25]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 26]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 27]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 28]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 29]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 30]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 31]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 32]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 33]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 34]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 35]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 36]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 37]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 38]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 39]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 40]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 41]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 42]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 43]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 44]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 45]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 46]]),
                ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 47]]),
            ];
            ::log::trace!("<CC1101 ALL REGS {:#?}", $stat_id);
        }), $cs)), $read_len) {
            cs_low();
            clock_data(::libftd2xx::ClockData::MsbPosIn, [0x40,
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[0],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[1],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[2],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[3],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[4],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[5],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[6],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[7],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[8],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[9],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[10],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[11],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[12],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[13],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[14],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[15],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[16],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[17],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[18],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[19],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[20],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[21],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[22],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[23],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[24],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[25],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[26],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[27],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[28],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[29],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[30],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[31],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[32],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[33],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[34],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[35],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[36],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[37],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[38],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[39],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[40],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[41],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[42],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[43],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[44],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[45],
                ($data as ::libftd2xx_cc1101::regs::Regs).into_bytes()[46],
            ]);
            cs_high();
            $($tail)*
        } -> [$($out)*]);
    };

    // Generic burst read
    ((($const_let:tt, (@cc1101 ($ft:tt, $buf:ident, ($($ret_ids:ident,)*), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     @{let $data_id:ident = read($msg:tt, $opcode:expr, $len:expr); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(concat!(">CC1101 ", stringify!($msg {})), $len);
        mpsse!((($const_let, (@cc1101 ($ft, $buf, ($($ret_ids,)* $data_id,), {
            $($post_reads)*
            let $data_id: [u8; $len] = $buf[$read_len..$read_len+$len].try_into().unwrap();
            ::log::trace!(concat!("<CC1101 ", stringify!($msg), " {:x?}"), $data_id);
        }), $cs)), $read_len) {
            cs_low();
            clock_bits_out(::libftd2xx::ClockBitsOut::MsbNeg, $opcode, 8);
            clock_data_in(::libftd2xx::ClockDataIn::MsbNeg, $len);
            cs_high();
            $($tail)*
        } -> [$($out)*]);
    };

    // Read RXFIFO
    ($passthru:tt {let $data_id:ident = read_rx_fifo($len:expr); $($tail:tt)*} -> [$($out:tt)*]) => {
        mpsse!($passthru @{let $data_id = read(RXFIFO, 0xFF, $len); $($tail)*} -> [$($out)*]);
    };

    // Read PATABLE
    ($passthru:tt {let $data_id:ident = read_pa_table($len:expr); $($tail:tt)*} -> [$($out:tt)*]) => {
        mpsse!($passthru @{let $data_id = read(PATABLE, 0xFE, $len); $($tail)*} -> [$($out)*]);
    };

    // Generic burst read with status
    ((($const_let:tt, (@cc1101 ($ft:tt, $buf:ident, ($($ret_ids:ident,)*), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     @{let ($stat_id:ident, $data_id:ident) = read($msg:tt, $opcode:expr, $len:expr); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(concat!(">CC1101 ", stringify!($msg {})), $len);
        mpsse!((($const_let, (@cc1101 ($ft, $buf, ($($ret_ids,)* $stat_id, $data_id,), {
            $($post_reads)*
            let $stat_id = ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len]]);
            let $data_id: [u8; $len] = $buf[$read_len+1..$read_len+1+$len].try_into().unwrap();
            ::log::trace!(concat!("<CC1101 ", stringify!($msg), " {:?} {:x?}"), $stat_id, $data_id);
        }), $cs)), $read_len) {
            cs_low();
            clock_bits(::libftd2xx::ClockBits::MsbPosIn, $opcode, 8);
            clock_data_in(::libftd2xx::ClockDataIn::MsbNeg, $len);
            cs_high();
            $($tail)*
        } -> [$($out)*]);
    };

    // Read RXFIFO with status
    ($passthru:tt {let ($stat_id:ident, $data_id:ident) = read_rx_fifo($len:expr); $($tail:tt)*} -> [$($out:tt)*]) => {
        mpsse!($passthru @{let ($stat_id, $data_id) = read(RXFIFO, 0xFF, $len); $($tail)*} -> [$($out)*]);
    };

    // Read PATABLE with status
    ($passthru:tt {let ($stat_id:ident, $data_id:ident) = read_pa_table($len:expr); $($tail:tt)*} -> [$($out:tt)*]) => {
        mpsse!($passthru @{let ($stat_id, $data_id) = read(PATABLE, 0xFE, $len); $($tail)*} -> [$($out)*]);
    };

    // Read all regs
    ((($const_let:tt, (@cc1101 ($ft:tt, $buf:ident, ($($ret_ids:ident,)*), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     {let $data_id:ident = read_all_regs(); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(">CC1101 ALL REGS");
        mpsse!((($const_let, (@cc1101 ($ft, $buf, ($($ret_ids,)* $data_id,), {
            $($post_reads)*
            let $data_id = ::libftd2xx_cc1101::regs::Regs::from_bytes($buf[$read_len..$read_len+47].try_into().unwrap());
            ::log::trace!("<CC1101 ALL REGS {:#?}", $data_id);
        }), $cs)), $read_len) {
            cs_low();
            clock_bits_out(::libftd2xx::ClockBitsOut::MsbNeg, 0xC0, 8);
            clock_data_in(::libftd2xx::ClockDataIn::MsbNeg, 47);
            cs_high();
            $($tail)*
        } -> [$($out)*]);
    };

    // Read all regs with status
    ((($const_let:tt, (@cc1101 ($ft:tt, $buf:ident, ($($ret_ids:ident,)*), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     {let ($stat_id:ident, $data_id:ident) = read_all_regs(); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(">CC1101 ALL REGS");
        mpsse!((($const_let, (@cc1101 ($ft, $buf, ($($ret_ids,)* $stat_id, $data_id,), {
            $($post_reads)*
            let $stat_id = ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len]]);
            let $data_id = ::libftd2xx_cc1101::regs::Regs::from_bytes($buf[$read_len+1..$read_len+48].try_into().unwrap());
            ::log::trace!("<CC1101 ALL REGS {:?} {:#?}", $stat_id, $data_id);
        }), $cs)), $read_len) {
            cs_low();
            clock_bits(::libftd2xx::ClockBits::MsbPosIn, 0xC0, 8);
            clock_data_in(::libftd2xx::ClockDataIn::MsbNeg, 47);
            cs_high();
            $($tail)*
        } -> [$($out)*]);
    };

    // Decode register constructs
    (@decode u8, $buf:ident, $read_len:expr) => {
        $buf[$read_len]
    };
    (@decode $tp:ty, $buf:ident, $read_len:expr) => {
        <$tp>::from_bytes([$buf[$read_len]])
    };

    // u24 register read for FREQ
    ((($const_let:tt, (@cc1101 ($ft:tt, $buf:ident, ($($ret_ids:ident,)*), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     @{let $data_id:ident = read_reg($opcode:expr, $enum_var:ident, ::libftd2xx_cc1101::regs::FREQ); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(concat!(">CC1101 ", stringify!($enum_var)));
        mpsse!((($const_let, (@cc1101 ($ft, $buf, ($($ret_ids,)* $data_id,), {
            $($post_reads)*
            let $data_id = u32::from_be_bytes([0, $buf[$read_len], $buf[$read_len + 1], $buf[$read_len + 2]]);
            ::log::trace!(concat!("<CC1101 ", stringify!($enum_var), " {:x?}"), $data_id);
        }), $cs)), $read_len) {
            cs_low();
            clock_bits_out(::libftd2xx::ClockBitsOut::MsbNeg, $opcode, 8);
            clock_bits_in(::libftd2xx::ClockBitsIn::MsbNeg, 8);
            clock_bits_out(::libftd2xx::ClockBitsOut::MsbNeg, $opcode + 1, 8);
            clock_bits_in(::libftd2xx::ClockBitsIn::MsbNeg, 8);
            clock_bits_out(::libftd2xx::ClockBitsOut::MsbNeg, $opcode + 2, 8);
            clock_bits_in(::libftd2xx::ClockBitsIn::MsbNeg, 8);
            $($tail)*
        } -> [$($out)*]);
    };

    // u24 register read for FREQ with statuses
    ((($const_let:tt, (@cc1101 ($ft:tt, $buf:ident, ($($ret_ids:ident,)*), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     @{let ($stat_id:ident, $data_id:ident) = read_reg($opcode:expr, $enum_var:ident, ::libftd2xx_cc1101::regs::FREQ); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(concat!(">CC1101 ", stringify!($enum_var)));
        mpsse!((($const_let, (@cc1101 ($ft, $buf, ($($ret_ids,)* $stat_id, $data_id,), {
            $($post_reads)*
            let $stat_id = [::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len]]),
                            ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 2]]),
                            ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 4]])];
            let $data_id = u32::from_be_bytes([0, $buf[$read_len + 1], $buf[$read_len + 3], $buf[$read_len + 5]]);
            ::log::trace!(concat!("<CC1101 ", stringify!($enum_var), " {:?} {:x?}"), $stat_id, $data_id);
        }), $cs)), $read_len) {
            cs_low();
            clock_bits(::libftd2xx::ClockBits::MsbPosIn, $opcode, 8);
            clock_bits_in(::libftd2xx::ClockBitsIn::MsbNeg, 8);
            clock_bits(::libftd2xx::ClockBits::MsbPosIn, $opcode + 1, 8);
            clock_bits_in(::libftd2xx::ClockBitsIn::MsbNeg, 8);
            clock_bits(::libftd2xx::ClockBits::MsbPosIn, $opcode + 2, 8);
            clock_bits_in(::libftd2xx::ClockBitsIn::MsbNeg, 8);
            $($tail)*
        } -> [$($out)*]);
    };

    // u16 register read for SYNC, WOREVT, WORTIME
    ((($const_let:tt, (@cc1101 ($ft:tt, $buf:ident, ($($ret_ids:ident,)*), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     @{let $data_id:ident = read_reg($opcode:expr, $enum_var:ident, u16); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(concat!(">CC1101 ", stringify!($enum_var)));
        mpsse!((($const_let, (@cc1101 ($ft, $buf, ($($ret_ids,)* $data_id,), {
            $($post_reads)*
            let $data_id = u16::from_be_bytes([$buf[$read_len], $buf[$read_len + 1]]);
            ::log::trace!(concat!("<CC1101 ", stringify!($enum_var), " {:x?}"), $data_id);
        }), $cs)), $read_len) {
            cs_low();
            clock_bits_out(::libftd2xx::ClockBitsOut::MsbNeg, $opcode, 8);
            clock_bits_in(::libftd2xx::ClockBitsIn::MsbNeg, 8);
            clock_bits_out(::libftd2xx::ClockBitsOut::MsbNeg, $opcode + 1, 8);
            clock_bits_in(::libftd2xx::ClockBitsIn::MsbNeg, 8);
            $($tail)*
        } -> [$($out)*]);
    };

    // u16 register read for SYNC, WOREVT, WORTIME with statuses
    ((($const_let:tt, (@cc1101 ($ft:tt, $buf:ident, ($($ret_ids:ident,)*), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     @{let ($stat_id:ident, $data_id:ident) = read_reg($opcode:expr, $enum_var:ident, u16); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(concat!(">CC1101 ", stringify!($enum_var)));
        mpsse!((($const_let, (@cc1101 ($ft, $buf, ($($ret_ids,)* $stat_id, $data_id,), {
            $($post_reads)*
            let $stat_id = [::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len]]),
                            ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 2]])];
            let $data_id = u16::from_be_bytes([$buf[$read_len + 1], $buf[$read_len + 3]]);
            ::log::trace!(concat!("<CC1101 ", stringify!($enum_var), " {:?} {:x?}"), $stat_id, $data_id);
        }), $cs)), $read_len) {
            cs_low();
            clock_bits(::libftd2xx::ClockBits::MsbPosIn, $opcode, 8);
            clock_bits_in(::libftd2xx::ClockBitsIn::MsbNeg, 8);
            clock_bits(::libftd2xx::ClockBits::MsbPosIn, $opcode + 1, 8);
            clock_bits_in(::libftd2xx::ClockBitsIn::MsbNeg, 8);
            $($tail)*
        } -> [$($out)*]);
    };

    // Generic register read
    ((($const_let:tt, (@cc1101 ($ft:tt, $buf:ident, ($($ret_ids:ident,)*), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     @{let $data_id:ident = read_reg($opcode:expr, $enum_var:ident, $($tp:tt)*); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(concat!(">CC1101 ", stringify!($enum_var)));
        mpsse!((($const_let, (@cc1101 ($ft, $buf, ($($ret_ids,)* $data_id,), {
            $($post_reads)*
            let $data_id = ::libftd2xx_cc1101::__mpsse_base!(@decode $($tp)*, $buf, $read_len);
            ::log::trace!(concat!("<CC1101 ", stringify!($enum_var), " {:x?}"), $data_id);
        }), $cs)), $read_len) {
            cs_low();
            clock_bits_out(::libftd2xx::ClockBitsOut::MsbNeg, $opcode, 8);
            clock_bits_in(::libftd2xx::ClockBitsIn::MsbNeg, 8);
            $($tail)*
        } -> [$($out)*]);
    };

    // Generic register read with status
    ((($const_let:tt, (@cc1101 ($ft:tt, $buf:ident, ($($ret_ids:ident,)*), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     @{let ($stat_id:ident, $data_id:ident) = read_reg($opcode:expr, $enum_var:ident, $($tp:tt)*); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(concat!(">CC1101 ", stringify!($enum_var)));
        mpsse!((($const_let, (@cc1101 ($ft, $buf, ($($ret_ids,)* $stat_id, $data_id,), {
            $($post_reads)*
            let $stat_id = ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len]]);
            let $data_id = ::libftd2xx_cc1101::__mpsse_base!(@decode $($tp)*, $buf, $read_len + 1);
            ::log::trace!(concat!("<CC1101 ", stringify!($enum_var), " {:?} {:x?}"), $stat_id, $data_id);
        }), $cs)), $read_len) {
            cs_low();
            clock_bits(::libftd2xx::ClockBits::MsbPosIn, $opcode, 8);
            clock_bits_in(::libftd2xx::ClockBitsIn::MsbNeg, 8);
            $($tail)*
        } -> [$($out)*]);
    };

    // Encode register constructs
    (@encode u8, $data:expr) => {
        $data
    };
    (@encode $tp:ty, $data:expr) => {
        ($data as $tp).into_bytes()[0];
    };

    // u24 register write for FREQ
    ($passthru:tt @{write_reg($opcode:expr, $enum_var:ident, $data:expr, ::libftd2xx_cc1101::regs::FREQ); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(concat!(">CC1101 ", stringify!($enum_var), " {:x?}"), $data);
        mpsse!($passthru {
            cs_low();
            clock_data_out(::libftd2xx::ClockDataOut::MsbNeg, [$opcode, ($data >> 16) & 0x3,
                                                               $opcode + 1, ($data >> 8) & 0xff,
                                                               $opcode + 2, $data & 0xff]);
            $($tail)*
        } -> [$($out)*]);
    };

    // u24 register write for FREQ with status sequence
    ((($const_let:tt, (@cc1101 ($ft:tt, $buf:ident, ($($ret_ids:ident,)*), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     @{let $stat_id:ident = write_reg($opcode:expr, $enum_var:ident, $data:expr, ::libftd2xx_cc1101::regs::FREQ); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(concat!(">CC1101 ", stringify!($enum_var), " {:x?}"), $data);
        mpsse!((($const_let, (@cc1101 ($ft, $buf, ($($ret_ids,)* $stat_id,), {
            $($post_reads)*
            let $stat_id = [::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len]]),
                            ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 1]]),
                            ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 2]]),
                            ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 3]]),
                            ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 4]]),
                            ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 5]])];
            ::log::trace!(concat!("<CC1101 ", stringify!($enum_var), " {:#?}"), $stat_id);
        }), $cs)), $read_len) {
            cs_low();
            clock_data(::libftd2xx::ClockData::MsbPosIn, [$opcode, ($data >> 16) & 0x3,
                                                          $opcode + 1, ($data >> 8) & 0xff,
                                                          $opcode + 2, $data & 0xff]);
            $($tail)*
        } -> [$($out)*]);
    };

    // u16 register write for SYNC, WOREVT
    ($passthru:tt @{write_reg($opcode:expr, $enum_var:ident, $data:expr, u16); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(concat!(">CC1101 ", stringify!($enum_var), " {:x?}"), $data);
        mpsse!($passthru {
            cs_low();
            clock_data_out(::libftd2xx::ClockDataOut::MsbNeg, [$opcode, ($data >> 8) & 0xff,
                                                               $opcode + 1, $data & 0xff]);
            $($tail)*
        } -> [$($out)*]);
    };

    // u16 register write for SYNC, WOREVT with status sequence
    ((($const_let:tt, (@cc1101 ($ft:tt, $buf:ident, ($($ret_ids:ident,)*), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     @{let $stat_id:ident = write_reg($opcode:expr, $enum_var:ident, $data:expr, u16); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(concat!(">CC1101 ", stringify!($enum_var), " {:x?}"), $data);
        mpsse!((($const_let, (@cc1101 ($ft, $buf, ($($ret_ids,)* $stat_id,), {
            $($post_reads)*
            let $stat_id = [::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len]]),
                            ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 1]]),
                            ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 2]]),
                            ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 3]])];
            ::log::trace!(concat!("<CC1101 ", stringify!($enum_var), " {:#?}"), $stat_id);
        }), $cs)), $read_len) {
            cs_low();
            clock_data(::libftd2xx::ClockData::MsbPosIn, [$opcode, ($data >> 8) & 0xff,
                                                          $opcode + 1, $data & 0xff]);
            $($tail)*
        } -> [$($out)*]);
    };

    // Generic register write
    ($passthru:tt @{write_reg($opcode:expr, $enum_var:ident, $data:expr, $($tp:tt)*); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(concat!(">CC1101 ", stringify!($enum_var), " {:x?}"), $data);
        mpsse!($passthru {
            cs_low();
            clock_data_out(::libftd2xx::ClockDataOut::MsbNeg, [$opcode, ::libftd2xx_cc1101::__mpsse_base!(@encode $($tp)*, $data)]);
            $($tail)*
        } -> [$($out)*]);
    };

    // Generic register write with status sequence
    ((($const_let:tt, (@cc1101 ($ft:tt, $buf:ident, ($($ret_ids:ident,)*), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     @{let $stat_id:ident = write_reg($opcode:expr, $enum_var:ident, $data:expr, $($tp:tt)*); $($tail:tt)*} -> [$($out:tt)*]) => {
        ::log::trace!(concat!(">CC1101 ", stringify!($enum_var), " {:x?}"), $data);
        mpsse!((($const_let, (@cc1101 ($ft, $buf, ($($ret_ids,)* $stat_id,), {
            $($post_reads)*
            let $stat_id = [::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len]]),
                            ::libftd2xx_cc1101::Status::from_bytes([$buf[$read_len + 1]])];
            ::log::trace!(concat!("<CC1101 ", stringify!($enum_var), " {:#?}"), $stat_id);
        }), $cs)), $read_len) {
            cs_low();
            clock_data(::libftd2xx::ClockData::MsbPosIn, [$opcode, ::libftd2xx_cc1101::__mpsse_base!(@encode $($tp)*, $data)]);
            $($tail)*
        } -> [$($out)*]);
    };

    // Read only if there is data to read
    (@transfer $ft:tt, [$($out:tt)*], $buf:ident, 0) => {
        $ft.write_all(&[$($out)*]).unwrap();
    };
    (@transfer $ft:tt, [$($out:tt)*], $buf:ident, $read_len:expr) => {
        $ft.write_all(&[$($out)* ::libftd2xx::MpsseCmd::SendImmediate as u8]).unwrap();
        ::log::trace!(">CC1101 WRITING {:x?}", &[$($out)* ::libftd2xx::MpsseCmd::SendImmediate as u8]);
        let mut $buf: [u8; $read_len] = [0; $read_len];
        $ft.read_all(&mut $buf).unwrap();
        ::log::trace!("<CC1101 READ {:x?}", &$buf);
    };

    // Perform write/read without returns
    ((($_const_let:tt, (@cc1101 ($ft:tt, $buf:ident, (), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     {} -> [$($out:tt)*]) => {
        {
            ::libftd2xx_cc1101::__mpsse_base!(@transfer $ft, [$($out)*], $buf, $read_len);
            $($post_reads)*
        };
    };

    // Perform write/read with returns
    ((($_const_let:tt, (@cc1101 ($ft:tt, $buf:ident, ($($ret_ids:ident,)*), {$($post_reads:tt)*}), $cs:tt)), $read_len:tt)
     {} -> [$($out:tt)*]) => {
        let ($($ret_ids,)*) = {
            ::libftd2xx_cc1101::__mpsse_base!(@transfer $ft, [$($out)*], $buf, $read_len);
            $($post_reads)*
            ($($ret_ids,)*)
        };
    };

    // Everything else handled by libftd2xx crate implementation.
    ($($tokens:tt)*) => {
        ::libftd2xx::mpsse!($($tokens)*);
    };
}

/// [CC1101 command strobes](https://www.ti.com/lit/ds/symlink/cc1101.pdf?ts=1623437777103&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FCC1101#%5B%7B%22num%22%3A395%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C69%2C431%2C0%5D)
#[derive(Debug, Copy, Clone, CC1101Commands)]
pub enum Command {
    /// Reset chip.
    SRES = 0x30,
    /// Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). If in RX (with CCA): Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround).
    SFSTXON = 0x31,
    /// Turn off crystal oscillator.
    SXOFF = 0x32,
    /// Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
    SCAL = 0x33,
    /// Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.
    SRX = 0x34,
    /// In IDLE state: Enable TX. Perform calibration first if MCSM0. FS_AUTOCAL=1. If in RX state and CCA is enabled: Only go to TX if channel is clear.
    STX = 0x35,
    /// Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable.
    SIDLE = 0x36,
    /// Start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if WORCTRL.RC_PD=0.
    SWOR = 0x38,
    /// Enter power down mode when CSn goes high.
    SPWD = 0x39,
    /// Flush the RX FIFO buffer. Only issue SFRXin IDLE or RXFIFO_OVERFLOW states.
    SFRX = 0x3A,
    /// Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states.
    SFTX = 0x3B,
    /// Reset real time clock to Event1 value.
    SWORRST = 0x3C,
    /// No operation. May be used to get access to the chip status byte.
    SNOP = 0x3D,
}

/// [CC1101 configuration registers](https://www.ti.com/lit/ds/symlink/cc1101.pdf?ts=1623437777103&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FCC1101#%5B%7B%22num%22%3A488%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C69%2C127%2C0%5D)
#[allow(non_camel_case_types)]
#[derive(CC1101Regs)]
pub enum RegAddrs {
    /// GDO2 output pin configuration
    #[gentype(::libftd2xx_cc1101::regs::IOCFG2)]
    IOCFG2 = 0x00,
    /// GDO1 output pin configuration
    #[gentype(::libftd2xx_cc1101::regs::IOCFG1)]
    IOCFG1 = 0x01,
    /// GDO0 output pin configuration
    #[gentype(::libftd2xx_cc1101::regs::IOCFG0)]
    IOCFG0 = 0x02,
    /// RX FIFO and TX FIFO thresholds
    #[gentype(::libftd2xx_cc1101::regs::FIFOTHR)]
    FIFOTHR = 0x03,
    /// Sync word
    #[gentype(u16)]
    SYNC = 0x04,
    /// Packet length
    #[gentype(u8)]
    PKTLEN = 0x06,
    /// Packet automation control 1
    #[gentype(::libftd2xx_cc1101::regs::PKTCTRL1)]
    PKTCTRL1 = 0x07,
    /// Packet automation control 0
    #[gentype(::libftd2xx_cc1101::regs::PKTCTRL0)]
    PKTCTRL0 = 0x08,
    /// Device address
    #[gentype(u8)]
    ADDR = 0x09,
    /// Channel number
    #[gentype(u8)]
    CHANNR = 0x0A,
    /// Frequency synthesizer control 1
    #[gentype(::libftd2xx_cc1101::regs::FSCTRL1)]
    FSCTRL1 = 0x0B,
    /// Frequency synthesizer control 0
    #[gentype(::libftd2xx_cc1101::regs::FSCTRL0)]
    FSCTRL0 = 0x0C,
    /// Frequency control word
    #[gentype(::libftd2xx_cc1101::regs::FREQ)]
    FREQ = 0x0D,
    /// Modem configuration 4
    #[gentype(::libftd2xx_cc1101::regs::MDMCFG4)]
    MDMCFG4 = 0x10,
    /// Modem configuration 3
    #[gentype(::libftd2xx_cc1101::regs::MDMCFG3)]
    MDMCFG3 = 0x11,
    /// Modem configuration 2
    #[gentype(::libftd2xx_cc1101::regs::MDMCFG2)]
    MDMCFG2 = 0x12,
    /// Modem configuration 1
    #[gentype(::libftd2xx_cc1101::regs::MDMCFG1)]
    MDMCFG1 = 0x13,
    /// Modem configuration 0
    #[gentype(::libftd2xx_cc1101::regs::MDMCFG0)]
    MDMCFG0 = 0x14,
    /// Modem deviation setting
    #[gentype(::libftd2xx_cc1101::regs::DEVIATN)]
    DEVIATN = 0x15,
    /// Main Radio Control State Machine configuration 2
    #[gentype(::libftd2xx_cc1101::regs::MCSM2)]
    MCSM2 = 0x16,
    /// Main Radio Control State Machine configuration 1
    #[gentype(::libftd2xx_cc1101::regs::MCSM1)]
    MCSM1 = 0x17,
    /// Main Radio Control State Machine configuration 0
    #[gentype(::libftd2xx_cc1101::regs::MCSM0)]
    MCSM0 = 0x18,
    /// Frequency Offset Compensation configuration
    #[gentype(::libftd2xx_cc1101::regs::FOCCFG)]
    FOCCFG = 0x19,
    /// Bit Synchronization configuration
    #[gentype(::libftd2xx_cc1101::regs::BSCFG)]
    BSCFG = 0x1A,
    /// AGC control 2
    #[gentype(::libftd2xx_cc1101::regs::AGCCTRL2)]
    AGCCTRL2 = 0x1B,
    /// AGC control 1
    #[gentype(::libftd2xx_cc1101::regs::AGCCTRL1)]
    AGCCTRL1 = 0x1C,
    /// AGC control 0
    #[gentype(::libftd2xx_cc1101::regs::AGCCTRL0)]
    AGCCTRL0 = 0x1D,
    /// High byte Event 0 timeout
    #[gentype(u16)]
    WOREVT = 0x1E,
    /// Wake On Radio control
    #[gentype(::libftd2xx_cc1101::regs::WORCTRL)]
    WORCTRL = 0x20,
    /// Front end RX configuration
    #[gentype(::libftd2xx_cc1101::regs::FREND1)]
    FREND1 = 0x21,
    /// Front end TX configuration
    #[gentype(::libftd2xx_cc1101::regs::FREND0)]
    FREND0 = 0x22,
    /// Frequency synthesizer calibration 3
    #[gentype(::libftd2xx_cc1101::regs::FSCAL3)]
    FSCAL3 = 0x23,
    /// Frequency synthesizer calibration 2
    #[gentype(::libftd2xx_cc1101::regs::FSCAL2)]
    FSCAL2 = 0x24,
    /// Frequency synthesizer calibration 1
    #[gentype(::libftd2xx_cc1101::regs::FSCAL1)]
    FSCAL1 = 0x25,
    /// Frequency synthesizer calibration 0
    #[gentype(::libftd2xx_cc1101::regs::FSCAL0)]
    FSCAL0 = 0x26,
    /// RC oscillator configuration 1
    #[gentype(::libftd2xx_cc1101::regs::RCCTRL1)]
    RCCTRL1 = 0x27,
    /// RC oscillator configuration 0
    #[gentype(::libftd2xx_cc1101::regs::RCCTRL0)]
    RCCTRL0 = 0x28,
    /// Frequency synthesizer calibration control
    #[gentype(u8)]
    FSTEST = 0x29,
    /// Production test
    #[gentype(u8)]
    PTEST = 0x2A,
    /// AGC test
    #[gentype(u8)]
    AGCTEST = 0x2B,
    /// Various test settings
    #[gentype(u8)]
    TEST2 = 0x2C,
    /// Various test settings
    #[gentype(u8)]
    TEST1 = 0x2D,
    /// Various test settings
    #[gentype(::libftd2xx_cc1101::regs::TEST0)]
    TEST0 = 0x2E,
}

/// [CC1101 status registers](https://www.ti.com/lit/ds/symlink/cc1101.pdf?ts=1623437777103&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FCC1101#%5B%7B%22num%22%3A1009%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C69%2C489%2C0%5D)
#[allow(non_camel_case_types)]
#[derive(CC1101ReadRegs)]
pub enum ReadRegAddrs {
    /// Part number for CC1101
    #[gentype(u8)]
    PARTNUM = 0xF0,
    /// Current version number
    #[gentype(u8)]
    VERSION = 0xF1,
    /// Frequency Offset Estimate
    #[gentype(u8)]
    FREQTEST = 0xF2,
    /// Demodulator estimate for Link Quality
    #[gentype(::libftd2xx_cc1101::regs::LQI)]
    LQI = 0xF3,
    /// Received signal strength indication
    #[gentype(u8)]
    RSSI = 0xF4,
    /// Control state machine state
    #[gentype(::libftd2xx_cc1101::regs::MARCSTATE)]
    MARCSTATE = 0xF5,
    /// High byte of WOR timer
    #[gentype(u16)]
    WORTIME = 0xF6,
    /// Current GDOx status and packet status
    #[gentype(::libftd2xx_cc1101::regs::PKTSTATUS)]
    PKTSTATUS = 0xF8,
    /// Current setting from PLL calibration module
    #[gentype(u8)]
    VCO_VC_DAC = 0xF9,
    /// Underflow and number of bytes in the TX FIFO
    #[gentype(::libftd2xx_cc1101::regs::TXBYTES)]
    TXBYTES = 0xFA,
    /// Overflow and number of bytes in the RX FIFO
    #[gentype(::libftd2xx_cc1101::regs::RXBYTES)]
    RXBYTES = 0xFB,
    /// Last RC oscillator calibration result
    #[gentype(::libftd2xx_cc1101::regs::RCCTRL_STATUS)]
    RCCTRL1_STATUS = 0xFC,
    /// Last RC oscillator calibration result
    #[gentype(::libftd2xx_cc1101::regs::RCCTRL_STATUS)]
    RCCTRL0_STATUS = 0x3D,
}
