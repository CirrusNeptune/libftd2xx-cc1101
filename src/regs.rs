//! This module contains bitfield types that may be used to cache
//! and manipulate all CC1101 registers.
//!
//! PDF reference links work in Firefox.

#![deny(missing_docs, unsafe_code, warnings)]
// TODO: Remove this when compiler stops warning about nothing
#![allow(unused_braces, clippy::upper_case_acronyms)]

use modular_bitfield_msb::prelude::*;
use static_assertions::assert_eq_size;

/// [General Purpose / Test Output Control Pins Configuration](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A227%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C69%2C391%2C0%5D)
///
/// Selects GDOx pin signal
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 6]
pub enum GDOCfg {
    /// Associated to the RX FIFO: Asserts when RX FIFO is filled at or above the RX FIFO threshold. De-asserts when RX FIFO is drained below the same threshold.
    RxFifoThreshold = 0,
    /// Associated to the RX FIFO: Asserts when RX FIFO is filled at or above the RX FIFO threshold or the end of packet is reached. De-asserts when the RX FIFO is empty.
    RxFifoThresholdEmpty = 1,
    /// Associated to the TX FIFO: Asserts when the TX FIFO is filled at or above the TX FIFO threshold. De-asserts when the TX FIFO is below the same threshold.
    TxFifoThreshold = 2,
    /// Associated to the TX FIFO: Asserts when TX FIFO is full. De-asserts when the TX FIFO is drained below the TX FIFO threshold.
    TxFifoThresholdEmpty = 3,
    /// Asserts when the RX FIFO has overflowed. De-asserts when the FIFO has been flushed.
    RxFifoOverflow = 4,
    /// Asserts when the TX FIFO has underflowed. De-asserts when the FIFO is flushed.
    TxFifoUnderflow = 5,
    /// Asserts when sync word has been sent / received, and de-asserts at the end of the packet. In RX, the pin will also de-assert when a packet is discarded due to address or maximum length filtering or when the radio enters RXFIFO_OVERFLOW state. In TX the pin will de-assert if the TX FIFO underflows.
    SyncWord = 6,
    /// Asserts when a packet has been received with CRC OK. De-asserts when the first byte is read from the RX FIFO.
    PacketReceived = 7,
    /// Preamble Quality Reached. Asserts when the PQI is above the programmed PQT value.De-asserted when the chip re-enters RX state (MARCSTATE=0x0D) or the PQI gets below the programmed PQT value.
    PreambleQualityReached = 8,
    /// Clear channel assessment. High when RSSI level is below threshold (dependent on the current CCA_MODE setting).
    ClearChannel = 9,
    /// Lock detector output. The PLL is in lock if the lock detector output has a positive transition or is constantly logic high. To check for PLL lock the lock detector output should be used as an interrupt for the MCU.
    LockDetected = 10,
    /// Serial Clock. Synchronous to the data in synchronous serial mode.In RX mode, data is set up on the falling edge by CC1101 when GDOx_INV=0. In TX mode, data is sampled by CC1101 on the rising edge of the serial clock when GDOx_INV=0.
    SerialClock = 11,
    /// Serial Synchronous Data Output. Used for synchronous serial mode.
    SerialSynchronousData = 12,
    /// Serial Data Output. Used for asynchronous serial mode.
    SerialAsynchronousData = 13,
    /// Carrier sense. High if RSSI level is above threshold.Cleared when entering IDLE mode.
    CarrierSense = 14,
    /// The last CRC comparison matched. Cleared when entering/restarting RX mode.
    CrcOk = 15,
    /// Can be used together with RX_SYMBOL_TICK for alternative serial RX output.
    RxHardData1 = 22,
    /// Can be used together with RX_SYMBOL_TICK for alternative serial RX output.
    RxHardData0 = 23,
    /// Note: PA_PD willhave the same signal level in SLEEP and TX states. To control an external PA or RX/TX switch in applications where the SLEEP state is used it is recommended to use GDOx_CFGx=0x2F instead.
    PaPd = 27,
    /// Note: LNA_PD will have the same signal level in SLEEP and RX states. To control an external LNA or RX/TX switch in applications where the SLEEP state is used it is recommended to use GDOx_CFGx=0x2F instead.
    LnaPd = 28,
    /// Can be used together with RX_HARD_DATA for alternative serial RX output.
    RxSymbolTick = 29,
    /// WOR_EVNT0
    WorEvnt0 = 36,
    /// WOR_EVNT1
    WorEvnt1 = 37,
    /// CLK_256
    Clk256 = 38,
    /// CLK_32k
    Clk32k = 39,
    /// CHIP_RDYn
    ChipRdyN = 41,
    /// XOSC_STABLE
    XOscStable = 43,
    /// High impedance (3-state)
    HighZ = 46,
    /// HW to 0 (HW1 achieved by setting GDOx_INV=1). Can be used to control an external LNA/PA or RX/TX switch.
    HwTo0 = 47,
    /// CLK_XOSC/1
    ClkXOsc1 = 48,
    /// CLK_XOSC/1.5
    ClkXOsc1_5 = 49,
    /// CLK_XOSC/2
    ClkXOsc2 = 50,
    /// CLK_XOSC/3
    ClkXOsc3 = 51,
    /// CLK_XOSC/4
    ClkXOsc4 = 52,
    /// CLK_XOSC/6
    ClkXOsc6 = 53,
    /// CLK_XOSC/8
    ClkXOsc8 = 54,
    /// CLK_XOSC/12
    ClkXOsc12 = 55,
    /// CLK_XOSC/16
    ClkXOsc16 = 56,
    /// CLK_XOSC/24
    ClkXOsc24 = 57,
    /// CLK_XOSC/32
    ClkXOsc32 = 58,
    /// CLK_XOSC/48
    ClkXOsc48 = 59,
    /// CLK_XOSC/64
    ClkXOsc64 = 60,
    /// CLK_XOSC/96
    ClkXOsc96 = 61,
    /// CLK_XOSC/128
    ClkXOsc128 = 62,
    /// CLK_XOSC/192
    ClkXOsc192 = 63,
}

/// Used in [`FIFOTHR.CLOSE_IN_RX`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A321%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C206%2C753%2C0%5D)
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum RXAttenuation {
    /// 0 dB
    Db0,
    /// 6 dB
    Db6,
    /// 12 dB
    Db12,
    /// 18 dB
    Db18,
}

/// Used in [`FIFOTHR.FIFO_THR`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A321%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C206%2C753%2C0%5D)
///
/// Set the threshold for the TX FIFO and RX FIFO. The threshold is exceeded when the number of bytes in the FIFO is equal to or higher than the threshold value.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 4]
pub enum FifoThreshold {
    /// 61 bytes in TX or 4 bytes in RX.
    Tx61Rx4,
    /// 57 bytes in TX or 8 bytes in RX.
    Tx57Rx8,
    /// 53 bytes in TX or 12 bytes in RX.
    Tx53Rx12,
    /// 49 bytes in TX or 16 bytes in RX.
    Tx49Rx16,
    /// 45 bytes in TX or 20 bytes in RX.
    Tx45Rx20,
    /// 41 bytes in TX or 24 bytes in RX.
    Tx41Rx24,
    /// 37 bytes in TX or 28 bytes in RX.
    Tx37Rx28,
    /// 33 bytes in TX or 32 bytes in RX.
    Tx33Rx32,
    /// 29 bytes in TX or 36 bytes in RX.
    Tx29Rx36,
    /// 25 bytes in TX or 40 bytes in RX.
    Tx25Rx40,
    /// 21 bytes in TX or 44 bytes in RX.
    Tx21Rx44,
    /// 17 bytes in TX or 48 bytes in RX.
    Tx17Rx48,
    /// 13 bytes in TX or 52 bytes in RX.
    Tx13Rx52,
    /// 9 bytes in TX or 56 bytes in RX.
    Tx9Rx56,
    /// 5 bytes in TX or 60 bytes in RX.
    Tx5Rx60,
    /// 1 byte in TX or 64 bytes in RX.
    Tx1Rx64,
}

/// Used in [`PKTCTRL1.ADR_CHK`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A584%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C215%2C513%2C0%5D)
///
/// Controls address check configuration of received packages.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum AdrChk {
    /// No address check
    NoAddressCheck,
    /// Address check, no broadcast
    AddressCheckNoBroadcast,
    /// Address check and 0 (0x00) broadcast
    AddressCheck0Broadcast,
    /// Address check and 0 (0x00) and 255 (0xFF) broadcast
    AddressCheck0_255Broadcast,
}

/// Used in [`PKTCTRL0.PKT_FORMAT`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A591%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C215%2C753%2C0%5D)
///
/// Format of RX and TX data
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum PktFormat {
    /// Normal mode, use FIFOs for RX and TX
    Normal,
    /// Synchronous serial mode,Data in on GDO0 and data out on either of the GDOxpins
    SynchronousSerial,
    /// Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX
    RandomTx,
    /// Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins
    AsynchronousSerial,
}

/// Used in [`PKTCTRL0.LENGTH_CONFIG`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A591%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C215%2C753%2C0%5D)
///
/// Configure the packet length
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum LengthConfig {
    /// Fixed packet length mode. Length configured in PKTLEN register
    Fixed,
    /// Variable packet length mode. Packet length configured by the first byte after sync word
    Variable,
    /// Infinite packet length mode
    Infinite,
}

/// Used in [`MDMCFG2.MOD_FORMAT`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A279%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C228%2C753%2C0%5D)
///
/// The modulation format of the radio signal
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 3]
pub enum ModFormat {
    /// 2-FSK
    Fsk2 = 0,
    /// GFSK
    Gfsk = 1,
    /// ASK/OOK
    AskOok = 3,
    /// 4-FSK
    Fsk4 = 4,
    /// MSK
    Msk = 7,
}

/// Used in [`MDMCFG2.SYNC_MODE`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A279%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C228%2C753%2C0%5D)
///
/// Combined sync-word qualifier mode.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 3]
pub enum SyncMode {
    /// No preamble/sync
    None,
    /// 15/16 sync word bits detected
    Sync15_16,
    /// 16/16 sync word bits detected
    Sync16_16,
    /// 30/32 sync word bits detected
    Sync30_32,
    /// No preamble/sync, carrier-sense above threshold
    CSAboveThres,
    /// 15/16 + carrier-sense above threshold
    #[allow(non_camel_case_types)]
    Sync15_16_CSAboveThres,
    /// 16/16 + carrier-sense above threshold
    #[allow(non_camel_case_types)]
    Sync16_16_CSAboveThres,
    /// 30/32 + carrier-sense above threshold
    #[allow(non_camel_case_types)]
    Sync30_32_CSAboveThres,
}

/// Used in [`MDMCFG1.NUM_PREAMBLE`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A603%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C230%2C753%2C0%5D)
///
/// Sets the minimum number of preamble bytes to be transmitted
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 3]
pub enum NumPreamble {
    /// 2 preamble bytes
    Bytes2,
    /// 3 preamble bytes
    Bytes3,
    /// 4 preamble bytes
    Bytes4,
    /// 6 preamble bytes
    Bytes6,
    /// 6 preamble bytes
    Bytes8,
    /// 12 preamble bytes
    Bytes12,
    /// 16 preamble bytes
    Bytes16,
    /// 24 preamble bytes
    Bytes24,
}

/// Used in [`MCSM2.RX_TIME`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A285%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C174%2C753%2C0%5D)
///
/// The RX timeout in μs is given by EVENT0*C(RX_TIME, WOR_RES)·26/X
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 3]
pub enum RxTime {
    /// 3.6058, 18.0288, 32.4519, 46.8750
    Time0,
    /// 1.8029, 9.0144, 16.2260, 23.4375
    Time1,
    /// 0.9014, 4.5072, 8.1130, 11.7188
    Time2,
    /// 0.4507, 2.2536, 4.0565, 5.8594
    Time3,
    /// 0.2254, 1.1268, 2.0282, 2.9297
    Time4,
    /// 0.1127, 0.5634, 1.0141, 1.4648
    Time5,
    /// 0.0563, 0.2817, 0.5071, 0.7324
    Time6,
    /// Until end of packet
    UntilEndOfPacket,
}

/// Used in [`MCSM1.CCA_MODE`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A628%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C175%2C753%2C0%5D)
///
/// Selects CCA_MODE; Reflected in CCA signal
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum CcaMode {
    /// Always
    Always,
    /// If RSSI below threshold
    RssiBelowThres,
    /// Unless currently receiving a packet
    ReceivingPacket,
    /// If RSSI below threshold unless currently receiving a packet
    RssiBelowThresUnlessReceivingPacket,
}

/// Used in [`MCSM1.RXOFF_MODE`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A628%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C175%2C753%2C0%5D)
///
/// Select what should happen when a packet has been received
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum RxoffMode {
    /// `IDLE`
    Idle,
    /// `FSTXON`
    FstxOn,
    /// `TX`
    Tx,
    /// Stay in `RX`
    StayInRx,
}

/// Used in [`MCSM1.TXOFF_MODE`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A628%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C175%2C753%2C0%5D)
///
/// Select what should happen when a packet has been sent (TX)
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum TxoffMode {
    /// `IDLE`
    Idle,
    /// `FSTXON`
    FstxOn,
    /// Stay in `TX`
    StayInTx,
    /// `RX`
    Rx,
}

/// Used in [`MCSM0.FS_AUTOCAL`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A283%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C175%2C753%2C0%5D)
///
/// Automatically calibrate when going to RX or TX, or back to IDLE
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum FsAutocal {
    /// Never (manually calibrate using `SCAL` strobe)
    Never,
    /// When going from `IDLE` to `RX` or `TX` (or `FSTXON`)
    FromIdleToRxOrTxOrFstxOn,
    /// When going from `RX` or `TX` back to `IDLE` automatically
    FromRxOrTxToIdle,
    /// Every 4th time when going from `RX` or `TX` to `IDLE` automatically
    Every4thFromRxOrTxToIdle,
}

/// Used in [`MCSM0.PO_TIMEOUT`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A283%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C175%2C753%2C0%5D)
///
/// Programs the number of times the six-bit ripple counter must expire after XOSC has stabilized before CHP_RDYn goes low.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum PoTimeout {
    /// Expire count = 1
    Count1,
    /// Expire count = 16
    Count16,
    /// Expire count = 64
    Count64,
    /// Expire count = 256
    Count256,
}

/// Used in [`FOCCFG.FOC_PRE_K`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A569%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C174%2C753%2C0%5D)
///
/// The frequency compensation loop gain to be used before a sync word is detected.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum FocPreK {
    /// K
    K,
    /// 2K
    K2,
    /// 3K
    K3,
    /// 4K
    K4,
}

/// Used in [`FOCCFG.FOC_POST_K`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A569%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C174%2C753%2C0%5D)
///
/// The frequency compensation loop gain to be used after a sync word is detected.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 1]
pub enum FocPostK {
    /// Same as FOC_PRE_K
    SameAsPreK,
    /// K/2
    KOver2,
}

/// Used in [`FOCCFG.FOC_LIMIT`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A569%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C174%2C753%2C0%5D)
///
/// The saturation point for the frequency offset compensation algorithm
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum FocLimit {
    /// +/- 0
    PM0,
    /// +/- BW_chan/8
    PMBWOver8,
    /// +/- BW_chan/4
    PMBWOver4,
    /// +/- BW_chan/2
    PMBWOver2,
}

/// Used in [`BSCFG.BS_PRE_KI`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A1077%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C208%2C753%2C0%5D)
///
/// The clock recovery feedback loop integral gain to be used before a sync word is detected (used to correct offsets in data rate)
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum BsPreKi {
    /// K_I
    KI,
    /// 2K_I
    KI2,
    /// 3K_I
    KI3,
    /// 4K_I
    KI4,
}

/// Used in [`BSCFG.BS_PRE_KP`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A1077%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C208%2C753%2C0%5D)
///
/// The clock recovery feedback loop proportional gain to be used before a sync word is detected.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum BsPreKp {
    /// K_P
    KP,
    /// 2K_P
    KP2,
    /// 3K_P
    KP3,
    /// 4K_P
    KP4,
}

/// Used in [`BSCFG.BS_POST_KI`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A1077%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C208%2C753%2C0%5D)
///
/// The clock recovery feedback loop integral gain to be used after a sync word is detected.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 1]
pub enum BsPostKi {
    /// Same as BS_PRE_KI
    SameAsBsPreKi,
    /// K_I/2
    KIOver2,
}

/// Used in [`BSCFG.BS_POST_KP`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A1077%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C208%2C753%2C0%5D)
///
/// The clock recovery feedback loop proportional gain to be used after a sync word is detected.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 1]
pub enum BsPostKp {
    /// Same as BS_PRE_KP
    SameAsBsPreKp,
    /// K_P
    KP,
}

/// Used in [`BSCFG.BS_LIMIT`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A1077%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C208%2C753%2C0%5D)
///
/// The saturation point for the data rate offset compensation algorithm.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum BsLimit {
    /// +/- 0
    PM0,
    /// +/- 3.125%
    PM3_125,
    /// +/- 6.25%
    PM6_25,
    /// +/- 12.5%
    PM12_5,
}

/// Used in [`AGCCTRL2.MAX_DVGA_GAIN`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A734%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C248%2C753%2C0%5D)
///
/// Reduces the maximum allowable DVGA gain.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum MaxDvgaGain {
    /// All gain settings can be used
    AllGainCanBeUsed,
    /// The highest gain setting can not be used
    HighestGainCannotBeUsed,
    /// The 2 highest gain settings can not be used
    TwoHighestGainCannotBeUsed,
    /// The 3 highest gain settings can not be used
    ThreeHighestGainCannotBeUsed,
}

/// Used in [`AGCCTRL2.MAX_LNA_GAIN`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A734%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C248%2C753%2C0%5D)
///
/// Sets the maximum allowable LNA + LNA 2 gain relative to the maximum possible gain.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 3]
pub enum MaxLnaGain {
    /// Maximum possible LNA + LNA 2 gain
    MaxPossibleLNAPlusLNA2,
    /// Approx. 2.6 dB below maximum possible gain
    Approx2_6BelowMax,
    /// Approx. 6.1 dB below maximum possible gain
    Approx6_1BelowMax,
    /// Approx. 7.4 dB below maximum possible gain
    Approx7_4BelowMax,
    /// Approx. 9.2 dB below maximum possible gain
    Approx9_2BelowMax,
    /// Approx. 11.5 dB below maximum possible gain
    Approx11_5BelowMax,
    /// Approx. 14.6 dB below maximum possible gain
    Approx14_6BelowMax,
    /// Approx. 17.1 dB below maximum possible gain
    Approx17_1BelowMax,
}

/// Used in [`AGCCTRL2.MAGN_TARGET`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A734%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C248%2C753%2C0%5D)
///
/// These bits set the target value for the averaged amplitude from the digital channel filter (1 LSB = 0 dB).
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 3]
pub enum MagnTarget {
    /// 24 dB
    Db24,
    /// 27 dB
    Db27,
    /// 30 dB
    Db30,
    /// 33 dB
    Db33,
    /// 36 dB
    Db36,
    /// 38 dB
    Db38,
    /// 40 dB
    Db40,
    /// 42 dB
    Db42,
}

/// Used in [`AGCCTRL1.CARRIER_SENSE_REL_THR`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A737%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C248%2C753%2C0%5D)
///
/// Sets the relative change threshold for asserting carrier sense.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum CarrierSenseRelThr {
    /// Relative carrier sense threshold disabled
    RelThrDisabled,
    /// 6 dB increase in RSSI value
    Db6IncreaseRssi,
    /// 10 dB increase in RSSI value
    Db10IncreaseRssi,
    /// 14 dB increase in RSSI value
    Db14IncreaseRssi,
}

/// Used in [`AGCCTRL1.CARRIER_SENSE_ABS_THR`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A737%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C248%2C753%2C0%5D)
///
/// Sets the absolute RSSI threshold for asserting carrier sense. The 2-complement signed threshold is programmed in steps of 1 dB and is relative to the MAGN_TARGET setting.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 4]
pub enum CarrierSenseAbsThr {
    /// Absolute carrier sense threshold disabled
    AbsThrDisabled = 0b1000,
    /// 7 dB below MAGN_TARGET setting
    Db7BelowTarget = 0b1001,
    /// 6 dB below MAGN_TARGET setting
    Db6BelowTarget = 0b1010,
    /// 5 dB below MAGN_TARGET setting
    Db5BelowTarget = 0b1011,
    /// 4 dB below MAGN_TARGET setting
    Db4BelowTarget = 0b1100,
    /// 3 dB below MAGN_TARGET setting
    Db3BelowTarget = 0b1101,
    /// 2 dB below MAGN_TARGET setting
    Db2BelowTarget = 0b1110,
    /// 1 dB below MAGN_TARGET setting
    Db1BelowTarget = 0b1111,
    /// At MAGN_TARGET setting
    AtTarget = 0b0000,
    /// 1 dB above MAGN_TARGET setting
    Db1AboveTarget = 0b0001,
    /// 2 dB above MAGN_TARGET setting
    Db2AboveTarget = 0b0010,
    /// 3 dB above MAGN_TARGET setting
    Db3AboveTarget = 0b0011,
    /// 4 dB above MAGN_TARGET setting
    Db4AboveTarget = 0b0100,
    /// 5 dB above MAGN_TARGET setting
    Db5AboveTarget = 0b0101,
    /// 6 dB above MAGN_TARGET setting
    Db6AboveTarget = 0b0110,
    /// 7 dB above MAGN_TARGET setting
    Db7AboveTarget = 0b0111,
}

/// Used in [`AGCCTRL0.HYST_LEVEL`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A713%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C248%2C753%2C0%5D)
///
/// Sets the level of hysteresis on the magnitude deviation (internal AGC signal that determine gain changes).
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum HystLevel {
    /// No hysteresis, small symmetric dead zone, high gain
    NoHysteresis,
    /// Low hysteresis, small asymmetric dead zone, medium gain
    LowHysteresis,
    /// Medium hysteresis, medium asymmetric dead zone, medium gain
    MediumHysteresis,
    /// Large hysteresis, large asymmetric dead zone, low gain
    LargeHysteresis,
}

/// Used in [`AGCCTRL0.WAIT_TIME`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A713%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C248%2C753%2C0%5D)
///
/// Sets the number of channel filter samples from a gain adjustment has been made until the AGC algorithm starts accumulating new samples.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum WaitTime {
    /// 8 channel filter samples
    Samples8,
    /// 16 channel filter samples
    Samples16,
    /// 24 channel filter samples
    Samples24,
    /// 32 channel filter samples
    Samples32,
}

/// Used in [`AGCCTRL0.AGC_FREEZE`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A713%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C248%2C753%2C0%5D)
///
/// Control when the AGC gain should be frozen.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum AgcFreeze {
    /// Normal operation. Always adjust gain when required.
    AlwaysAdjust,
    /// The gain setting is frozen when a sync word has been found.
    GainFrozenOnSyncWord,
    /// Manually freeze the analogue gain setting and continue to adjust the digital gain.
    FreezeAnalogueAdjustDigital,
    /// Manually freezes both the analogue and the digital gain setting. Used for manually overriding the gain.
    FreezeAnalogueAndDigital,
}

/// Used in [`AGCCTRL0.FILTER_LENGTH`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A713%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C248%2C753%2C0%5D)
///
/// 2-FSK, 4-FSK, MSK: Sets the averaging length for the amplitude from the channel filter. ASK, OOK: Sets the OOK/ASK decision boundary for OOK/ASK reception.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum FilterLength {
    /// 8 channel filter samples
    Samples8,
    /// 16 channel filter samples
    Samples16,
    /// 24 channel filter samples
    Samples24,
    /// 32 channel filter samples
    Samples32,
}

/// Used in [`WORCTRL.EVENT1`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A839%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C224%2C660%2C0%5D)
///
/// Timeout setting from register block. Decoded to Event 1 timeout. RC oscillator clock frequency equals FXOSC/750, which is 34.7 –36 kHz, depending on crystal frequency.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 3]
pub enum Event1 {
    /// 4 (0.111 – 0.115 ms)
    Time4,
    /// 6 (0.167 – 0.173 ms)
    Time6,
    /// 8 (0.222 – 0.230 ms)
    Time8,
    /// 12 (0.333 – 0.346 ms)
    Time12,
    /// 16 (0.444 – 0.462 ms)
    Time16,
    /// 24 (0.667 – 0.692 ms)
    Time24,
    /// 32 (0.889 – 0.923 ms)
    Time32,
    /// 48 (1.333 – 1.385 ms)
    Time48,
}

/// Used in [`WORCTRL.WOR_RES`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A839%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C224%2C660%2C0%5D)
///
/// Controls the Event 0 resolution as well as maximum timeout of the WOR module and maximum timeout under normal RX operation.
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 2]
pub enum WorRes {
    /// 1 Period
    Period1,
    /// 2^5 Periods
    Period2_5,
    /// 2^10 Periods
    Period2_10,
    /// 2^15 Periods
    Period2_15,
}

/// Used in [`MARCSTATE.MARC_STATE`](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A500%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C197%2C753%2C0%5D)
///
/// Main Radio Control FSM State
#[repr(u8)]
#[derive(Debug, BitfieldSpecifier)]
#[bits = 5]
pub enum MarcState {
    /// `SLEEP`
    Sleep,
    /// `IDLE`
    Idle,
    /// `XOFF`
    Xoff,
    /// `VCOON_MC`
    VcoonMc,
    /// `REGON_MC`
    RegonMc,
    /// `MANCAL`
    Mancal,
    /// `VCOON`
    Vcoon,
    /// `REGON`
    Regon,
    /// `STARTCAL`
    Startcal,
    /// `BWBOOST`
    Bwboost,
    /// `FS_LOCK`
    FsLock,
    /// `IFADCON`
    Ifadcon,
    /// `ENDCAL`
    Endcal,
    /// `RX`
    Rx,
    /// `RX_END`
    RxEnd,
    /// `RX_RST`
    RxRst,
    /// `TXRX_SWITCH`
    TxrxSwitch,
    /// `RXFIFO_OVERFLOW`
    RxFifoOverflow,
    /// `FSTXON`
    Fstxon,
    /// `TX`
    Tx,
    /// `TX_END`
    TxEnd,
    /// `RXTX_SWITCH`
    RxtxSwitch,
    /// `TXFIFO_UNDERFLOW`
    TxfifoUnderflow,
}

/// [`GDO2` Output Pin Configuration](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A256%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C214%2C735%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct IOCFG2 {
    #[skip]
    __: bool,
    pub gdo2_inv: bool,
    pub gdo2_cfg: GDOCfg,
}

/// [`GDO1` Output Pin Configuration](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A256%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C214%2C623%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct IOCFG1 {
    pub gdo_ds: bool,
    pub gdo1_inv: bool,
    pub gdo1_cfg: GDOCfg,
}

/// [`GDO0` Output Pin Configuration](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A256%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C214%2C511%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct IOCFG0 {
    pub temp_sensor_enable: bool,
    pub gdo0_inv: bool,
    pub gdo0_cfg: GDOCfg,
}

/// [RX FIFO and TX FIFO Thresholds](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A321%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C206%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct FIFOTHR {
    #[skip]
    __: bool,
    pub adc_retention: bool,
    pub close_in_rx: RXAttenuation,
    pub fifo_thr: FifoThreshold,
}

/// [Packet Automation Control 1](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A584%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C215%2C513%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct PKTCTRL1 {
    pub pqt: B3,
    #[skip]
    __: bool,
    pub crc_autoflush: bool,
    pub append_status: bool,
    pub adr_chk: AdrChk,
}

/// [Packet Automation Control 0](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A591%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C215%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct PKTCTRL0 {
    #[skip]
    __: bool,
    pub white_data: bool,
    pub pkt_format: PktFormat,
    #[skip]
    __: bool,
    pub crc_en: bool,
    pub length_config: LengthConfig,
}

/// [Frequency Synthesizer Control 1](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A578%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C210%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct FSCTRL1 {
    #[skip]
    __: B3,
    pub freq_if: B5,
}

/// [Frequency Synthesizer Control 0](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A578%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C210%2C581%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct FSCTRL0 {
    pub freq_off: B8,
}

/// [Modem Configuration 4](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A553%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C228%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct MDMCFG4 {
    pub chanbw_e: B2,
    pub chanbw_m: B2,
    pub drate_e: B4,
}

/// [Modem Configuration 3](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A553%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C228%2C580%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct MDMCFG3 {
    pub drate_m: B8,
}

/// [Modem Configuration 2](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A279%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C228%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct MDMCFG2 {
    pub dem_dcfilt_off: bool,
    pub mod_format: ModFormat,
    pub manchester_en: bool,
    pub sync_mode: SyncMode,
}

/// [Modem Configuration 1](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A603%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C230%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct MDMCFG1 {
    pub fec_en: bool,
    pub num_preamble: NumPreamble,
    #[skip]
    __: B2,
    pub chanspc_e: B2,
}

/// [Modem Configuration 0](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A603%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C230%2C448%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct MDMCFG0 {
    pub chanspc_m: B8,
}

/// [Modem Deviation Setting](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A682%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C224%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct DEVIATN {
    #[skip]
    __: B1,
    pub deviation_e: B3,
    #[skip]
    __: B1,
    pub deviation_m: B3,
}

/// [Main Radio Control StateMachine Configuration 2](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A285%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C174%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct MCSM2 {
    #[skip]
    __: B3,
    pub rx_time_rssi: bool,
    pub rx_time_qual: bool,
    pub rx_time: RxTime,
}

/// [Main Radio Control StateMachine Configuration 1](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A628%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C175%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct MCSM1 {
    #[skip]
    __: B2,
    pub cca_mode: CcaMode,
    pub rxoff_mode: RxoffMode,
    pub txoff_mode: TxoffMode,
}

/// [Main Radio Control StateMachine Configuration 0](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A283%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C175%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct MCSM0 {
    #[skip]
    __: B2,
    pub fs_autocal: FsAutocal,
    pub po_timeout: PoTimeout,
    pub pin_ctrl_en: bool,
    pub xosc_force_on: bool,
}

/// [Frequency Offset Compensation Configuration](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A569%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C174%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct FOCCFG {
    #[skip]
    __: B2,
    pub foc_bs_cs_gate: bool,
    pub foc_pre_k: FocPreK,
    pub foc_post_k: FocPostK,
    pub foc_limit: FocLimit,
}

/// [Bit Synchronization Configuration](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A1077%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C208%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct BSCFG {
    pub bs_pre_ki: BsPreKi,
    pub bs_pre_kp: BsPreKp,
    pub bs_post_ki: BsPostKi,
    pub bs_post_kp: BsPostKp,
    pub bs_limit: BsLimit,
}

/// [AGC Control 2](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A734%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C248%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct AGCCTRL2 {
    pub max_dvga_gain: MaxDvgaGain,
    pub max_lna_gain: MaxLnaGain,
    pub magn_target: MagnTarget,
}

/// [AGC Control 1](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A737%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C248%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct AGCCTRL1 {
    #[skip]
    __: B1,
    pub agc_lna_priority: bool,
    pub carrier_sense_rel_thr: CarrierSenseRelThr,
    pub carrier_sense_abs_thr: CarrierSenseAbsThr,
}

/// [AGC Control 0](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A713%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C248%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct AGCCTRL0 {
    pub hyst_level: HystLevel,
    pub wait_time: WaitTime,
    pub agc_freeze: AgcFreeze,
    pub filter_length: FilterLength,
}

/// [Wake On Radio Control](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A839%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C224%2C660%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct WORCTRL {
    pub rc_pd: bool,
    pub event1: Event1,
    pub rc_cal: bool,
    #[skip]
    __: B1,
    pub wor_res: WorRes,
}

/// [Front End RX Configuration](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A527%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C220%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct FREND1 {
    pub lna_current: B2,
    pub lna2mix_current: B2,
    pub lodiv_buf_current_rx: B2,
    pub mix_current: B2,
}

/// [Front End TX Configuration](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A527%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C220%2C630%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct FREND0 {
    #[skip]
    __: B2,
    pub lodiv_buf_current_tx: B2,
    #[skip]
    __: B1,
    pub pa_power: B3,
}

/// [Frequency Synthesizer Calibration 3](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A527%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C205%2C430%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct FSCAL3 {
    pub fscal3: B2,
    pub chp_curr_cal_en: B2,
    pub fscal3_res: B4,
}

/// [Frequency Synthesizer Calibration 2](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A493%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C205%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct FSCAL2 {
    #[skip]
    __: B2,
    pub vco_core_h_en: bool,
    pub fscal2_res: B5,
}

/// [Frequency Synthesizer Calibration 1](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A493%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C205%2C594%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct FSCAL1 {
    #[skip]
    __: B2,
    pub fscal1_res: B6,
}

/// [Frequency Synthesizer Calibration 0](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A493%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C205%2C446%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct FSCAL0 {
    #[skip]
    __: B1,
    pub fscal0_res: B7,
}

/// [RC Oscillator Configuration 1](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A493%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C217%2C345%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct RCCTRL1 {
    #[skip]
    __: B1,
    pub rcctrl1: B7,
}

/// [RC Oscillator Configuration 0](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A493%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C217%2C254%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct RCCTRL0 {
    #[skip]
    __: B1,
    pub rcctrl0: B7,
}

/// [Various Test Settings](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A260%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C239%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct TEST0 {
    pub test0a: B6,
    pub vco_sel_cal_en: bool,
    pub test0b: bool,
}

/// [Demodulator Estimate for Link Quality](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A260%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C222%2C342%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct LQI {
    pub crc_ok: bool,
    pub lqi_est: B7,
}

/// [Main Radio Control State Machine State](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A500%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C197%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct MARCSTATE {
    #[skip]
    __: B3,
    pub marc_state: MarcState,
}

/// [Current GDOx Status and Packet Status](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A502%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C198%2C753%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct PKTSTATUS {
    pub crc_ok: bool,
    pub cs: bool,
    pub pqt_reached: bool,
    pub cca: bool,
    pub sfd: bool,
    pub gdo2: bool,
    #[skip]
    __: B1,
    pub gdo0: bool,
}

/// [RX Overflow and Number of Bytes](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A502%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C226%2C269%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct RXBYTES {
    pub rxfifo_overflow: bool,
    pub num_rxbytes: B7,
}

/// [TX Underflow and Number of Bytes](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A502%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C224%2C361%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
pub struct TXBYTES {
    pub txfifo_underflow: bool,
    pub num_txbytes: B7,
}

/// [Last RC Oscillator Calibration Result](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A502%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C191%2C178%2C0%5D)
#[bitfield]
#[derive(Debug, Copy, Clone, BitfieldSpecifier)]
#[allow(non_camel_case_types)]
pub struct RCCTRL_STATUS {
    #[skip]
    __: B1,
    pub status: B7,
}

/// [Complete set of all CC1101 registers](https://www.ti.com/lit/ds/symlink/cc1101.pdf#%5B%7B%22num%22%3A488%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C69%2C127%2C0%5D)
///
/// This is exposed to the closure passed to [`crate::CC1101::initialize`] and keeps a cached shadow file
/// of all registers to avoid having to read/modify/write.
#[bitfield]
#[derive(Debug, Copy, Clone)]
pub struct Regs {
    pub iocfg2: IOCFG2,
    pub iocfg1: IOCFG1,
    pub iocfg0: IOCFG0,
    pub fifothr: FIFOTHR,
    pub sync: B16,
    pub pktlen: B8,
    pub pktctrl1: PKTCTRL1,
    pub pktctrl0: PKTCTRL0,
    pub addr: B8,
    pub channr: B8,
    pub fsctrl1: FSCTRL1,
    pub fsctrl0: FSCTRL0,
    #[skip]
    __: B2,
    pub freq: B22,
    pub mdmcfg4: MDMCFG4,
    pub mdmcfg3: MDMCFG3,
    pub mdmcfg2: MDMCFG2,
    pub mdmcfg1: MDMCFG1,
    pub mdmcfg0: MDMCFG0,
    pub deviatn: DEVIATN,
    pub mcsm2: MCSM2,
    pub mcsm1: MCSM1,
    pub mcsm0: MCSM0,
    pub foccfg: FOCCFG,
    pub bscfg: BSCFG,
    pub agcctrl2: AGCCTRL2,
    pub agcctrl1: AGCCTRL1,
    pub agcctrl0: AGCCTRL0,
    pub worevt: B16,
    pub worctrl: WORCTRL,
    pub frend1: FREND1,
    pub frend0: FREND0,
    pub fscal3: FSCAL3,
    pub fscal2: FSCAL2,
    pub fscal1: FSCAL1,
    pub fscal0: FSCAL0,
    pub rcctrl1: RCCTRL1,
    pub rcctrl0: RCCTRL0,
    pub fstest: B8,
    pub ptest: B8,
    pub agctest: B8,
    pub test2: B8,
    pub test1: B8,
    pub test0: TEST0,
}

assert_eq_size!([u8; 47], Regs);
