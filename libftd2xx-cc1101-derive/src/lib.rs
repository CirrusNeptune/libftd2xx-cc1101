extern crate proc_macro;
use proc_macro::TokenStream;
use quote::{quote, quote_spanned};
use syn::spanned::Spanned as _;

macro_rules! format_err {
    ( $spanned:expr, $($msg:tt)* ) => {{
        ::syn::Error::new(
            <_ as ::syn::spanned::Spanned>::span(&$spanned),
            format_args!($($msg)*)
        )
    }}
}

#[proc_macro_derive(CC1101Commands)]
pub fn generate_cc1101_commands(input: TokenStream) -> TokenStream {
    let input = syn::parse::<syn::DeriveInput>(input).unwrap();
    match syn::Item::from(input) {
        syn::Item::Enum(enum_item) => {
            let span = enum_item.span();
            let variants = enum_item.variants.iter().map(|variant| {
                let enum_ident = &variant.ident;
                let name = enum_ident.to_string().to_lowercase();
                let ident = syn::Ident::new(&name, variant.ident.span());
                quote!(
                    ($passthru:tt {#ident(); $($tail:tt)*} -> [$($out:tt)*]) => {
                        mpsse!($passthru {command_strobe(::libftd2xx_cc1101::Command::#enum_ident); $($tail)*} -> [$($out)*]);
                    };
                    ($passthru:tt {let $stat_id:ident = #ident(); $($tail:tt)*} -> [$($out:tt)*]) => {
                        mpsse!($passthru {let $stat_id = command_strobe(::libftd2xx_cc1101::Command::#enum_ident); $($tail)*} -> [$($out)*]);
                    };
                )
            });
            TokenStream::from(quote_spanned!(span=>
                /// Internally-shadowed macro to support generated [`mpsse`] macro.
                #[macro_export]
                macro_rules! __mpsse_base2 {
                    #( #variants )*

                    // Everything else handled by base implementation.
                    ($($tokens:tt)*) => {
                        ::libftd2xx_cc1101::__mpsse_base!($($tokens)*);
                    };
                }
            ))
        }
        other_input => TokenStream::from(
            format_err!(other_input, "only enum types supported").to_compile_error(),
        ),
    }
}

#[proc_macro_derive(CC1101Regs, attributes(gentype))]
pub fn generate_cc1101_regs(input: TokenStream) -> TokenStream {
    let input = syn::parse::<syn::DeriveInput>(input).unwrap();
    match syn::Item::from(input) {
        syn::Item::Enum(enum_item) => {
            let span = enum_item.span();
            let variants = enum_item.variants.iter().map(|variant| {
                let enum_ident = &variant.ident;
                let name = enum_ident.to_string().to_lowercase();
                let ident = syn::Ident::new(&name, variant.ident.span());
                let set_ident = syn::Ident::new(&format!("set_{}", name), variant.ident.span());
                for attr in &variant.attrs {
                    if attr.path.is_ident("gentype") {
                        let path = &attr.path;
                        let args = &attr.tokens;
                        let gentype: syn::MetaList =
                            syn::parse2::<_>(quote! { #path #args }).unwrap();
                        let span = gentype.span();
                        match gentype.nested.first().unwrap() {
                            syn::NestedMeta::Meta(meta) => {
                                match meta {
                                    syn::Meta::Path(gentype_path) => {
                                        return quote!(
                                            ($passthru:tt {let $data_id:ident = #ident(); $($tail:tt)*} -> [$($out:tt)*]) => {
                                                mpsse!($passthru @{let $data_id = read_reg(::libftd2xx_cc1101::RegAddrs::#enum_ident as u8 + 0x80, #enum_ident, #gentype_path); $($tail)*} -> [$($out)*]);
                                            };
                                            ($passthru:tt {let ($stat_id:ident, $data_id:ident) = #ident(); $($tail:tt)*} -> [$($out:tt)*]) => {
                                                mpsse!($passthru @{let ($stat_id, $data_id) = read_reg(::libftd2xx_cc1101::RegAddrs::#enum_ident as u8 + 0x80, #enum_ident, #gentype_path); $($tail)*} -> [$($out)*]);
                                            };
                                            ($passthru:tt {#set_ident($data:expr); $($tail:tt)*} -> [$($out:tt)*]) => {
                                                mpsse!($passthru @{write_reg(::libftd2xx_cc1101::RegAddrs::#enum_ident as u8, #enum_ident, $data, #gentype_path); $($tail)*} -> [$($out)*]);
                                            };
                                            ($passthru:tt {let $stat_id:ident = #set_ident($data:expr); $($tail:tt)*} -> [$($out:tt)*]) => {
                                                mpsse!($passthru @{let $stat_id = write_reg(::libftd2xx_cc1101::RegAddrs::#enum_ident as u8, #enum_ident, $data, #gentype_path); $($tail)*} -> [$($out)*]);
                                            };
                                        );
                                    },
                                    _ => {}
                                }
                            },
                            _ => {}
                        }
                        return format_err!(
                            span,
                            "encountered invalid value type for #[gentype(...)]"
                        ).to_compile_error();
                    }
                }
                format_err!(variant, "#[gentype(...)] attr not provided").to_compile_error()
            });
            TokenStream::from(quote_spanned!(span=>
                /// Internally-shadowed macro to support generated [`mpsse`] macro.
                #[macro_export]
                macro_rules! __mpsse_base3 {
                    #( #variants )*

                    // Everything else handled by __mpsse_base2 implementation.
                    ($($tokens:tt)*) => {
                        ::libftd2xx_cc1101::__mpsse_base2!($($tokens)*);
                    };
                }
            ))
        }
        other_input => TokenStream::from(
            format_err!(other_input, "only enum types supported").to_compile_error(),
        ),
    }
}

#[proc_macro_derive(CC1101ReadRegs, attributes(gentype))]
pub fn generate_cc1101_read_regs(input: TokenStream) -> TokenStream {
    let input = syn::parse::<syn::DeriveInput>(input).unwrap();
    match syn::Item::from(input) {
        syn::Item::Enum(enum_item) => {
            let span = enum_item.span();
            let variants = enum_item.variants.iter().map(|variant| {
                let enum_ident = &variant.ident;
                let name = enum_ident.to_string().to_lowercase();
                let ident = syn::Ident::new(&name, variant.ident.span());
                for attr in &variant.attrs {
                    if attr.path.is_ident("gentype") {
                        let path = &attr.path;
                        let args = &attr.tokens;
                        let gentype: syn::MetaList =
                            syn::parse2::<_>(quote! { #path #args }).unwrap();
                        let span = gentype.span();
                        match gentype.nested.first().unwrap() {
                            syn::NestedMeta::Meta(meta) => {
                                match meta {
                                    syn::Meta::Path(gentype_path) => {
                                        return quote!(
                                            ($passthru:tt {let $data_id:ident = #ident(); $($tail:tt)*} -> [$($out:tt)*]) => {
                                                mpsse!($passthru @{let $data_id = read_reg(::libftd2xx_cc1101::ReadRegAddrs::#enum_ident as u8, #enum_ident, #gentype_path); $($tail)*} -> [$($out)*]);
                                            };
                                            ($passthru:tt {let ($stat_id:ident, $data_id:ident) = #ident(); $($tail:tt)*} -> [$($out:tt)*]) => {
                                                mpsse!($passthru @{let ($stat_id, $data_id) = read_reg(::libftd2xx_cc1101::ReadRegAddrs::#enum_ident as u8, #enum_ident, #gentype_path); $($tail)*} -> [$($out)*]);
                                            };
                                        );
                                    },
                                    _ => {}
                                }
                            },
                            _ => {}
                        }
                        return format_err!(
                            span,
                            "encountered invalid value type for #[gentype(...)]"
                        ).to_compile_error();
                    }
                }
                format_err!(variant, "#[gentype(...)] attr not provided").to_compile_error()
            });
            TokenStream::from(quote_spanned!(span=>
                /// Extended compile-time mpsse command array generation macro.
                ///
                /// See documentation of [`libftd2xx::mpsse`] for general information on how this
                /// macro functions.
                ///
                /// # Commands
                ///
                /// In addition to the commands in [`libftd2xx::mpsse`], the following
                /// CC1101-specific commands are available:
                ///
                /// * Command Strobes
                ///     * [`sres()`][`crate::Command::SRES`]
                ///     * [`sfstxon()`][`crate::Command::SFSTXON`]
                ///     * [`sxoff()`][`crate::Command::SXOFF`]
                ///     * [`scal()`][`crate::Command::SCAL`]
                ///     * [`srx()`][`crate::Command::SRX`]
                ///     * [`stx()`][`crate::Command::STX`]
                ///     * [`sidle()`][`crate::Command::SIDLE`]
                ///     * [`swor()`][`crate::Command::SWOR`]
                ///     * [`spwd()`][`crate::Command::SPWD`]
                ///     * [`sfrx()`][`crate::Command::SFRX`]
                ///     * [`sftx()`][`crate::Command::SFTX`]
                ///     * [`sworrst()`][`crate::Command::SWORRST`]
                ///     * [`snop()`][`crate::Command::SNOP`]
                /// * Configuration Register Access
                ///     * `read_all_regs()` `->` [`Regs`][`crate::regs::Regs`]
                ///     * `write_all_regs(regs: Regs)`
                ///     * [`iocfg2()`][`crate::RegAddrs::IOCFG2`] `->` [`IOCFG2`][`crate::regs::IOCFG2`]
                ///     * [`set_iocfg2(v: IOCFG2)`][`crate::RegAddrs::IOCFG2`]
                ///     * [`iocfg1()`][`crate::RegAddrs::IOCFG1`] `->` [`IOCFG1`][`crate::regs::IOCFG1`]
                ///     * [`set_iocfg1(v: IOCFG1)`][`crate::RegAddrs::IOCFG1`]
                ///     * [`iocfg0()`][`crate::RegAddrs::IOCFG0`] `->` [`IOCFG0`][`crate::regs::IOCFG0`]
                ///     * [`set_iocfg0(v: IOCFG0)`][`crate::RegAddrs::IOCFG0`]
                ///     * [`fifothr()`][`crate::RegAddrs::FIFOTHR`] `->` [`FIFOTHR`][`crate::regs::FIFOTHR`]
                ///     * [`set_fifothr(v: FIFOTHR)`][`crate::RegAddrs::FIFOTHR`]
                ///     * [`sync()`][`crate::RegAddrs::SYNC`] `->` [`u16`]
                ///     * [`set_sync(v: u16)`][`crate::RegAddrs::SYNC`]
                ///     * [`pktlen()`][`crate::RegAddrs::PKTLEN`] `->` [`u8`]
                ///     * [`set_pktlen(v: u8)`][`crate::RegAddrs::PKTLEN`]
                ///     * [`pktctrl1()`][`crate::RegAddrs::PKTCTRL1`] `->` [`PKTCTRL1`][`crate::regs::PKTCTRL1`]
                ///     * [`set_pktctrl1(v: PKTCTRL1)`][`crate::RegAddrs::PKTCTRL1`]
                ///     * [`pktctrl0()`][`crate::RegAddrs::PKTCTRL0`] `->` [`PKTCTRL0`][`crate::regs::PKTCTRL0`]
                ///     * [`set_pktctrl0(v: PKTCTRL0)`][`crate::RegAddrs::PKTCTRL0`]
                ///     * [`addr()`][`crate::RegAddrs::ADDR`] `->` [`u8`]
                ///     * [`set_addr(v: u8)`][`crate::RegAddrs::ADDR`]
                ///     * [`channr()`][`crate::RegAddrs::CHANNR`] `->` [`u8`]
                ///     * [`set_channr(v: u8)`][`crate::RegAddrs::CHANNR`]
                ///     * [`fsctrl1()`][`crate::RegAddrs::FSCTRL1`] `->` [`FSCTRL1`][`crate::regs::FSCTRL1`]
                ///     * [`set_fsctrl1(v: FSCTRL1)`][`crate::RegAddrs::FSCTRL1`]
                ///     * [`fsctrl0()`][`crate::RegAddrs::FSCTRL0`] `->` [`FSCTRL0`][`crate::regs::FSCTRL0`]
                ///     * [`set_fsctrl0(v: FSCTRL0)`][`crate::RegAddrs::FSCTRL0`]
                ///     * [`freq()`][`crate::RegAddrs::FREQ`] `->` [`u32`]
                ///     * [`set_freq(v: u32)`][`crate::RegAddrs::FREQ`]
                ///     * [`mdmcfg4()`][`crate::RegAddrs::MDMCFG4`] `->` [`MDMCFG4`][`crate::regs::MDMCFG4`]
                ///     * [`set_mdmcfg4(v: MDMCFG4)`][`crate::RegAddrs::MDMCFG4`]
                ///     * [`mdmcfg3()`][`crate::RegAddrs::MDMCFG3`] `->` [`MDMCFG3`][`crate::regs::MDMCFG3`]
                ///     * [`set_mdmcfg3(v: MDMCFG3)`][`crate::RegAddrs::MDMCFG3`]
                ///     * [`mdmcfg2()`][`crate::RegAddrs::MDMCFG2`] `->` [`MDMCFG2`][`crate::regs::MDMCFG2`]
                ///     * [`set_mdmcfg2(v: MDMCFG2)`][`crate::RegAddrs::MDMCFG2`]
                ///     * [`mdmcfg1()`][`crate::RegAddrs::MDMCFG1`] `->` [`MDMCFG1`][`crate::regs::MDMCFG1`]
                ///     * [`set_mdmcfg1(v: MDMCFG1)`][`crate::RegAddrs::MDMCFG1`]
                ///     * [`mdmcfg0()`][`crate::RegAddrs::MDMCFG0`] `->` [`MDMCFG0`][`crate::regs::MDMCFG0`]
                ///     * [`set_mdmcfg0(v: MDMCFG0)`][`crate::RegAddrs::MDMCFG0`]
                ///     * [`deviatn()`][`crate::RegAddrs::DEVIATN`] `->` [`DEVIATN`][`crate::regs::DEVIATN`]
                ///     * [`set_deviatn(v: DEVIATN)`][`crate::RegAddrs::DEVIATN`]
                ///     * [`mcsm2()`][`crate::RegAddrs::MCSM2`] `->` [`MCSM2`][`crate::regs::MCSM2`]
                ///     * [`set_mcsm2(v: MCSM2)`][`crate::RegAddrs::MCSM2`]
                ///     * [`mcsm1()`][`crate::RegAddrs::MCSM1`] `->` [`MCSM1`][`crate::regs::MCSM1`]
                ///     * [`set_mcsm1(v: MCSM1)`][`crate::RegAddrs::MCSM1`]
                ///     * [`mcsm0()`][`crate::RegAddrs::MCSM0`] `->` [`MCSM0`][`crate::regs::MCSM0`]
                ///     * [`set_mcsm0(v: MCSM0)`][`crate::RegAddrs::MCSM0`]
                ///     * [`foccfg()`][`crate::RegAddrs::FOCCFG`] `->` [`FOCCFG`][`crate::regs::FOCCFG`]
                ///     * [`set_foccfg(v: FOCCFG)`][`crate::RegAddrs::FOCCFG`]
                ///     * [`bscfg()`][`crate::RegAddrs::BSCFG`] `->` [`BSCFG`][`crate::regs::BSCFG`]
                ///     * [`set_bscfg(v: BSCFG)`][`crate::RegAddrs::BSCFG`]
                ///     * [`agcctrl2()`][`crate::RegAddrs::AGCCTRL2`] `->` [`AGCCTRL2`][`crate::regs::AGCCTRL2`]
                ///     * [`set_agcctrl2(v: AGCCTRL2)`][`crate::RegAddrs::AGCCTRL2`]
                ///     * [`agcctrl1()`][`crate::RegAddrs::AGCCTRL1`] `->` [`AGCCTRL1`][`crate::regs::AGCCTRL1`]
                ///     * [`set_agcctrl1(v: AGCCTRL1)`][`crate::RegAddrs::AGCCTRL1`]
                ///     * [`agcctrl0()`][`crate::RegAddrs::AGCCTRL0`] `->` [`AGCCTRL0`][`crate::regs::AGCCTRL0`]
                ///     * [`set_agcctrl0(v: AGCCTRL0)`][`crate::RegAddrs::AGCCTRL0`]
                ///     * [`worevt()`][`crate::RegAddrs::WOREVT`] `->` [`u16`]
                ///     * [`set_worevt(v: u16)`][`crate::RegAddrs::WOREVT`]
                ///     * [`worctrl()`][`crate::RegAddrs::WORCTRL`] `->` [`WORCTRL`][`crate::regs::WORCTRL`]
                ///     * [`set_worctrl(v: WORCTRL)`][`crate::RegAddrs::WORCTRL`]
                ///     * [`frend1()`][`crate::RegAddrs::FREND1`] `->` [`FREND1`][`crate::regs::FREND1`]
                ///     * [`set_frend1(v: FREND1)`][`crate::RegAddrs::FREND1`]
                ///     * [`frend0()`][`crate::RegAddrs::FREND0`] `->` [`FREND0`][`crate::regs::FREND0`]
                ///     * [`set_frend0(v: FREND0)`][`crate::RegAddrs::FREND0`]
                ///     * [`fscal3()`][`crate::RegAddrs::FSCAL3`] `->` [`FSCAL3`][`crate::regs::FSCAL3`]
                ///     * [`set_fscal3(v: FSCAL3)`][`crate::RegAddrs::FSCAL3`]
                ///     * [`fscal2()`][`crate::RegAddrs::FSCAL2`] `->` [`FSCAL2`][`crate::regs::FSCAL2`]
                ///     * [`set_fscal2(v: FSCAL2)`][`crate::RegAddrs::FSCAL2`]
                ///     * [`fscal1()`][`crate::RegAddrs::FSCAL1`] `->` [`FSCAL1`][`crate::regs::FSCAL1`]
                ///     * [`set_fscal1(v: FSCAL1)`][`crate::RegAddrs::FSCAL1`]
                ///     * [`fscal0()`][`crate::RegAddrs::FSCAL0`] `->` [`FSCAL0`][`crate::regs::FSCAL0`]
                ///     * [`set_fscal0(v: FSCAL0)`][`crate::RegAddrs::FSCAL0`]
                ///     * [`rcctrl1()`][`crate::RegAddrs::RCCTRL1`] `->` [`RCCTRL1`][`crate::regs::RCCTRL1`]
                ///     * [`set_rcctrl1(v: RCCTRL1)`][`crate::RegAddrs::RCCTRL1`]
                ///     * [`rcctrl0()`][`crate::RegAddrs::RCCTRL0`] `->` [`RCCTRL0`][`crate::regs::RCCTRL0`]
                ///     * [`set_rcctrl0(v: RCCTRL0)`][`crate::RegAddrs::RCCTRL0`]
                ///     * [`fstest()`][`crate::RegAddrs::FSTEST`] `->` [`u8`]
                ///     * [`set_fstest(v: u8)`][`crate::RegAddrs::FSTEST`]
                ///     * [`ptest()`][`crate::RegAddrs::PTEST`] `->` [`u8`]
                ///     * [`set_ptest(v: u8)`][`crate::RegAddrs::PTEST`]
                ///     * [`agctest()`][`crate::RegAddrs::AGCTEST`] `->` [`u8`]
                ///     * [`set_agctest(v: u8)`][`crate::RegAddrs::AGCTEST`]
                ///     * [`test2()`][`crate::RegAddrs::TEST2`] `->` [`u8`]
                ///     * [`set_test2(v: u8)`][`crate::RegAddrs::TEST2`]
                ///     * [`test1()`][`crate::RegAddrs::TEST1`] `->` [`u8`]
                ///     * [`set_test1(v: u8)`][`crate::RegAddrs::TEST1`]
                ///     * [`test0()`][`crate::RegAddrs::TEST0`] `->` [`TEST0`][`crate::regs::TEST0`]
                ///     * [`set_test0(v: TEST0)`][`crate::RegAddrs::TEST0`]
                /// * Status Register Access
                ///     * [`partnum()`][`crate::ReadRegAddrs::PARTNUM`] `->` [`u8`]
                ///     * [`version()`][`crate::ReadRegAddrs::VERSION`] `->` [`u8`]
                ///     * [`freqtest()`][`crate::ReadRegAddrs::FREQTEST`] `->` [`u8`]
                ///     * [`lqi()`][`crate::ReadRegAddrs::LQI`] `->` [`LQI`][`crate::regs::LQI`]
                ///     * [`rssi()`][`crate::ReadRegAddrs::RSSI`] `->` [`u8`]
                ///     * [`marcstate()`][`crate::ReadRegAddrs::MARCSTATE`] `->` [`MARCSTATE`][`crate::regs::MARCSTATE`]
                ///     * [`wortime()`][`crate::ReadRegAddrs::WORTIME`] `->` [`u8`]
                ///     * [`pktstatus()`][`crate::ReadRegAddrs::PKTSTATUS`] `->` [`PKTSTATUS`][`crate::regs::PKTSTATUS`]
                ///     * [`vco_vc_dac()`][`crate::ReadRegAddrs::VCO_VC_DAC`] `->` [`u8`]
                ///     * [`txbytes()`][`crate::ReadRegAddrs::TXBYTES`] `->` [`TXBYTES`][`crate::regs::TXBYTES`]
                ///     * [`rxbytes()`][`crate::ReadRegAddrs::RXBYTES`] `->` [`RXBYTES`][`crate::regs::RXBYTES`]
                ///     * [`rcctrl1_status()`][`crate::ReadRegAddrs::RCCTRL1_STATUS`] `->` [`RCCTRL_STATUS`][`crate::regs::RCCTRL_STATUS`]
                ///     * [`rcctrl0_status()`][`crate::ReadRegAddrs::RCCTRL0_STATUS`] `->` [`RCCTRL_STATUS`][`crate::regs::RCCTRL_STATUS`]
                /// * FIFO Access
                ///     * `read_rx_fifo(len: u16)` `->` `[u8; N]`
                ///     * `write_tx_fifo(data: [u8; N])`
                /// * PA Table Access
                ///     * `read_pa_table(len: u16)` `->` `[u8; N]`
                ///     * `write_pa_table(data: [u8; N])`
                #[macro_export]
                macro_rules! mpsse {
                    #( #variants )*

                    // Everything else handled by __mpsse_base3 implementation.
                    ($($tokens:tt)*) => {
                        ::libftd2xx_cc1101::__mpsse_base3!($($tokens)*);
                    };
                }
            ))
        }
        other_input => TokenStream::from(
            format_err!(other_input, "only enum types supported").to_compile_error(),
        ),
    }
}
