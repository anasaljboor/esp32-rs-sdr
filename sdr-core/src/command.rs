//! Command-line parsing for the burst protocol (`docs/protocol.md`).
//!
//! Parsing is purely syntactic plus the compile-time range checks the C
//! firmware applies before it touches the radio. Anything that does not match
//! becomes [`Command::Unknown`], which the firmware answers with
//! `ERR command`, exactly like the C receivers' fall-through.

use crate::parse;
use crate::tuning;

/// Payload format of a capture reply.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    /// `CAP`: raw 32-bit dump words.
    Raw32,
    /// `CAP16`: signed 8-bit I then 8-bit (negated) Q.
    Iq8,
    /// `CAP20`: 10-bit I and Q packed LSB first.
    Iq10,
}

impl Format {
    /// The C firmware's numeric format code (0, 16 or 20).
    pub const fn code(self) -> u32 {
        match self {
            Format::Raw32 => 0,
            Format::Iq8 => 16,
            Format::Iq10 => 20,
        }
    }

    fn from_bits(bits: u32) -> Option<Self> {
        match bits {
            16 => Some(Format::Iq8),
            20 => Some(Format::Iq10),
            _ => None,
        }
    }
}

/// Target-specific limits the parser enforces.
#[derive(Clone, Copy, Debug)]
pub struct Limits {
    /// Largest snapshot length in samples (`IQ_WORDS`).
    pub max_words: u32,
    /// Highest accepted rate code (`BURST_MAX_RATE`).
    pub max_rate: u32,
    /// Narrowest and widest `BANDWIDTH` in MHz.
    pub bandwidth_mhz: (u32, u32),
}

/// Smallest snapshot the protocol accepts.
pub const MIN_WORDS: u32 = 256;

/// One parsed command line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    Sync(u64),
    Info,
    Caps,
    Version,
    Transport,
    Release,
    BaudQuery,
    /// `BAUD 1000000` or `BAUD 2000000`.
    Baud(u32),
    /// Any other `BAUD ...` line.
    BaudInvalid,
    Range,
    Limits,
    Freq(u32),
    /// `FOFS <kHz>`: PLL offset applied from the next tune.
    Fofs(i32),
    /// `FOFS` with a malformed or out-of-range argument (`ERR args`).
    FofsInvalid,
    GainQuery,
    GainHardware,
    /// Manual index; the firmware checks it against the calibrated maximum.
    GainManual(u32),
    /// Approximate analog bandwidth in MHz, 0 = widest.
    Bandwidth(u32),
    LpfAuto,
    Lpf(u32),
    LpfQuery,
    Capture { samples: u32, rate: u32, format: Format },
    RxRun { samples: u32, rate: u32, repeats: u32, format: Format },
    Unknown,
}

impl Command {
    pub fn parse(line: &str, limits: &Limits) -> Command {
        use Command::*;
        let samples_ok = |n: u32| (MIN_WORDS..=limits.max_words).contains(&n);
        let (word, rest) = line.split_once(' ').unwrap_or((line, ""));
        match (word, rest) {
            ("INFO", "") => Info,
            ("CAPS", "") => Caps,
            ("VERSION?", "") => Version,
            ("TRANSPORT?", "") => Transport,
            ("RELEASE", "") => Release,
            ("RANGE?", "") => Range,
            ("LIMITS?", "") => Limits,
            ("GAIN?", "") => GainQuery,
            ("LPF?", "") => LpfQuery,
            ("BAUD?", "") => BaudQuery,
            ("BAUD", _) => match rest {
                "1000000" => Baud(1_000_000),
                "2000000" => Baud(2_000_000),
                _ => BaudInvalid,
            },
            ("SYNC", _) => parse::uint64(rest).map_or(Unknown, Sync),
            ("FREQ", _) => match parse::exact::<1>(rest) {
                Some([mhz]) if tuning::frequency_valid(mhz) => Freq(mhz),
                _ => Unknown,
            },
            ("FOFS", _) => parse::int_in(rest, -1000, 1000).map_or(FofsInvalid, Fofs),
            ("GAIN", "HARDWARE") => GainHardware,
            ("GAIN", _) => match rest.strip_prefix("MANUAL ").and_then(parse::exact::<1>) {
                Some([code]) => GainManual(code),
                None => Unknown,
            },
            ("BANDWIDTH", _) => match parse::exact::<1>(rest) {
                Some([0]) => Bandwidth(0),
                Some([mhz]) if (limits.bandwidth_mhz.0..=limits.bandwidth_mhz.1).contains(&mhz) => {
                    Bandwidth(mhz)
                }
                _ => Unknown,
            },
            ("LPF", "AUTO") => LpfAuto,
            ("LPF", _) => match parse::exact::<1>(rest) {
                Some([code]) if code <= 63 => Lpf(code),
                _ => Unknown,
            },
            ("CAP" | "CAP16" | "CAP20", _) => {
                let format = match word {
                    "CAP" => Format::Raw32,
                    "CAP16" => Format::Iq8,
                    _ => Format::Iq10,
                };
                match parse::exact::<2>(rest) {
                    Some([n, rate]) if samples_ok(n) && rate <= limits.max_rate => Capture {
                        samples: n,
                        rate,
                        format,
                    },
                    _ => Unknown,
                }
            }
            ("RXRUN", _) => match parse::exact::<4>(rest) {
                Some([n, rate, repeats, bits])
                    if samples_ok(n)
                        && rate <= limits.max_rate
                        && (1..=1000).contains(&repeats) =>
                {
                    Format::from_bits(bits).map_or(Unknown, |format| RxRun {
                        samples: n,
                        rate,
                        repeats,
                        format,
                    })
                }
                _ => Unknown,
            },
            _ => Unknown,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const S3: Limits = Limits {
        max_words: 16380,
        max_rate: 6,
        bandwidth_mhz: (13, 69),
    };

    fn p(s: &str) -> Command {
        Command::parse(s, &S3)
    }

    #[test]
    fn session_commands() {
        assert_eq!(p("INFO"), Command::Info);
        assert_eq!(p("INFO x"), Command::Unknown);
        assert_eq!(p("SYNC 18446744073709551615"), Command::Sync(u64::MAX));
        assert_eq!(p("SYNC -1"), Command::Unknown);
        assert_eq!(p("RELEASE"), Command::Release);
        assert_eq!(p("BAUD 1000000"), Command::Baud(1_000_000));
        assert_eq!(p("BAUD 115200"), Command::BaudInvalid);
        assert_eq!(p("BAUD"), Command::BaudInvalid);
    }

    #[test]
    fn captures() {
        assert_eq!(
            p("CAP16 4096 0"),
            Command::Capture { samples: 4096, rate: 0, format: Format::Iq8 }
        );
        assert_eq!(
            p("CAP20 16380 6"),
            Command::Capture { samples: 16380, rate: 6, format: Format::Iq10 }
        );
        assert_eq!(
            p("CAP 256 1"),
            Command::Capture { samples: 256, rate: 1, format: Format::Raw32 }
        );
        // Rates 2..5 pass the parser; the S3 capture path answers `ERR rate`.
        assert!(matches!(p("CAP16 4096 3"), Command::Capture { rate: 3, .. }));
        assert_eq!(p("CAP16 255 0"), Command::Unknown);
        assert_eq!(p("CAP16 16381 0"), Command::Unknown);
        assert_eq!(p("CAP16 4096 7"), Command::Unknown);
        assert_eq!(p("CAP16 4096"), Command::Unknown);
    }

    #[test]
    fn rxrun() {
        assert_eq!(
            p("RXRUN 4096 0 10 16"),
            Command::RxRun { samples: 4096, rate: 0, repeats: 10, format: Format::Iq8 }
        );
        assert_eq!(p("RXRUN 4096 0 0 16"), Command::Unknown);
        assert_eq!(p("RXRUN 4096 0 1001 16"), Command::Unknown);
        assert_eq!(p("RXRUN 4096 0 1 0"), Command::Unknown);
    }

    #[test]
    fn receiver_settings() {
        assert_eq!(p("FREQ 2442"), Command::Freq(2442));
        assert_eq!(p("FREQ 99"), Command::Unknown);
        assert_eq!(p("FREQ 6001"), Command::Unknown);
        assert_eq!(p("FOFS -250"), Command::Fofs(-250));
        assert_eq!(p("FOFS 2000"), Command::FofsInvalid);
        assert_eq!(p("GAIN MANUAL 40"), Command::GainManual(40));
        assert_eq!(p("GAIN MANUAL"), Command::Unknown);
        assert_eq!(p("GAIN HARDWARE"), Command::GainHardware);
        assert_eq!(p("BANDWIDTH 0"), Command::Bandwidth(0));
        assert_eq!(p("BANDWIDTH 20"), Command::Bandwidth(20));
        assert_eq!(p("BANDWIDTH 5"), Command::Unknown);
        assert_eq!(p("LPF 63"), Command::Lpf(63));
        assert_eq!(p("LPF 64"), Command::Unknown);
        assert_eq!(p("LPF AUTO"), Command::LpfAuto);
    }
}
