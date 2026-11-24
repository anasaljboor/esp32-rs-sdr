//! A fixed-capacity text buffer for reply lines, usable with `write!`.

use core::fmt;

pub struct Line<const N: usize> {
    buf: [u8; N],
    len: usize,
    overflowed: bool,
}

impl<const N: usize> Line<N> {
    pub const fn new() -> Self {
        Self { buf: [0; N], len: 0, overflowed: false }
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }

    pub fn as_str(&self) -> &str {
        // Only `write_str` fills the buffer, and it copies whole `&str`s.
        core::str::from_utf8(self.as_bytes()).unwrap_or("")
    }

    /// True if a write did not fit and was dropped.
    pub fn overflowed(&self) -> bool {
        self.overflowed
    }
}

impl<const N: usize> Default for Line<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> fmt::Write for Line<N> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let end = self.len + s.len();
        if end > N {
            self.overflowed = true;
            return Err(fmt::Error);
        }
        self.buf[self.len..end].copy_from_slice(s.as_bytes());
        self.len = end;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::fmt::Write;

    #[test]
    fn formats_and_rejects_overflow() {
        let mut l = Line::<32>::new();
        write!(l, "DATA {} {:08x}", 4096, 0xab).unwrap();
        assert_eq!(l.as_str(), "DATA 4096 000000ab");
        let mut small = Line::<4>::new();
        assert!(write!(small, "hello").is_err());
        assert!(small.overflowed());
        assert_eq!(small.as_str(), "");
    }
}
