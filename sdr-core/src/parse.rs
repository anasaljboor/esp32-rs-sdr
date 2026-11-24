//! Strict decimal parsing for serial commands (`main/common/burst_parse.h`).
//!
//! Unlike `sscanf("%u")`, these reject signs, overflow and stray characters
//! before a command touches receiver state.

/// Parse up to `out.len()` space-separated unsigned decimal fields.
///
/// Returns the number of fields, or `None` for malformed text, overflow or
/// more fields than `out` can hold. Leading, trailing and repeated spaces are
/// allowed; an empty string yields `Some(0)`.
pub fn uints(s: &str, out: &mut [u32]) -> Option<usize> {
    let mut n = 0;
    for field in s.split(' ').filter(|f| !f.is_empty()) {
        if n == out.len() {
            return None;
        }
        out[n] = uint(field)?;
        n += 1;
    }
    Some(n)
}

/// Parse exactly `N` space-separated unsigned fields.
pub fn exact<const N: usize>(s: &str) -> Option<[u32; N]> {
    let mut v = [0u32; N];
    (uints(s, &mut v)? == N).then_some(v)
}

/// Parse one unsigned decimal field with no sign and no other characters.
pub fn uint(field: &str) -> Option<u32> {
    if field.is_empty() || !field.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    field.parse().ok()
}

/// Parse one unsigned 64-bit field (used by `SYNC`).
pub fn uint64(s: &str) -> Option<u64> {
    let s = s.trim_matches(' ');
    if s.is_empty() || !s.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    s.parse().ok()
}

/// Parse exactly one optionally signed decimal integer within `[low, high]`.
pub fn int_in(s: &str, low: i32, high: i32) -> Option<i32> {
    let s = s.trim_matches(' ');
    let (negative, digits) = match s.as_bytes().first()? {
        b'-' => (true, &s[1..]),
        b'+' => (false, &s[1..]),
        _ => (false, s),
    };
    let magnitude = i64::from(uint(digits)?);
    let value = if negative { -magnitude } else { magnitude };
    (i64::from(low)..=i64::from(high))
        .contains(&value)
        .then_some(value as i32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accepts_plain_fields() {
        let mut v = [0; 4];
        assert_eq!(uints("1 22  333 ", &mut v), Some(3));
        assert_eq!(&v[..3], &[1, 22, 333]);
        assert_eq!(uints("", &mut v), Some(0));
    }

    #[test]
    fn rejects_signs_overflow_and_junk() {
        let mut v = [0; 2];
        assert_eq!(uints("+1", &mut v), None);
        assert_eq!(uints("-1", &mut v), None);
        assert_eq!(uints("4294967296", &mut v), None);
        assert_eq!(uints("12x", &mut v), None);
        assert_eq!(uints("1 2 3", &mut v), None);
        assert_eq!(uints("4294967295", &mut v), Some(1));
    }

    #[test]
    fn exact_count() {
        assert_eq!(exact::<2>("4096 0"), Some([4096, 0]));
        assert_eq!(exact::<2>("4096"), None);
        assert_eq!(exact::<2>("4096 0 1"), None);
    }

    #[test]
    fn signed_range() {
        assert_eq!(int_in("-1000", -1000, 1000), Some(-1000));
        assert_eq!(int_in("+5", -1000, 1000), Some(5));
        assert_eq!(int_in("1001", -1000, 1000), None);
        assert_eq!(int_in("--5", -1000, 1000), None);
        assert_eq!(int_in("", -1000, 1000), None);
        assert_eq!(int_in("-", -1000, 1000), None);
    }
}
