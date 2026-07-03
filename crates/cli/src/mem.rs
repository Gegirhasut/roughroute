//! Peak-RSS / total-RAM observability for the `batch` builder (dev/CI only).
//!
//! Linux-only, pure `std::fs` — no libc, no new dependency, and nothing here
//! touches the offline core or the routing runtime (spec §2.1). It answers one
//! question: how much resident memory did building a region actually cost, so
//! a run's headroom against the machine's RAM limit is visible before bigger
//! regions are added (the Austria OOM was RAM during construction, not graph
//! size — docs/DECISIONS.md D18/D20).
//!
//! Peak RSS is read from `VmHWM` in `/proc/self/status` (kibibytes). That is a
//! process-wide high-water mark, so to get a *per-region* number the builder
//! resets it between regions by writing `5` to `/proc/self/clear_refs` (resets
//! the peak down to the current RSS; Linux ≥ 4.0 with `CONFIG_PROC_PAGE_MONITOR`).
//! Where that reset isn't available the number is the process peak so far, and
//! the caller labels it as such.

use std::fs;

/// Total system RAM in KiB (`MemTotal` from `/proc/meminfo`), if readable.
pub(crate) fn total_ram_kib() -> Option<u64> {
    read_status_kib(&fs::read_to_string("/proc/meminfo").ok()?, "MemTotal")
}

/// Peak resident set size of this process in KiB (`VmHWM` from
/// `/proc/self/status`), if readable. Linux tracks this as a high-water mark,
/// so it only ever rises unless reset via [`reset_peak_rss`].
pub(crate) fn peak_rss_kib() -> Option<u64> {
    read_status_kib(&fs::read_to_string("/proc/self/status").ok()?, "VmHWM")
}

/// Reset this process's peak-RSS high-water mark (`VmHWM`) down to its current
/// RSS, so a subsequent [`peak_rss_kib`] reflects only work done after this
/// call. Returns whether the reset was accepted; on failure the peak keeps
/// accumulating across regions and the caller reports it as process-wide.
pub(crate) fn reset_peak_rss() -> bool {
    fs::write("/proc/self/clear_refs", "5").is_ok()
}

/// Parse a `Key:   <number> kB` line (the `/proc` status format) and return the
/// number in KiB. The `kB` unit in these files means KiB (1024 bytes).
fn read_status_kib(contents: &str, key: &str) -> Option<u64> {
    for line in contents.lines() {
        let Some(rest) = line.strip_prefix(key) else { continue };
        // Require the field to be exactly `key:` (not a longer key sharing the
        // prefix), then take the first integer that follows.
        let Some(rest) = rest.strip_prefix(':') else { continue };
        if let Some(num) = rest.split_whitespace().next() {
            if let Ok(v) = num.parse::<u64>() {
                return Some(v);
            }
        }
    }
    None
}

/// Human-readable memory size from KiB, e.g. `1.94 GiB` or `812 MiB`.
pub(crate) fn fmt_kib(kib: u64) -> String {
    let mib = kib as f64 / 1024.0;
    if mib >= 1024.0 {
        format!("{:.2} GiB", mib / 1024.0)
    } else {
        format!("{mib:.0} MiB")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_vmhwm_and_memtotal_from_proc_format() {
        let status = "Name:\troughroute\nVmPeak:\t  123456 kB\nVmHWM:\t    5508 kB\n\
                      VmRSS:\t    5508 kB\n";
        assert_eq!(read_status_kib(status, "VmHWM"), Some(5508));
        // Must not confuse VmHWM with the VmPeak/VmRSS lines around it.
        assert_eq!(read_status_kib(status, "VmRSS"), Some(5508));

        let meminfo = "MemTotal:        6084384 kB\nMemFree:          609000 kB\n\
                       MemAvailable:    4200000 kB\n";
        assert_eq!(read_status_kib(meminfo, "MemTotal"), Some(6_084_384));
        assert_eq!(read_status_kib(meminfo, "MemAvailable"), Some(4_200_000));
    }

    #[test]
    fn missing_or_malformed_fields_return_none() {
        assert_eq!(read_status_kib("Name:\tx\n", "VmHWM"), None);
        assert_eq!(read_status_kib("", "MemTotal"), None);
        // A key that only appears as a longer name must not match.
        assert_eq!(read_status_kib("VmHWMX:\t7 kB\n", "VmHWM"), None);
        // Non-numeric value.
        assert_eq!(read_status_kib("VmHWM:\tnope kB\n", "VmHWM"), None);
    }

    #[test]
    fn fmt_kib_switches_units_sensibly() {
        assert_eq!(fmt_kib(512 * 1024), "512 MiB");
        assert_eq!(fmt_kib(2 * 1024 * 1024), "2.00 GiB");
        // 1.94 GiB-ish
        assert_eq!(fmt_kib(2_040_109), "1.95 GiB");
        assert_eq!(fmt_kib(0), "0 MiB");
    }
}
