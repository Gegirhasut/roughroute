//! Build-time HTTP + disk-safety helpers shared by `build --pbf-url` and
//! `batch`. Dev/CI only — the routing runtime never downloads (spec §2.1).
//!
//! Everything here funnels through one timeout-configured [`agent`] so a
//! stalled connection can't hang an unattended run forever, and every
//! download passes the same [`gate_download`] safety check (hard size
//! ceiling + disk headroom) before a single byte is fetched.

use std::error::Error;
use std::fs;
use std::path::Path;
use std::process::Command;
use std::time::Duration;

/// Free space that must remain after a download (beyond the estimated
/// artifacts), so a build never runs the disk to the wire.
pub(crate) const HEADROOM_FLOOR_BYTES: u64 = 1 << 30; // 1 GiB
/// Assumed download size when the server sends no Content-Length.
pub(crate) const UNKNOWN_PBF_ESTIMATE_BYTES: u64 = 5 << 30; // 5 GiB, deliberately harsh
/// Hard safety ceiling on a single `.pbf`: regardless of disk headroom, a
/// probed (or unknown) size above this aborts rather than downloading. A
/// blunt guard against an accidentally huge region (a continent, a whole
/// country the size of the US) eating an unattended run's disk or wall-clock
/// budget; raise it deliberately if a legitimately larger source is added.
///
/// Raised 800 MB → 1.2 GB (2026-07-04) to admit Austria (~803 MB pbf) as a
/// deliberate CI RAM-limit test on the ~16 GB runner (docs/DECISIONS.md D22).
/// This only relaxes the *size* gate; the `df` disk-headroom check below is
/// unchanged, and there is still no RAM gate — that's the point of the test.
pub(crate) const HARD_MAX_PBF_BYTES: u64 = 1_200_000_000; // 1.2 GB (decimal)

/// A timeout-configured agent for all build-time downloads. `timeout_read`
/// (no data for this long → fail) is the real protection against a stalled
/// connection hanging forever; the generous overall cap accommodates a slow
/// but progressing download of anything under [`HARD_MAX_PBF_BYTES`].
pub(crate) fn agent() -> ureq::Agent {
    ureq::AgentBuilder::new()
        .timeout_connect(Duration::from_secs(30))
        .timeout_read(Duration::from_secs(120))
        .timeout_write(Duration::from_secs(30))
        .timeout(Duration::from_secs(2 * 60 * 60))
        .build()
}

/// HEAD the URL and read `Content-Length` (following redirects — Geofabrik's
/// `-latest.osm.pbf` URLs 302 to a dated file, so a redirect-naive probe
/// would see the tiny HTML redirect body's length instead of the real one).
/// `ureq`'s agent follows redirects for HEAD the same as GET.
pub(crate) fn probe_content_length(agent: &ureq::Agent, url: &str) -> Option<u64> {
    agent
        .head(url)
        .call()
        .ok()
        .and_then(|resp| resp.header("content-length").and_then(|v| v.parse::<u64>().ok()))
}

/// `2 × content_length + 1 GiB` (D17): source + built graph + floor. A harsh
/// default when the length is unknown.
pub(crate) fn estimated_need_bytes(content_length: Option<u64>) -> u64 {
    match content_length {
        Some(len) => len.saturating_mul(2).saturating_add(HEADROOM_FLOOR_BYTES),
        None => UNKNOWN_PBF_ESTIMATE_BYTES,
    }
}

/// The shared pre-download gate (CLAUDE.md "Disk usage"): probe the source
/// size, refuse anything over the hard ceiling *or* of unknown size, and
/// refuse when disk headroom in `dest_dir` is short. `label` names the thing
/// being downloaded for error messages. Returns the probed length on success.
///
/// `Err` means "do not download" — the caller maps it to its own abort. No
/// byte is fetched here beyond a HEAD request.
pub(crate) fn gate_download(
    agent: &ureq::Agent,
    url: &str,
    dest_dir: &Path,
    label: &str,
) -> Result<u64, String> {
    let content_length = probe_content_length(agent, url);
    let len = match content_length {
        Some(len) if len > HARD_MAX_PBF_BYTES => {
            return Err(format!(
                "{label}: .pbf is {:.1} MB, over the {:.0} MB hard safety ceiling — refusing to \
                 download; pick a smaller source or raise HARD_MAX_PBF_BYTES deliberately",
                len as f64 / 1_000_000.0,
                HARD_MAX_PBF_BYTES as f64 / 1_000_000.0,
            ));
        }
        Some(len) => {
            eprintln!("  .pbf size: {:.1} MB (within the safety ceiling)", len as f64 / 1_000_000.0);
            len
        }
        None => {
            return Err(format!(
                "{label}: could not determine .pbf size via HEAD request; refusing to download \
                 without a size safety check"
            ));
        }
    };

    let need = estimated_need_bytes(Some(len));
    let avail = available_bytes(dest_dir)
        .map_err(|e| format!("cannot determine free disk space: {e}"))?;
    if avail < need {
        return Err(format!(
            "insufficient disk for {label}: {:.1} GiB free, {:.1} GiB needed \
             (download + build + 1 GiB floor)",
            avail as f64 / (1u64 << 30) as f64,
            need as f64 / (1u64 << 30) as f64,
        ));
    }
    Ok(len)
}

/// Download `url` to `to`, returning bytes written. Uses the timeout agent.
pub(crate) fn download(agent: &ureq::Agent, url: &str, to: &Path) -> Result<u64, String> {
    let fail = |e: String| format!("download failed: {e}");
    let response = agent.get(url).call().map_err(|e| fail(e.to_string()))?;
    let mut file = fs::File::create(to).map_err(|e| fail(e.to_string()))?;
    let copied =
        std::io::copy(&mut response.into_reader(), &mut file).map_err(|e| fail(e.to_string()))?;
    eprintln!("  downloaded {:.1} MB", copied as f64 / (1024.0 * 1024.0));
    Ok(copied)
}

/// Free bytes on the filesystem holding `path`, via `df -P -B1` (POSIX output
/// format; this is dev/CI tooling for Linux/macOS runners).
pub(crate) fn available_bytes(path: &Path) -> Result<u64, Box<dyn Error>> {
    let output = Command::new("df").arg("-P").arg("-B1").arg(path).output()?;
    if !output.status.success() {
        return Err(format!("df exited with {}", output.status).into());
    }
    parse_df_available(&String::from_utf8_lossy(&output.stdout))
}

/// Parse the "Available" column of `df -P` output (second line, 4th field).
fn parse_df_available(df_output: &str) -> Result<u64, Box<dyn Error>> {
    let line = df_output.lines().nth(1).ok_or("df output has no data line")?;
    let field = line.split_whitespace().nth(3).ok_or("df line has no Available field")?;
    Ok(field.parse::<u64>()?)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn estimated_need_scales_with_known_length_and_is_harsh_when_unknown() {
        let need = estimated_need_bytes(Some(300_000_000));
        assert_eq!(need, 300_000_000 * 2 + HEADROOM_FLOOR_BYTES);
        assert_eq!(estimated_need_bytes(None), UNKNOWN_PBF_ESTIMATE_BYTES);
    }

    #[test]
    fn hard_max_pbf_ceiling_is_1_2gb() {
        // Pin the constant so a casual edit doesn't silently loosen the
        // safety gate. Raised 800 MB → 1.2 GB to admit Austria for the CI
        // RAM-limit test (docs/DECISIONS.md D22).
        assert_eq!(HARD_MAX_PBF_BYTES, 1_200_000_000);
    }

    #[test]
    fn df_output_parses() {
        let out = "Filesystem     1-blocks       Used  Available Capacity Mounted on\n\
                   /dev/sda1    105089261568 46349357056 53355900928      47% /\n";
        assert_eq!(parse_df_available(out).unwrap(), 53_355_900_928);
        assert!(parse_df_available("").is_err());
        assert!(parse_df_available("header only\n").is_err());
    }
}
