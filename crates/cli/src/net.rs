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
/// deliberate CI RAM-limit test on the ~16 GB runner (docs/DECISIONS.md D22),
/// then 1.2 GB → 6 GB (2026-07-04, D24) after the D23 compact build was
/// confirmed on CI (Austria 6.28 → 1.44 GiB peak): big-country extracts
/// (Germany 4.79 GB, France 5.04 GB as probed) now fit the runner's RAM.
/// This only relaxes the *size* gate; the `df` disk-headroom check below
/// (2 × pbf + 1 GiB) is unchanged and still guards disk, and there is still
/// no RAM gate — the per-region peak-RSS log (D21) is the visibility.
///
/// Overridable per run without a rebuild, by precedence: the `--max-pbf-gb`
/// CLI flag, then the [`ENV_MAX_PBF_BYTES`] env var (decimal bytes; used when
/// it parses as `u64 > 0`), then this constant as the documented default
/// (see [`resolve_max_pbf_ceiling`]). An in-effect override is announced on
/// stderr, naming its source, so an unattended log still records that the
/// guard was deliberately moved; it shifts only this *size* gate — the `df`
/// disk-headroom check below (2 × pbf + 1 GiB) is unaffected.
pub(crate) const HARD_MAX_PBF_BYTES: u64 = 6_000_000_000; // 6 GB (decimal)

/// Env var (decimal bytes) that overrides [`HARD_MAX_PBF_BYTES`] for a single
/// run — the middle tier of precedence, below the `--max-pbf-gb` flag. Set to
/// a `u64 > 0` to raise or lower the ceiling deliberately without recompiling;
/// anything else (unset, zero, unparseable) keeps the default.
pub(crate) const ENV_MAX_PBF_BYTES: &str = "ROUGHROUTE_MAX_PBF_BYTES";

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

/// Parse a raw [`ENV_MAX_PBF_BYTES`] value into an override ceiling. `Some(n)`
/// only when the string is present and parses as a `u64 > 0`; an absent, zero,
/// or unparseable value yields `None` (→ caller falls back to the
/// [`HARD_MAX_PBF_BYTES`] default). Strictly decimal — no units, no
/// whitespace. Pure over its input so it's testable without touching the
/// process environment.
fn parse_max_pbf_override(raw: Option<&str>) -> Option<u64> {
    match raw?.parse::<u64>() {
        Ok(n) if n > 0 => Some(n),
        _ => None,
    }
}

/// Which input set the effective ceiling, for honest logging.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CeilingSource {
    /// The `--max-pbf-gb` CLI flag.
    Flag,
    /// The [`ENV_MAX_PBF_BYTES`] environment variable.
    Env,
    /// Neither was set (or was invalid) — the [`HARD_MAX_PBF_BYTES`] default.
    Default,
}

/// Convert decimal GB (as the `--max-pbf-gb` flag takes) to decimal bytes:
/// `6.0 → 6_000_000_000`, `6.5 → 6_500_000_000`. `round` guards against an
/// f64 representation landing a hair under an exact byte count.
pub(crate) fn gb_to_bytes(gb: f64) -> u64 {
    (gb * 1_000_000_000.0).round() as u64
}

/// clap `value_parser` for `--max-pbf-gb`: a finite, strictly-positive number
/// of GB. Rejects `<= 0`, `NaN`, and non-numbers at parse time so the gate
/// never sees a nonsensical ceiling.
pub(crate) fn parse_max_pbf_gb(s: &str) -> Result<f64, String> {
    let gb: f64 = s.parse().map_err(|_| format!("`{s}` is not a number of GB"))?;
    if gb.is_finite() && gb > 0.0 {
        Ok(gb)
    } else {
        Err("must be a positive number of GB".to_string())
    }
}

/// Resolve the effective `.pbf` size ceiling by precedence — `--max-pbf-gb`
/// (already converted to bytes) beats [`ENV_MAX_PBF_BYTES`] beats the
/// [`HARD_MAX_PBF_BYTES`] default. Pure over its arguments (the caller reads
/// env and passes it in as `env_raw`) so precedence is testable without
/// touching real env or args.
fn resolve_max_pbf_bytes(flag_bytes: Option<u64>, env_raw: Option<&str>) -> u64 {
    if let Some(bytes) = flag_bytes {
        return bytes;
    }
    parse_max_pbf_override(env_raw).unwrap_or(HARD_MAX_PBF_BYTES)
}

/// Which input the same precedence picks, for naming the source in the log.
/// Pure counterpart to [`resolve_max_pbf_bytes`].
fn max_pbf_source(flag_bytes: Option<u64>, env_raw: Option<&str>) -> CeilingSource {
    if flag_bytes.is_some() {
        CeilingSource::Flag
    } else if parse_max_pbf_override(env_raw).is_some() {
        CeilingSource::Env
    } else {
        CeilingSource::Default
    }
}

/// Resolve the ceiling for a real run: apply the `--max-pbf-gb` flag (bytes)
/// over the live [`ENV_MAX_PBF_BYTES`] env var over the default, and — when
/// the result differs from the compiled default — announce it on stderr,
/// naming the source, so an unattended log still shows the safety guard was
/// deliberately moved rather than silently changed.
pub(crate) fn resolve_max_pbf_ceiling(flag_bytes: Option<u64>) -> u64 {
    let env_raw = std::env::var(ENV_MAX_PBF_BYTES).ok();
    let bytes = resolve_max_pbf_bytes(flag_bytes, env_raw.as_deref());
    if bytes != HARD_MAX_PBF_BYTES {
        let via = match max_pbf_source(flag_bytes, env_raw.as_deref()) {
            CeilingSource::Flag => "via --max-pbf-gb",
            CeilingSource::Env => "via ROUGHROUTE_MAX_PBF_BYTES",
            CeilingSource::Default => "default", // unreachable while bytes != default
        };
        let direction = if bytes > HARD_MAX_PBF_BYTES { "raised" } else { "lowered" };
        eprintln!(
            "  .pbf size ceiling {direction} to {:.2} GB ({via}; compiled default {:.0} GB)",
            bytes as f64 / 1_000_000_000.0,
            HARD_MAX_PBF_BYTES as f64 / 1_000_000_000.0,
        );
    }
    bytes
}

/// The shared pre-download gate (CLAUDE.md "Disk usage"): probe the source
/// size, refuse anything over `max_pbf_bytes` *or* of unknown size, and refuse
/// when disk headroom in `dest_dir` is short. `label` names the thing being
/// downloaded for error messages. Returns the probed length on success.
///
/// `max_pbf_bytes` is the already-resolved ceiling (flag > env > default, via
/// [`resolve_max_pbf_ceiling`]) — this function neither reads env nor logs the
/// override, so the caller owns that once per run.
///
/// `Err` means "do not download" — the caller maps it to its own abort. No
/// byte is fetched here beyond a HEAD request.
pub(crate) fn gate_download(
    agent: &ureq::Agent,
    url: &str,
    dest_dir: &Path,
    label: &str,
    max_pbf_bytes: u64,
) -> Result<u64, String> {
    let content_length = probe_content_length(agent, url);
    let len = match content_length {
        Some(len) if len > max_pbf_bytes => {
            return Err(format!(
                "{label}: .pbf is {:.1} MB, over the {:.1} GB ceiling — refusing to download; \
                 pick a smaller source, or raise it with --max-pbf-gb (or ROUGHROUTE_MAX_PBF_BYTES)",
                len as f64 / 1_000_000.0,
                max_pbf_bytes as f64 / 1_000_000_000.0,
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
    fn hard_max_pbf_ceiling_is_6gb() {
        // Pin the constant so a casual edit doesn't silently loosen the
        // safety gate. Raised 1.2 GB → 6 GB for the big-country streaming-
        // build proof (Germany/France; docs/DECISIONS.md D24) after D23 cut
        // peak build RAM ~4.4×.
        assert_eq!(HARD_MAX_PBF_BYTES, 6_000_000_000);
    }

    #[test]
    fn max_pbf_override_parses_positive_u64_else_falls_back() {
        // Unset → no override (caller keeps HARD_MAX_PBF_BYTES). Pure helper,
        // so this asserts the fallback intent without mutating real env.
        assert_eq!(parse_max_pbf_override(None), None);
        // Valid positive decimal bytes → parsed through.
        assert_eq!(parse_max_pbf_override(Some("8000000000")), Some(8_000_000_000));
        assert_eq!(parse_max_pbf_override(Some("1")), Some(1));
        // Zero → ignored (a zero ceiling would refuse everything, clearly not
        // the intent) → default.
        assert_eq!(parse_max_pbf_override(Some("0")), None);
        // Garbage → ignored → default. Strictly decimal: no units, no sign,
        // no whitespace, no fractions.
        for garbage in ["", "6GB", "6_000", "-5", "3.5", " 42", "42 ", "0x10"] {
            assert_eq!(parse_max_pbf_override(Some(garbage)), None, "{garbage:?}");
        }
    }

    #[test]
    fn gb_flag_converts_decimal_gb_to_decimal_bytes() {
        assert_eq!(gb_to_bytes(6.0), 6_000_000_000);
        assert_eq!(gb_to_bytes(6.5), 6_500_000_000);
        assert_eq!(gb_to_bytes(12.0), 12_000_000_000);
    }

    #[test]
    fn gb_flag_parser_rejects_zero_negative_and_nonnumbers() {
        assert_eq!(parse_max_pbf_gb("6"), Ok(6.0));
        assert_eq!(parse_max_pbf_gb("6.5"), Ok(6.5));
        for bad in ["0", "-1", "-6.5", "abc", "", "NaN", "inf"] {
            assert!(parse_max_pbf_gb(bad).is_err(), "{bad:?} should be rejected");
        }
    }

    #[test]
    fn ceiling_resolves_by_precedence_flag_over_env_over_default() {
        // Pure over (flag_bytes, env_raw): no real env or args are touched.
        // Flag wins over env, even when both are set.
        assert_eq!(
            resolve_max_pbf_bytes(Some(gb_to_bytes(12.0)), Some("8000000000")),
            12_000_000_000
        );
        // No flag → the env override is used.
        assert_eq!(resolve_max_pbf_bytes(None, Some("8000000000")), 8_000_000_000);
        // No flag, invalid env → the compiled default.
        assert_eq!(resolve_max_pbf_bytes(None, Some("garbage")), HARD_MAX_PBF_BYTES);
        // Neither set → the compiled default.
        assert_eq!(resolve_max_pbf_bytes(None, None), HARD_MAX_PBF_BYTES);
        // A flag still wins even when the env is invalid.
        assert_eq!(resolve_max_pbf_bytes(Some(5_000_000_000), Some("nonsense")), 5_000_000_000);
        // The source (which drives the log wording) tracks the same precedence.
        assert_eq!(max_pbf_source(Some(9), Some("8000000000")), CeilingSource::Flag);
        assert_eq!(max_pbf_source(None, Some("8000000000")), CeilingSource::Env);
        assert_eq!(max_pbf_source(None, Some("garbage")), CeilingSource::Default);
        assert_eq!(max_pbf_source(None, None), CeilingSource::Default);
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
