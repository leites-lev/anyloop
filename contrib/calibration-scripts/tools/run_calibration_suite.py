#!/usr/bin/env python3
"""Run an Anyloop calibration-suite manifest and combine its PDF reports."""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import re
import statistics


ROOT = Path(__file__).resolve().parents[3]


def root_path(value):
    path = Path(value)
    return path if path.is_absolute() else ROOT / path


def load_manifest(path):
    with path.open(encoding="utf-8") as stream:
        manifest = json.load(stream)
    runs = manifest.get("runs")
    if not isinstance(runs, list) or not runs:
        raise ValueError("manifest needs a non-empty 'runs' array")
    for index, run in enumerate(runs):
        if not isinstance(run, dict) or not all(
                isinstance(run.get(key), str) and run[key]
                for key in ("name", "config", "pdf")):
            raise ValueError(
                f"runs[{index}] needs non-empty name, config and pdf strings")
        amplitudes = run.get("prbs_amplitudes")
        if amplitudes is not None:
            if not isinstance(amplitudes, list) or len(amplitudes) < 1 \
                    or not all(isinstance(value, (int, float)) and value != 0
                               for value in amplitudes):
                raise ValueError(
                    f"runs[{index}].prbs_amplitudes needs at least one "
                    "nonzero numbers")
            spread = run.get("max_delay_spread_frames")
            if not isinstance(spread, (int, float)) or spread <= 0:
                raise ValueError(
                    f"runs[{index}] with prbs_amplitudes needs a positive "
                    "max_delay_spread_frames")
    args = manifest.get("anyloop_args", [])
    if not isinstance(args, list) or not all(isinstance(arg, str) for arg in args):
        raise ValueError("'anyloop_args' must be an array of strings")
    if not isinstance(manifest.get("output_file"), str):
        raise ValueError("manifest needs an 'output_file' string")
    roi = manifest.get("roi_refresh")
    if not isinstance(roi, dict):
        raise ValueError("manifest needs an 'roi_refresh' object")
    if not all(isinstance(roi.get(key), str) and roi[key]
               for key in ("park_config", "script", "reference_config")):
        raise ValueError(
            "roi_refresh needs park_config, script and reference_config strings")
    if not isinstance(roi.get("settle_seconds"), (int, float)) \
            or roi["settle_seconds"] < 0:
        raise ValueError("roi_refresh.settle_seconds must be non-negative")
    if "probe_gain" in roi and not isinstance(roi["probe_gain"], (int, float)):
        raise ValueError("roi_refresh.probe_gain must be numeric")
    if not isinstance(roi.get("max_attempts", 3), int) \
            or roi.get("max_attempts", 3) < 1:
        raise ValueError("roi_refresh.max_attempts must be a positive integer")
    configs = roi.get("configs")
    if not isinstance(configs, list) or not configs \
            or not all(isinstance(value, str) and value for value in configs):
        raise ValueError("roi_refresh.configs must be a non-empty string array")
    before_runs = roi.get("before_runs", [])
    if not isinstance(before_runs, list) \
            or not all(isinstance(value, int) and value >= 0
                       for value in before_runs):
        raise ValueError("roi_refresh.before_runs must be an array of "
                         "non-negative run indexes")
    if any(value > len(runs) for value in before_runs):
        raise ValueError("roi_refresh.before_runs cannot exceed the number "
                         "of runs")
    survey = manifest.get("com_survey")
    if not isinstance(survey, dict):
        raise ValueError("manifest needs a 'com_survey' object")
    if not all(isinstance(survey.get(key), str) and survey[key]
               for key in ("config", "capture", "analyzer", "result")):
        raise ValueError(
            "com_survey needs config, capture, analyzer and result strings")
    if not isinstance(survey.get("fs_hz"), (int, float)) \
            or survey["fs_hz"] <= 0:
        raise ValueError("com_survey.fs_hz must be positive")
    targets = survey.get("apply_to")
    if not isinstance(targets, list) or not targets \
            or not all(isinstance(value, str) and value for value in targets):
        raise ValueError("com_survey.apply_to must be a non-empty string array")
    camera_targets = survey.get("camera_apply_to")
    if not isinstance(camera_targets, list) or not camera_targets \
            or not all(isinstance(value, str) and value
                       for value in camera_targets):
        raise ValueError(
            "com_survey.camera_apply_to must be a non-empty string array")
    if not isinstance(survey.get("max_exposure_us"), (int, float)) \
            or survey["max_exposure_us"] <= 0:
        raise ValueError("com_survey.max_exposure_us must be positive")
    if not isinstance(survey.get("pwm_hz"), (int, float)) \
            or survey["pwm_hz"] < 0:
        raise ValueError("com_survey.pwm_hz must be non-negative")
    if not isinstance(survey.get("pwm_duty"), (int, float)) \
            or not 0 < survey["pwm_duty"] <= 1:
        raise ValueError("com_survey.pwm_duty must be in (0, 1]")
    update = manifest.get("update_run")
    if update is not None:
        required = ("config", "source_config", "x_gain_run", "y_gain_run",
                    "x_delay_run", "y_delay_run")
        if not isinstance(update, dict) or not all(
                isinstance(update.get(key), str) and update[key]
                for key in required):
            raise ValueError(
                "update_run needs config plus x/y gain and delay run names")
        names = {run["name"] for run in runs}
        for key in ("x_gain_run", "y_gain_run",
                    "x_delay_run", "y_delay_run"):
            if update[key] not in names:
                raise ValueError(f"update_run.{key} names no configured run")
        shared = update.get("shared_configs", [])
        if not isinstance(shared, list) or not all(
                isinstance(value, str) and value for value in shared):
            raise ValueError("update_run.shared_configs must be a string array")
    return manifest


def apply_survey(result_path, target_paths, max_rejected_fraction=0.25):
    with result_path.open(encoding="utf-8") as stream:
        result = json.load(stream)
    recommendation = result.get("recommendation")
    if recommendation == "none":
        raise SystemExit(
            "COM survey recommends neither tracker; refusing to drive the sweep")
    if recommendation != "fit_com":
        raise SystemExit(f"invalid COM survey recommendation: {recommendation!r}")

    if result.get("beam_clipped") is not False:
        raise SystemExit(
            "COM survey did not validate beam clearance inside the ROI; "
            "refusing to drive the sweep")
    rejected = result.get("fit_rejected_fraction")
    if not isinstance(rejected, (int, float)) or rejected > max_rejected_fraction:
        raise SystemExit(
            f"COM survey {recommendation} rejected {rejected!r} of frames "
            f"(limit {max_rejected_fraction}); refusing to drive the sweep")

    suggested = result.get(recommendation)
    if not isinstance(suggested, dict):
        raise SystemExit(f"COM survey omitted {recommendation} parameters")
    # Preserve estimator choice and compute policy. Acceptance, PWM and
    # reacquisition gates are measured by the survey and intentionally apply.
    preserve = {
        "max_iter", "max_us", "moment_output", "fit_gaussian",
        "fit_slope", "robust_k", "robust_iter", "row_time",
    }
    for path in target_paths:
        with path.open(encoding="utf-8") as stream:
            config = json.load(stream)
        matches = [device for device in config.get("pipeline", [])
                   if device.get("uri") == "anyloop:fit_com"]
        if len(matches) != 1:
            raise SystemExit(
                f"{path} needs exactly one fit_com device, "
                f"found {len(matches)}")
        device = matches[0]
        old_params = device.get("params", {})
        new_params = {key: value for key, value in old_params.items()
                      if key.startswith("_")}
        new_params.update(suggested)
        for key in preserve:
            if key in old_params:
                new_params[key] = old_params[key]
        device["uri"] = "anyloop:fit_com"
        device["params"] = new_params
        temporary = path.with_name(f".{path.name}.com-survey.tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(config, stream, indent="\t")
            stream.write("\n")
        os.replace(temporary, path)
        print(f"[COM survey] applied {recommendation} to {path}", flush=True)


def camera_device(config):
    matches = [device for device in config.get("pipeline", [])
               if "asi_source" in device.get("uri", "")]
    if len(matches) != 1:
        raise SystemExit(
            f"configuration needs exactly one asi_source, found {len(matches)}")
    return matches[0]


def read_camera_settings(path):
    with path.open(encoding="utf-8") as stream:
        config = json.load(stream)
    params = camera_device(config).get("params", {})
    exposure, gain = params.get("exposure"), params.get("gain")
    if not isinstance(exposure, (int, float)) or exposure <= 0 \
            or not isinstance(gain, (int, float)):
        raise SystemExit(f"{path} needs numeric exposure > 0 and gain")
    return exposure, gain


def measured_camera_fs(log_path, fallback):
    """Use the settled ASI camera rate, not an ROI-specific stale constant."""
    text = log_path.read_text(encoding="utf-8", errors="replace")
    rates = [float(value) for value in
             re.findall(r"camera ([0-9]+(?:\.[0-9]+)?) Hz", text)]
    if len(rates) < 3:
        raise SystemExit(
            f"cannot measure a settled camera rate from {log_path}")
    # asi_source's first report includes camera startup. The median of the
    # remainder rejects occasional host-side stalls without hiding a new ROI
    # rate. This measured value is passed to the COM analysis; changing ROI
    # therefore cannot silently retain the old 64x64 rate.
    settled = rates[1:]
    measured = statistics.median(settled)
    print(f"[COM survey] measured camera rate {measured:g} Hz "
          f"(manifest reference {fallback:g} Hz)", flush=True)
    return measured


def apply_camera(result_path, target_paths, lock_exposure=False):
    with result_path.open(encoding="utf-8") as stream:
        result = json.load(stream)
    suggested = result.get("camera")
    if not isinstance(suggested, dict) \
            or not all(isinstance(suggested.get(key), (int, float))
                       for key in ("exposure", "gain")):
        raise SystemExit("COM survey omitted camera exposure/gain suggestion")
    changed = False
    for path in target_paths:
        with path.open(encoding="utf-8") as stream:
            config = json.load(stream)
        params = camera_device(config).setdefault("params", {})
        old = (params.get("exposure"), params.get("gain"))
        new = (old[0] if lock_exposure else round(suggested["exposure"]),
               round(suggested["gain"]))
        if old == new:
            continue
        params["exposure"], params["gain"] = new
        temporary = path.with_name(f".{path.name}.com-camera.tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(config, stream, indent="\t")
            stream.write("\n")
        os.replace(temporary, path)
        print(f"[COM survey] camera {old} -> {new} in {path}", flush=True)
        changed = True
    return changed


def prbs_variant(config_path, pdf_path, amplitude):
    """Write one temporary amplitude variant of a PRBS configuration."""
    with config_path.open(encoding="utf-8") as stream:
        config = json.load(stream)
    matches = [device for device in config.get("pipeline", [])
               if device.get("uri") == "anyloop:prbs_test"]
    if len(matches) != 1:
        raise SystemExit(
            f"{config_path} needs exactly one prbs_test device, "
            f"found {len(matches)}")
    params = matches[0].setdefault("params", {})
    params["amplitude"] = amplitude
    params["output_file"] = str(pdf_path)
    fd, name = tempfile.mkstemp(prefix=".prbs-", suffix=".json", dir=ROOT)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        json.dump(config, stream, indent="\t")
        stream.write("\n")
    return Path(name)


def read_prbs_delay(pdf_path):
    dat_path = pdf_path.with_suffix(".dat")
    if not dat_path.is_file():
        raise SystemExit(f"PRBS run did not create {dat_path}")
    text = dat_path.read_text(encoding="utf-8")
    quality = re.search(r"^# quality (PASS|FAIL)(?:: (.*))?$", text,
                        re.MULTILINE)
    if not quality or quality.group(1) != "PASS":
        reason = quality.group(2) if quality else "missing quality verdict"
        raise SystemExit(f"{dat_path.name} is not a valid delay: {reason}")
    match = re.search(
        r"^# recommended_delay ([+-]?[0-9.]+) fr source \S+$", text,
        re.MULTILINE)
    if not match:
        raise SystemExit(f"{dat_path.name} has no recommended delay")
    return float(match.group(1))


def read_gain(pdf_path):
    """Read the validated local gain used by the closed-loop controller."""
    dat_path = pdf_path.with_suffix(".dat")
    if not dat_path.is_file():
        raise SystemExit(f"gain run did not create {dat_path}")
    text = dat_path.read_text(encoding="utf-8")
    match = re.search(
        r"^# fit slope ([+-]?[0-9.eE+-]+) \+/- ([0-9.eE+-]+) "
        r"intercept [+-]?[0-9.eE+-]+ R2 ([0-9.eE+-]+)",
        text, re.MULTILINE)
    if not match:
        raise SystemExit(f"{dat_path.name} has no usable gain fit")
    full_gain, full_error, r2 = map(float, match.groups())
    small = re.search(
        r"^# small_signal \|cmd-bias\|<=[0-9.eE+-]+: slope "
        r"([+-]?[0-9.eE+-]+) \+/- ([0-9.eE+-]+) over ([0-9]+) levels$",
        text, re.MULTILINE)
    if not small:
        raise SystemExit(
            f"{dat_path.name} has no small-signal gain fit; refusing to "
            "install a full-range average into the controller")
    gain, error = map(float, small.groups()[:2])
    levels = int(small.group(3))
    if full_gain <= 0.0 or gain <= 0.0:
        raise SystemExit(f"{dat_path.name} measured non-positive gain {gain:g}")
    if r2 < 0.90:
        raise SystemExit(f"{dat_path.name} gain R2 {r2:.4f} < 0.90")
    if levels < 5:
        raise SystemExit(
            f"{dat_path.name} small-signal fit has only {levels} levels")
    if error > 0.15 * gain:
        raise SystemExit(
            f"{dat_path.name} local-gain uncertainty {error:g} exceeds 15%")
    disagreement = abs(gain - full_gain) / gain
    if disagreement > 0.25:
        raise SystemExit(
            f"{dat_path.name} local gain {gain:g} and full-span gain "
            f"{full_gain:g} differ by {100.0*disagreement:.1f}%; this is "
            "too nonlinear to update the controller safely")
    return gain


def _numeric_fs(params):
    """The config's fs, or None when it is "auto" (measured at startup)."""
    try:
        value = float(params.get("fs", 0.0))
    except (TypeError, ValueError):
        return None
    return value if value > 0.0 else None


def install_delay(params, axes, fs_hz, x_delay, y_delay, target_fs=None):
    """Install a measured delay, in whatever form the config asked for.

    A config on "delay": "auto" measures the delay itself every run and derives
    the part it cannot measure from delay_ident_ms -- the identification, in
    MILLISECONDS. Writing frame counts over that would replace a self-correcting
    arrangement with numbers that are only valid at one frame rate, so the
    identification goes in as time and the config keeps measuring."""
    if params.get("delay") == "auto" or params.get("fs") == "auto":
        params["delay_ident_ms"] = round(1e3 * x_delay / fs_hz, 4)
        if axes["y"].get("delay") != "auto":
            axes["y"]["delay_ident_ms"] = round(1e3 * y_delay / fs_hz, 4)
        return
    scale = (target_fs / fs_hz) if target_fs else 1.0
    for target, value in ((params, x_delay * scale), (axes["y"], y_delay * scale)):
        whole = int(value)
        target["delay"] = whole
        target["delay_frac"] = round(value - whole, 6)


def update_run_config(path, source_path, fs_hz, x_gain, y_gain,
                      x_delay, y_delay):
    """Atomically install a complete accepted calibration into an FSP run."""
    with path.open(encoding="utf-8") as stream:
        config = json.load(stream)
    with source_path.open(encoding="utf-8") as stream:
        source = json.load(stream)
    # Install the exact camera and estimator that produced the calibration.
    config_camera = camera_device(config)
    source_camera = camera_device(source)
    config_camera["params"] = source_camera.get("params", {}).copy()
    config_fit = [device for device in config.get("pipeline", [])
                  if device.get("uri") == "anyloop:fit_com"]
    source_fit = [device for device in source.get("pipeline", [])
                  if device.get("uri") == "anyloop:fit_com"]
    if len(config_fit) != 1 or len(source_fit) != 1:
        raise SystemExit("run and calibration source need one fit_com each")
    config_fit[0]["params"] = source_fit[0].get("params", {}).copy()
    matches = [device for device in config.get("pipeline", [])
               if device.get("uri") == "anyloop:fsp"]
    if len(matches) != 1:
        raise SystemExit(f"{path} needs exactly one fsp device")
    params = matches[0].setdefault("params", {})
    axes = {axis: params.setdefault(axis, {}) for axis in ("x", "y")}
    if params.get("fs") != "auto":
        params["fs"] = fs_hz
    if x_gain is not None:
        axes["x"]["K"] = x_gain
        axes["y"]["K"] = y_gain
    if x_delay is not None:
        install_delay(params, axes, fs_hz, x_delay, y_delay)
    params["_auto_calibration"] = (
        f"Automatically applied by run_calibration_suite: fs={fs_hz:.6g} Hz, "
        f"Kx={x_gain if x_gain is not None else 'preserved'}, "
        f"Ky={y_gain if y_gain is not None else 'preserved'}, "
        f"delay_x={x_delay if x_delay is not None else 'preserved'} frames, "
        f"delay_y={y_delay if y_delay is not None else 'preserved'} frames. "
        "Camera, ROI and fit_com parameters were applied by the same survey.")
    temporary = path.with_name(f".{path.name}.calibration.tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(config, stream, indent="\t")
        stream.write("\n")
    os.replace(temporary, path)
    print(f"[calibration] updated run configuration {path}", flush=True)


def update_shared_plant(path, source_fs, x_gain, y_gain, x_delay, y_delay):
    """Propagate physical plant gain/delay without changing waveform settings."""
    with path.open(encoding="utf-8") as stream:
        config = json.load(stream)
    matches = [device for device in config.get("pipeline", [])
               if device.get("uri") == "anyloop:fsp"]
    if len(matches) != 1:
        raise SystemExit(f"{path} needs exactly one fsp device")
    params = matches[0].setdefault("params", {})
    axes = {axis: params.setdefault(axis, {}) for axis in ("x", "y")}
    target_fs = _numeric_fs(params)
    if target_fs is None and params.get("fs") != "auto":
        raise SystemExit(f"{path} has no positive fsp.fs")
    if x_gain is not None:
        axes["x"]["K"], axes["y"]["K"] = x_gain, y_gain
    converted = None
    if x_delay is not None:
        # A target on "auto" takes the identification in ms and converts it at
        # the rate it measures for itself; only a fixed-fs target needs the
        # frame count converting here.
        converted = target_fs is not None
        install_delay(params, axes, source_fs, x_delay, y_delay, target_fs)
    params["_shared_plant_calibration"] = (
        f"Shared plant calibration propagated from a {source_fs:.6g} Hz run; "
        f"gain {'updated' if x_gain is not None else 'preserved'}, delay "
        f"{'converted in physical time to this frame rate' if converted else 'preserved'}.")
    temporary = path.with_name(f".{path.name}.plant-calibration.tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(config, stream, indent="\t")
        stream.write("\n")
    os.replace(temporary, path)
    print(f"[calibration] propagated shared plant values to {path}", flush=True)


def main():
    (ROOT / "data" / "calibration" / "run_staging").mkdir(
        parents=True, exist_ok=True)
    parser = argparse.ArgumentParser(
        description="Run gain/delay calibrations and combine their PDFs")
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--dry-run", action="store_true",
                        help="validate and print commands without driving hardware")
    parser.add_argument("--com-only", action="store_true",
                        help="refresh ROI and validate/apply fit_com parameters, "
                             "then stop before actuator calibration runs")
    parser.add_argument("--skip-gain", action="store_true",
                        help="skip gain tests and preserve existing K values")
    parser.add_argument("--skip-delay", action="store_true",
                        help="skip delay tests and preserve existing delays")
    opts = parser.parse_args()

    manifest_path = opts.manifest.resolve()
    manifest = load_manifest(manifest_path)
    opts.skip_gain = opts.skip_gain or bool(manifest.get("skip_gain", False))
    opts.skip_delay = opts.skip_delay or bool(manifest.get("skip_delay", False))
    anyloop = root_path(manifest.get("anyloop", "build/anyloop"))
    anyloop_args = manifest.get("anyloop_args", [])
    output = root_path(manifest["output_file"])
    pdfunite = shutil.which("pdfunite")

    # Every shipped calibration pipeline uses parport_dac's direct MMIO BAR
    # mapping. Fail before the first park command with an actionable message;
    # otherwise parport_dac reports EPERM and subprocess.check=True obscures
    # the cause under a Python CalledProcessError traceback.
    if not opts.dry_run and os.geteuid() != 0:
        flags = []
        if opts.com_only:
            flags.append("--com-only")
        if opts.skip_gain:
            flags.append("--skip-gain")
        if opts.skip_delay:
            flags.append("--skip-delay")
        command = " ".join([
            "sudo", "chrt", "-f", "80", "taskset", "-c", "2",
            sys.executable, str(Path(__file__).resolve()),
            str(manifest_path), *flags,
        ])
        raise SystemExit(
            "Calibration requires root because parport_dac uses the MMIO "
            "backend (CAP_SYS_RAWIO). Re-run as:\n\n  " + command)

    if not anyloop.is_file():
        raise SystemExit(f"Anyloop executable not found: {anyloop}")
    if not pdfunite:
        raise SystemExit("pdfunite is required to create the combined report")

    roi = manifest["roi_refresh"]
    park_config = root_path(roi["park_config"])
    roi_script = root_path(roi["script"])
    roi_reference = root_path(roi["reference_config"])
    roi_configs = [root_path(value) for value in roi["configs"]]
    for path in [park_config, roi_script, roi_reference, *roi_configs]:
        if not path.is_file():
            raise SystemExit(f"ROI refresh input not found: {path}")
    park_command = [str(anyloop), *anyloop_args, str(park_config)]
    roi_command = [
        sys.executable, str(roi_script),
        "--settle", str(roi["settle_seconds"]),
        "--ref-config", str(roi_reference), "--write",
    ]
    if "probe_gain" in roi:
        roi_command.extend(["--gain", str(roi["probe_gain"])])
    for config in roi_configs:
        roi_command.extend(["--config", str(config)])
    def refresh_roi(reason):
        print(f"[park FSM at 0 V: {reason}] {' '.join(park_command)}",
              flush=True)
        print(f"[refresh ROI: {reason}] {' '.join(roi_command)}", flush=True)
        if not opts.dry_run:
            subprocess.run(park_command, cwd=ROOT, check=True)
            for attempt in range(roi.get("max_attempts", 3)):
                result = subprocess.run(roi_command, cwd=ROOT)
                if result.returncode == 0:
                    break
                if attempt + 1 < roi.get("max_attempts", 3):
                    print(f"[refresh ROI] probe attempt {attempt + 1} "
                          "did not find a coherent beam; retrying", flush=True)
            else:
                raise SystemExit(
                    f"ROI probe failed {roi.get('max_attempts', 3)} times")

    # Index zero is the initial refresh before the COM survey. Additional
    # indexes let long suites re-check a walking beam between drive tests;
    # len(runs) gives a final refresh immediately before steering is started.
    refresh_before_runs = set(roi.get("before_runs", [0, len(manifest["runs"])]))
    refresh_roi("before COM survey")

    survey = manifest["com_survey"]
    survey_config = root_path(survey["config"])
    survey_capture = root_path(survey["capture"])
    survey_analyzer = root_path(survey["analyzer"])
    survey_result = root_path(survey["result"])
    survey_targets = [root_path(value) for value in survey["apply_to"]]
    camera_targets = [root_path(value)
                      for value in survey["camera_apply_to"]]
    for path in [survey_config, survey_analyzer, *survey_targets,
                 *camera_targets]:
        if not path.is_file():
            raise SystemExit(f"COM survey input not found: {path}")
    capture_command = [str(anyloop), *anyloop_args, str(survey_config)]
    exposure, gain = read_camera_settings(survey_config)
    measured_fs = float(survey["fs_hz"])
    analyze_command = [
        str(survey_analyzer), str(survey_capture),
        "--fs", str(survey["fs_hz"]),
        "--exposure-us", str(exposure),
        "--max-exposure-us", str(survey["max_exposure_us"]),
        "--gain", str(gain), "--json", str(survey_result),
        "--pwm-hz", str(survey["pwm_hz"]),
        "--pwm-duty", str(survey["pwm_duty"]),
    ]
    print(f"[COM survey capture] {' '.join(capture_command)}", flush=True)
    print(f"[COM survey analyze] {' '.join(analyze_command)}", flush=True)
    if not opts.dry_run:
        # If the first survey changes exposure/gain, capture once more so the
        # tracker recommendation and gates describe the settings the sweep
        # will actually use. Never proceed from an unvalidated camera change.
        for attempt in range(4):
            capture_log = ROOT / "data/calibration/run_staging/com_survey_capture.log"
            with capture_log.open("w", encoding="utf-8") as stream:
                subprocess.run(capture_command, cwd=ROOT, check=True,
                               stdout=stream, stderr=subprocess.STDOUT)
            exposure, gain = read_camera_settings(survey_config)
            measured_fs = measured_camera_fs(capture_log, survey["fs_hz"])
            analyze_command = [
                str(survey_analyzer), str(survey_capture),
                "--fs", str(measured_fs),
                "--exposure-us", str(exposure),
                "--max-exposure-us", str(survey["max_exposure_us"]),
                "--gain", str(gain), "--json", str(survey_result),
                "--pwm-hz", str(survey["pwm_hz"]),
                "--pwm-duty", str(survey["pwm_duty"]),
            ]
            analyzer_log = ROOT / "data/calibration/run_staging/com_survey_analyzer.log"
            with analyzer_log.open("w", encoding="utf-8") as stream:
                subprocess.run(analyze_command, cwd=ROOT, check=True,
                               stderr=stream)
            with survey_result.open(encoding="utf-8") as stream:
                survey_values = json.load(stream)
            # Passing fit parameters are already adequate for the live beam.
            # Do not chase a brightness target after a pass; camera changes
            # are reserved for a failed fit (darkness or saturation), then
            # validated by the next capture.
            changed = False
            if survey_values.get("recommendation") != "fit_com":
                changed = apply_camera(
                    survey_result, camera_targets,
                    survey.get("lock_exposure", False))
            if not changed:
                break
            if attempt < 3:
                print("[COM survey] camera settings changed; recapturing "
                      "before the sweeps", flush=True)
        else:
            raise SystemExit(
                "COM survey camera recommendation did not converge after "
                "four captures; refusing an unvalidated sweep")
        apply_survey(survey_result, survey_targets,
                     survey.get("max_rejected_fraction", 0.25))

    if opts.com_only:
        print("[COM survey] fit_com-only validation complete; actuator "
              "calibrations were not run", flush=True)
        return 0

    inputs = []
    run_results = {}
    selected_runs = []
    update_names = manifest.get("update_run", {})
    gain_names = {update_names.get("x_gain_run"),
                  update_names.get("y_gain_run")}
    delay_names = {update_names.get("x_delay_run"),
                   update_names.get("y_delay_run")}
    for run in manifest["runs"]:
        if opts.skip_gain and run["name"] in gain_names:
            print(f"[{run['name']}] skipped by --skip-gain", flush=True)
            continue
        if opts.skip_delay and run["name"] in delay_names:
            print(f"[{run['name']}] skipped by --skip-delay", flush=True)
            continue
        selected_runs.append(run)
    for run_index, run in enumerate(selected_runs):
        if roi.get("before_each_run", False) or (
                run_index in refresh_before_runs and run_index != 0):
            refresh_roi(f"before run {run_index + 1}: {run['name']}")
        config = root_path(run["config"])
        pdf = root_path(run["pdf"])
        if not config.is_file():
            raise SystemExit(f"configuration not found: {config}")
        amplitudes = run.get("prbs_amplitudes")
        variants = [(None, pdf)]
        temporary_configs = []
        if amplitudes:
            variants = []
            for amplitude in amplitudes:
                tag = f"{abs(amplitude):g}".replace(".", "p")
                if amplitude < 0:
                    tag = "m" + tag
                variant_pdf = pdf.with_name(f"{pdf.stem}_a{tag}{pdf.suffix}")
                variants.append((amplitude, variant_pdf))
        delays = []
        try:
            for variant_index, (amplitude, run_pdf) in enumerate(variants):
                label = run["name"] if amplitude is None \
                    else f"{run['name']} amplitude {amplitude:g}"
                if roi.get("before_each_variant", False) and (
                        not roi.get("before_each_run", False)
                        or variant_index > 0):
                    refresh_roi(f"before {label}")
                # Build amplitude variants only after the ROI refresh, so the
                # temporary config includes the newly written crop instead of
                # retaining the position from before the refresh.
                run_config = config
                if amplitude is not None:
                    run_config = prbs_variant(config, run_pdf, amplitude)
                    temporary_configs.append(run_config)
                command = [str(anyloop), *anyloop_args, str(run_config)]
                print(f"[{label}] {' '.join(command)}", flush=True)
                inputs.append(run_pdf)
                if opts.dry_run:
                    continue
                previous_mtime = (run_pdf.stat().st_mtime_ns
                                  if run_pdf.exists() else None)
                started = time.time_ns()
                subprocess.run(command, cwd=ROOT, check=True)
                if not run_pdf.is_file():
                    raise SystemExit(f"{label} did not create {run_pdf}")
                if previous_mtime is not None \
                        and run_pdf.stat().st_mtime_ns == previous_mtime:
                    raise SystemExit(
                        f"{label} left the existing {run_pdf.name} unchanged; "
                        "refusing to combine a stale report")
                if run_pdf.stat().st_mtime_ns < started:
                    raise SystemExit(
                        f"{label} produced an unexpectedly old {run_pdf}")
                if amplitudes:
                    delays.append((amplitude, read_prbs_delay(run_pdf)))
                elif any(device.get("uri") == "anyloop:gain_test"
                         for device in json.loads(
                             run_config.read_text(encoding="utf-8"))
                         .get("pipeline", [])):
                    run_results[run["name"]] = read_gain(run_pdf)
        finally:
            for temporary_config in temporary_configs:
                temporary_config.unlink(missing_ok=True)
        if delays:
            values = [value for _, value in delays]
            spread = max(values) - min(values)
            limit = run["max_delay_spread_frames"]
            summary = ", ".join(f"{amp:g} -> {value:.3f} fr"
                                for amp, value in delays)
            print(f"[{run['name']} consistency] {summary}; "
                  f"spread {spread:.3f} fr", flush=True)
            if spread > limit:
                raise SystemExit(
                    f"{run['name']} delay changes by {spread:.3f} frames "
                    f"across amplitudes (limit {limit:.3f}); tracking or "
                    "plant dynamics are amplitude-dependent")
            run_results[run["name"]] = statistics.median(values)

    if len(manifest["runs"]) in refresh_before_runs:
        refresh_roi("after calibrations / before steering_par_fsp")

    update = manifest.get("update_run")
    if update and not opts.dry_run:
        target = root_path(update["config"])
        x_gain = None if opts.skip_gain else run_results[update["x_gain_run"]]
        y_gain = None if opts.skip_gain else run_results[update["y_gain_run"]]
        x_delay = (None if opts.skip_delay
                   else run_results[update["x_delay_run"]])
        y_delay = (None if opts.skip_delay
                   else run_results[update["y_delay_run"]])
        update_run_config(
            target, root_path(update["source_config"]), measured_fs,
            x_gain, y_gain, x_delay, y_delay)
        for shared in update.get("shared_configs", []):
            shared_path = root_path(shared)
            if shared_path.resolve() == target.resolve():
                continue
            update_shared_plant(shared_path, measured_fs,
                                x_gain, y_gain, x_delay, y_delay)

    if not inputs:
        print("[combine] no actuator reports requested", flush=True)
        return 0
    print(f"[combine] {output}", flush=True)
    if opts.dry_run:
        return 0

    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{output.stem}.", suffix=".pdf", dir=output.parent)
    os.close(fd)
    temporary = Path(temporary_name)
    temporary.unlink()
    try:
        subprocess.run([pdfunite, *(str(pdf) for pdf in inputs), str(temporary)],
                       cwd=ROOT, check=True)
        os.replace(temporary, output)
    finally:
        temporary.unlink(missing_ok=True)
    print(f"combined calibration report written to {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
