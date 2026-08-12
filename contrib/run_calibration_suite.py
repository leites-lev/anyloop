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


ROOT = Path(__file__).resolve().parent.parent


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
    # These are loop/timing policy, not optical properties measured by this
    # open-loop survey. Keep the values already validated in each live config.
    preserve = {"max_iter", "max_us", "reacquire_after"}
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


def apply_camera(result_path, target_paths):
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
        new = (round(suggested["exposure"]), round(suggested["gain"]))
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


def main():
    parser = argparse.ArgumentParser(
        description="Run gain/delay calibrations and combine their PDFs")
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--dry-run", action="store_true",
                        help="validate and print commands without driving hardware")
    parser.add_argument("--com-only", action="store_true",
                        help="refresh ROI and validate/apply fit_com parameters, "
                             "then stop before actuator calibration runs")
    opts = parser.parse_args()

    manifest_path = opts.manifest.resolve()
    manifest = load_manifest(manifest_path)
    anyloop = root_path(manifest.get("anyloop", "build/anyloop"))
    anyloop_args = manifest.get("anyloop_args", [])
    output = root_path(manifest["output_file"])
    pdfunite = shutil.which("pdfunite")

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
    analyze_command = [
        str(survey_analyzer), str(survey_capture),
        "--fs", str(survey["fs_hz"]),
        "--exposure-us", str(exposure),
        "--max-exposure-us", str(survey["max_exposure_us"]),
        "--gain", str(gain), "--json", str(survey_result),
    ]
    print(f"[COM survey capture] {' '.join(capture_command)}", flush=True)
    print(f"[COM survey analyze] {' '.join(analyze_command)}", flush=True)
    if not opts.dry_run:
        # If the first survey changes exposure/gain, capture once more so the
        # tracker recommendation and gates describe the settings the sweep
        # will actually use. Never proceed from an unvalidated camera change.
        for attempt in range(4):
            capture_log = ROOT / "com_survey_capture.log"
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
            ]
            analyzer_log = ROOT / "com_survey_analyzer.log"
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
                changed = apply_camera(survey_result, camera_targets)
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
    for run_index, run in enumerate(manifest["runs"]):
        if run_index in refresh_before_runs and run_index != 0:
            refresh_roi(f"before run {run_index + 1}: {run['name']}")
        config = root_path(run["config"])
        pdf = root_path(run["pdf"])
        if not config.is_file():
            raise SystemExit(f"configuration not found: {config}")
        amplitudes = run.get("prbs_amplitudes")
        variants = [(None, config, pdf)]
        temporary_configs = []
        if amplitudes:
            variants = []
            for amplitude in amplitudes:
                tag = f"{abs(amplitude):g}".replace(".", "p")
                if amplitude < 0:
                    tag = "m" + tag
                variant_pdf = pdf.with_name(f"{pdf.stem}_a{tag}{pdf.suffix}")
                variant_config = prbs_variant(config, variant_pdf, amplitude)
                temporary_configs.append(variant_config)
                variants.append((amplitude, variant_config, variant_pdf))
        delays = []
        try:
            for amplitude, run_config, run_pdf in variants:
                label = run["name"] if amplitude is None \
                    else f"{run['name']} amplitude {amplitude:g}"
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

    if len(manifest["runs"]) in refresh_before_runs:
        refresh_roi("after calibrations / before steering_par_fsp")

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
