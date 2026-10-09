import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).resolve().parents[2] / "scripts/eval/ray_coverage_recall.py"
SPEC = importlib.util.spec_from_file_location("argus_ray_protocol", SCRIPT)
rays = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(rays)


def truth_frame(index, count=1, present=True):
    return {"frame": index, "distance_m": 60.0 if present else None,
            "box_min": [0, -60, -1] if present else [],
            "rays_replaced": count, "rays_added": 0}


def prediction(index, alert="1", distance="60"):
    return {"frame": str(index), "alert": alert, "forward_m": distance}


def test_missing_predictions_and_zero_rays_remain_misses():
    truth = {"frames": [truth_frame(0, 0), truth_frame(1, 4), truth_frame(2, 0, False)]}
    frames = rays.analyze([], truth, 3)
    assert len(frames) == 3
    assert [frame["active"] for frame in frames] == [True, True, False]
    assert not any(frame["prediction_received"] for frame in frames)
    assert rays.summarize(frames)[0] == {
        "bin": "0-8", "frames": 2, "matched": 0, "missed": 2, "recall": 0.0}


def test_matched_distance_and_missing_inactive_frame():
    truth = {"frames": [truth_frame(0), truth_frame(1), truth_frame(2, 0, False)]}
    frames = rays.analyze([prediction(0, distance="62.5"), prediction(1, distance="63.1")],
                          truth, 3)
    assert [frame["matched"] for frame in frames] == [True, False, False]
    assert rays.summarize(frames)[0]["recall"] == 0.5


def test_zero_ray_target_cannot_match_background_alert():
    frames = rays.analyze([prediction(0)], {"frames": [truth_frame(0, 0)]}, 3)
    assert frames[0]["active"]
    assert frames[0]["alert"]
    assert not frames[0]["matched"]


def test_nonfinite_truth_cannot_be_excluded_as_inactive():
    frame = truth_frame(0)
    frame["distance_m"] = float("nan")
    with pytest.raises(ValueError, match="truth distance"):
        rays.analyze([], {"frames": [frame]}, 3)


@pytest.mark.parametrize("rows", [
    [prediction(0), prediction(0)], [prediction(1)], [prediction(0, alert="2")],
    [prediction(0, distance="nan")],
])
def test_invalid_predictions_are_rejected(rows):
    with pytest.raises(ValueError):
        rays.analyze(rows, {"frames": [truth_frame(0)]}, 3)


@pytest.mark.parametrize("value", ["0:1:1:60", "1:nan:1:60", "1:1:1:inf", "1:1:1"])
def test_invalid_cases_are_rejected(value):
    with pytest.raises(ValueError):
        rays.parse_case(value)


@pytest.mark.parametrize("maximum", [0, -1, float("nan"), float("inf")])
def test_invalid_range_is_rejected(maximum):
    with pytest.raises(ValueError):
        rays.detector_command("detector", "scene.frames", maximum)


def test_detect_uses_explicit_range_and_keeps_stderr(tmp_path, monkeypatch):
    commands = []

    def run(command, **kwargs):
        commands.append(command)
        return SimpleNamespace(stdout="frame,alert,clusters,raw,filtered,tracks,forward_m,"
                               "range_m,anom_points,anom_cells\n0,0,0,0,0,0,-1,-1,0,0\n",
                               stderr="candidate detail\n")

    monkeypatch.setattr(rays.subprocess, "run", run)
    output = tmp_path / "predictions.csv"
    rows = rays.detect("detector", "scene.frames", output)
    assert commands == [["detector", "scene.frames", "--fusion", "--geom-max-range", "150.0"]]
    assert rows[0]["frame"] == "0"
    assert output.with_suffix(".stderr.txt").read_text() == "candidate detail\n"


def test_malformed_csv_cannot_be_silently_filtered(tmp_path, monkeypatch):
    monkeypatch.setattr(rays.subprocess, "run", lambda *args, **kwargs:
                        SimpleNamespace(stdout="frame,alert\n0,1\n", stderr=""))
    with pytest.raises(ValueError, match="header"):
        rays.detect("detector", "scene.frames", tmp_path / "predictions.csv", 45)


def test_paired_ranges_share_exact_generated_scene(tmp_path, monkeypatch):
    names = ("source", "poses", "detector", "generator")
    source, poses, detector, generator = [tmp_path / name for name in names]
    for path in (source, poses, detector, generator):
        path.write_text(path.name)
    generated = []

    def generate(*args):
        generated.append(args)
        Path(args[3]).write_bytes(b"identical synthetic scene")
        truth = {"frames": [truth_frame(0)]}
        Path(args[4]).write_text(json.dumps(truth))
        return truth

    def detect(binary, scene, output, maximum):
        output.write_text("prediction")
        output.with_suffix(".stderr.txt").write_text("")
        return [prediction(0, alert="0" if maximum == 45 else "1")]

    monkeypatch.setattr(rays, "generate", generate)
    monkeypatch.setattr(rays, "detect", detect)
    output = tmp_path / "report.json"
    rays.main(["--frames", str(source), "--poses", str(poses), "--detector", str(detector),
               "--generator", str(generator), "--cases", "1:1:1:60", "--frames-n", "1",
               "--start-frame", "0", "--geom-max-ranges", "45,150", "--out", str(output)])
    report = json.loads(output.read_text())
    assert len(generated) == 1
    assert [case["matched_frames"] for case in report["cases"]] == [0, 1]
    assert report["cases"][0]["scene"]["sha256"] == report["cases"][1]["scene"]["sha256"]
    assert report["receipts"]["source"]["bytes"] == len("source")
    assert report["evaluation_kind"] == "synthetic_nearest_distance_diagnostic"
