import copy
import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from argus_eval.benchmark import digest, evaluate, freeze, match, score_scene


PROTOCOL = {'max_center_error_m': 1.0, 'max_event_gap_ns': 150000000}


def obj(identity='person', x=0, y=20, z=0):
    return {'object_id': identity, 'position_m': [x, y, z]}


def detection(identity='track', x=0, y=20, z=0):
    return {'track_id': identity, 'position_m': [x, y, z]}


def frame(stamp=100000000, objects=None, reviewed=True, path=0):
    return {'stamp_ns': stamp, 'objects': objects or [], 'reviewed': reviewed, 'path_m': path}


def predicted(stamp=100000000, detections=None):
    return {'stamp_ns': stamp, 'detections': detections or []}


def annotations(frames=None):
    return {'frames': frames if frames is not None else [frame(objects=[obj()])],
            'reference_distance_verified': False, 'synthetic': False, 'coordinate_frame': 'lidar'}


def test_missing_prediction_and_low_coverage_still_count_as_misses():
    truth = annotations([frame(objects=[{**obj(), 'rays': 2}]),
                         frame(200000000, [obj('other')])])
    result = score_scene(truth, [predicted(detections=[detection()])], PROTOCOL)
    assert result['object_recall'] == 0.5
    assert result['object_frame_recall'] == 0.5
    assert result['prediction_coverage'] == 0.5
    assert result['missed_objects'] == 1


def test_same_range_wrong_lateral_position_is_false_detection():
    result = score_scene(annotations(), [predicted(detections=[detection(x=5)])], PROTOCOL)
    assert result['object_recall'] == 0
    assert result['false_track_events'] == 1


def test_matching_maximizes_cardinality_in_conflicting_assignment():
    pairs = match([obj('a', x=0), obj('b', x=1.5)],
                  [detection('one', x=0.7), detection('two', x=-0.8)], 1.0)
    assert pairs == {0: 1, 1: 0}


def test_duplicate_detections_cannot_inflate_recall():
    result = score_scene(annotations(),
                         [predicted(detections=[detection('one'), detection('two')])], PROTOCOL)
    assert result['detected_objects'] == 1
    assert result['false_detections'] == 1


def test_first_detection_uses_chronology_and_ground_truth_range():
    truth = annotations([frame(objects=[obj(y=40)]), frame(200000000, [obj(y=20)])])
    result = score_scene(truth, [predicted(200000000, [detection(y=20.5)])], PROTOCOL)
    assert result['objects']['person'] == {
        'first_detection_stamp_ns': 200000000, 'first_detection_range_m': 20.0}
    assert result['object_frame_recall'] == 0.5


@pytest.mark.parametrize('gap,expected', [(100000000, 1), (200000000, 2)])
def test_false_tracks_are_events_not_alert_frames(gap, expected):
    truth = annotations([frame(), frame(100000000 + gap)])
    preds = [predicted(detections=[detection()]),
             predicted(100000000 + gap, [detection()])]
    result = score_scene(truth, preds, PROTOCOL)
    assert result['false_detections'] == 2
    assert result['false_track_events'] == expected


@pytest.mark.parametrize('reviewed', [True, False])
def test_missing_or_unreviewed_frame_breaks_false_event(reviewed):
    truth = annotations([frame(), frame(200000000, reviewed=reviewed), frame(300000000)])
    preds = [predicted(detections=[detection()]), predicted(300000000, [detection()])]
    result = score_scene(truth, preds, PROTOCOL)
    assert result['false_track_events'] == 2


def test_kilometric_rate_requires_verified_complete_reference_distance():
    truth = annotations([frame(), frame(200000000, path=100)])
    preds = [predicted(detections=[detection()]), predicted(200000000)]
    assert score_scene(truth, preds, PROTOCOL)['false_track_events_per_km'] is None
    truth['reference_distance_verified'] = True
    assert score_scene(truth, preds, PROTOCOL)['false_track_events_per_km'] == 10
    truth['frames'][1]['reviewed'] = False
    assert score_scene(truth, preds, PROTOCOL)['false_track_events_per_km'] is None


@pytest.mark.parametrize('preds', [[], [predicted()]])
def test_lost_predictions_cannot_report_zero_false_events_per_kilometer(preds):
    truth = annotations([frame(), frame(200000000, path=100)])
    truth['reference_distance_verified'] = True
    result = score_scene(truth, preds, PROTOCOL)
    assert result['false_track_events'] == 0
    assert result['false_track_events_per_km'] is None
    assert result['prediction_coverage'] < 1


def test_empty_denominators_do_not_report_perfect_recall_or_zero_rate():
    truth = annotations([frame()])
    truth['reference_distance_verified'] = True
    result = score_scene(truth, [], PROTOCOL)
    assert result['object_recall'] is None
    assert result['false_track_events_per_km'] is None


@pytest.mark.parametrize('mutation', ['duplicate_stamp', 'extra_stamp', 'nan', 'duplicate_track'])
def test_invalid_predictions_are_rejected(mutation):
    preds = [predicted(detections=[detection()])]
    if mutation == 'duplicate_stamp':
        preds.append(preds[0])
    elif mutation == 'extra_stamp':
        preds[0]['stamp_ns'] = 1
    elif mutation == 'nan':
        preds[0]['detections'][0]['position_m'][0] = float('nan')
    else:
        preds[0]['detections'].append(preds[0]['detections'][0])
    with pytest.raises(ValueError):
        score_scene(annotations(), preds, PROTOCOL)


def write_json(path, value):
    path.write_text(json.dumps(value), encoding='utf-8')


@pytest.fixture
def dataset(tmp_path):
    source = tmp_path / 'source.txt'
    source.write_text('test recording', encoding='utf-8')
    config = tmp_path / 'params.yaml'
    config.write_text('detector: fixed', encoding='utf-8')
    truth = annotations()
    truth['source_sha256'] = digest(source)
    write_json(tmp_path / 'labels.json', truth)
    spec = {
        'protocol': PROTOCOL, 'detector_config': 'params.yaml',
        'detector_revision': '1' * 40, 'run_command': 'test detector', 'scenes': [{
            'scene_id': 'scene', 'group_id': 'recording', 'split': 'holdout',
            'unseen_at_freeze': True, 'source': 'source.txt', 'annotations': 'labels.json'}]}
    write_json(tmp_path / 'spec.json', spec)
    return tmp_path, spec


def prediction_receipt(root, manifest):
    receipt = {'manifest_sha256': digest(root / 'manifest.json'),
               'detector_revision': manifest['detector_revision'],
               'detector_config_sha256': manifest['detector_config_sha256'], 'scenes': {
                   'scene': {'source_sha256': manifest['scenes'][0]['source_sha256'],
                             'frames': [predicted(detections=[detection()])]}}}
    write_json(root / 'predictions.json', receipt)
    return receipt


def test_frozen_end_to_end_report_and_overwrite_protection(dataset):
    root, _ = dataset
    manifest = freeze(root / 'spec.json', root / 'manifest.json')
    prediction_receipt(root, manifest)
    result = evaluate(root / 'manifest.json', root / 'predictions.json')['scenes']['scene']
    assert result['object_recall'] == 1
    assert result['independence'] == 'declared_unseen_real_holdout'
    with pytest.raises(FileExistsError):
        freeze(root / 'spec.json', root / 'manifest.json')


def test_frozen_bundle_can_be_relocated_without_changing_receipt(dataset):
    root, _ = dataset
    manifest = freeze(root / 'spec.json', root / 'manifest.json')
    prediction_receipt(root, manifest)
    destination = root.parent / (root.name + '-moved')
    shutil.copytree(root, destination)
    result = evaluate(destination / 'manifest.json', destination / 'predictions.json')
    assert result['scenes']['scene']['object_recall'] == 1


def test_manifest_paths_are_relative_to_output_not_spec(dataset):
    root, _ = dataset
    output = root / 'reports'
    output.mkdir()
    freeze(root / 'spec.json', output / 'manifest.json')
    manifest = json.loads((output / 'manifest.json').read_text(encoding='utf-8'))
    assert manifest['detector_config'] == '../params.yaml'
    assert manifest['scenes'][0]['annotations'] == '../labels.json'


@pytest.mark.parametrize('filename', ['source.txt', 'labels.json', 'params.yaml', 'manifest.json'])
def test_changed_inputs_invalidate_frozen_receipt(dataset, filename):
    root, _ = dataset
    manifest = freeze(root / 'spec.json', root / 'manifest.json')
    prediction_receipt(root, manifest)
    with (root / filename).open('a', encoding='utf-8') as stream:
        stream.write(' ')
    with pytest.raises(ValueError):
        evaluate(root / 'manifest.json', root / 'predictions.json')


@pytest.mark.parametrize('kind', ['used_holdout', 'group_leak', 'content_leak', 'wrong_source'])
def test_invalid_dataset_provenance_is_rejected(dataset, kind):
    root, spec = dataset
    if kind == 'used_holdout':
        spec['scenes'][0]['unseen_at_freeze'] = False
    elif kind == 'wrong_source':
        truth = json.loads((root / 'labels.json').read_text(encoding='utf-8'))
        truth['source_sha256'] = 'wrong'
        write_json(root / 'labels.json', truth)
    else:
        extra = copy.deepcopy(spec['scenes'][0])
        extra.update(scene_id='other', split='development', unseen_at_freeze=False)
        if kind == 'content_leak':
            extra['group_id'] = 'renamed-recording'
        spec['scenes'].append(extra)
    write_json(root / 'spec.json', spec)
    with pytest.raises(ValueError):
        freeze(root / 'spec.json', root / 'manifest.json')


def test_cli_freeze_and_score(dataset):
    root, _ = dataset
    script = Path(__file__).resolve().parents[1] / 'argus_eval' / 'benchmark.py'
    subprocess.run([sys.executable, str(script), 'freeze', str(root / 'spec.json'),
                    str(root / 'manifest.json')], check=True)
    prediction_receipt(root, json.loads((root / 'manifest.json').read_text(encoding='utf-8')))
    subprocess.run([sys.executable, str(script), 'score', str(root / 'manifest.json'),
                    str(root / 'predictions.json'), str(root / 'report.json')], check=True)
    result = json.loads((root / 'report.json').read_text(encoding='utf-8'))
    assert result['scenes']['scene']['detected_objects'] == 1


@pytest.mark.parametrize(
    'field', ['manifest_sha256', 'detector_config_sha256',
              'detector_revision', 'source_sha256', 'scenes'])
def test_incompatible_prediction_receipt_is_rejected(dataset, field):
    root, _ = dataset
    manifest = freeze(root / 'spec.json', root / 'manifest.json')
    receipt = prediction_receipt(root, manifest)
    if field == 'source_sha256':
        receipt['scenes']['scene'][field] = 'wrong'
    elif field == 'scenes':
        receipt['scenes'] = {}
    else:
        receipt[field] = 'wrong'
    write_json(root / 'predictions.json', receipt)
    with pytest.raises(ValueError):
        evaluate(root / 'manifest.json', root / 'predictions.json')


@pytest.mark.parametrize('kind', ['synthetic', 'incomplete', 'development'])
def test_development_and_partial_data_do_not_gain_independent_label(dataset, kind):
    root, spec = dataset
    truth = json.loads((root / 'labels.json').read_text(encoding='utf-8'))
    if kind == 'synthetic':
        truth['synthetic'] = True
    elif kind == 'incomplete':
        truth['frames'].append(frame(200000000, reviewed=False))
    else:
        spec['scenes'][0].update(split='development', unseen_at_freeze=False)
    write_json(root / 'labels.json', truth)
    write_json(root / 'spec.json', spec)
    manifest = freeze(root / 'spec.json', root / 'manifest.json')
    prediction_receipt(root, manifest)
    result = evaluate(root / 'manifest.json', root / 'predictions.json')['scenes']['scene']
    assert result['independence'] == 'development_or_incomplete'


def test_nonmonotonic_reference_distance_is_rejected():
    truth = annotations([frame(path=10), frame(200000000, path=9)])
    truth['reference_distance_verified'] = True
    with pytest.raises(ValueError):
        score_scene(truth, [], PROTOCOL)


def test_unreviewed_objects_and_empty_timeline_are_rejected():
    with pytest.raises(ValueError):
        score_scene(annotations([frame(objects=[obj()], reviewed=False)]), [], PROTOCOL)
    with pytest.raises(ValueError):
        score_scene(annotations([]), [], PROTOCOL)
