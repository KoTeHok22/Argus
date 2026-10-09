import argparse
import hashlib
import json
import math
import os
import re
from pathlib import Path


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def frozen_path(path, output):
    try:
        return Path(os.path.relpath(path, Path(output).resolve().parent)).as_posix()
    except ValueError:
        return str(path)


def number(value, name, minimum=0):
    if type(value) not in (int, float) or not math.isfinite(value) or value < minimum:
        raise ValueError(f'{name}: expected a finite number >= {minimum}')
    return value


def identifier(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError('Object and scene identifiers must be nonempty strings')
    return value


def position(item):
    value = item['position_m']
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError('position_m must contain three coordinates')
    return [number(x, 'position_m', -math.inf) for x in value]


def validate_frames(frames, truth):
    previous = -1
    for frame in frames:
        stamp = frame['stamp_ns']
        if type(stamp) is not int or stamp < 0 or stamp <= previous:
            raise ValueError('Frame timestamps must be unique, increasing source timestamps')
        previous = stamp
        if truth and type(frame['reviewed']) is not bool:
            raise ValueError('reviewed must be a boolean')
        items = frame['objects'] if truth else frame['detections']
        identities = [identifier(item['object_id' if truth else 'track_id']) for item in items]
        if len(set(identities)) != len(identities):
            raise ValueError('Duplicate object or track identifier in a frame')
        for item in items:
            position(item)
        if truth and not frame['reviewed'] and items:
            raise ValueError('Unreviewed frames cannot contain scored objects')


def freeze(spec_path, output):
    spec_path = Path(spec_path).resolve()
    spec = read_json(spec_path)
    root = spec_path.parent
    if not re.fullmatch(r'[0-9a-f]{40}', spec['detector_revision']):
        raise ValueError('detector_revision must be a full Git commit identifier')
    identifier(spec['run_command'])
    protocol = spec['protocol']
    if number(protocol['max_center_error_m'], 'max_center_error_m') == 0:
        raise ValueError('Matching tolerance must be positive')
    number(protocol['max_event_gap_ns'], 'max_event_gap_ns')
    groups = {}
    sources = {}
    scene_ids = set()
    for scene in spec['scenes']:
        scene_id = identifier(scene['scene_id'])
        if scene_id in scene_ids:
            raise ValueError('Duplicate scene identifier')
        scene_ids.add(scene_id)
        split = scene['split']
        if split not in ('development', 'holdout'):
            raise ValueError('Unknown split')
        group = identifier(scene['group_id'])
        if group in groups and groups[group] != split:
            raise ValueError('A recording group cannot cross dataset splits')
        groups[group] = split
        if type(scene['unseen_at_freeze']) is not bool:
            raise ValueError('unseen_at_freeze must be a boolean')
        if split == 'holdout' and not scene['unseen_at_freeze']:
            raise ValueError('Previously used scenes cannot be declared holdout')
        for key in ('source', 'annotations'):
            path = (root / scene[key]).resolve()
            scene[key] = frozen_path(path, output)
            scene[key + '_sha256'] = digest(path)
        source_hash = scene['source_sha256']
        if source_hash in sources and sources[source_hash] != split:
            raise ValueError('Identical source bytes cannot cross dataset splits')
        sources[source_hash] = split
        annotations = read_json(Path(output).resolve().parent / scene['annotations'])
        validate_frames(annotations['frames'], True)
        if not annotations['frames']:
            raise ValueError('Annotation timeline cannot be empty')
        if annotations['source_sha256'] != source_hash:
            raise ValueError('Annotations refer to a different source')
        if annotations['coordinate_frame'] != 'lidar':
            raise ValueError('Annotations must use the source LiDAR coordinate frame')
        if type(annotations['synthetic']) is not bool:
            raise ValueError('synthetic must be a boolean')
    if not scene_ids:
        raise ValueError('Dataset cannot be empty')
    config = (root / spec['detector_config']).resolve()
    spec['detector_config'] = frozen_path(config, output)
    spec['detector_config_sha256'] = digest(config)
    spec['schema_version'] = 1
    with Path(output).open('x', encoding='utf-8') as stream:
        json.dump(spec, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    return spec


def match(objects, detections, tolerance):
    candidates = []
    for obj in objects:
        edges = [(math.dist(position(obj), position(det)), index)
                 for index, det in enumerate(detections)]
        candidates.append([index for distance, index in sorted(edges) if distance <= tolerance])
    owners = {}

    def assign(obj_index, visited):
        for det_index in candidates[obj_index]:
            if det_index in visited:
                continue
            visited.add(det_index)
            if det_index not in owners or assign(owners[det_index], visited):
                owners[det_index] = obj_index
                return True
        return False

    for index in range(len(objects)):
        assign(index, set())
    return {obj: det for det, obj in owners.items()}


def score_scene(annotations, predictions, protocol):
    frames = annotations['frames']
    if not frames:
        raise ValueError('Annotation timeline cannot be empty')
    validate_frames(frames, True)
    validate_frames(predictions, False)
    timeline = {frame['stamp_ns'] for frame in frames}
    predicted = {frame['stamp_ns']: frame for frame in predictions}
    if not set(predicted).issubset(timeline):
        raise ValueError('Predictions contain timestamps absent from the source timeline')
    objects = {}
    presences = matches = false_detections = false_events = reviewed = received = 0
    previous_false = set()
    previous_stamp = None
    for frame in frames:
        stamp = frame['stamp_ns']
        received += stamp in predicted
        if not frame['reviewed']:
            previous_false = set()
            previous_stamp = None
            continue
        reviewed += 1
        detections = predicted.get(stamp, {'detections': []})['detections']
        pairs = match(frame['objects'], detections, protocol['max_center_error_m'])
        presences += len(frame['objects'])
        matches += len(pairs)
        for index, obj in enumerate(frame['objects']):
            record = objects.setdefault(obj['object_id'], {
                'first_detection_stamp_ns': None, 'first_detection_range_m': None})
            if index in pairs and record['first_detection_stamp_ns'] is None:
                record['first_detection_stamp_ns'] = stamp
                record['first_detection_range_m'] = math.dist(position(obj), [0, 0, 0])
        used = set(pairs.values())
        current_false = {
            det['track_id'] for index, det in enumerate(detections) if index not in used}
        if previous_stamp is None or stamp - previous_stamp > protocol['max_event_gap_ns']:
            previous_false = set()
        false_events += len(current_false - previous_false)
        false_detections += len(current_false)
        previous_false = current_false
        previous_stamp = stamp
    detected = sum(record['first_detection_stamp_ns'] is not None for record in objects.values())
    distance = None
    if annotations.get('reference_distance_verified') is True and reviewed == len(frames):
        distances = [number(frame['path_m'], 'path_m') for frame in frames]
        if any(right < left for left, right in zip(distances, distances[1:])):
            raise ValueError('Reference path must be monotonic')
        distance = distances[-1] - distances[0]
    return {
        'source_frames': len(frames), 'prediction_frames': received,
        'prediction_coverage': received / len(frames), 'reviewed_frames': reviewed,
        'annotation_coverage': reviewed / len(frames), 'objects': objects,
        'total_objects': len(objects), 'detected_objects': detected,
        'missed_objects': len(objects) - detected,
        'object_recall': detected / len(objects) if objects else None,
        'object_frame_recall': matches / presences if presences else None,
        'matched_object_frames': matches, 'total_object_frames': presences,
        'false_detections': false_detections, 'false_track_events': false_events,
        'reference_distance_m': distance,
        'false_track_events_per_km': false_events * 1000 / distance if distance else None,
        'metric_scope': 'fully_reviewed' if reviewed == len(frames) else 'reviewed_frames_only',
    }


def evaluate(manifest_path, predictions_path):
    root = Path(manifest_path).resolve().parent
    manifest = read_json(manifest_path)
    predictions = read_json(predictions_path)
    if manifest['schema_version'] != 1:
        raise ValueError('Unsupported manifest version')
    if predictions['manifest_sha256'] != digest(manifest_path):
        raise ValueError('Prediction manifest receipt does not match the frozen manifest')
    if predictions['detector_revision'] != manifest['detector_revision']:
        raise ValueError('Detector revision does not match the frozen manifest')
    config_hash = digest(root / manifest['detector_config'])
    if (config_hash != manifest['detector_config_sha256']
            or predictions['detector_config_sha256'] != config_hash):
        raise ValueError('Detector configuration changed after freezing')
    expected = {scene['scene_id'] for scene in manifest['scenes']}
    if set(predictions['scenes']) != expected:
        raise ValueError('Prediction scenes must exactly match the manifest')
    report = {'manifest_sha256': digest(manifest_path), 'scenes': {}}
    for scene in manifest['scenes']:
        for key in ('source', 'annotations'):
            if digest(root / scene[key]) != scene[key + '_sha256']:
                raise ValueError(f'{key} changed after freezing')
        annotations = read_json(root / scene['annotations'])
        prediction = predictions['scenes'][scene['scene_id']]
        if prediction['source_sha256'] != scene['source_sha256']:
            raise ValueError('Predictions refer to a different source')
        result = score_scene(annotations, prediction['frames'], manifest['protocol'])
        result['split'] = scene['split']
        result['independence'] = ('declared_unseen_real_holdout'
                                  if scene['split'] == 'holdout' and scene['unseen_at_freeze']
                                  and not annotations['synthetic']
                                  and result['annotation_coverage'] == 1
                                  else 'development_or_incomplete')
        report['scenes'][scene['scene_id']] = result
    return report


def main():
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest='command', required=True)
    lock = commands.add_parser('freeze')
    lock.add_argument('spec')
    lock.add_argument('output')
    run = commands.add_parser('score')
    run.add_argument('manifest')
    run.add_argument('predictions')
    run.add_argument('output')
    args = parser.parse_args()
    if args.command == 'freeze':
        freeze(args.spec, args.output)
    else:
        report = evaluate(args.manifest, args.predictions)
        with Path(args.output).open('x', encoding='utf-8') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')


if __name__ == '__main__':
    main()
