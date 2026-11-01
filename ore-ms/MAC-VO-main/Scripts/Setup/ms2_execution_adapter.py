"""Explicit MS2 source frames behind the unchanged camera-local v3 interface."""
from contextlib import contextmanager
from copy import deepcopy
import hashlib
import json
from pathlib import Path

from DataLoader.SequenceBase import SequenceBase
from DataLoader.Dataset.MS2 import MS2ManifestSequence, THERMAL_RECIPE


def paired_rows(source):
    """Copy official ID/time associations; never pair by nearest time or support."""
    rows = []
    for index, record in enumerate(source['records']):
        row = {'source_stem': record['stem']}
        for camera, native in (('rgb', 'rgb'), ('thermal', 'thr')):
            for side in ('left', 'right'):
                value = record['cameras'][native][side]
                row[camera + '_' + side] = {
                    'index': index, 'source_stem': record['stem'],
                    'time_ns': value['recorded_time_ns'], 'path': value['path']}
        rows.append(row)
    return rows


def prepare(root, source_manifest, output):
    root, source_manifest = Path(root).resolve(), Path(source_manifest).resolve()
    if not source_manifest.is_relative_to(root):
        raise ValueError('source manifest must be inside the MS2 path root')
    payload = source_manifest.read_bytes()
    source = json.loads(payload)
    # Native validation checks schema, calibration, IDs and all four clocks.
    MS2ManifestSequence(root, source_manifest, 'rgb')
    contract = {'schema_version': 1, 'dataset': 'ms2',
        'source_manifest_relative': str(source_manifest.relative_to(root)),
        'source_manifest_sha256': hashlib.sha256(payload).hexdigest(),
        'source_sequence': source['sequence'], 'thermal_recipe': THERMAL_RECIPE,
        'pairs': paired_rows(source), 'scope': 'provided source pairing; camera-local only',
        'physical_frame_or_clock_verified': False, 'reference_labels_used': False}
    with Path(output).open('x') as stream:
        json.dump(contract, stream, indent=2); stream.write('\n')
    return contract


def execution_contract(dataset, manifest):
    """Dataset identity for the otherwise unchanged v3 output report."""
    path = Path(manifest)
    contract = json.loads(path.read_text())
    return {'dataset': 'ms2', 'paired_manifest_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'source_manifest_sha256': contract['source_manifest_sha256'],
        'source_sequence': contract['source_sequence'], 'thermal_recipe': THERMAL_RECIPE,
        'torch_threads': 1, 'opencv_threads': 1,
        'recorded_time_semantics': 'unverified-source-recorded-time; exact left timestamps retained',
        'baseline_semantics': 'provided rectified positive left-disparity model',
        'scope': 'camera-local runtime/preservation only; no pose/depth reference read',
        'physical_motion_admitted': False, 'checkpoint_training_recipe_verified': False}


class MS2ExecutionSequence(SequenceBase):
    """Wrapper leaves native rows, pixels, frame types and thermal recipe intact."""
    def __init__(self, dataset, manifest, modality, *, acknowledge_provisional_geometry=False):
        if acknowledge_provisional_geometry is not True or modality not in ('rgb', 'thermal'):
            raise ValueError('explicit camera-local acknowledgement and camera required')
        root = Path(dataset).resolve()
        contract = json.loads(Path(manifest).read_text())
        source_path = (root / contract['source_manifest_relative']).resolve()
        if not source_path.is_relative_to(root):
            raise ValueError('source manifest outside MS2 root')
        payload = source_path.read_bytes()
        if hashlib.sha256(payload).hexdigest() != contract['source_manifest_sha256']:
            raise ValueError('source manifest hash changed')
        source = json.loads(payload)
        if (contract['schema_version'] != 1 or contract['dataset'] != 'ms2' or
                contract['thermal_recipe'] != THERMAL_RECIPE or
                contract['source_sequence'] != source['sequence'] or
                contract['pairs'] != paired_rows(source)):
            raise ValueError('paired manifest differs from exact native source contract')
        native = 'rgb' if modality == 'rgb' else 'thr'
        self.source = MS2ManifestSequence(root, source_path, native,
            thermal_transform=THERMAL_RECIPE if native == 'thr' else None)
        if self.source.manifest != source:
            raise ValueError('source manifest changed during initialization')
        self.rows = deepcopy(contract['pairs'])
        self.K, self.baseline = self.source.K, self.source.baseline
        # Provided rectified model uses positive left-disparity baseline.
        self.signed_baseline = self.baseline
        self.height, self.width = self.source.height, self.source.width
        super().__init__(len(self.rows))

    def __getitem__(self, local_index):
        # Match native clipping/local-frame-index semantics without changing rows.
        self.source.indices = self.indices
        return self.source[local_index]

    @classmethod
    def is_valid_config(cls, config):
        raise ValueError('use the explicit bounded MS2 execution entrypoint')


@contextmanager
def adapted_runner():
    """Process-local opt-in; restore the original runner even on failure."""
    import cv2
    import torch
    from Scripts.Setup import run_td3_execution_v3 as runner
    original = runner.TD3ManifestSequence
    cv_threads, torch_threads = cv2.getNumThreads(), torch.get_num_threads()
    runner.TD3ManifestSequence = MS2ExecutionSequence
    cv2.setNumThreads(1)
    torch.set_num_threads(1)
    try:
        yield runner
    finally:
        runner.TD3ManifestSequence = original
        cv2.setNumThreads(cv_threads)
        torch.set_num_threads(torch_threads)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('root', 'source_manifest', 'output'):
        parser.add_argument('--' + name.replace('_', '-'), type=Path, required=True)
    args = parser.parse_args()
    prepare(args.root, args.source_manifest, args.output)
