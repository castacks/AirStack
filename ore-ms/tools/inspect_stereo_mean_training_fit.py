"""Post-fit training-objective check; never selects or changes a checkpoint."""
from copy import deepcopy

import numpy as np
import torch

from stereo_mean_adaptation import OUT, build, freeze_check, load_mean_head, load_pair, read, setup, write


def main():
    freeze_check(); setup()
    dataset = read(OUT/'dataset.json')
    base = build(); adapted = deepcopy(base); load_mean_head(adapted)
    rows = []
    with torch.inference_mode():
        for record in dataset['records']:
            if record['role'] != 'train':
                continue
            for row in record['rows']:
                a, b, target, valid = load_pair(row)
                original, _ = base.inference(a, b); mean, _ = adapted.inference(a, b)
                values = [t[0, 0][valid].cpu().numpy() for t in [original, mean]]
                y = target[valid].cpu().numpy()
                metrics = {}
                for name, value in zip(['base', 'adapted'], values):
                    e = np.abs(value-y)
                    metrics[name] = float(np.where(e < 1., .5*e**2, e-.5).mean())
                dest = OUT/'training-fit'/f'{record["condition"]}-{row["source_id"]}.npz'
                dest.parent.mkdir(exist_ok=True)
                np.savez_compressed(dest, target_at_valid=y, base_at_valid=values[0], adapted_at_valid=values[1])
                rows.append({'condition': record['condition'], 'source_id': row['source_id'], 'valid_pixels': int(valid.sum()),
                             'smooth_l1_px': metrics})
    write(OUT/'training-fit-check.json', {'status': 'PASS_MEASUREMENT_ONLY', 'posthoc_after_evaluation': True,
          'checkpoint_selection_or_updates': False, 'optimizer_steps_added': 0, 'rows': rows,
          'equal_frame_mean_loss': {m: float(np.mean([r['smooth_l1_px'][m] for r in rows])) for m in ['base', 'adapted']}})
    print(read(OUT/'training-fit-check.json')['equal_frame_mean_loss'])


if __name__ == '__main__':
    main()
