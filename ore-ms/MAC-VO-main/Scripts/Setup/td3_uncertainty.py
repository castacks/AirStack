"""Opt-in bounded research recording; no selection, covariance or pose corrections."""
import json
from pathlib import Path
import numpy as np
import torch

from Utility.Point import filterPointsInRange


def array(value):
    return value.detach().cpu().numpy().copy()


class UncertaintyRecorder:
    """Wrap a single synchronous MACVO instance; snapshot before mutable covariance flooring."""
    def __init__(self, system, output):
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.pairs = []
        self.current = None
        selector = system.KeypointSelector.select_point
        outlier = system.OutlierFilter.filter
        retrieve = system.Frontend.retrieve_pixels

        def select(frame, count, depth0, depth1, match):
            # Integer fixed grid, 16px pitch, same 32px border as configured selector.
            margin = system.edge_width
            u = torch.arange(margin, frame.width-margin, 16, device=match.flow.device)
            v = torch.arange(margin, frame.height-margin, 16, device=match.flow.device)
            vv, uu = torch.meshgrid(v, u, indexing='ij')
            grid = torch.stack([uu.flatten(), vv.flatten()], dim=1)
            record = {"time0_ns": np.array(frame.frame_ns, dtype=np.int64),
                      "K": array(frame.K), "height_width": np.array([frame.height, frame.width])}

            def sample(prefix, kp):
                end = kp + retrieve(kp, match.flow).T
                inbound = filterPointsInRange(end, (margin, frame.width-margin), (margin, frame.height-margin))
                safe = end.clone()
                # Invalid target samples are NaN in saved arrays; clamp only to permit gathering.
                safe[:, 0] = safe[:, 0].nan_to_num().clamp(0, frame.width-1)
                safe[:, 1] = safe[:, 1].nan_to_num().clamp(0, frame.height-1)
                d1 = array(retrieve(safe, depth1.depth).reshape(-1))
                d1[~array(inbound)] = np.nan
                record.update({prefix+"_uv0": array(kp), prefix+"_uv1": array(end),
                               prefix+"_depth0": array(retrieve(kp, depth0.depth).reshape(-1)),
                               prefix+"_depth1": d1, prefix+"_inbound": array(inbound),
                               prefix+"_raw_flow_cov": array(retrieve(kp, match.cov).T)})
            sample("grid", grid)
            kp = selector(frame, count, depth0, depth1, match)
            sample("selected", kp)
            self.current = record
            self.pairs.append(record)
            return kp

        def observe(values):
            index = sum(key.endswith('_depth_variance_before') for key in self.current)
            for key, value in values.items():
                if value is not None:
                    self.current[f"endpoint{index}_{key}"] = array(value)

        def filter_observations(observations, device):
            mask = outlier(observations, device)
            self.current['outlier_keep_inbound'] = array(mask)
            return mask

        system.KeypointSelector.select_point = select
        system.OutlierFilter.filter = filter_observations
        system.ObsCovModel.diagnostic_observer = observe

    def finish(self):
        for index, record in enumerate(self.pairs):
            np.savez_compressed(self.output / f"pair-{index:03d}.npz", **record)
        (self.output / "metadata.json").write_text(json.dumps({
            "pairs": len(self.pairs), "grid_pitch_px": 16,
            "raw_flow_cov_units": "px^2; temporal learned covariance before floors, channels uu,vv,uv",
            "endpoint0_flow": "fixed pixel quantization prior, not learned",
            "depth_variance": "local weighted depth variance; not learned stereo variance",
            "scope": "static-scene consistency diagnostic against recorded SLAM; not calibration",
        }, indent=2)+'\n')
