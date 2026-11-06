#!/usr/bin/env python3
"""AirStack adapter around upstream RayFronts, without modifying its submodule."""
import os
import sys
# Match `python -m rayfronts.mapping_server`: launch cwd is /opt/rayfronts.
sys.path.insert(0, os.getcwd())
from rayfronts.mapping_server import MappingServer, main

# Both matched modes use float32 cosine/softmax, matching the CPU fused-map
# query. Feature extraction/alignment remains the original mixed-precision path.
if os.getenv('RAYFRONTS_QUERY_COMPRESSED', 'True').lower() == 'false':
    import rayfronts.mapping.semantic_ray_frontiers_map as mapper_module
    from rayfronts.utils import compute_cos_sim
    def float_query(q, features, softmax=False):
        return compute_cos_sim(q.float(), features.float(), softmax=softmax)
    mapper_module.compute_cos_sim = float_query

if os.getenv('RAVEN_REUSABLE_FEATURE_MAP', 'false').lower() == 'true':
    from raven_nav.feature_map import export_feature_snapshot
    original = MappingServer.run_queries

    def run_queries(self):
        query = self.mapper.feature_query
        reference = {}
        def capture(*args, **kwargs):
            result = query(*args, **kwargs)
            if result is not None and 'vox_sim' in result:
                reference['scores'] = result['vox_sim'][:, :64].T.float().cpu().numpy()
            return result
        with self._query_lock:
            self.mapper.feature_query = capture
            try:
                original(self)
                export_feature_snapshot(self, reference)
            finally:
                self.mapper.feature_query = query
    MappingServer.run_queries = run_queries

if __name__ == '__main__':
    import hydra
    import rayfronts.mapping_server as server_module
    # Upstream's relative Hydra config path assumes execution with -m. Once
    # imported by an adapter it becomes a config-module lookup, but that folder
    # is not a Python package. Preserve the exact configs via an absolute path.
    config_dir = os.path.join(os.path.dirname(server_module.__file__), 'configs')
    hydra.main(version_base=None, config_path=config_dir, config_name='default')(main.__wrapped__)()
