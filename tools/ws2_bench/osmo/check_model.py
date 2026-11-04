"""Real checkpoint inference check inside a task image; does not start a flight."""
import time
from pathlib import Path


def check(planner):
    import numpy as np
    started = time.monotonic()
    image = np.random.default_rng(42).integers(0, 256, (480, 640, 3), dtype=np.uint8)
    result = {'planner': planner, 'kind': 'synthetic_input_inference_check'}
    if planner == 'mononav':
        import sys
        sys.path.insert(0, str(Path.cwd() / 'ZoeDepth'))
        import torch
        import open3d as o3d
        from PIL import Image
        from zoedepth.models.builder import build_model
        from zoedepth.utils.config import get_config
        # ZoeDepth overrides .to() to update the device used by infer_pil.
        # .cuda() moves parameters but leaves that input-device property stale.
        model = build_model(get_config('zoedepth', 'infer')).to('cuda').eval()
        with torch.inference_mode():
            output = np.asarray(model.infer_pil(Image.fromarray(image), output_type='numpy'))
        # Exercise the TSDF CUDA kernels too, not just torch's GPU discovery.
        device = o3d.core.Device('CUDA:0')
        grid = o3d.t.geometry.VoxelBlockGrid(
            attr_names=('tsdf', 'weight', 'color'),
            attr_dtypes=(o3d.core.float32,) * 3, attr_channels=((1,), (1,), (3,)),
            voxel_size=.05, block_resolution=8, block_count=1000, device=device)
        depth = o3d.t.geometry.Image(o3d.core.Tensor(np.full((48, 64), 2000, np.uint16), device=device))
        color = o3d.t.geometry.Image(o3d.core.Tensor(image[::10, ::10].copy(), device=device))
        intrinsics = o3d.core.Tensor([[50., 0., 32.], [0., 50., 24.], [0., 0., 1.]], dtype=o3d.core.float64)
        pose = o3d.core.Tensor(np.eye(4), dtype=o3d.core.float64)
        blocks = grid.compute_unique_block_coordinates(depth, intrinsics, pose, 1000., 3.)
        grid.integrate(blocks, depth, color, intrinsics, pose, 1000., 3.)
        result.update(framework=torch.__version__, device=torch.cuda.get_device_name(),
                      tsdf_points=int(grid.extract_point_cloud().point.positions.shape[0]),
                      depth_mean=float(output.mean()))
    elif planner == 'kim':
        import tensorflow as tf
        from airstack_integration.runtime import CollisionAvoidanceRuntime
        runtime = CollisionAvoidanceRuntime('save_model/D3QN_V_3_single.h5',
            fcrn_checkpoint='airstack_models/NYU_FCRN-checkpoint/NYU_FCRN.ckpt')
        try:
            depth = runtime.estimate_depth(image)
            history = np.repeat(depth[None, ..., None], runtime.history_channels, axis=-1)
            action, output = runtime.predict(history)
            result.update(framework=tf.__version__, device=str(tf.config.list_physical_devices('GPU')),
                          depth_mean=float(depth.mean()), action=action, q_values=output.tolist())
            if tf.config.list_physical_devices('GPU'):
                trace = tf.compat.v1.RunMetadata()
                runtime.session.run(runtime.fcrn_output,
                    feed_dict={runtime.fcrn_input: np.zeros((1, 228, 304, 3), np.float32)},
                    options=tf.compat.v1.RunOptions(trace_level=tf.compat.v1.RunOptions.FULL_TRACE),
                    run_metadata=trace)
                gpu_nodes = [n.node_name for d in trace.step_stats.dev_stats if 'GPU' in d.device.upper()
                             for n in d.node_stats if 'conv' in n.node_name.lower()]
                if not gpu_nodes:
                    raise RuntimeError('FCRN convolution execution on GPU was not verified')
                result['gpu_convolution_nodes'] = len(gpu_nodes)
        finally:
            runtime.close()
    else:
        import torch
        from airstack_integration.model import dronet, Depthwise_Separable
        model = dronet(depth_mult=.125, block_class=Depthwise_Separable, bypass=False).eval()
        model.load_state_dict(torch.load('tiny-pulp-dronet-v3/model/tiny-pulp-dronet-v3-dw-pw-0.125.pth',
                                        map_location='cpu', weights_only=True), strict=True)
        with torch.inference_mode():
            values = model(torch.zeros((1, 1, 200, 200)))
        output = np.array([float(v.item()) for v in values])
        result.update(framework=torch.__version__, device='cpu', outputs=output.tolist())
    if not np.isfinite(output).all():
        raise RuntimeError('Non-finite model output')
    result.update(ok=True, wall_seconds=time.monotonic()-started)
    return result
