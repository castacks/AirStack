"""Lossy PCA for communication only; never mutate local mapper evidence.

Fit a source-specific, centered 100-D basis once from the first available full
language-feature map (all semantic classes/background, no query filtering).
Keep that basis fixed for the trial. A full/resync carries basis and mean;
subsequent coefficient deltas reuse the acknowledged decoder state.
"""
import hashlib
import numpy as np


def wire_fingerprint(basis, mean):
    from raven_nav.native_map import basis_fingerprint
    return hashlib.sha256((basis_fingerprint(basis)).encode() +
                          np.asarray(mean,dtype='<f4').tobytes()).hexdigest()


def project_wire(server, coefficients, basis, mean):
    import torch
    coefficients=np.asarray(coefficients,dtype=np.float32)
    basis=np.asarray(basis,dtype=np.float32);mean=np.asarray(mean,dtype=np.float32)
    if coefficients.ndim!=2 or basis.ndim!=2 or coefficients.shape[1]!=basis.shape[1] or mean.shape!=(basis.shape[0],):
        raise ValueError('incompatible wire PCA dimensions')
    if not len(coefficients):return np.empty((0,basis.shape[0]),dtype=np.float32)
    with torch.inference_mode():
        device=server.mapper.device
        b=torch.from_numpy(basis.copy()).to(device);m=torch.from_numpy(mean.copy()).to(device)
        pieces=[]
        for start in range(0,len(coefficients),4096):
            c=torch.from_numpy(coefficients[start:start+4096].copy()).to(device)
            pieces.append((c@b.T+m).float().cpu().numpy())
    return np.concatenate(pieces)


class WirePCA:
    def __init__(self, dimension=100, max_fit_voxels=20000, seed=17):
        self.dimension,self.max_fit_voxels,self.seed=dimension,max_fit_voxels,seed
        self.basis=self.mean=None
        self.fit_voxels=0

    def encode(self, features, device):
        import torch
        features=np.asarray(features,dtype=np.float32)
        if features.ndim!=2 or features.shape[1]<self.dimension or not np.isfinite(features).all():
            raise ValueError('wire PCA needs finite full feature vectors')
        if len(features)<self.dimension and self.basis is None:
            return None  # wait for enough observations; do not change rank
        with torch.inference_mode():
            if self.basis is None:
                indices=np.linspace(0,len(features)-1,min(len(features),self.max_fit_voxels),dtype=np.int64)
                sample=torch.from_numpy(features[indices].copy()).to(device)
                mean=sample.mean(dim=0)
                dev = torch.device(device)
                devices=[dev.index if dev.index is not None else torch.cuda.current_device()] if dev.type=='cuda' else []
                # Preserve the mapper's RNG state and all feature tensors.
                with torch.random.fork_rng(devices=devices):
                    torch.manual_seed(self.seed)
                    _,_,basis=torch.pca_lowrank(sample-mean,q=self.dimension,center=False,niter=3)
                self.basis=basis.float().cpu().numpy();self.mean=mean.float().cpu().numpy()
                self.fit_voxels=len(indices)
            if features.shape[1]!=self.basis.shape[0]:
                raise ValueError('wire feature dimension changed during a trial')
            b=torch.from_numpy(self.basis.copy()).to(device);m=torch.from_numpy(self.mean.copy()).to(device)
            pieces=[]
            for start in range(0,len(features),4096):
                f=torch.from_numpy(features[start:start+4096].copy()).to(device)
                pieces.append(((f-m)@b).float().cpu().numpy())
        return np.concatenate(pieces) if pieces else np.empty((0,self.dimension),dtype=np.float32)
