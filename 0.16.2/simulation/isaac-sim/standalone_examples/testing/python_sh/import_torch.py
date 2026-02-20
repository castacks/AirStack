# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import torch

print(torch.__path__[0])
assert "omni.isaac.ml_archive" in torch.__path__[0]
print(f"Cuda available: {torch.cuda.is_available()}")
assert torch.cuda.is_available()


@torch.jit.script
def add(a, b):
    return a + b


a = torch.ones((10, 2), device="cuda:0")
b = torch.ones((10, 2), device="cuda:0")
c = add(a, b)
d = a + b
assert torch.allclose(c, d)
