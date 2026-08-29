import omni.client
for p in ("omniverse://airlab-nucleus.andrew.cmu.edu:443/NVIDIA/Environments/2024_1/DomeLights/Clear/mealie_road.hdr",
          "omniverse://airlab-nucleus.andrew.cmu.edu:443/NVIDIA/Environments/2024_1/DomeLights/Clear/"):
    r = omni.client.stat(p)
    print(r[0], p)
