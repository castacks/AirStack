# M00 configuration comparison

This compares the development environment to EXT E4/D5; it does not approve equivalence.

| Item | Supplied controlled reference | M00 observation | Disposition |
|---|---|---|---|
| AirStack | a6dad8caf54722e5eba3481367e914ce213e6135 plus frozen R4 | ore_proj 169c83a58bbb387e9a9086ef49ca2dd5887a7eb7 before M00 | Different source; no reproduction claim |
| Simulator image | E4 qualifying image ID sha256:6bdb5e2f0fe077500c89fda5c1f447a92362bbd3cc924916a82ca0f1187aa246; sealed manifest erratum/A3 approval pending | Running image ID sha256:dfe91c2ed7680c5bfc7ccfc1f4bc424fe96fd65790353cf07595f12baae849cf | Different image ID; no substitution |
| Robot image | sha256:48f82de86ece4d455a6ee22fe9dcbec0bb358edfd67cd26f056cc413edd9f7af | sha256:3b98f4333f33dbbe734363f19f9ea65f52f4af8a98f4ac3e78f724124af9c290 | Different image ID |
| PX4 source / binary | v1.16.1 94cb2012792b2ae89f0b147cfee53ee31ae550be; binary 413033b866e6b1f4ccee01542085687a200df18f3a0badc874d0bcb7b7979248 | Not attested | UNK; blocks equivalence |
| Pegasus / vehicle | 8c7a664095396bc9c4f4609531445d4cf6390fe4; airframe10015; Iris USD09f8390b… | Runtime not attested | UNK |
| ROS / middleware / MAVROS | Jazzy / FastDDS / MAVROS2.16.0, domain1 robot_1 | Passive subscriptions received data in domain1; exact deployed versions not checked | Partial observation, no equivalence |
| Fence / parameters | Frozen 19m horizontal, 10m AGL converted to home-relative each session; successor BOUNDARY 18m/8.5m and margins separate | No current full readback acquired | UNK; never apply successor geometry to frozen record |
| Scene / physics / controller | Controlled world and frozen R4/R5C packages | No exact runtime binding or physics qualification | UNK |
| NumPy | Existing AirStack deployment fix1.26.4 | 1.26.4 observed | Compatibility fact only |

Image config IDs above are compared to the E4 image IDs; the source `.env` registry manifest digest is a different identifier type and is not compared directly with a config ID. No image was pulled, built or changed. Every future D5 item needs a justified class before authorization; source/identity differences currently preclude labeling this environment CFG-SIL-01 EQ.
