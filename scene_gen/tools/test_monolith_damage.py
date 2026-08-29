#!/usr/bin/env python3
"""Pure-Python offline tests for generic monolith damage."""

import random
from disaster import monolith_damage as md


def main():
    tested = 0
    for seed in range(16):
        rng = random.Random(seed)
        for recipe in md.RECIPES:
            d = md.Descriptor(rng.uniform(9, 60), rng.uniform(10, 70),
                              rng.uniform(12, 115), storey_m=rng.uniform(3.0, 4.2),
                              construction=rng.choice(("urm", "rc", "rc_glass")))
            issues = md.validate(d, recipe, seed)
            assert not issues, (recipe, seed, issues)
            rubble = md.rubble_specs(d, recipe, seed)
            assert 55 <= len(rubble) <= 360
            assert all(q["kind"] in ("chunk", "slab", "beam") for q in rubble)
            assert md.stepped_profile(d, recipe) == md.stepped_profile(d, recipe)
            tested += 1
    print("[monolith_damage] offline checks ok: {} cases".format(tested))


if __name__ == "__main__":
    main()
