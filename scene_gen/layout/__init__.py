"""layout stage — see GENERATION.md, "The three stages".

Both subdividers need the same two config readers, and they were duplicated
verbatim in each. They live here so there is one copy.
"""

def _rng_range(v, fallback):
    """Accept ``[lo, hi]`` or a scalar for a range-valued knob."""
    if isinstance(v, (list, tuple)) and len(v) >= 2:
        return float(v[0]), float(v[1])
    if isinstance(v, (int, float)):
        return float(v), float(v)
    return fallback


def _weighted(weights: dict, rng, fallback: str):
    """Draw a key from a ``{name: weight}`` mapping."""
    items = [(k, float(v)) for k, v in (weights or {}).items() if float(v) > 0.0]
    total = sum(w for _k, w in items)
    if total <= 0.0:
        return fallback
    r = rng.random() * total
    for k, w in items:
        r -= w
        if r <= 0.0:
            return k
    return items[-1][0]
