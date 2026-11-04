"""EXP123 fixed-trace store occupancy; standalone publication artifacts."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from retention_capacity_study import ROOT, OUT, read, name

fig, ax = plt.subplots(figsize=(9, 4.3), constrained_layout=True)
for cap, label in [(None, 'Unlimited'), (2097152, '2 MiB'), (4194304, '4 MiB')]:
    rows = read(OUT / 'cases/buffered_probe' / name(cap) / 'retention.json')['arrivals']
    ax.plot([i / 2 for i, r in enumerate(rows)],
            [r['resident_bytes'] / 1048576 for r in rows], label=label, lw=1.2)
ax.axvline(51, color='black', ls=':', lw=1)
ax.text(52, 1.0, 'Recorded RGB50 query at group51\n2 MiB: miss; 4 MiB: hit', fontsize=9)
ax.set(xlabel='Source group (within-group arrival order retained)', ylabel='Store-owned encoded payload (MiB)',
       title='Fixed EXP122 buffered trace: byte-capacity storage replay', ylim=(0, 4.5))
ax.legend(loc='lower right'); ax.grid(alpha=.2)
fig.text(.5, -.03, 'Excludes staged input, payload leases, IPC, decoding, GPU and total process memory.',
         ha='center', fontsize=8)
for suffix in ['png', 'pdf']:
    path = ROOT / 'docs/research/figures' / ('retention-capacity-2026-11-03.' + suffix)
    assert not path.exists(); fig.savefig(path, dpi=180, bbox_inches='tight')
