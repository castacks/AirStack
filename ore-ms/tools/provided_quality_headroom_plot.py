"""Display frozen EXP130 scores, with independent metric axes."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from provided_quality_headroom import OUT,read
scores=read(OUT/'scores.json');groups=scores['groups'];cases=list(groups)
fig,axes=plt.subplots(1,2,figsize=(11,4.3),layout='constrained')
for ax,metric,label in zip(axes,['translation_m','rotation_deg'],['Translation discrepancy (m)','Rotation discrepancy (degrees)']):
 x=np.arange(len(cases));width=.24
 for offset,key,title,color in [(-1,'rgb_mean','Fixed RGB','#3477ae'),(0,'thermal_mean','Fixed thermal','#d47725'),(1,'oracle_mean','Retrospective oracle','#39815e')]:
  ax.bar(x+offset*width,[groups[c][metric][key] for c in cases],width,label=title,color=color)
 ax.set_xticks(x,['salon\nearly (95)','salon\nlate (38)','salon3\nmiddle (95)','salon3\nlate (95)']);ax.set_ylabel(label);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
axes[0].legend(frameon=False,fontsize=8)
fig.suptitle('EXP130: matched native intervals against recorded SLAM\nNominal body/clock conventions; physical ground truth unverified',fontsize=11)
for suffix in ['png','svg']:
 path=OUT/('quality-headroom.'+suffix);assert not path.exists();fig.savefig(path,dpi=180)
