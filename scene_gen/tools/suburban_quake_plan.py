"""Draw the shared suburban layout and earthquake decisions without Kit."""
import argparse
import collections
import json
import os
import sys
sys.path.insert(0, os.path.dirname(__file__))
import fence_png
from disaster import quake_suburban as qs
import scene_generator as sg


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--config",default="suburb_earthquake_250")
    p.add_argument("--out",default="scene_gen/_plans/suburban_earthquake_250")
    a=p.parse_args()
    scene=fence_png.build(config_name=a.config,house_instances=[])
    cfg=scene["cfg"]
    region=(-125,-125,125,125)
    field=sg.make_damage_field(cfg["disaster"]["field"],region)
    houses=qs.plan_houses(scene["house_instances"],cfg,field,cfg["seed"])
    os.makedirs(os.path.dirname(a.out),exist_ok=True)
    with open(a.out+".json","w") as f:
        json.dump(dict(region=region,houses=houses),f,indent=2)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Ellipse
    fig,ax=plt.subplots(figsize=(10,10))
    colors=dict(zip(qs.MODES,["#91aa87","#e8c55b","#8376b7","#e68c45","#ca5257","#772f36"]))
    for h in houses:
        points=[qs._world(x*h["W"]/2,y*h["D"]/2,h) for x,y in [(-1,-1),(1,-1),(1,1),(-1,1)]]
        ax.add_patch(Polygon(points,facecolor=colors[h["mode"]],edgecolor="black",linewidth=.4))
        ax.text(h["x"],h["y"],h["id"][-3:],ha="center",fontsize=6)
    for edge in scene["net"].edges.values():
        pts = edge.pts
        ax.plot([p[0] for p in pts],[p[1] for p in pts],color="#888888",linewidth=2,zorder=0)
    soil=cfg["disaster"].get("soft_soil")
    if soil:
        ax.add_patch(Ellipse(soil["center"],soil["rx_m"]*2,soil["ry_m"]*2,
                            angle=soil["angle_deg"],fill=False,ls="--",ec="#6a5b37"))
    tally=collections.Counter(h["mode"] for h in houses)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color=colors[m],label=f"{m}: {tally[m]}") for m in qs.MODES],loc="upper left",fontsize=8)
    ax.set(xlim=(-125,125),ylim=(-125,125),aspect="equal",xlabel="metres",ylabel="metres",
           title="250 × 250 m suburban earthquake — synthetic damage sample\nDashed: susceptible soil; house IDs match the live plan")
    fig.savefig(a.out+".png",dpi=160)
    print(dict(houses=len(houses),damage=tally,out=a.out))


if __name__=="__main__":
    main()
