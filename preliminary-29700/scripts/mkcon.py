import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt, pandas as pd, numpy as np
from pathlib import Path
S=Path("/private/tmp/claude-501/-Users-russia9-study-8sem-cw-biap/5471bc84-79c9-4fcb-8799-bf0f41c32a3f/scratchpad")
BLUE,ORANGE,CRIT="#2a78d6","#eb6834","#d03b3b"
INK,SEC,MUTED,GRID,SURF="#0b0b0b","#52514e","#898781","#e1e0d9","#fcfcfb"
GOOD="#0ca30c"
plt.rcParams.update({"figure.facecolor":SURF,"axes.facecolor":SURF,"savefig.facecolor":SURF,
 "font.family":"DejaVu Sans","font.size":10,"axes.edgecolor":"#c3c2b7","axes.labelcolor":SEC,
 "axes.titlecolor":INK,"axes.labelsize":10,"xtick.color":MUTED,"ytick.color":MUTED,
 "xtick.labelsize":9,"ytick.labelsize":9,"grid.color":GRID,"grid.linewidth":0.7,
 "legend.frameon":False,"axes.spines.top":False,"axes.spines.right":False})
df=pd.read_csv(S/"winner_traj.csv",sep=";")
aut=df[(df["stage"]<=3)&(df["t"]>0.05)]
TK=aut["t"].max(); SEPS=[66.3,109.2]
sub=aut[(aut["Mach"]<1.1)&(aut["q"]>1.0)]          # strict <, matches the Go bucketing
mid=aut[(aut["Mach"]>=1.1)&(aut["H"]<=94000)&(aut["q"]>1.0)]
fig,axs=plt.subplots(2,2,figsize=(11.5,7.4),gridspec_kw={"hspace":0.46,"wspace":0.26})
def panel(ax,x,y,lim,ylab,title,ach,xmax,note=None):
    ax.axhline(lim,color=CRIT,lw=1.3,ls=(0,(5,3)),zorder=4)
    ax.text(xmax*0.985,lim,f" limit {lim:g}",ha="right",va="bottom",color=CRIT,fontsize=8.5)
    ax.plot(x,y,color=BLUE,lw=2.0,zorder=3)
    ax.fill_between(x,0,y,color=BLUE,alpha=0.08,lw=0,zorder=2)
    ax.set_ylabel(ylab); ax.set_xlabel("time, s")
    ax.set_title(title,loc="left",pad=20,fontsize=11.5,fontweight="bold")
    ax.text(0.0,1.055,f"{ach}",transform=ax.transAxes,ha="left",va="bottom",
            fontsize=9.5,color=GOOD,fontweight="bold")
    for t in SEPS:
        if t<=xmax: ax.axvline(t,color=MUTED,lw=0.8,ls=(0,(4,3)),zorder=1)
    ax.set_xlim(0,xmax); ax.set_ylim(0,lim*1.3); ax.grid(True,axis="y",zorder=0); ax.set_axisbelow(True)
    if note: ax.text(xmax*0.5,lim*1.30*0.955,note,ha="center",va="top",fontsize=8.5,color=MUTED)
panel(axs[0][0],sub["t"],sub["alpha"].abs(),1.5,"|α|, deg",
      "ε₁ — angle of attack, M ≤ 1.1","1.50 / 1.50 deg   ✓",23.2,"regime ends at t = 23.2 s")
panel(axs[0][1],mid["t"],mid["alpha"].abs(),10.0,"|α|, deg",
      "ε₂ — angle of attack, M > 1.1, H ≤ 94 km","9.99 / 10.00 deg   ✓",102.5,
      "regime ends at t = 102.5 s (94 km)")
panel(axs[1][0],aut["t"],aut["omega"].abs(),3.0,"|ϑ̇|, deg/s",
      "ϑ̇ — programmed pitch rate","3.00 / 3.00 deg/s   ✓",TK)
panel(axs[1][1],aut["t"],aut["q"]/1000,120.0,"q, kPa",
      "q — dynamic pressure","76.2 / 120 kPa   ✓",TK,"peak at t = 37.0 s")
fig.suptitle("§4.4 constructive-ballistic limits — all four satisfied",x=0.09,y=1.0,
             ha="left",fontsize=13.5,fontweight="bold",color=INK)
fig.savefig(S/"constraints.png",dpi=170,bbox_inches="tight"); print("ok")
