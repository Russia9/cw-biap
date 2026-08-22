import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt, numpy as np, pandas as pd
from pathlib import Path
S = Path("/private/tmp/claude-501/-Users-russia9-study-8sem-cw-biap/5471bc84-79c9-4fcb-8799-bf0f41c32a3f/scratchpad")
RZ=6371000.0
BLUE,ORANGE = "#2a78d6","#eb6834"
INK,SEC,MUTED,GRID,SURF = "#0b0b0b","#52514e","#898781","#e1e0d9","#fcfcfb"
plt.rcParams.update({"figure.facecolor":SURF,"axes.facecolor":SURF,"savefig.facecolor":SURF,
 "font.family":"DejaVu Sans","font.size":10,"axes.edgecolor":"#c3c2b7","axes.labelcolor":SEC,
 "axes.titlecolor":INK,"axes.labelsize":10,"xtick.color":MUTED,"ytick.color":MUTED,
 "xtick.labelsize":9,"ytick.labelsize":9,"grid.color":GRID,"grid.linewidth":0.7,
 "legend.frameon":False,"legend.fontsize":9,"axes.spines.top":False,"axes.spines.right":False})
df=pd.read_csv(S/"winner_traj.csv",sep=";")
df["dr"]=RZ*np.arctan2(df["x"],RZ+df["y"])/1000.0; df["Hkm"]=df["H"]/1000.0
aut=df[df["stage"]<=3]; put=df[df["stage"]==4]; BO=aut.iloc[-1]
apo=df.loc[df["Hkm"].idxmax()]; imp=df.iloc[-1]

fig,ax=plt.subplots(figsize=(11.5,5.8))
ax.fill_between(df["dr"],0,df["Hkm"],color=BLUE,alpha=0.09,lw=0,zorder=1)
ax.plot(put["dr"],put["Hkm"],color=BLUE,lw=2.2,zorder=3,label="ballistic leg (ПУТ)")
ax.plot(aut["dr"],aut["Hkm"],color=ORANGE,lw=3.0,zorder=4,label="powered leg (АУТ)")
for x,y,lbl,ha,dx,dy in [
  (BO["dr"],BO["Hkm"],f"burnout\n{BO['Hkm']:.0f} km · {BO['V']:.0f} m/s · θ {BO['theta']:.1f}°","left",16,-4),
  (apo["dr"],apo["Hkm"],f"apogee {apo['Hkm']:.0f} km","center",0,14),
  (imp["dr"],0,f"impact\n{imp['dr']:.0f} km","right",-12,20)]:
    ax.plot([x],[y],"o",ms=7,color=INK,mec=SURF,mew=1.8,zorder=6)
    ax.annotate(lbl,xy=(x,y),xytext=(dx,dy),textcoords="offset points",ha=ha,va="bottom",
                color=SEC,fontsize=9.5,zorder=8,linespacing=1.35)
ax.set_xlabel("downrange, km"); ax.set_ylabel("altitude, km")
ax.set_title("Trajectory  —  m₀ = 29 724 kg,  range 12 418 km",loc="left",pad=14,fontweight="bold")
ax.grid(True,zorder=0); ax.set_axisbelow(True); ax.set_xlim(-200,13100); ax.set_ylim(0,2180)
ax.legend(loc="lower center",bbox_to_anchor=(0.52,0.03),ncol=2)

ins=ax.inset_axes([0.055,0.56,0.285,0.375], zorder=10)
ins.set_facecolor(SURF)
ins.patch.set_alpha(1.0)
ins.fill_between(aut["dr"],0,aut["Hkm"],color=ORANGE,alpha=0.10,lw=0)
ins.plot(aut["dr"],aut["Hkm"],color=ORANGE,lw=2.4)
seps=[66.3,109.2]
for t in seps:
    r=aut.iloc[(aut["t"]-t).abs().argmin()]
    ins.plot([r["dr"]],[r["Hkm"]],"o",ms=5,color=INK,mec=SURF,mew=1.4,zorder=5)
    ins.annotate(f"{int(round(t))} s",xy=(r["dr"],r["Hkm"]),xytext=(4,-11),
                 textcoords="offset points",fontsize=8,color=SEC)
ins.set_xlim(0,300); ins.set_ylim(0,200)
ins.set_title("powered leg (detail)",fontsize=9.5,color=SEC,loc="left",pad=5)
ins.tick_params(labelsize=8); ins.grid(True); ins.set_axisbelow(True)
ins.set_xlabel("km",fontsize=8,labelpad=1); ins.set_ylabel("km",fontsize=8,labelpad=1)
for k,sp in ins.spines.items():
    sp.set_visible(True); sp.set_edgecolor("#c3c2b7"); sp.set_linewidth(0.9)
fig.savefig(S/"trajectory.png",dpi=170,bbox_inches="tight"); print("ok")
