"""Remote ensemble driver. Builds book-clean designs at several K_V, optionally
adds pitch arcs, and runs N seeds each in parallel isolated workdirs."""
import argparse, contextlib, copy, io, json, math, os, shutil, subprocess, sys
from pathlib import Path
ROOT=Path(os.path.expanduser("~/cw-biap-claude")); os.chdir(ROOT); sys.path.insert(0,str(ROOT))
import main as M
PY_=str(ROOT/".venv"/"bin"/"python")
AERO=str(ROOT/"openfoam"/"results"/"averages.csv")
WORK=ROOT/"work"; WORK.mkdir(exist_ok=True)
BASE=copy.deepcopy(M.STAGES)
C={k:getattr(M,k) for k in ("ETA","EPSILON","A_OMEGA_3","N_TAIL","K_V","D_K_BAR","ALPHA_C","BETA_C","L_FULL")}
PKG=dict(ALPHA_C=0.004,A_OMEGA_3=0.015,N_TAIL=0.008,BETA_C=math.radians(20),D_K_BAR=0.30,L_FULL=12000)

def build(**o):
    for k,v in C.items(): setattr(M,k,v)
    for k,v in o.items(): setattr(M,k,v)
    M.STAGES=copy.deepcopy(BASE); M.PROPS=[M.stage_props(s,i) for i,s in enumerate(M.STAGES,1)]
    with contextlib.redirect_stdout(io.StringIO()):
        th=[M.calc_thrust(i) for i in (1,2,3)]; w=[M.calc_weights(i) for i in (1,2,3)]
        pav=(((th[0].P_ud_0+th[0].P_ud_v)/2)+th[1].P_ud_v+th[2].P_ud_v)/3
        mu=M.emit_trajectory(th,pav); kk=[w[i].a_dv+(M.A_OMEGA_3 if i==2 else 0.0) for i in range(3)]
        sub=M.emit_masses(w,kk,mu,th)
    dV=sum(9.80665*th[i].P_ud_v*math.log(1/(1-mu)) for i in range(3))
    return M.traj_stage_fields(th,sub), sub, dV

def window(cfg,i):
    t=0.0
    for j,s in enumerate(cfg["stages"]):
        if j==i: return t,t+s["burn_time"]
        t+=s["burn_time"]

def split_longest(cfg,si):
    lo,hi=window(cfg,si); arcs=cfg["stages"][si]["pitch"]
    edges=[lo]+[a["t_end"] for a in arcs if "t_end" in a]+[hi]
    _,k=max((edges[j+1]-edges[j],j) for j in range(len(arcs)))
    prev=arcs[k-1]["theta_deg"] if k>0 else 90.0
    arcs.insert(k,{"t_end":round((edges[k]+edges[k+1])/2,6),
                   "theta_deg":round((prev+arcs[k]["theta_deg"])/2,4),
                   "shape":arcs[k].get("shape","cos"),"k":arcs[k].get("k",1.2)})
    return cfg

def make_cfg(warm,kv,extra_arcs):
    f,sub,dV=build(**dict(PKG,K_V=kv)); c=copy.deepcopy(warm)
    old=[s["burn_time"] for s in warm["stages"]]
    for st,w in zip(c["stages"],f): st.update(w)
    ob=[0.0]; nb=[0.0]
    for a,b in zip(old,[x["burn_time"] for x in f]): ob.append(ob[-1]+a); nb.append(nb[-1]+b)
    for i,st in enumerate(c["stages"]):
        for arc in st.get("pitch",[]):
            if "t_end" in arc:
                fr=(arc["t_end"]-ob[i])/(ob[i+1]-ob[i]); arc["t_end"]=round(nb[i]+fr*(nb[i+1]-nb[i]),6)
    for si in extra_arcs: c=split_longest(c,si)
    return c,sub,dV

ap=argparse.ArgumentParser()
ap.add_argument("--warm",required=True); ap.add_argument("--spec",required=True)
ap.add_argument("--maxiter",type=int,default=1500); ap.add_argument("--sigma0",type=float,default=1.0)
ap.add_argument("--tag",default="run")
a=ap.parse_args()
warm=json.loads(Path(a.warm).read_text())
# spec: "kv:arcs:seed,seed,...;kv:arcs:seeds;..."   arcs = extra arcs like "0" or "01" or ""
jobs=[]
for part in a.spec.split(";"):
    kv,arcs,seeds=part.split(":")
    for s in seeds.split(","):
        jobs.append((float(kv),tuple(int(x) for x in arcs),int(s)))
procs=[]
for kv,arcs,seed in jobs:
    cfg,sub,dV=make_cfg(warm,kv,arcs)
    name=f"{a.tag}_kv{int(kv*1000)}_a{len(sum([s['pitch'] for s in cfg['stages']],[]))}_s{seed}"
    d=WORK/name
    if d.exists(): shutil.rmtree(d)
    shutil.copytree(ROOT/"traj",d,ignore=shutil.ignore_patterns("out","outcmaes","__pycache__","*.csv"))
    (d/"out").mkdir(exist_ok=True); shutil.copy(ROOT/"traj"/"out"/"traj-sim", d/"out"/"traj-sim")
    (d/"rocket.json").write_text(json.dumps(cfg,indent=2,ensure_ascii=False)+"\n")
    log=open(WORK/f"{name}.log","w")
    procs.append((name,kv,sub.m0[0],dV,len(sum([s['pitch'] for s in cfg['stages']],[])),
        subprocess.Popen([PY_,"optimize.py","--aero",AERO,"--target","14000","--maxiter",str(a.maxiter),
                          "--seed",str(seed),"--sigma0",str(a.sigma0)],cwd=d,stdout=log,stderr=log),log,d))
print(f"launched {len(procs)} runs, maxiter={a.maxiter}",flush=True)
for *_,p,log,_ in procs: p.wait(); log.close()
res=[]
for name,kv,m0,dV,na,p,log,d in procs:
    txt=(WORK/f"{name}.log").read_text()
    rng=next((l for l in txt.splitlines() if "impact range" in l),"")
    bo=next((l for l in txt.splitlines() if "burnout (" in l),"")
    try: v=float(rng.split(":")[1].split("km")[0])
    except Exception: v=0.0
    res.append((kv,m0,na,name.split("_s")[-1],v,bo.strip(),dV,d))
res.sort(key=lambda r:(-r[0],-r[4]))
print("\n"+"="*92)
for kv,m0,na,seed,v,bo,dV,d in res:
    print(f"K_V={kv:.3f} m0={m0:6.0f} arcs={na} seed={seed:>4}: {v:9.1f} km  {'PASS' if v>=12000 else ''}  {bo.split(':',1)[-1].strip()}")
print("\n--- best per (K_V, arcs) ---")
best={}
for kv,m0,na,seed,v,bo,dV,d in res:
    if v>best.get((kv,na),(0,))[0]: best[(kv,na)]=(v,m0,d)
for (kv,na),(v,m0,d) in sorted(best.items(),key=lambda x:-x[0][0]):
    print(f"K_V={kv:.3f} arcs={na} m0={m0:6.0f}: {v:9.1f} km  {'PASS' if v>=12000 else 'short %.0f'%(12000-v)}")
    shutil.copy(d/"out"/"best.json", WORK/f"BEST_{a.tag}_kv{int(kv*1000)}_a{na}.json")
