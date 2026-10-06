#!/usr/bin/env python3
"""Common TEX-CUP benchmark evaluation.

Usage: evaluate.py --est est.csv --out results_dir [--gt ground_truth.log]
                   [--name pipeline] [--cut HH:MM:SS] [--end HH:MM:SS]
est.csv columns (header required): utc_sec (seconds of day 2019-05-09, UTC) and either
  ecef_x,ecef_y,ecef_z (metres, ANTENNA position) OR lat_deg,lon_deg,h_ell.

Metrics protocol (current defaults):
- Window = [18:09:40, 19:16:59 UTC] by default, inclusive, sampled at 1 Hz GT epochs.
- A window epoch is SOLVED if the pipeline has an estimate within +/-0.5 s.
- <1.0 m and <1.5 m percentages use the FULL window as denominator:
  an unsolved epoch counts as NOT meeting the threshold.
- RMS/median/max are over solved epochs only (they are undefined for gaps);
  availability_pct tells you how much is missing.
"""
import argparse, csv, json, math, os, sys
from pathlib import Path
import numpy as np

def default_ground_truth():
    """Use the downloaded dataset root, without falling back to saved results."""
    configured = os.environ.get('TEXCUP_DATA')
    if configured:
        return Path(configured).expanduser()/'ground_truth.log'
    root = Path(__file__).resolve().parents[1]
    return root/'data/tex_cup/ground_truth.log'


GT_LOG=str(default_ground_truth())
A=6378137.0; E2=6.69437999014e-3

def lla2ecef(lat,lon,h):
    la,lo=np.radians(lat),np.radians(lon)
    N=A/np.sqrt(1-E2*np.sin(la)**2)
    return np.stack([(N+h)*np.cos(la)*np.cos(lo),(N+h)*np.cos(la)*np.sin(lo),(N*(1-E2)+h)*np.sin(la)],-1)

def ecef2enu(p,ref_lla):
    la,lo=math.radians(ref_lla[0]),math.radians(ref_lla[1])
    ref=lla2ecef(*ref_lla)
    R=np.array([[-math.sin(lo),math.cos(lo),0],
                [-math.sin(la)*math.cos(lo),-math.sin(la)*math.sin(lo),math.cos(la)],
                [math.cos(la)*math.cos(lo),math.cos(la)*math.sin(lo),math.sin(la)]])
    return (p-ref)@R.T

def load_gt(path=None):
    ts=[];lla=[]
    source = Path(default_ground_truth() if path is None else path).expanduser()
    if not source.is_file():
        raise FileNotFoundError(
            f'Ground truth not found: {source}. Download it from the TEX-CUP archive '
            'following data/README.md, then pass --gt /path/to/ground_truth.log '
            'or set TEXCUP_DATA to the prepared dataset directory.')
    # The publisher's comment header contains a Latin-1 unit character;
    # timestamps and numeric data are ASCII in both publisher and legacy files.
    with source.open(encoding='latin-1') as stream:
        for line in stream:
            if line.startswith('#') or not line.strip(): continue
            p=line.split()
            if len(p)<6 or '/' not in p[0]: continue
            hh,mm,ss=p[1].split(':')
            ts.append(int(hh)*3600+int(mm)*60+float(ss))
            lla.append((float(p[3]),float(p[4]),float(p[5])))
    return np.array(ts),np.array(lla)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--est',required=True); ap.add_argument('--out',required=True)
    ap.add_argument('--name',default='pipeline')
    ap.add_argument('--gt',default=GT_LOG,
                    help='Downloaded GT path; defaults to TEXCUP_DATA or repository data/tex_cup')
    ap.add_argument('--cut',default='18:09:40',help='window start, UTC HH:MM:SS')
    ap.add_argument('--end',default='19:16:59',help='inclusive window end, UTC HH:MM:SS (common camera endpoint)')
    a=ap.parse_args()
    hh,mm,ss=a.cut.split(':'); cut=int(hh)*3600+int(mm)*60+float(ss)
    eh,em,es=a.end.split(':'); end=int(eh)*3600+int(em)*60+float(es)
    try:
        gt_t,gt_lla=load_gt(a.gt)
    except FileNotFoundError as error:
        ap.error(str(error))
    gt_ecef=lla2ecef(gt_lla[:,0],gt_lla[:,1],gt_lla[:,2])
    rows=list(csv.DictReader(open(a.est)))
    if not rows: sys.exit('empty est csv')
    et=np.array([float(r['utc_sec']) for r in rows])
    if 'ecef_x' in rows[0]:
        ep=np.stack([[float(r['ecef_x']),float(r['ecef_y']),float(r['ecef_z'])] for r in rows])
    else:
        ep=lla2ecef(np.array([float(r['lat_deg']) for r in rows]),
                    np.array([float(r['lon_deg']) for r in rows]),
                    np.array([float(r['h_ell']) for r in rows]))
    o=np.argsort(et); et,ep=et[o],ep[o]
    # per-estimate errors (GT interpolated at est times)
    m=(et>=cut)&(et<=end)&(et>=gt_t[0])&(et<=gt_t[-1])
    et,ep=et[m],ep[m]
    gi=np.stack([np.interp(et,gt_t,gt_ecef[:,i]) for i in range(3)],-1)
    ref=gt_lla[0]
    d=ecef2enu(ep,ref)-ecef2enu(gi,ref)
    hor=np.hypot(d[:,0],d[:,1]); ver=np.abs(d[:,2])
    # 1 Hz window grid over GT epochs
    grid=gt_t[(gt_t>=cut)&(gt_t<=end)]
    if len(et):
        idx=np.clip(np.searchsorted(et,grid),0,len(et)-1)
        idx0=np.clip(idx-1,0,len(et)-1)
        pick=np.where(np.abs(et[idx]-grid)<=np.abs(et[idx0]-grid),idx,idx0)
        dt=np.abs(et[pick]-grid)
        solved=dt<=0.5
        ghor=np.where(solved,hor[pick],np.nan); gver=np.where(solved,ver[pick],np.nan)
    else:
        solved=np.zeros(len(grid),bool); ghor=np.full(len(grid),np.nan); gver=ghor
    nw=len(grid); ns=int(solved.sum())
    sh=ghor[solved]; sv=gver[solved]
    res=dict(name=a.name,cut_utc=a.cut,end_utc=a.end,
        n_window=nw,n_solved=ns,availability_pct=float(100*ns/nw) if nw else 0.0,
        hor_lt_1m_pct_of_window=float(100*np.sum(sh<1.0)/nw) if nw else 0.0,
        hor_lt_1p5m_pct_of_window=float(100*np.sum(sh<1.5)/nw) if nw else 0.0,
        hor_rms_solved=float(np.sqrt(np.mean(sh**2))) if ns else None,
        hor_median_solved=float(np.median(sh)) if ns else None,
        hor_max_solved=float(sh.max()) if ns else None,
        ver_rms_solved=float(np.sqrt(np.mean(sv**2))) if ns else None,
        hor_lt_1m_pct_of_solved=float(100*np.mean(sh<1.0)) if ns else None,
        hor_lt_1p5m_pct_of_solved=float(100*np.mean(sh<1.5)) if ns else None,
        t_first=float(et[0]) if len(et) else None,t_last=float(et[-1]) if len(et) else None)
    import pathlib; out=pathlib.Path(a.out); out.mkdir(parents=True,exist_ok=True)
    json.dump(res,open(out/'eval.json','w'),indent=1)
    np.savetxt(out/'hor_err_timeseries.csv',
               np.stack([grid,ghor,gver],-1),delimiter=',',
               header='utc_sec,hor_err_m(nan=no solution),ver_err_m',comments='')
    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        fig,ax=plt.subplots(figsize=(12,4))
        ax.plot((grid-18*3600)/60,ghor,lw=0.8)
        gap=(~solved)
        if gap.any(): ax.plot((grid[gap]-18*3600)/60,np.zeros(gap.sum()),'|',color='crimson',ms=8,label='no solution')
        ax.set_xlabel('minutes since 18:00 UTC'); ax.set_ylabel('horizontal error (m)')
        ax.set_title(f'{a.name}: avail {res["availability_pct"]:.1f}%, <1m {res["hor_lt_1m_pct_of_window"]:.1f}% / <1.5m {res["hor_lt_1p5m_pct_of_window"]:.1f}% of window, RMS(solved) {res["hor_rms_solved"]:.3f} m')
        if gap.any(): ax.legend()
        ax.grid(alpha=.3); plt.tight_layout(); plt.savefig(out/'error.png',dpi=110)
    except Exception as ex: print('plot skipped:',ex)
    print(json.dumps(res,indent=1))
if __name__=='__main__': main()
