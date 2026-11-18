import json, sys
from pathlib import Path
import cv2, numpy as np
E="/Users/woonkil/projects/dofbot-rgbd-manipulation/evidence/G06/pa_261118-090521/"; D="/Users/woonkil/projects/dofbot-rgbd-manipulation/data/g04/pa/"; S="pa_261118-090521_"
L=lambda p: json.load(open(p))
# M1: mac_timing m_detect_done - m_start
lat=[]
for i in range(1,21):
    m=dict(l.split() for l in open(E+S+"snap%02d.mac_timing.txt"%i).read().strip().splitlines())
    lat.append(float(m["m_detect_done"])-float(m["m_start"]))
print("M1 n=%d min=%.4f max=%.4f (<=3.0: %s)"%(len(lat),min(lat),max(lat),max(lat)<=3.0))
# M1': stamp_minus_req, recv_minus_stamp over color+depth, 20 meta
smr=[];rms=[]; ids=[]
for i in range(1,21):
    mt=L(E+S+"snap%02d.meta.json"%i)
    for k in ("color","depth"): smr.append(mt[k]["stamp_minus_req"]); rms.append(mt[k]["recv_minus_stamp"])
    ids.append(mt["request_id"]==open(E+S+"snap%02d.req_id.txt"%i).read().strip() and mt["ok"] is True and mt["stale_dropped"]=={"color":0,"depth":0})
print("M1' rows=%d min stamp-req=%.5f (>=0 fresh) recv-stamp range [%.5f, %.5f]"%(len(smr),min(smr),min(rms),max(rms)))
print("M1'' request_id==req_id.txt & ok & stale_dropped 0 for all 20:",all(ids))
# M2b bias
det=lambda n: L(E+S+n+".detect.json")
sn=[det("snap%02d"%i) for i in range(1,21)]; bs=[det("base_"+t) for t in ("a1","a2","b1","b2")]
mean=lambda xs: sum(xs)/len(xs)
for nm,arr in (("P-A 20",sn),("기준 4",bs)):
    print(nm,"cx %.4f cy %.4f side %.4f yaw %.4f"%(mean([a['center'][0] for a in arr]),mean([a['center'][1] for a in arr]),mean([a['side_px'] for a in arr]),mean([a['yaw_deg'] for a in arr])))
dc=((mean([a['center'][0] for a in sn])-mean([a['center'][0] for a in bs]))**2+(mean([a['center'][1] for a in sn])-mean([a['center'][1] for a in bs]))**2)**.5
print("bias center %.4f side %.4f yaw %.4f"%(dc,abs(mean([a['side_px'] for a in sn])-mean([a['side_px'] for a in bs])),abs(mean([a['yaw_deg'] for a in sn])-mean([a['yaw_deg'] for a in bs]))))
print("center y 순서: A1 %.2f A2 %.2f start1 %.2f start2 %.2f B1 %.2f B2 %.2f"%(bs[0]['center'][1],bs[1]['center'][1],sn[0]['center'][1],sn[10]['center'][1],bs[2]['center'][1],bs[3]['center'][1]))
# M3 images
def st(n):
    im=cv2.imread(D+S+n+".color.png"); dp=np.load(D+S+n+".depth_mm.npy"); return float(im.mean()), float((dp>0).sum()/dp.size)
bb=[st("base_"+t) for t in ("a1","a2","b1","b2")]; ss=[st("snap%02d"%i) for i in range(1,21)]
lo,hi=min(b[0] for b in bb)-2.0,max(b[0] for b in bb)+2.0; vl,vh=min(b[1] for b in bb)-0.02,max(b[1] for b in bb)+0.02
u1=mean([s[0] for s in ss[:10]]); u2=mean([s[0] for s in ss[10:]])
print("M3 bright range [%.3f,%.3f] unit means %.3f %.3f ok=%s; valid range [%.4f,%.4f] snaps [%.4f,%.4f] ok=%s"%(lo,hi,u1,u2,lo<=u1<=hi and lo<=u2<=hi,vl,vh,min(s[1] for s in ss),max(s[1] for s in ss),all(vl<=s[1]<=vh for s in ss)))
# M4
for nm in ("probe_start","probe_end"):
    p=L(E+S+nm+".json"); print("M4",nm,"color %.3f depth %.3f robot_commands %s joint_read %s"%(p["rate"]["color"]["arrival_hz"],p["rate"]["depth"]["arrival_hz"],p["robot_commands"],p["joint_read"]))
# auto-exit timing
print("autoexit: launch epoch(log name) 1794961789 -> SIGINT 1794961835.48 = %.1f s; started_epoch 1794961805.218 -> +%.1f s"%(1794961835.48-1794961789,1794961835.48-1794961805.218))
