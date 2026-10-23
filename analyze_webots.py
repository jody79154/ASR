"""Summarize only CSVs measured by Webots; never use offline metrics as outcomes."""
from pathlib import Path
import csv,json,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parent
out=root/'results/webots';out.mkdir(exist_ok=True)
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
def number(r,k):return float(r[k])
summary={'source':'Actual Webots CSV logs only','date':'2026-10-23','A':{},'B':{}}
fig,ax=plt.subplots(1,2,figsize=(12,5))
a=rows(root/'results/tables/A_webots_log.csv')
if a:
    pts=np.array([[number(r,'x'),number(r,'y')] for r in a]);err=np.linalg.norm(pts-[5.45,5.45],axis=1)
    reached=np.flatnonzero(err<.16)
    summary['A']={'samples':len(a),'duration_s':number(a[-1],'t'),'final_position':pts[-1].tolist(),'final_goal_distance_m':float(err[-1]),'goal_reached':bool(len(reached)),'first_goal_time_s':number(a[reached[0]],'t') if len(reached) else None,'travel_distance_m':float(np.linalg.norm(np.diff(pts,axis=0),axis=1).sum()),'replans':int(a[-1]['replans']),'landmark_detection_rows':sum(int(r['landmarks'])>0 for r in a)}
    ax[0].plot(*pts.T,label='Measured GPS trajectory');ax[0].scatter([5.45],[5.45],marker='*',s=100,label='Goal')
    planfile=root/'results/webots/A_plans.json'
    if planfile.exists():
        plans=json.loads(planfile.read_text());errors=[]
        for record,point in zip(a,pts):
            available=[q for q in plans if q['time']<=number(record,'t') and q['points']]
            if not available:continue
            line=np.asarray(available[-1]['points']);u=line[:-1];d=np.diff(line,axis=0)
            if len(d):
                fraction=np.clip(np.sum((point-u)*d,axis=1)/np.maximum(np.sum(d*d,axis=1),1e-12),0,1)
                errors.append(float(np.min(np.linalg.norm(point-(u+fraction[:,None]*d),axis=1))))
        summary['A']['rms_distance_to_active_plan_m']=float(np.sqrt(np.mean(np.square(errors)))) if errors else None
        for i,q in enumerate(plans):
            line=np.asarray(q['points'])
            if len(line):ax[0].plot(*line.T,'--',alpha=.25,color='orange',label='Saved replan paths' if i==0 else None)
    for x,y,w,h in [(1.25,1.15,.35,2.55),(2.75,2.2,.35,2.8),(4.1,.7,.35,2.75)]:ax[0].add_patch(plt.Rectangle((x,y),w,h,color='gray'))
    ax[0].set(xlim=(0,6),ylim=(0,6),title='A: actual Webots navigation',xlabel='x (m)',ylabel='y (m)');ax[0].legend(fontsize=8)
estimates=[]
for p in sorted((root/'results/tables').glob('B_webots_*.csv')):
    data=rows(p)
    if not data:continue
    pts=np.array([[number(r,'x'),number(r,'y')] for r in data]);name=p.stem.removeprefix('B_webots_')
    estimates.append(number(data[-1],'estimate'))
    summary['B'][name]={'samples':len(data),'duration_s':number(data[-1],'time'),'travel_distance_m':float(np.linalg.norm(np.diff(pts,axis=0),axis=1).sum()),'final_position':pts[-1].tolist(),'final_estimate':number(data[-1],'estimate'),'received_packets':sum(int(r['neighbours']) for r in data),'steps_with_neighbours':sum(int(r['neighbours'])>0 for r in data),'final_goal_distance_m':number(data[-1],'goal_distance'),'model_loaded':data[-1]['model_loaded']}
    if 'safe_goal_distance' in data[-1]:
        summary['B'][name]['safe_goal_distance_m']=number(data[-1],'safe_goal_distance')
        summary['B'][name]['safe_goal_reached']=number(data[-1],'safe_goal_distance')<=.08
    ax[1].plot(*pts.T,label=name)
if estimates:summary['B_final_consensus_spread']=max(estimates)-min(estimates)
ax[1].set(xlim=(0,3),ylim=(0,3),title='B: actual Webots trajectories',xlabel='x (m)',ylabel='y (m)')
if estimates:ax[1].legend()
for component in ('A','B'):
    p=root/f'results/webots/{component}_physics.csv'
    if p.exists():
        data=rows(p)
        summary[f'{component}_physics']={'observed_until_s':number(data[-1],'time') if data else 0,'sample_rows_with_contact_above_floor':sum(int(r['contacts_above_floor'])>0 for r in data),'minimum_upright_dot':min((number(r,'roll_proxy') for r in data),default=None),'note':'Contacts sampled each 0.032 s physics step; count excludes contacts at or below z=0.012 m. This is a defined diagnostic, not a safety certification.'}
for a in ax:a.set_aspect('equal');a.grid(alpha=.2)
fig.tight_layout();fig.savefig(out/'actual_webots_trajectories.png',dpi=180)
(out/'measured_results.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
