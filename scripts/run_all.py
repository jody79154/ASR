"""Run A1–A4 and B1–B3 offline reference tests; charts use Matplotlib only."""
import sys,json,csv,time,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np,matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from sklearn.neural_network import MLPClassifier
from sklearn.metrics import classification_report,ConfusionMatrixDisplay,accuracy_score
from sklearn.model_selection import train_test_split
from joblib import dump
from src.core import *
F=ROOT/'results/figures';T=ROOT/'results/tables';F.mkdir(exist_ok=True,parents=True);T.mkdir(exist_ok=True,parents=True)
results={};world=make_map();blocked=inflate(world,.18);start=rc((.5,.5));goal=rc((5.45,5.45));

# A1 simulated camera observation (NOT actual Webots frame), LiDAR and late fusion.
import cv2
pose=(.55,.55,.7);angles=np.linspace(-np.pi,np.pi,180,endpoint=False)
scans=lidar_2d(pose,world,angles,seed=1);grid=Grid();grid.scan(pose,scans,angles)
frame=np.zeros((240,320,3),np.uint8);frame[:]=(90,90,90)
cv2.rectangle(frame,(135,92),(169,171),(0,0,245),-1)
dets,mask=detect_red(frame);marks=fuse(grid,pose,dets,scans,angles)
cv2.imwrite(str(ROOT/'data/camera_example_SYNTHETIC.png'),frame)
np.savetxt(ROOT/'data/lidar_example_SYNTHETIC.csv',np.column_stack([angles,scans]),delimiter=',',header='relative_angle_rad,range_m',comments='')
fig,axs=plt.subplots(1,3,figsize=(11,3.3));axs[0].imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));axs[0].set_title('Synthetic camera frame');axs[1].imshow(mask,cmap='gray');axs[1].set_title('Red HSV segmentation');axs[2].imshow(grid.logodds,origin='lower',extent=[0,6,0,6],cmap='coolwarm',vmin=-3,vmax=3);axs[2].scatter([pose[0]],[pose[1]],marker='*',s=100);axs[2].set_title('LiDAR + camera evidence');
for ax in axs:ax.set_xticks([]);ax.set_yticks([])
fig.tight_layout();fig.savefig(F/'A1_perception.png',dpi=140);plt.close(fig)
results['A1']={'synthetic_frame_detections':len(dets),'fused_landmarks':len(marks),'lidar_beams':len(scans)}

# A2 same 8-connected graph, 0 versus admissible Euclidean heuristic.
fig,axs=plt.subplots(1,2,figsize=(9,4.1));planning={}
for ax,alg in zip(axs,['dijkstra','astar']):
    t0=time.perf_counter();path,length,expanded=search(blocked,start,goal,alg);runtime=[]
    for _ in range(7):
        tick=time.perf_counter();search(blocked,start,goal,alg);runtime.append((time.perf_counter()-tick)*1e3)
    planning[alg]={'length_m':round(length,3),'nodes_expanded':len(expanded),'runtime_ms_median':round(float(np.median(runtime)),3)}
    ax.imshow(blocked,origin='lower',extent=(0,6,0,6),cmap='Greys',alpha=.65)
    if expanded:
        pt=np.array([xy(q) for q in expanded]);ax.scatter(pt[:,0],pt[:,1],s=2,alpha=.13)
    if path:
        coords=np.array([xy(q) for q in path]);ax.plot(coords[:,0],coords[:,1],lw=2)
    ax.scatter([.5,5.45],[.5,5.45],c=['green','red'],s=50);ax.set_title(f'{alg}: {len(expanded)} nodes');ax.set_aspect('equal')
fig.tight_layout();fig.savefig(F/'A2_planning.png',dpi=140);plt.close(fig);results['A2']=planning

# A3 heading step simulated with 0.35 s actuator lag and 0.15 rad/s constant bias.
def heading_response(gains):
    dt=.01;setpoint=np.pi/2;th=0.;w=0.;pid=PID(*gains,dt,2);trace=[]
    for k in range(800):
        err=wrap(setpoint-th);u=pid.step(err);w+=dt*(u-w)/.35;th+=(w+.15)*dt;trace.append(th)
    t=np.arange(800)*dt;a=np.array(trace); overshoot=max(0,float((np.max(a)-setpoint)/setpoint*100))
    cross=np.where(a>=.9*setpoint)[0];rise=float(t[cross[0]]) if len(cross) else None
    good=np.abs(a-setpoint)<.02*setpoint
    settles=next((float(t[i]) for i in range(len(t)) if good[i:].all()),None)
    return t,a,dict(overshoot_pct=round(overshoot,2),rise_s=rise,settle_s=settles,steady_state_error_deg=round(abs(np.rad2deg(setpoint-a[-1])),3))
pidsets={'P low':(1.5,0,0),'P high':(12,0,0),'PI':(4,2,0),'PID':(10,.6,2)};fig,ax=plt.subplots(figsize=(8,3.6));tuning={}
for name,gains in pidsets.items():
    t,a,metric=heading_response(gains);tuning[name]=dict(gains=gains,**metric);ax.plot(t,np.rad2deg(a),label=name)
ax.axhline(90,ls=':',c='black');ax.set(xlabel='Time (s)',ylabel='Heading (degrees)',title='A3: heading step and PID tuning');ax.legend(ncol=4,fontsize=8);fig.tight_layout();fig.savefig(F/'A3_pid.png',dpi=140);plt.close(fig);results['A3']=tuning

# A4 path following uses shared planner + PID and discretised differential-drive kinematics.
path,length,_=search(blocked,start,goal,'astar');targets=np.array([xy(p) for p in path]);curr=np.array([.55,.55,0.]);pid=PID(5.,0.05,.22,.05,2.);traj=[];errors=[];times=[];idx=0;replans=0
for k in range(2600):
    dist=np.linalg.norm(targets[:,:2]-curr[:2],axis=1)
    near=int(np.argmin(dist));idx=max(idx,near)
    idx=min(idx+3,len(targets)-1)
    target=targets[idx];e=wrap(np.arctan2(target[1]-curr[1],target[0]-curr[0])-curr[2]);w=pid.step(e)
    v=.15 if abs(e)<.65 else .065
    if np.linalg.norm(curr[:2]-targets[-1])<.13:break
    curr=np.asarray(step_pose(curr,v,w,.05));traj.append(curr.copy());times.append(k*.05)
    errors.append(float(np.min(np.linalg.norm(targets-curr[:2],axis=1))))
traj=np.asarray(traj);success=bool(np.linalg.norm(curr[:2]-targets[-1])<.13)
fig,ax=plt.subplots(figsize=(6,5));ax.imshow(blocked,origin='lower',extent=(0,6,0,6),cmap='Greys',alpha=.45);ax.plot(targets[:,0],targets[:,1],label='Planned');ax.plot(traj[:,0],traj[:,1],label='Driven');ax.legend();ax.set(xlabel='x (m)',ylabel='y (m)',title='A4 offline path tracking');fig.tight_layout();fig.savefig(F/'A4_trajectory.png',dpi=140);plt.close(fig)
results['A4']={'goal_reached':success,'initial_path_m':round(length,3),'distance_driven_m':round(float(np.linalg.norm(np.diff(traj[:,:2],axis=0),axis=1).sum()),3),'time_s':round(float(len(traj)*.05),2),'rms_tracking_error_m':round(float(np.sqrt(np.mean(np.square(errors))),),3),'final_goal_error_m':round(float(np.linalg.norm(curr[:2]-targets[-1])),3),'replans':replans,'note':'Offline known-map follower. Webots closed-loop replan logic is in controller; no Webots execution claimed.'}

# B1–B2 four peers with range-limited comms, connected ring for consensus benchmark.
A=np.zeros((4,4));
for i in range(4):A[i,(i-1)%4]=1;A[i,(i+1)%4]=1
hist,eps=consensus([.12,.7,.4,.9],A,50);fig,ax=plt.subplots(figsize=(7,3.2))
for i in range(4):ax.plot(hist[:,i],label=f'Robot {i+1}')
ax.axhline(np.mean(hist[0]),ls=':',c='black');ax.set(xlabel='Consensus round',ylabel='Coverage estimate',title='B2 distributed consensus');ax.legend(ncol=4,fontsize=8);fig.tight_layout();fig.savefig(F/'B2_consensus.png',dpi=140);plt.close(fig)
rng=np.random.default_rng(42);points=rng.uniform(.15,2.85,(240,2));comparison={};fig,ax=plt.subplots(figsize=(7,3.2))
for topo in ['global','ring']:
    positions,curve=pso(points,topo,seed=5);comparison[topo]={'final_cost':round(float(curve[-1]),5),'positions':positions.round(3).tolist()};ax.plot(curve,label=topo)
random_positions=rng.uniform(.22,2.78,(4,2));comparison['random_baseline_cost']=round(coverage_cost(random_positions,points),5)
ax.axhline(comparison['random_baseline_cost'],label='random baseline',ls=':');ax.set(xlabel='PSO iteration',ylabel='Mean squared nearest-agent distance',title='B2 PSO coverage convergence');ax.legend();fig.tight_layout();fig.savefig(F/'B2_pso.png',dpi=140);plt.close(fig)
results['B2']={'epsilon':eps,'final_consensus_spread':float(np.ptp(hist[-1])),'pso':comparison}

# B3 imitative ML policy, source=synthetic labelled expert; no false Webots training claim.
X,y=synthetic_dataset();Xtr,Xte,ytr,yte=train_test_split(X,y,test_size=.2,stratify=y,random_state=0)
net=MLPClassifier(hidden_layer_sizes=(16,16),max_iter=350,random_state=0,early_stopping=True).fit(Xtr,ytr)
pred=net.predict(Xte);acc=float(accuracy_score(yte,pred));report=classification_report(yte,pred,output_dict=True,zero_division=0)
dump(net,ROOT/'data/navigation_mlp.joblib')
fig,axs=plt.subplots(1,2,figsize=(9,3.6));axs[0].plot(net.loss_curve_);axs[0].set(xlabel='Epoch',ylabel='Loss',title='B3 training loss');ConfusionMatrixDisplay.from_predictions(yte,pred,display_labels=['forward','left','right'],ax=axs[1],cmap='Blues',colorbar=False);fig.tight_layout();fig.savefig(F/'B3_training.png',dpi=140);plt.close(fig)
# Offline mock coordination trade-off, measured from deterministic grid-visitation proxy rather than Webots.
def proxy_coverage(distributed,seed):
    rng=np.random.default_rng(seed);visited=set();positions=rng.integers(0,20,(4,2));log=[];messages=0
    for t in range(160):
        for i in range(4):
            visited.add(tuple(positions[i]));near=[(r,c) for r in range(max(0,positions[i,0]-1),min(20,positions[i,0]+2)) for c in range(max(0,positions[i,1]-1),min(20,positions[i,1]+2))]
            unexplored=[p for p in near if p not in visited]
            if unexplored:positions[i]=unexplored[rng.integers(len(unexplored))]
            else:positions[i]=np.clip(positions[i]+rng.integers(-1,2,2),0,19)
        if not distributed:
            # central proxy: deterministic reassignment away from visited cells every 8 steps
            if t%8==0:
                free=[(r,c) for r in range(20) for c in range(20) if (r,c) not in visited]
                for i in range(min(4,len(free))):positions[i]=free[(i*len(free)//4)%len(free)]
            messages+=4
        else:messages+=sum(np.linalg.norm(positions[i]-positions[j])<=5 for i in range(4) for j in range(i+1,4))
        log.append(len(visited)/400)
    return np.array(log),messages
fig,ax=plt.subplots(figsize=(7,3.2));central={}
for mode in ['centralised','distributed']:
    traces=[];msgs=[]
    for seed in range(8):
        tr,ms=proxy_coverage(mode=='distributed',seed);traces.append(tr);msgs.append(ms)
    arr=np.mean(traces,axis=0);central[mode]={'coverage_pct':round(100*float(arr[-1]),2),'messages_mean':float(np.mean(msgs))};ax.plot(arr*100,label=mode)
ax.set(xlabel='Offline proxy step',ylabel='Cells visited (%)',title='B3 coordination comparison (proxy only)');ax.legend();fig.tight_layout();fig.savefig(F/'B3_coordination.png',dpi=140);plt.close(fig)
results['B3']={'synthetic_test_accuracy':round(acc,4),'per_class_recall':{str(i):round(report[str(i)]['recall'],3) for i in range(3)},'synthetic_train_samples':len(Xtr),'synthetic_test_samples':len(Xte),'proxy_comparison':central,'limitation':'Synthetic offline model, not Webots sensor measurements; closed-loop ML success must be verified on Webots.'}
with open(T/'metrics.json','w') as f:json.dump(results,f,indent=2)
with open(T/'metrics.csv','w',newline='') as f:
    w=csv.writer(f);w.writerow(['section','metric','value'])
    for section,vals in results.items():
        for k,v in vals.items():w.writerow([section,k,str(v)])
print(json.dumps(results,indent=2));print('Saved offline evidence to',ROOT/'results')
