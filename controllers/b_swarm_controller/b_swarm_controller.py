"""Component B1–B3: peer control, radio consensus, MLP action and safe fallback."""
import sys,math,csv,struct
from pathlib import Path
import numpy as np
from controller import Robot
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from src.core import consensus,expert,wrap,wheels,inflate,search,rc,xy
robot=Robot();dt=int(robot.getBasicTimeStep());name=robot.getName();idx=int(name[-1])-1
left=robot.getDevice('left wheel motor');right=robot.getDevice('right wheel motor')
for motor in [left,right]:motor.setPosition(float('inf'));motor.setVelocity(0)
gps=robot.getDevice('gps');gps.enable(dt);imu=robot.getDevice('imu');imu.enable(dt)
ps=[robot.getDevice('ps'+str(i)) for i in range(8)]
for s in ps:s.enable(dt)
emitter=robot.getDevice('emitter');receiver=robot.getDevice('receiver');receiver.enable(dt)
try:
 from joblib import load
 net=load(ROOT/'data/navigation_mlp_local.joblib')
except Exception as e:
 print('MLP unavailable, using expert fallback:',e);net=None
# Assign four PSO-computed coverage waypoints. Offline global-best result, not live PSO.
import json
try:
 coord=np.asarray(json.loads((ROOT/'results/tables/metrics.json').read_text())['B2']['pso']['ring']['positions'][idx])
except Exception:coord=np.array([[.6,.6],[2.4,.6],[.6,2.4],[2.4,2.4]][idx])
out=ROOT/f'results/tables/B_webots_{name}.csv';f=open(out,'w',newline='');log=csv.writer(f);log.writerow(['time','x','y','estimate','action','neighbours','goal_distance','left_speed','right_speed','max_proximity','model_loaded','safe_goal_distance','safe_goal_x','safe_goal_y'])
estimate=.2+idx*.18;step=0
# Known static arena geometry, not a learned map or distributed PSO.
# Each robot plans locally; radio remains range-limited and decentralized.
blocked=np.ones((60,60),dtype=bool);blocked[1:29,1:29]=False
blocked[10:16,8:12]=True;blocked[17:22,19:23]=True
blocked=inflate(blocked,.20);route=None
# PSO ignores obstacle clearance. Project its station to a free grid cell.
free=np.argwhere(~blocked)
terminal=min((tuple(p) for p in free),key=lambda p:math.dist(xy(p),coord))
safe_coord=xy(terminal)
print(name, 'requested station',coord,'safe station',safe_coord,flush=True)
while robot.step(dt)!=-1:
    x,y=gps.getValues()[:2];yaw=imu.getRollPitchYaw()[2]
    measured=np.clip(np.array([s.getValue() for s in ps])/1000.,0,1)
    # Adapt custom counterclockwise ring to the trained expert's clockwise layout.
    proximity=measured[[0,7,6,5,4,3,2,1]]
    emitter.send(struct.pack('iddd',idx,x,y,estimate))
    heard=[]
    while receiver.getQueueLength():
        data=receiver.getBytes()
        if len(data)==struct.calcsize('iddd'):
            j,nx,ny,val=struct.unpack('iddd',data)
            if j!=idx:heard.append(val)
        receiver.nextPacket()
    if heard:estimate+=.25*sum(h-estimate for h in heard) # epsilon < 1/3 for 3 peers
    action=int(net.predict(proximity[None,:])[0]) if net is not None else expert(proximity)
    # Safety override takes precedence over neural predictions
    if max(proximity[0],proximity[7])>.85:action=1 if proximity[6]<proximity[1] else 2
    if route is None:
        route,_,_=search(blocked,rc((x,y)),terminal,'astar')
        if not route:print(name,'NO SAFE PATH: stopping',flush=True)
    while route and math.dist((x,y),xy(route[0]))<.05:route.pop(0)
    target=xy(route[0]) if route else safe_coord
    bearing=wrap(math.atan2(target[1]-y,target[0]-x)-yaw)
    if not route or math.dist((x,y),safe_coord)<=.05:lv,rv=0.,0.
    elif max(proximity[0],proximity[7])>.55:
        if action==0:action=1 if max(proximity[5:7])<max(proximity[1:3]) else 2
        lv,rv=(-2.,2.) if action==1 else (2.,-2.)
    else:
        v=.065 if abs(bearing)<.3 else 0.
        w=float(np.clip(4*bearing,-1.4,1.4))
        lv,rv=wheels(v,w,r=.026,L=.13,max_speed=6.5)
    left.setVelocity(lv);right.setVelocity(rv)
    log.writerow([robot.getTime(),x,y,estimate,action,len(heard),math.dist((x,y),coord),lv,rv,float(proximity.max()),net is not None,math.dist((x,y),safe_coord),*safe_coord]);step+=1
    f.flush()
    if robot.getTime()>=120:break
left.setVelocity(0);right.setVelocity(0);f.close()
