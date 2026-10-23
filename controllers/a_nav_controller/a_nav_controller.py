"""Component A1–A4 live Webots loop; Webots installation needed to validate devices/geometry."""
import sys,csv,math,json
from pathlib import Path
import numpy as np
from controller import Robot
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from src.core import Grid,PID,wrap,rc,xy,inflate,search,detect_red,fuse,wheels
robot=Robot();dt=int(robot.getBasicTimeStep());seconds=dt/1000
left=robot.getDevice('left wheel motor');right=robot.getDevice('right wheel motor')
for m in [left,right]:m.setPosition(float('inf'));m.setVelocity(0)
gps=robot.getDevice('gps');gps.enable(dt);imu=robot.getDevice('imu');imu.enable(dt)
lidar=robot.getDevice('lidar');lidar.enable(dt);camera=robot.getDevice('camera');camera.enable(dt)
angles=lidar.getFov()/2-(np.arange(lidar.getHorizontalResolution())+.5)*lidar.getFov()/lidar.getHorizontalResolution()
grid=Grid();pid=PID(5,.05,.22,seconds,2)
path=[];step=0;lastplan=0
evidence=ROOT/'results/webots';evidence.mkdir(parents=True,exist_ok=True)
plans=[]
out=ROOT/'results/tables/A_webots_log.csv';out.parent.mkdir(parents=True,exist_ok=True)
f=open(out,'w',newline='');writer=csv.writer(f);writer.writerow(['t','x','y','heading','target_x','target_y','v','w','landmarks','replans'])
while robot.step(dt)!=-1:
    pos=gps.getValues();pose=(pos[0],pos[1],imu.getRollPitchYaw()[2]);ranges=np.array(lidar.getRangeImage(),dtype=float)
    if step%5==0:
        # Device bearing order must be verified using a single box at robot's left.
        grid.scan(pose,ranges,angles)
        raw=np.frombuffer(camera.getImage(),dtype=np.uint8)
        if raw.size==camera.getWidth()*camera.getHeight()*4:
            bgr=raw.reshape(camera.getHeight(),camera.getWidth(),4)[:,:,:3].copy()
            detections,_=detect_red(bgr);landmarks=fuse(grid,pose,detections,ranges,angles,camera.getWidth(),camera.getFov())
        else:landmarks=[]
        occupancy=inflate(grid.occupied(),.25)
        here=rc(pose[:2]);goal=rc((5.45,5.45));occupancy[here]=False;occupancy[goal]=False
        if not path or any(occupancy[r,c] for r,c in path[:min(12,len(path))]):
            newpath,_,_=search(occupancy,here,goal,'astar')
            path=newpath;lastplan+=1
            plans.append({'time':robot.getTime(),'points':[xy(p) for p in path]})
        if step%250==0:
            camera.saveImage(str(evidence/f'A_camera_{step:05d}.png'),100)
            np.savetxt(evidence/f'A_lidar_{step:05d}.csv',np.column_stack((angles,ranges)),delimiter=',',header='bearing_rad,range_m')
    if path:
        # Delete waypoints already reached
        while len(path)>1 and math.dist(pose[:2],xy(path[0]))<.09:path.pop(0)
        target=xy(path[0]);err=wrap(math.atan2(target[1]-pose[1],target[0]-pose[0])-pose[2]);w=pid.step(err);v=.10 if abs(err)<.35 else 0.
        if math.dist(pose[:2],(5.45,5.45))<.16:v=w=0
    else:target=pose[:2];v=w=0
    lv,rv=wheels(v,w)
    left.setVelocity(lv);right.setVelocity(rv)
    writer.writerow([robot.getTime(),*pose,*target,v,w,len(landmarks) if step%5==0 else 0,lastplan]);step+=1
    if step%100==0:f.flush()
    if robot.getTime()>=180:break
left.setVelocity(0);right.setVelocity(0)
f.close()
(evidence/'A_plans.json').write_text(json.dumps(plans))
np.savetxt(evidence/'A_map_logodds.csv',grid.logodds,delimiter=',')
