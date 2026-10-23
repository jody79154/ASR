"""Measurement only: never modifies robot pose or control commands."""
from controller import Supervisor
from pathlib import Path
import csv,json
robot=Supervisor();dt=int(robot.getBasicTimeStep())
root=Path(__file__).resolve().parents[2]
component=robot.getCustomData() or 'A'
out=root/'results'/'webots';out.mkdir(parents=True,exist_ok=True)
nodes=[]
children=robot.getRoot().getField('children')
for i in range(children.getCount()):
    n=children.getMFNode(i)
    if n.getTypeName()=='Robot' and n.getField('name').getSFString()!='observer':nodes.append(n)
f=(out/f'{component}_physics.csv').open('w',newline='');w=csv.writer(f)
w.writerow(['time','name','x','y','z','roll_proxy','contacts_above_floor'])
limit=180 if component=='A' else 120
step=0
while robot.step(dt)!=-1:
    t=robot.getTime()
    if True:
        for n in nodes:
            p=n.getPosition();o=n.getOrientation()
            contacts=n.getContactPoints(True)
            above=sum(c.point[2]>.012 for c in contacts)
            w.writerow([t,n.getField('name').getSFString(),*p,o[8],above])
        f.flush()
    if step==20:robot.exportImage(str(out/f'{component}_start.png'),100)
    if t>=limit+.064:
        f.close();robot.exportImage(str(out/f'{component}_end.png'),100)
        print(f'ACTUAL WEBOTS TEST {component} finished at {t:.3f} seconds',flush=True)
        robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
        break
    step+=1
