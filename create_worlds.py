from pathlib import Path
import re
P=Path(__file__).resolve().parent
def repaired(s,component):
 s=s.replace('TexturedBackground {}','Background { skyColor [ 0.16 0.19 0.24 ] }').replace('TexturedBackgroundLight {}','DirectionalLight { direction -0.3 -0.5 -1 intensity 1 }')
 s=s.replace('Physics { mass','Physics { density -1 mass')
 s=s.replace('size 0.18 0.18 0.1','size 0.18 0.14 0.04').replace('size 0.14 0.14 0.1','size 0.14 0.11 0.03')
 s=re.sub(r'endPoint Solid \{ translation (0 [-.0-9]+ 0) children',r'endPoint Solid { translation \1 rotation 1 0 0 -1.57079632679 children',s)
 s=s.replace('0.0632 name','0.024 name').replace('0.0604 name','0.017 name')
 s=re.sub(r'Viewpoint \{[^}]+\}', 'Viewpoint { orientation 0 1 0 1.57079632679 position '+('3 3 10' if component=='A' else '1.5 1.5 6')+' }',s)
 return s+'\nRobot { name "observer" supervisor TRUE controller "test_observer" customData "'+component+'" }\n'
header='#VRML_SIM R2023b utf8\nWorldInfo { basicTimeStep 32 coordinateSystem "ENU" }\nViewpoint { orientation -0.25 0.92 0.28 1.2 position 8 -8 9 }\nTexturedBackground {}\nTexturedBackgroundLight {}\n'
def box(name,x,y,z,sx,sy,sz,color):
 return f'''Solid {{ translation {x} {y} {z} name "{name}" children [ Shape {{ appearance PBRAppearance {{ baseColor {color} roughness 1 }} geometry Box {{ size {sx} {sy} {sz} }} }} ] boundingObject Box {{ size {sx} {sy} {sz} }} }}\n'''
def robot(name,x,y,controller,mode):
 half=.08 if mode=='A' else .065; radius=.033 if mode=='A' else .026; body=.18 if mode=='A' else .14; h=.10
 s=f'''Robot {{ translation {x} {y} {h/2+radius*.4} name "{name}" controller "{controller}" children [
 Shape {{ appearance PBRAppearance {{ baseColor 0.10 0.26 0.65 roughness 1 }} geometry Box {{ size {body} {body} {h} }} }}
 GPS {{ name "gps" }}
 InertialUnit {{ name "imu" }}
'''
 if mode=='A':
  s+=''' Lidar { name "lidar" translation 0 0 0.09 horizontalResolution 180 numberOfLayers 1 fieldOfView 6.28318 minRange 0.04 maxRange 3.5 }
 Camera { name "camera" translation 0.09 0 0.07 width 320 height 240 fieldOfView 1.1 }
'''
 else:
  s+=''' Emitter { name "emitter" channel 1 range 1.0 }
 Receiver { name "receiver" channel 1 }
'''
  import math
  for i in range(8):
   angle=2*math.pi*i/8
   # sensor local forward +X via rotation around Z
   s+=f' DistanceSensor {{ name "ps{i}" translation {0.065*math.cos(angle):.4f} {0.065*math.sin(angle):.4f} 0 rotation 0 0 1 {angle:.5f} type "infra-red" lookupTable [ 0 1000 0 0.08 600 0 0.25 0 0 ] }}\n'
 for side,sy in [('left',half),('right',-half)]:
  s+=f''' HingeJoint {{ jointParameters HingeJointParameters {{ axis 0 1 0 anchor 0 {sy} 0 }} device [ RotationalMotor {{ name "{side} wheel motor" maxVelocity 7 }} PositionSensor {{ name "{side} wheel sensor" }} ] endPoint Solid {{ translation 0 {sy} 0 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.08 0.08 0.08 roughness 1 }} geometry Cylinder {{ radius {radius} height 0.025 }} }} ] name "{name}_{side}_wheel" boundingObject Cylinder {{ radius {radius} height 0.025 }} physics Physics {{ mass 0.04 }} }} }}\n'''
 s+=f''' ] boundingObject Box {{ size {body} {body} {h} }} physics Physics {{ mass 0.45 }} }}\n'''
 return s
A=header+'Floor { size 6 6 tileSize 0.5 }\n'
# 6 x 6 centred on origin => coordinate arena shifted by 3,3 globally for obstacles/robots.
A=A.replace('Floor { size 6 6 tileSize 0.5 }','Solid { translation 3 3 -0.06 children [ Shape { appearance PBRAppearance { baseColor 0.7 0.74 0.74 roughness 1 } geometry Box { size 6 6 0.1 } } ] boundingObject Box { size 6 6 0.1 } }')
for i,(x,y,sx,sy) in enumerate([(3,-.025,6,.05),(3,6.025,6,.05),(-.025,3,.05,6),(6.025,3,.05,6),(1.425,2.425,.35,2.55),(2.925,3.6,.35,2.8),(4.275,2.075,.35,2.75)]):
 A+=box('wall'+str(i),x,y,.25,sx,sy,.5,'0.43 0.44 0.49')
for i,(x,y) in enumerate([(1.1,4.5),(3.65,1.35),(5.1,4.6)]):A+=box('red_landmark'+str(i),x,y,.15,.2,.2,.3,'0.95 0.03 0.04')
A+=robot('navigator',.55,.55,'a_nav_controller','A')
(P/'worlds/component_A_fixed.wbt').write_text(repaired(A,'A'))
B=header+'Solid { translation 1.5 1.5 -0.06 children [ Shape { appearance PBRAppearance { baseColor 0.77 0.77 0.74 roughness 1 } geometry Box { size 3 3 0.1 } } ] boundingObject Box { size 3 3 0.1 } }\n'
for i,(x,y,sx,sy) in enumerate([(1.5,-.025,3,.05),(1.5,3.025,3,.05),(-.025,1.5,.05,3),(3.025,1.5,.05,3),(1.0,1.25,.25,.5),(2.1,1.95,.35,.35)]):B+=box('swarm_wall'+str(i),x,y,.22,sx,sy,.44,'0.47 0.49 0.53')
for i,(x,y) in enumerate([(.4,.4),(2.55,.4),(.4,2.55),(2.55,2.55)],1):B+=robot('epuck'+str(i),x,y,'b_swarm_controller','B')
(P/'worlds/component_B_fixed.wbt').write_text(repaired(B,'B'))
from style_worlds import apply_style
apply_style(P/'worlds')
print('Recreated repaired worlds with original starting positions and parquet floor.')
