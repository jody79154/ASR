"""ASR700 shared algorithms. Study guide units 2–6; assignment A1–B3."""
import heapq, math, time
import numpy as np
from scipy.ndimage import binary_dilation

RES=.10
RECTS=[(1.25,1.6,1.15,3.7),(2.75,3.10,2.2,5.0),(4.1,4.45,.7,3.45)]
LANDMARKS=[(1.1,4.5),(3.65,1.35),(5.1,4.6)]

def make_map(n=60):
    a=np.zeros((n,n),bool)
    for x0,x1,y0,y1 in RECTS:
        a[int(y0/RES):int(y1/RES)+1,int(x0/RES):int(x1/RES)+1]=True
    a[[0,-1],:]=True;a[:,[0,-1]]=True
    return a

def inflate(blocked,radius=.16):
    k=int(np.ceil(radius/RES)); yy,xx=np.mgrid[-k:k+1,-k:k+1]
    return binary_dilation(blocked,structure=(xx*xx+yy*yy<= (radius/RES)**2))

def rc(xy):return (int(np.clip(xy[1]/RES,0,59)),int(np.clip(xy[0]/RES,0,59)))
def xy(cell):return ((cell[1]+.5)*RES,(cell[0]+.5)*RES)

def search(blocked,start,goal,algorithm='astar',heuristic='euclidean'):
    moves=[(-1,0,1),(1,0,1),(0,-1,1),(0,1,1),(-1,-1,2**.5),(-1,1,2**.5),(1,-1,2**.5),(1,1,2**.5)]
    def h(p):
        dy=abs(p[0]-goal[0]);dx=abs(p[1]-goal[1]);return math.hypot(dx,dy) if heuristic=='euclidean' else max(dx,dy)+(2**.5-1)*min(dx,dy)
    pq=[(0.,0,start)]; count=0; costs={start:0.};prev={};visited=set();expanded=[]
    while pq:
        _,_,u=heapq.heappop(pq)
        if u in visited:continue
        visited.add(u);expanded.append(u)
        if u==goal:break
        for dr,dc,step in moves:
            v=(u[0]+dr,u[1]+dc)
            if not(0<=v[0]<blocked.shape[0] and 0<=v[1]<blocked.shape[1]) or blocked[v]:continue
            if dr and dc and (blocked[u[0],v[1]] or blocked[v[0],u[1]]):continue # prevent corner cutting
            ng=costs[u]+step
            if ng<costs.get(v,float('inf')):
                costs[v]=ng;prev[v]=u;count+=1
                heapq.heappush(pq,(ng+(h(v) if algorithm=='astar' else 0),count,v))
    if goal not in costs or goal not in visited:return [],float('inf'),expanded
    path=[goal]
    while path[-1]!=start:path.append(prev[path[-1]])
    return list(reversed(path)),costs[goal]*RES,expanded

class Grid:
    def __init__(self,n=60):self.logodds=np.zeros((n,n),dtype=float)
    def scan(self,pose,ranges,angles,max_range=3.5):
        x,y,theta=pose
        for d,a in zip(ranges,angles):
            hit=np.isfinite(d) and d<max_range-.01
            r=min(float(d),max_range) if np.isfinite(d) else max_range
            angle=theta+a
            for s in np.arange(0,max(0,r-RES),RES):
                ix,iy=x+s*np.cos(angle),y+s*np.sin(angle)
                if 0<=ix<6 and 0<=iy<6:
                    r0,c0=rc((ix,iy));self.logodds[r0,c0]-=.3
            if hit:
                ix,iy=x+r*np.cos(angle),y+r*np.sin(angle)
                if 0<=ix<6 and 0<=iy<6:
                    r0,c0=rc((ix,iy));self.logodds[r0,c0]+=.85
        np.clip(self.logodds,-5,5,out=self.logodds)
    def occupied(self):return self.logodds>.65

class PID:
    def __init__(self,kp,ki,kd,dt,limit=2.):self.kp,self.ki,self.kd,self.dt,self.limit=kp,ki,kd,dt,limit;self.i=0;self.old=None
    def step(self,error):
        d=0 if self.old is None else (error-self.old)/self.dt
        self.old=error; raw=self.kp*error+self.ki*self.i+self.kd*d
        out=float(np.clip(raw,-self.limit,self.limit))
        if abs(raw)<self.limit:self.i=np.clip(self.i+error*self.dt,-3,3)
        return out

def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def wheels(v,w,r=.033,L=.16,max_speed=6.67):return tuple(np.clip([(v-w*L/2)/r,(v+w*L/2)/r],-max_speed,max_speed))

def step_pose(p,v,w,dt):
    x,y,a=p;return (x+v*np.cos(a)*dt,y+v*np.sin(a)*dt,wrap(a+w*dt))

def lidar_2d(p,world,angles,max_range=3.5,seed=0):
    rng=np.random.default_rng(seed); results=[]
    for a in angles:
        t=p[2]+a;dx,dy=np.cos(t),np.sin(t);rngs=np.arange(.04,max_range+.001,.035)
        px=p[0]+rngs*dx;py=p[1]+rngs*dy
        rr=np.clip((py/RES).astype(int),0,59);cc=np.clip((px/RES).astype(int),0,59)
        collision=(px<=0)|(py<=0)|(px>=6)|(py>=6)|world[rr,cc]
        q=np.argmax(collision) if np.any(collision) else len(rngs)-1
        results.append(min(max_range,max(.04,float(rngs[q]+rng.normal(0,.012)))))
    return np.array(results)

def detect_red(bgr,min_area=60):
    import cv2
    img=cv2.GaussianBlur(bgr,(5,5),0);hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    mask=cv2.bitwise_or(cv2.inRange(hsv,(0,120,70),(10,255,255)),cv2.inRange(hsv,(170,120,70),(180,255,255)))
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    cnt,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    det=[]
    for c in cnt:
        if cv2.contourArea(c)>=min_area:
            x,y,w,h=cv2.boundingRect(c);det.append((x+w/2,y+h/2,(x,y,w,h)))
    return det,mask

def fuse(grid,pose,detects,ranges,angles,w=320,hfov=1.1):
    marks=[]
    for cx,cy,_ in detects:
        bearing=(.5-cx/w)*hfov # positive bearing left
        idx=np.argsort(abs(angles-bearing))[:5]
        rr=np.array(ranges)[idx];rr=rr[np.isfinite(rr)&(rr<3.5)]
        if not len(rr):continue
        distance=float(np.min(rr));x=pose[0]+distance*np.cos(pose[2]+bearing);y=pose[1]+distance*np.sin(pose[2]+bearing)
        if 0<=x<6 and 0<=y<6:
            row,col=rc((x,y));grid.logodds[row,col]+=1.25;marks.append((x,y))
    return marks

def consensus(initial,A,iters=50,eps=None):
    x=np.array(initial,dtype=float);deg=A.sum(axis=1); eps=float(.9/max(deg)) if eps is None else eps
    hist=[x.copy()]
    for _ in range(iters):x=x+eps*(A@x-deg*x);hist.append(x.copy())
    return np.array(hist),eps

def coverage_cost(z,points):
    stations=np.asarray(z).reshape(-1,2)
    return float(np.min(((points[:,None,:]-stations[None,:,:])**2).sum(axis=2),axis=1).mean())

def pso(points,topology='global',seed=0,n=28,iters=65,robots=4):
    rng=np.random.default_rng(seed);D=robots*2;x=rng.uniform(.22,2.78,(n,D));v=np.zeros_like(x)
    p=x.copy();f=np.array([coverage_cost(q,points) for q in x]);pf=f.copy();hist=[]
    for _ in range(iters):
        if topology=='global':best=np.repeat(p[np.argmin(pf)][None,:],n,axis=0)
        else:
            ids=[min([(i-1)%n,i,(i+1)%n],key=lambda j:pf[j]) for i in range(n)]
            best=p[ids]
        v=.72*v+1.5*rng.random((n,D))*(p-x)+1.5*rng.random((n,D))*(best-x)
        x=np.clip(x+v,.22,2.78)
        val=np.array([coverage_cost(q,points) for q in x]);improve=val<pf
        p[improve]=x[improve];pf[improve]=val[improve];hist.append(float(pf.min()))
    return p[np.argmin(pf)].reshape(robots,2),np.array(hist)

def expert(s):
    # 8 epuck-like sensors ordered clockwise around body, front at indices 0,7
    front=max(s[0],s[7]);left=max(s[5],s[6]);right=max(s[1],s[2])
    if front<.4:return 0
    return 1 if left<right else 2

def synthetic_dataset(n=9000,seed=23):
    rng=np.random.default_rng(seed);X=rng.beta(1.5,4,(n,8))
    # deliberate inclusion of minority edge/collision cases
    idx=rng.choice(n,n//2,replace=False)
    X[idx,0]=rng.uniform(.35,1,len(idx));X[idx,7]=rng.uniform(.25,1,len(idx))
    return X,np.array([expert(s) for s in X])
