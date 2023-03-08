"""
    This code is the implementation of an agent-based system for design and generating fluid shapes
    Fine tuned for a certain scale, the parameters can be changed to suit every other scale and use-case
    The agents need a curve (open or closed) as a guide
    This code is meant to be copy pasted inside a ghPthon component
    Inputs : 
            points : List of points representing the agents (point3d)
            time : Time period representing the number of iterations of the code (Integer)
            curve : Curve used as a guide for the agents to follow (curve)
            min_dist : The minimal distance allowed between agents (float) 
            max_dist : The maximum distance allowed between agents (float)
            Scale_dist : A scale factor that increases speed and acceleration (float)
            Amplitude : Amplitude for ondulation and rotation (float)
            mode : to switch between Runners, Swimmers and Spinners (Only integers 0, 1 and 2 allowed)
    
    Outputs : 
            final_path : Tree of points representing the final complete points trajectories (point3d)

"""



import Rhino.Geometry as rg
import ghpythonlib.components as ghc
from ghpythonlib.treehelpers import list_to_tree
import random
import math

#Defining a class for the general Behavior of our Boids(agents)
class Boids:
    
    #Initialize the class
    def __init__(self, p, v, curve, minD, maxD, scaleD):
        self.p = p
        self.v = v
        self.minD = minD
        self.maxD = maxD
        self.scaleD = scaleD
        self.pts = self.p
        self.vRep = []
        self.vAtt = []
        self.vStab = []
        self.repulsion()
        self.attraction()
        self.stable()
        self.V_final = []
        self.Sum_behavior()
        self.path = []
        self.draw_path()
    
    
    #Repulsion of the agents if they get too close to each other, closer than the mininum distance (min_dist)
    def repulsion(self):
        for i in range(len(self.pts)):
            Vrep = []
            VrepF = []
            for j in range(len(self.pts)):
                if j==i:
                    continue
                d = self.pts[i].DistanceTo(self.pts[j])
                if d <= self.minD and len(self.pts) > 1:
                    a = ghc.UnitVector(-((self.pts[j]-self.pts[i])) * ((self.minD-d) + 0.1 * (d-self.minD)))
                    vrep = ghc.UnitVector(self.v[i]) + a
                    Vrep.append(vrep)
                else:
                    Vrep.append(1)
            for k in range(len(Vrep)):
                if Vrep[k] != 1:
                    VrepF.append(Vrep[k])
            if len(VrepF) == 0:
                self.vRep.append(1)
            else:
                Addition, values = ghc.MassAddition(VrepF)
                self.vRep.append(Addition)
    
    
    #Attraction of the agents if they move too far away from each other, farther than the maximum distance (max_dist)
    def attraction(self):
        for i in range(len(self.pts)):
            Vatt = []
            VattF = []
            Cp, ind, dist = ghc.ClosestPoints(self.pts[i], self.pts, 15)
            for j in range(len(self.pts)):
                d = self.pts[i].DistanceTo(self.pts[j])
                if j == i or j not in ind:
                    continue
                if d >= self.maxD and len(self.pts) > 1:
                    a = ghc.UnitVector(((self.pts[j]-self.pts[i]) * ((d-self.minD) + 0.1 * (d-self.minD))))
                    vt = ghc.UnitVector(self.v[i]) + a
                    Vatt.append(vt)
                else:
                    Vatt.append(1)
            for k in range(len(Vatt)):
                if Vatt[k] != 1:
                    VattF.append(Vatt[k])
            if len(VattF) == 0:
                self.vAtt.append(1)
            else:
                Addition, values = ghc.MassAddition(VattF)
                self.vAtt.append(Addition)
    
    
    #Keep the same motion if in a stable state
    def stable(self):
        for i in range(len(self.pts)):
            Vstab = []
            VstabF = []
            for j in range(len(self.pts)):
                if j == i:
                    continue
                d = self.pts[i].DistanceTo(self.pts[j])
                if self.minD < d < self.maxD and len(self.pts) > 1:
                    vstab = ghc.UnitVector(self.v[i])
                    Vstab.append(vstab)
                else:
                    Vstab.append(1)
            for k in range(len(Vstab)):
                if Vstab[k] != 1:
                    VstabF.append(Vstab[k])
            if len(VstabF) == 0:
                self.vStab.append(1)
            else:
                Addition, values = ghc.MassAddition(VstabF)
                self.vStab.append(Addition)
    
    
    #Make the sum of all the behaviors of the agents and determine the final behavior for every one of them
    def Sum_behavior(self):
        for a, b, c in zip(self.vRep, self.vAtt, self.vStab):
            if a == 1 and b == 1 and c == 1:
                continue
            elif a != 1 and b == 1 and c == 1:
                final = self.scaleD * ghc.UnitVector(a * b * c)
            elif a != 1 and b != 1 and c == 1:
                final = self.scaleD * ghc.UnitVector((a + b)*c)
            elif a != 1 and b != 1 and c != 1:
                final = self.scaleD * ghc.UnitVector(a + b + c)
            elif a == 1 and b != 1 and c != 1:
                final = self.scaleD * ghc.UnitVector(a * (b + c))
            elif a == 1 and b == 1 and c != 1:
                final = self.scaleD * ghc.UnitVector(a * b * c)
            elif a != 1 and b == 1 and c != 1:
                final = self.scaleD * ghc.UnitVector((a + c) * b)
            elif a == 1 and b != 1 and c == 1:
                final = self.scaleD * ghc.UnitVector(a * b * c)
                
            self.V_final.append(final)
    
    
    #Draw the path of the moved agents
    def draw_path(self):
        for i in range(len(self.V_final)):
            pt, trans = ghc.Move(self.p[i], self.V_final[i])
            self.path.append(pt)

#A sub class for agents that slide straight along the curve
class Runners(Boids):
    pass


#A sub class for agents that ondulate along the curve
class Swimmers(Boids):
    def __init__(self, p, v, curve, minD, maxD, scaleD, amp, freq, ran):
        Boids.__init__(self, p, v, curve, minD, maxD, scaleD)
        self.amp = amp
        self.freq = freq
        self.ran = ran
        self.swim_path = []
        self.Swim()
    
    
    def Swim(self):
        for p1, p2 in zip(self.p, self.path):
            curve = rg.Curve.CreateInterpolatedCurve([p1,p2],3)
            plane = ghc.PerpFrame(curve, 0.5)
            vec = plane.Normal
            x = self.ran * self.amp*math.sin(self.freq)
            pt = rg.Point3d(x,0,1*self.amp)
            Opt , x = ghc.Orient(pt, ghc.XYPlane(rg.Point3d(0,0,0)), plane)
            self.swim_path.append(Opt)


#A sub class for agents that rotate around the curve
class Spinners(Boids):
    def __init__(self, p, v, curve, minD, maxD, scaleD, amp, freq, ran, waves):
        Boids.__init__(self, p, v, curve, minD, maxD, scaleD)
        self.amp = amp
        self.freq = freq
        self.ran = ran
        self.waves = waves
        self.spin_path = []
        self.Spin()
    
    
    def Spin(self):
        for p1, p2 in zip(self.p, self.path):
            curve = rg.Curve.CreateInterpolatedCurve([p1,p2],3)
            plane = ghc.PerpFrame(curve, 0.5)
            vec = plane.Normal
            x = 1.9*math.cos(self.waves/1.5)
            y = 1.9*math.sin(self.waves/1.5) 
            pt = rg.Point3d(x,y,1.5)
            Opt , x = ghc.Orient(pt, ghc.XYPlane(rg.Point3d(0,0,0)), plane)
            self.spin_path.append(Opt)




#Running the code

try:
    #Create initial random vectors for the agents
    final_path = []
    vectors = []
    
    for _ in range(len(points)):
        vs = rg.Vector3d(random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1))
        vectors.append(vs)
    
    flockstart = Boids(points, vectors, curve, min_dist, max_dist, Scale_dist)
    flockstart = flockstart.path
    
    
    #Create the path for the agents to follow
    pullp = []
    count = 1
    
    for i in range(time):
        pointVec = ghc.Average(flockstart)
        p , ti, D = ghc.CurveClosestPoint(pointVec,curve)
        d = ghc.Length(curve)
        t = ti+ (Scale_dist/d)
        
        #Make the t parameter start from 0 every time we come back to the start of the curve (Useful in the case of a closed curve to have an endless motion)
        if t > count:
            t = t - count
            count += 1
        evalC, T, Ang = ghc.EvaluateCurve(curve, t)
        pullp.append(evalC)
        vec = evalC - pointVec
        vss = []
        for j in range(len(vectors)):
            vs = rg.Vector3d(random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1))
            vsSum = 0.3*vs + vec
            vss.append(vsSum)
        #Mode 0 engages the runners that go straight
        if mode == 0:
            flock = Runners(flockstart, vss, curve, min_dist, max_dist, Scale_dist)
            final_path.append(flock.path)
            flockstart = flock.path
        #Mode 1 engages the swimmers that ondulate
        if mode == 1:
            flock = Swimmers(flockstart, vss, curve, min_dist, max_dist, Scale_dist, Amplitude, 0.01, random.uniform(-1,1))
            final_path.append(flock.swim_path)
            flockstart = flock.swim_path
        #Mode 2 engages the spinners that rotate
        if mode == 2:
            flock = Spinners(flockstart, vss, curve, min_dist, max_dist, Scale_dist, Amplitude, 0.3, random.uniform(-1,1),i)
            final_path.append(flock.spin_path)
            flockstart = flock.spin_path
    
    
    final_path = list_to_tree(final_path)
except Exception:
    print("Sorry, missing or wrong Data")





