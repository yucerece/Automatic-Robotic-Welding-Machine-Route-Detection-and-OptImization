# This code is adapted from the A* 3D path planning implementation by zhm-real.
# The original implementation can be found at:
# https://github.com/zhm-real/PathPlanning/blob/master/Search_based_Planning/Search_3D/Astar3D.py

from OCC.Display.SimpleGui import init_display
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.STEPControl import STEPControl_Reader, STEPControl_Writer, STEPControl_AsIs
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_SOLID, TopAbs_FACE, TopAbs_COMPOUND
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCC.Extend.TopologyUtils import TopologyExplorer
from OCC.Core.IFSelect import IFSelect_RetDone, IFSelect_ItemsByEntity
from OCC.Core.ShapeFix import ShapeFix_ShapeTolerance
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section
from OCC.Core.TopoDS import TopoDS_Compound, TopoDS_Solid, TopoDS_Edge
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve2d
from OCC.Core.gp import gp_Pnt
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeShell, BRepBuilderAPI_MakeSolid
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_Sewing
from OCC.Core.BRepClass3d import BRepClass3d_SolidClassifier

import os
import numpy as np
from robodk import robolink    # RoboDK API
from robodk.robomath import *
from robodk.robolink import *    # Robot toolbox
from robodk import *
import math
import pandas as pd
import time
from queue import PriorityQueue
from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox

from functions import *

RDK = robolink.Robolink()
step_reader = STEPControl_Reader()
my_box = step_reader.ReadFile('example.stp')
step_reader.TransferRoot()
shape = step_reader.Shape()
explorer = TopExp_Explorer()
explorer.Init(shape, TopAbs_SOLID)

robot = RDK.Item("Fanuc ARC Mate 120iC/12L", ITEM_TYPE_ROBOT)
kazan = RDK.Item("example", ITEM_TYPE_OBJECT)
robotPose = robot.Pose().Pos()
kazanpos = kazan.Pose().Pos()
#partFrame = RDK.Item("Fanuc ARC Mate 120iC/12L Base").Pose()
kazan3 = RDK.Item("example-3", ITEM_TYPE_OBJECT)

import numpy as np
import matplotlib.pyplot as plt
import queue
import time

class Weighted_A_star(object):
    def __init__(self, start, goal, obstacles, resolution=0.5):
        self.Alldirec = {(1, 0, 0): 1, (0, 1, 0): 1, (0, 0, 1): 1,
                         (-1, 0, 0): 1, (0, -1, 0): 1, (0, 0, -1): 1,
                         (1, 1, 0): np.sqrt(2), (1, 0, 1): np.sqrt(2), (0, 1, 1): np.sqrt(2),
                         (-1, -1, 0): np.sqrt(2), (-1, 0, -1): np.sqrt(2), (0, -1, -1): np.sqrt(2),
                         (1, -1, 0): np.sqrt(2), (-1, 1, 0): np.sqrt(2), (1, 0, -1): np.sqrt(2),
                         (-1, 0, 1): np.sqrt(2), (0, 1, -1): np.sqrt(2), (0, -1, 1): np.sqrt(2),
                         (1, 1, 1): np.sqrt(3), (-1, -1, -1): np.sqrt(3),
                         (1, -1, -1): np.sqrt(3), (-1, 1, -1): np.sqrt(3), (-1, -1, 1): np.sqrt(3),
                         (1, 1, -1): np.sqrt(3), (1, -1, 1): np.sqrt(3), (-1, 1, 1): np.sqrt(3)}
        self.settings = 'CollisionChecking'
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.obstacles = obstacles
        self.resolution = resolution
        self.g = {self.start: 0, self.goal: np.inf}
        self.Parent = {}
        self.CLOSED = set()
        self.V = []
        self.done = False
        self.Path = []
        self.ind = 0
        self.x0, self.xt = self.start, self.goal
        self.OPEN = queue.PriorityQueue()  # store [priority, point]
        self.OPEN.put((self.g[self.x0] + self.heuristic_fun(self.x0), self.x0))  # item, priority = g + h
        self.lastpoint = self.x0

    def heuristic_fun(self, node):
        return np.linalg.norm(np.array(node) - np.array(self.goal))

    def get_neighbors(self, node):
        x, y, z = node
        neighbors = [
            (x + self.resolution, y, z),
            (x - self.resolution, y, z),
            (x, y + self.resolution, z),
            (x, y - self.resolution, z),
            (x, y, z + self.resolution),
            (x, y, z - self.resolution)
        ]
        valid_neighbors = [
            (nx, ny, nz) for nx, ny, nz in neighbors
            if not self.is_collide((nx, ny, nz))
        ]
        return valid_neighbors

    def is_collide(self, point):
        for obs in self.obstacles:
            for vert in obs:
                if np.linalg.norm(np.array(point) - np.array(vert)) < self.resolution:
                    return True
        return False

    def run(self, N=None):
        xt = self.xt
        xi = self.x0
        while not self.OPEN.empty():  # while xt not reached and open is not empty
            _, xi = self.OPEN.get()
            if xi not in self.CLOSED:
                self.V.append(np.array(xi))
            self.CLOSED.add(xi)  # add the point in CLOSED set
            if np.linalg.norm(np.array(xi) - np.array(xt)) < self.resolution:
                break
            for xj in self.get_neighbors(xi):
                if xj not in self.g:
                    self.g[xj] = np.inf
                a = self.g[xi] + np.linalg.norm(np.array(xi) - np.array(xj))
                if a < self.g[xj]:
                    self.g[xj] = a
                    self.Parent[xj] = xi
                    self.OPEN.put((a + self.heuristic_fun(xj), xj))
            if N:
                if len(self.CLOSED) % N == 0:
                    break
            if self.ind % 100 == 0:
                print('number of nodes expanded = ' + str(len(self.V)))
            self.ind += 1

        self.lastpoint = xi
        if self.lastpoint in self.CLOSED:
            self.done = True
            self.Path = self.path()
            if N is None:
                self.visualization()
            return True

        return False

    def path(self):
        path = []
        x = self.lastpoint
        start = self.x0
        while x != start:
            path.append(x)
            x = self.Parent[x]
        path.append(start)
        path.reverse()
        return path

    def visualization(self):
        #visualization of the path and obstacles takes too for the graph to be shown
        pass
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for vert in self.obstacles:
            ax.scatter(vert[0], vert[1], vert[2], c='r', marker='o')
        path = np.array(self.Path)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], c='b')
        plt.show()
        """

if __name__ == '__main__':
    intersection, intersection_info, curveList, curveListinstersectionindex, solids, mainfaces = set_curves()
    #grupFace = points_of_each_solid(solids)
    #obstacles = grupFace.values()
    grupFace = points_of_each_solid(solids)
    #mainfaces_ = points_of_each_face(mainfaces)

    reference_frame = RDK.Item('KazanFrame', ITEM_TYPE_FRAME)
    ref_reference = reference_frame.Pose()
    
    # Tek boyutlu engel listesi oluşturma
    obstacles = []
    for points in grupFace.values():
        for point in points:
            point_liste = []
            x_ref, y_ref, z_ref = ref_reference * point
            point_liste.append(x_ref)
            point_liste.append(y_ref)
            point_liste.append(z_ref)
            obstacles.append(point_liste)

    print(len(obstacles))

    csv_file1 = "Welding_Datas/curve_tampons.csv"
    df1 = pd.read_csv(csv_file1)
    first_curve = int(df1.iloc[0][0]) 
    last_curve = int(df1.iloc[-1][0]) 

    
    target_name = "lefttop"  # Hedefin adı
    target = RDK.Item(target_name, ITEM_TYPE_TARGET)
    pose = target.Pose().Pos()
    xs, ys, zs = pose
    start = [int(xs), int(ys), int(zs)]

    #print(xs-int(xs), ys-int(ys), zs-int(zs))
    frac_start = [xs - int(xs), ys - int(ys), zs - int(zs)]

    #target_name = "target"  # Hedefin adı
    target_name = f"target{first_curve}_1"
    target = RDK.Item(target_name, ITEM_TYPE_TARGET)
    pose = target.Pose().Pos()
    xe, ye, ze = pose
    goal = [int(xe),int(ye), int(ze)]

    astar = Weighted_A_star(start, goal, obstacles, resolution=25)
    start_time = time.time()
    astar.run()
    print("Zaman:", time.time() - start_time)
    
    if astar.done:
        print("Yol bulundu")
        print(astar.Path)
    else:
        print("Yol bulunamadı")

    columns = ['x', 'y', 'z']
    df = pd.DataFrame(astar.Path, columns=['x', 'y', 'z'])
    df.loc[len(df)] = frac_start
    csv_file = "Welding_Datas/first_shortest_path.csv"
    df.to_csv(csv_file, mode='w', index=False)


    #GETTING BACK TO START POINT OF WELDING MACHINE
    target_name = f"target{last_curve}_10"  # Hedefin adı
    target = RDK.Item(target_name, ITEM_TYPE_TARGET)
    pose = target.Pose().Pos()
    xs, ys, zs = pose
    start = [int(xs), int(ys), int(zs)]

    #print(xs-int(xs), ys-int(ys), zs-int(zs))
    frac_start = [xs - int(xs), ys - int(ys), zs - int(zs)]

    #target_name = "target"  # name of the target
    target_name = "leftbottom"
    target = RDK.Item(target_name, ITEM_TYPE_TARGET)
    pose = target.Pose().Pos()
    xe, ye, ze = pose
    goal = [int(xe),int(ye), int(ze)]

    astar = Weighted_A_star(start, goal, obstacles, resolution=25)
    start_time = time.time()
    astar.run()
    print("Zaman:", time.time() - start_time)
    
    if astar.done:
        print("Yol bulundu")
        print(astar.Path)
    else:
        print("Yol bulunamadı")

    columns = ['x', 'y', 'z']
    df = pd.DataFrame(astar.Path, columns=['x', 'y', 'z'])
    df.loc[len(df)] = frac_start
    csv_file = "Welding_Datas/second_shortest_path.csv"
    df.to_csv(csv_file, mode = 'w', index=False)