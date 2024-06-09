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
import numpy as np
from queue import PriorityQueue

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

intersection, intersection_info, curveList, curveListinstersectionindex, solids, mainfaces = set_curves()
grupFace = points_of_each_solid(solids)

reference_frame = RDK.Item('KazanFrame', ITEM_TYPE_FRAME)
ref_reference = reference_frame.Pose()

df = pd.read_csv('Welding_Datas/CurvePoints.csv')
tampon = pd.read_csv('Welding_Datas/curve_tampons.csv')
first_shortest_path = pd.read_csv('Welding_Datas/first_shortest_path.csv')
second_shortest_path = pd.read_csv('Welding_Datas/second_shortest_path.csv')

RDK.setCollisionActive(COLLISION_ON)
robot.setSpeed(10)

start_time = time.time()

righttop = RDK.Item('righttop')
robot.MoveL(righttop)

rxfirst = df.iloc[0]['rx']
ryfirst = df.iloc[0]['ry']
rzfirst = df.iloc[0]['rz']

rxlast = df.iloc[-1]['rx']
rylast = df.iloc[-1]['ry']
rzlast = df.iloc[-1]['rz']

liste1 = first_shortest_path[:-1]
liste1_x_y_z = first_shortest_path.iloc[-1]

liste2 = second_shortest_path[:-1]
liste2_x_y_z = second_shortest_path.iloc[-1]

#With the a* algorithm, it goes to the starting point where the welding will be made without collision
for index, row in liste1.iterrows():
    x,y,z = row[0], row[1], row[2]

    x += liste1_x_y_z['x']
    y += liste1_x_y_z['y']
    z += liste1_x_y_z['z']

    pose_point_in_reference = KUKA_2_Pose([x, y, z, rzfirst, ryfirst, rxfirst])
    
    try:
        # Hedefe doğrudan robotu hareket ettir
        robot.setSpeed(10)
        robot.MoveL(pose_point_in_reference)
    except StoppedError as e:
        if "Collision detected" in str(e):
            print("Collision detected, closing collision control.")
            continue
        else:
            raise e

#Welds with saved angles and buffer points
for index, row in df.iterrows():
    x,y,z,rx,ry,rz = row[0], row[1], row[2], row[3], row[4], row[5]
    x_ref, y_ref, z_ref = ref_reference * [x, y, z]
    i = int(index / 10)
    curve, tamponx, tampony, tamponz = tampon.iloc[i]
    x_ref += tamponx
    y_ref += tampony
    z_ref += tamponz
    pose_point_in_reference = KUKA_2_Pose([x_ref, y_ref, z_ref, rz, ry, rx])
    try:
        robot.setSpeed(10)
        robot.MoveL(pose_point_in_reference)
    except StoppedError as e:
        if "Collision detected" in str(e):
            print("Collision detected, closing collision control.")
            print(x,y,z,rx,ry,rz)
            continue
        else:
            raise e

robot.setSpeed(10)
#Returns from the last point of welding with a* algorithm without collision
for index, row in liste2.iterrows():
    x,y,z = row[0], row[1], row[2]

    x += liste2_x_y_z['x']
    y += liste2_x_y_z['y']
    z += liste2_x_y_z['z']

    pose_point_in_reference = KUKA_2_Pose([x, y, z, rzlast, rylast, rxlast])
    
    try:
        robot.setSpeed(10)
        # Hedefe doğrudan robotu hareket ettir
        robot.MoveL(pose_point_in_reference)
    except StoppedError as e:
        if "Collision detected" in str(e):
            print("Collision detected, closing collision control.")
            print(x,y,z,rzlast, rylast, rxlast)
            continue
        else:
            raise e

robot.setSpeed(10)
target1 = RDK.Item('Target 1')
robot.MoveL(target1)
print("Time:", time.time() - start_time)
RDK.setCollisionActive(COLLISION_OFF)