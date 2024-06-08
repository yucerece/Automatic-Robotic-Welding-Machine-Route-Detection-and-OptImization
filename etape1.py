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
from openpyxl import load_workbook

from functions import *

RDK = robolink.Robolink()
step_reader = STEPControl_Reader()
my_box = step_reader.ReadFile('example.stp')
step_reader.TransferRoot()
shape = step_reader.Shape()
explorer = TopExp_Explorer()
explorer.Init(shape, TopAbs_SOLID)

robot=RDK.Item("Fanuc ARC Mate 120iC/12L",ITEM_TYPE_ROBOT)
kazan = RDK.Item("example", ITEM_TYPE_OBJECT)
robotPose = robot.Pose().Pos()
kazanpos=kazan.Pose().Pos()
#partFrame=RDK.Item("Fanuc ARC Mate 120iC/12L Base").Pose()
kazan3=RDK.Item("example-3",ITEM_TYPE_OBJECT)

intersection, intersection_info, curveList, curveListinstersectionindex, solids, mainfaces = set_curves()

#solid information about the curves if needed
print(curveListinstersectionindex[131]) 

#create_curves(curveList) #we use this, if we add curves to RoboDK. Running once is enough.

#this only used for this project, normally  no needed and shouldn't be used
solid0Liste = split_solid0_into_parts(mainfaces)

#start and end points of each solid
grupFace = points_of_each_solid(solids)

#start and end points of each face
faceList = points_of_each_face(mainfaces)

#get the no of the curve from the interface
curve = int(sys.argv[1])-1

#set the robot speed
robot.setSpeed(100)

#solids who intersects and who create the curve selected
intersectSolid1 = curveListinstersectionindex[curve][1]
intersectSolid2 = curveListinstersectionindex[curve][0]

#points of the selected curve
#print(curveList[curve])

#find_way function identify the curve's position and on which axis
way, side = find_way(curveList, curve)

print(intersectSolid1, intersectSolid2)

#rotation_angles functions find the appropriate angles to approach the curve
x_rot, y_rot, z_rot = rotation_angles(intersectSolid1, intersectSolid2, curve, curveList, grupFace, solid0Liste)

#the start and end points of some curves are reversed, the positions of the curves are added to the new list together with their angles
curveListNew = []
for i, curve_points in enumerate(curveList[curve]): 
    if (way == "straight-right") or (way == "left-bottom") or (way == "straight-bottom") or (way == "bottom-right"): 
        curveListNew.insert(0,[curve_points[0], curve_points[1], curve_points[2]] + [x_rot, y_rot, z_rot]) #x + sağa, y - bana, mavi
    elif (way == "straight-left") or (way == "right-bottom") or (way == "bottom-left"):
        curveListNew.append([curve_points[0], curve_points[1], curve_points[2]] + [x_rot, y_rot, z_rot]) #x + sağa, y - bana, mavi

reference_frame = RDK.Item('KazanFrame', ITEM_TYPE_FRAME)
ref_reference = reference_frame.Pose()

collision_off_points = []
yey_point = False

rzliste = []
ryliste = []

#If the angle at which the curve is to be approached causes a collision, trial and error is used to determine 
#the angles at which the collision will not occur for each point.
def findThePath(x,y,z,rx,ry,rz,tampon): 
    rxc = int(rx)
    ryc = int(ry)
    rzc = int(rz)

    rxr = rx
    ryr = ry
    rzr = rz

    yey_point = False

    x_ref, y_ref, z_ref = ref_reference * [x, y, z]

    x_ref += tampon[0]
    y_ref += tampon[1]
    z_ref += tampon[2]

    try:
        pose_point_in_reference = KUKA_2_Pose([x_ref, y_ref, z_ref, rz, ry, rx])
        robot.MoveL(pose_point_in_reference) 
    except TargetReachError as e:
        print("hata")

    RDK.setCollisionActive(COLLISION_ON)
    if(RDK.Collisions() > 0):
        RDK.setCollisionActive(COLLISION_OFF)
    else: #collision yok
        RDK.setCollisionActive(COLLISION_OFF)
        liste = [x, y, z, rx, ry, rz]
        return liste

    combined_range_z = list(range(rzc, rzc-45, -10)) + list(range(rzc, rzc+50, 10))
    for increment in combined_range_z:
        rz = increment
        try:
            pose_point_in_reference = KUKA_2_Pose([x_ref, y_ref, z_ref, rz, ry, rx])
            robot.MoveL(pose_point_in_reference) 
        except TargetReachError as e:
            print("hata z")
            return -1

        RDK.setCollisionActive(COLLISION_ON)
        if(RDK.Collisions() > 0):
            RDK.setCollisionActive(COLLISION_OFF)
            if increment  == (rzc+50-10):
                if len(rzliste) != 0:
                    rz = sum(rzliste) / len(rzliste)
                else: 
                    rz = rzr #move to the first position if the list is empty

                combined_range_y = list(range(ryc, ryc-70, -10)) + list(range(ryc, ryc+50, 10))
                for incrementy in combined_range_y:
                    ry = incrementy
                    try:
                        pose_point_in_reference = KUKA_2_Pose([x_ref, y_ref, z_ref, rz, ry, rx])
                        robot.MoveL(pose_point_in_reference)
                    except TargetReachError as e:
                        print("error y")
                        return -1

                    RDK.setCollisionActive(COLLISION_ON)
                    if(RDK.Collisions() > 0):
                        RDK.setCollisionActive(COLLISION_OFF)
                        #rx değiştir 
                        if incrementy == (ryc+50-10):
                            if len(ryliste) != 0:
                                ry = sum(ryliste) / len(ryliste)
                            else:
                                ry = ryr
                            
                            combined_range_x = list(range(rxc, rxc-45, -10)) + list(range(rxc, rxc+50, 10))
                            for incrementx in combined_range_x:
                                rx = incrementx
                                try:
                                    pose_point_in_reference = KUKA_2_Pose([x_ref, y_ref, z_ref, rz, ry, rx])
                                    robot.MoveL(pose_point_in_reference)
                                except TargetReachError as e:
                                    print("x hata")
                                    continue

                                RDK.setCollisionActive(COLLISION_ON)
                                if(RDK.Collisions() > 0):
                                    RDK.setCollisionActive(COLLISION_OFF)
                                    continue
                                else:
                                    RDK.setCollisionActive(COLLISION_OFF)
                                    ryliste.append(ry)
                                    rzliste.append(rz)
                                    liste = [x, y, z, rx, ry, rz]
                                    return liste
                                    yey_point = True
                                    break
                        if yey_point == True:
                            break
                        continue
                    else:
                        RDK.setCollisionActive(COLLISION_OFF)
                        ryliste.append(ry)
                        rzliste.append(rz)
                        liste = [x, y, z, rx, ry, rz]
                        return liste
                        yey_point = True
                        break
            if yey_point == True:
                break
            continue
        else: #no collision, right angles are found
            RDK.setCollisionActive(COLLISION_OFF)
            ryliste.append(ry)
            rzliste.append(rz)
            liste = [x, y, z, rx, ry, rz]
            return liste
            break
    return -1

start_time_time = time.time()
i = 0
indexlist = []
tekrar = False
tekrar_num = [] #tekrar edilecek index
lol = 0
#First, since the robot is 3D, collisions occur when the desired points are approached. For this reason, 
#buffer values are found with the testCollision function. Then the non-collision angles of each point are determined with the findThePath function. 
while i < len(curveListNew):
    indexlist.append(i)
    point_relative = curveListNew[i]

    x, y, z, rx, ry, rz = point_relative

    if lol == -1:
        ry -= 70

    # Convert the coordinates of the point to the reference frame
    x_ref, y_ref, z_ref = ref_reference * [x, y, z]
    if i % 10 == 0:
        x_range = (-3, 3)
        y_range = (-3, 3)
        z_range = (-3, 3)
        step = 0.5
        start_time = time.time()
        tampon_rots = testCollision(x_range, y_range, z_range, x_ref, y_ref, z_ref, rx, ry, rz, step)
        end_time = time.time()
        elapsed_time = end_time - start_time
        print(f"1. deneme: {elapsed_time:.4f} saniye")
        if tampon_rots != -1:
            tampon = tampon_rots[:3]
        else:
            if "straight" in way:
                target1 = RDK.Item('Target 1')
                robot.MoveL(target1)
                start_time = time.time()
                tampon_rots = testCollision(x_range, y_range, z_range, x_ref, y_ref, z_ref, rx, ry, rz+180, step)
                end_time = time.time()
                elapsed_time = end_time - start_time
                print(f"2. deneme: {elapsed_time:.4f} saniye")
                if tampon_rots != -1:
                    target1 = RDK.Item('Target 1')
                    robot.MoveL(target1)
                    start_time = time.time()
                    tampon_rots = testCollision(x_range, y_range, z_range, x_ref, y_ref, z_ref, rx, ry-30, rz, step)
                    end_time = time.time()
                    elapsed_time = end_time - start_time
                    print(f"3. deneme: {elapsed_time:.4f} saniye")
            if "bottom" in way:
                target1 = RDK.Item('Target 1')
                robot.MoveL(target1)
                tampon_rots = testCollision(x_range, y_range, z_range, x_ref, y_ref, z_ref, rx, ry+45, rz, step)
                if tampon_rots != -1:
                    target1 = RDK.Item('Target 1')
                    robot.MoveL(target1)
                    start_time = time.time()
                    tampon_rots = testCollision(x_range, y_range, z_range, x_ref, y_ref, z_ref, rx, ry, rz-30, step)
                    end_time = time.time()
                    elapsed_time = end_time - start_time
                    print(f"3. deneme: {elapsed_time:.4f} saniye")
        tampon = tampon_rots[:3]
        krx, kry, krz = tampon_rots[3:]
        if (int(rx) == int(krx)) and (int(ry) == int(kry)) and (int(rz) == int(krz)):
            pass
        else:
            new_values = [krx, kry, krz]
            curveListNew = [sublist[:-3] + new_values for sublist in curveListNew]
            rx, ry, rz = krx, kry, krz
        print(tampon)
        print("son : ",rx, ry, rz)

    # Noktanın açılarını bulun
    lol = findThePath(x, y, z, rx, ry, rz, tampon)

    if lol == -1:
        print("error ", i+1)
        # Go back to the previous point instead of moving to the next point
        print(ry)
        break
    else:
        # Save non-collision point
        collision_off_points.append(lol) 
    
    if len(collision_off_points) == 1:
        target_name = f"target{curve+1}_1"
        target1 = RDK.AddTarget(target_name) 
        RDK.Update()
    elif len(collision_off_points) == 5:
        target_name = f"target{curve+1}_5"
        target5 = RDK.AddTarget(target_name) 
        RDK.Update()
    elif len(collision_off_points) == 10:
        target_name = f"target{curve+1}_10"
        target10 = RDK.AddTarget(target_name) 
        RDK.Update()
        
    # Move to the next point
    i += 1

  #Go to point
  #See if there's colission
  #otherwise continue the for loop and get the next point
  #If there is collision, try all rx, ry, rz angles
    yey_point = False
    #rz değişöesi
if len(curveListNew) != len(collision_off_points):
    #exit()
    pass
else:
    # Sütun adları
    columns = ['x', 'y', 'z', 'rx', 'ry', 'rz']
    print(collision_off_points)
    df1 = pd.DataFrame(collision_off_points, columns=columns)
    csv_file1 = "Welding_Datas/CurvePoints.csv"
    if os.path.exists(csv_file1):
        df1.to_csv(csv_file1, mode='a', header=False, index=False)
    else:
        df1.to_csv(csv_file1, mode='w', header=True, index=False)


    curve_tampon = [[curve+1] + tampon]
    print(curve_tampon)
    columns2 = ['curve', 'tamponx', 'tampony', 'tamponz']
    df2 = pd.DataFrame(curve_tampon, columns=columns2)
    # İkinci CSV dosyasına ekleme
    csv_file2 = "Welding_Datas/curve_tampons.csv"
    if os.path.exists(csv_file2):
        df2.to_csv(csv_file2, mode='a', header=False, index=False)
    else:
        df2.to_csv(csv_file2, mode='w', header=True, index=False)

    print("Veriler CSV dosyasına eklendi.")

target1 = RDK.Item('Target 1')
robot.MoveL(target1)
print("Zaman:", time.time() - start_time_time)