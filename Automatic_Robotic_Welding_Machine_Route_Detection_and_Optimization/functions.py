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
from OCC.Core.TopoDS import TopoDS_Compound, TopoDS_Solid, TopoDS_Edge, topods_Solid
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve2d
from OCC.Core.gp import gp_Pnt
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepClass3d import BRepClass3d_SolidClassifier
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
import itertools

RDK = robolink.Robolink()

robot=RDK.Item("Fanuc ARC Mate 120iC/12L",ITEM_TYPE_ROBOT)
kazan = RDK.Item("example", ITEM_TYPE_OBJECT)
robotPose = robot.Pose().Pos()
kazanpos=kazan.Pose().Pos()
#partFrame=RDK.Item("Fanuc ARC Mate 120iC/12L Base").Pose()

#detects curves and adds them to RoboDK
def set_curves():
    step_reader = STEPControl_Reader()
    my_box = step_reader.ReadFile('/Users/eceyucer/Desktop/Kodlama/GSU/ Bitirme/deneme3/example.stp')
    step_reader.TransferRoot()
    shape = step_reader.Shape()
    explorer = TopExp_Explorer()
    explorer.Init(shape, TopAbs_SOLID)
    #process of dividing cad data into solid parts 
    solids = []  # 
    while explorer.More():
        solid = explorer.Current()
        solids.append(solid)
        explorer.Next()

    #The process of finding all surfaces of cad data 
    explorer.Init(shape, TopAbs_FACE)
    mainfaces = []  # the process of separating the main part into facets
    while explorer.More():
        face = explorer.Current()
        mainfaces.append(face)
        explorer.Next()

    tolerance = ShapeFix_ShapeTolerance()

    intersection = []
    intersection_info = []
    for i in range(len(solids)):
        for j in range(i + 1, len(solids)):
            solid1 = solids[i]
            solid2 = solids[j]

            #cad has a large tolerance value due to the large gap spacing
            tolerance.SetTolerance(solid1, 1)  # Set the tolerance value to 1
            tolerance.SetTolerance(solid2, 1)  # Set the tolerance value to 1

            section = BRepAlgoAPI_Section(solid1, solid2)
            section.Build()
            intersection_shape = section.Shape()

            intersection.append(intersection_shape)
            intersection_info.append((i, j, intersection_shape))

    curveListinstersectionindex = []
    curveList = []
    i = 0
    for index, intersection2 in enumerate(intersection):
        for edge in TopologyExplorer(intersection2).edges():
            curve = BRepAdaptor_Curve(edge)
            start_param = curve.FirstParameter()
            end_param = curve.LastParameter()
            num_samples = 10  # Örnekleme sayısını artırdım
            params = np.linspace(start_param, end_param, num_samples)
            points = [curve.Value(param) for param in params]
            
            # Create a new list to store each curve and add this curve to this list
            curve_points = []
            for point in points:
                curve_points.append([point.X(), point.Y(), point.Z()])  # Add the coordinates of each point as a matrix
            curveList.append(curve_points)  # Add the generated list to the master list to store each curve
            curveListinstersectionindex.append(intersection_info[index][:2])

    return intersection, intersection_info, curveList, curveListinstersectionindex, solids, mainfaces

def create_curves(curveList):
    kazan3=RDK.Item("example-3",ITEM_TYPE_OBJECT)
    curve_items = []
    sayac = 0
    for curve_points in curveList:
        egri = kazan3.AddCurve(curve_points, False, PROJECTION_ALONG_NORMAL_RECALC)  # Create a curve using each curve matrix
        if egri.Valid():
            sayac += 1
            egri.setName("Curve" + str(sayac))  # Name the curve
            curve_items.append(egri)
        else:
            print("Hata: Eğri geçersiz")

#solid0 is divided into 3 parts and the points of each part are returned
def split_solid0_into_parts(mainfaces):
    #solid bottom 
    # Create a BRepOffsetAPI_Sewing object
    sewing = BRepOffsetAPI_Sewing()
    # Add faces to the sewing object
    for face in mainfaces[:91]:
        sewing.Add(face)
    # Perform sewing operation
    sewing.Perform()
    # Get the result as a single shape
    solidbottom1 = sewing.SewedShape()
    solid_builder = BRepBuilderAPI_MakeSolid(solidbottom1)
    # Perform the operation
    solid_builder.Build()
    # Get the resulting solid
    solidbottom = solid_builder.Solid()

    #solid bigside
    sewing = BRepOffsetAPI_Sewing()
    for face in mainfaces[91:209]:
        sewing.Add(face)
    sewing.Perform()
    solidbottom1 = sewing.SewedShape()
    solid_builder = BRepBuilderAPI_MakeSolid(solidbottom1)
    solid_builder.Build()
    solidbigside = solid_builder.Solid()

    #solid smallside
    sewing = BRepOffsetAPI_Sewing()
    for face in mainfaces[209:248]:
        sewing.Add(face)
    sewing.Perform()
    solidbottom1 = sewing.SewedShape()
    solid_builder = BRepBuilderAPI_MakeSolid(solidbottom1)
    solid_builder.Build()
    solidsmallside = solid_builder.Solid()

    solid0 = [solidbottom, solidbigside, solidsmallside]

    solid0Liste = {}
    for index, solid in enumerate(solid0):
        liste = []
        for face in TopologyExplorer(solid0[index]).faces():
            brep_surface = BRepAdaptor_Surface(face)
            brep_surfaceValueF = brep_surface.Value(
                brep_surface.FirstUParameter(), brep_surface.FirstVParameter())
            brep_surfaceValueL = brep_surface.Value(
                brep_surface.LastUParameter(), brep_surface.LastVParameter())
            firstx, firsty, firstz = brep_surfaceValueF.X(
            ), brep_surfaceValueF.Y(), brep_surfaceValueF.Z()
            lastx, lasty, lastz = brep_surfaceValueL.X(
            ), brep_surfaceValueL.Y(), brep_surfaceValueL.Z()
            liste.append([firstx, firsty, firstz])
            liste.append([lastx, lasty, lastz])
        if index == 0:
            solid0Liste['solidbottom'] = liste.copy()
        elif index == 1:
            solid0Liste['solidbigside'] = liste.copy()
        elif index == 2:
            solid0Liste['solidsmallside'] = liste.copy()

    return solid0Liste

def interpolate_points(start, end, step_size=1.0):
    start = np.array(start)
    end = np.array(end)
    distance = np.linalg.norm(end - start)
    num_steps = int(distance / step_size)
    step_vector = (end - start) / num_steps
    points = []

    for i in range(num_steps + 1):
        point = start + i * step_vector
        points.append(point.tolist())

    return points

def points_of_each_solid(solids):
    #number of faces and points of each solid object 
    grupFace = {}
    for index in range(len(solids)):
        liste = []
        for face in TopologyExplorer(solids[index]).faces():
            brep_surface = BRepAdaptor_Surface(face)
            brep_surfaceValueF = brep_surface.Value(
                brep_surface.FirstUParameter(), brep_surface.FirstVParameter())
            brep_surfaceValueL = brep_surface.Value(
                brep_surface.LastUParameter(), brep_surface.LastVParameter())
            firstx, firsty, firstz = brep_surfaceValueF.X(
            ), brep_surfaceValueF.Y(), brep_surfaceValueF.Z()
            lastx, lasty, lastz = brep_surfaceValueL.X(
            ), brep_surfaceValueL.Y(), brep_surfaceValueL.Z()
            #liste.append([firstx, firsty, firstz])
            #liste.append([lastx, lasty, lastz])
            first_point = [firstx, firsty, firstz]
            last_point = [lastx, lasty, lastz]
            interpolated_points = interpolate_points(first_point, last_point, step_size=10.0)
            for point in interpolated_points:
                liste.append(point)
        
        for pair in itertools.combinations(interpolated_points, 2):
            interpolated_points = interpolate_points(pair[0], pair[1], step_size=10.0)
            for point in interpolated_points:
                liste.append(point)
        
        grupFace['solid'+str(index)] = liste.copy()
    
    return grupFace

def points_of_each_face(mainfaces):
    faceList = {}
    for index in range(len(mainfaces)):
        listes = []
        for face in TopologyExplorer(mainfaces[index]).faces():
            brep_surface = BRepAdaptor_Surface(face)
            brep_surfaceValueF = brep_surface.Value(
                brep_surface.FirstUParameter(), brep_surface.FirstVParameter())
            brep_surfaceValueL = brep_surface.Value(
                brep_surface.LastUParameter(), brep_surface.LastVParameter())
            firstx, firsty, firstz = brep_surfaceValueF.X(
            ), brep_surfaceValueF.Y(), brep_surfaceValueF.Z()
            lastx, lasty, lastz = brep_surfaceValueL.X(
            ), brep_surfaceValueL.Y(), brep_surfaceValueL.Z()
            #listes.append([firstx, firsty, firstz, lastx, lasty, lastz])
            listes.append([firstx, firsty, firstz])
            listes.append([lastx, lasty, lastz])
        faceList['face'+str(index)] = listes.copy()

    return faceList

which_way = None
side = None
def find_way(curveList, curve):
    reference_frame = RDK.Item('KazanFrame', ITEM_TYPE_FRAME)
    ref_reference = reference_frame.Pose()
    counter_x = 1
    counter_y = 1
    counter_z = 1
    for index, points in enumerate(curveList[curve]):
        if index == 0:
            x_point, y_point, z_point = points
            x_point, y_point, z_point = int(x_point), int(y_point), int(z_point)
            continue
        else:
            if x_point == int(points[0]):
                counter_x += 1
            if y_point == int(points[1]):
                counter_y += 1
            if z_point == int(points[2]):
                counter_z += 1

    #Find on which axis the linear curves are located according to the values of x, y and z
    global which_way
    global side

    if (int(curveList[curve][0][1]) == int(curveList[curve][1][1])) and (int(curveList[curve][0][0]) == int(curveList[curve][1][0])) and (int(curveList[curve+1][0][1]) == int(curveList[curve+1][1][1])) and (int(curveList[curve+1][0][0]) == int(curveList[curve+1][1][0])):
            which_way = "straight-left"  
            side = "solidbigside"
            x,y,z = curveList[curve][0]
            x_ref, y_ref, z_ref = ref_reference * [x, y, z]
            print(x_ref, y_ref, z_ref)
            if x_ref < 1200:
                which_way = "straight-right"
                side = "solidsmallside"
    elif (int(curveList[curve][0][1]) == int(curveList[curve][1][1])) and (int(curveList[curve][0][0]) == int(curveList[curve][1][0])) and (int(curveList[curve-1][0][1]) == int(curveList[curve-1][1][1])) and (int(curveList[curve-1][0][0]) == int(curveList[curve-1][1][0])):
            which_way = "straight-right" 
            side = "solidbigside"
            x,y,z = curveList[curve][0]
            x_ref, y_ref, z_ref = ref_reference * [x, y, z]
            print(x_ref, y_ref, z_ref)
            if x_ref < 1200:
                which_way = "straight-left"
                side = "solidsmallside"
    elif (counter_y == len(curveList[curve])) and (counter_z == len(curveList[curve])): #x decreasing variable y and z constant
        which_way = "straight-bottom" 
        side = "solidbottom"
    elif (counter_x == len(curveList[curve])) and (counter_z == len(curveList[curve])): #x and z constant y increasing variable
        if int(curveList[curve][0][1]) < int(curveList[curve][1][1]): #x and z constant y increasing variable
            which_way = "bottom-left" 
            side = "solidbottom"
        elif int(curveList[curve][0][1]) > int(curveList[curve][1][1]): #x and z constant y decreasing variable
            which_way = "bottom-right" 
            side = "solidbottom"

    return which_way, side

x_rot = 0  
y_rot = 0 
z_rot = 0
def rotation_angles(intersectSolid1, intersectSolid2, curve, curveList, grupFace, solid0Liste):
    global which_way
    global side
    print(which_way, side)
    if intersectSolid1 != 0:
        solid1 = f"solid{intersectSolid1}"
        # Create separate lists for each coordinate
        x_coords = [point[0] for point in grupFace[solid1]]
        y_coords = [point[1] for point in grupFace[solid1]]
        z_coords = [point[2] for point in grupFace[solid1]]
        # Calculate the mean x, y and z coordinates
        avg_x_11 = sum(x_coords) / len(x_coords)
        avg_y_11 = sum(y_coords) / len(y_coords)
        avg_z_11 = sum(z_coords) / len(z_coords)
        point1 = [avg_x_11, avg_y_11, avg_z_11]
    else:
        # Create separate lists for each coordinate
        x_coords = [point[0] for point in solid0Liste[side]]
        y_coords = [point[1] for point in solid0Liste[side]]
        z_coords = [point[2] for point in solid0Liste[side]]
        # Calculate the mean x, y and z coordinates
        avg_x_0 = sum(x_coords) / len(x_coords)
        avg_y_0 = sum(y_coords) / len(y_coords)
        avg_z_0 = sum(z_coords) / len(z_coords)
        point1 = [avg_x_0, avg_y_0, avg_z_0]

    if intersectSolid2 != 0:
        solid1 = f"solid{intersectSolid2}"
        # Create separate lists for each coordinate
        x_coords = [point[0] for point in grupFace[solid1]]
        y_coords = [point[1] for point in grupFace[solid1]]
        z_coords = [point[2] for point in grupFace[solid1]]
        # Calculate the mean x, y and z coordinates
        avg_x_11 = sum(x_coords) / len(x_coords)
        avg_y_11 = sum(y_coords) / len(y_coords)
        avg_z_11 = sum(z_coords) / len(z_coords)
        point2 = [avg_x_11, avg_y_11, avg_z_11]
    else:
        # Create separate lists for each coordinate
        x_coords = [point[0] for point in solid0Liste[side]]
        y_coords = [point[1] for point in solid0Liste[side]]
        z_coords = [point[2] for point in solid0Liste[side]]
        # Calculate the mean x, y and z coordinates
        avg_x_0 = sum(x_coords) / len(x_coords)
        avg_y_0 = sum(y_coords) / len(y_coords)
        avg_z_0 = sum(z_coords) / len(z_coords)
        point2 = [avg_x_0, avg_y_0, avg_z_0]

    # Create separate lists for each coordinate
    x_coords = [point[0] for point in curveList[curve]]
    y_coords = [point[1] for point in curveList[curve]]
    z_coords = [point[2] for point in curveList[curve]]
    # Calculate the mean x, y and z coordinates
    avg_x = sum(x_coords) / len(x_coords)
    avg_y = sum(y_coords) / len(y_coords)
    avg_z = sum(z_coords) / len(z_coords)
    point = [avg_x, avg_y, avg_z]

    #Calculate vectors from the given points
    vector1 = [point1[0] - point[0], point1[1] - point[1], point1[2] - point[2]]
    vector2 = [point2[0] - point[0], point2[1] - point[1], point2[2] - point[2]]

    #Calculate the dot product, the magnitudes of the two vectors, 
    dot_product = np.dot(vector1, vector2)
    magnitudes = np.linalg.norm(vector1) * np.linalg.norm(vector2)
    # Calculate the angle between the vectors in radians
    angle = np.arccos(dot_product / magnitudes)
    #Convert the angle to degrees
    angle = np.degrees(angle)
    angle = round(angle,-1)

    # Calculate the x-axis, y-axis, z-axis rotation between the two vectors
    x_rotation = np.arctan2(vector2[1], vector2[2]) - np.arctan2(vector1[1], vector1[2])
    y_rotation = np.arctan2(vector2[2], vector2[0]) - np.arctan2(vector1[2], vector1[0])
    z_rotation = np.arctan2(vector2[0], vector2[1]) - np.arctan2(vector1[0], vector1[1])

    global x_rot
    global y_rot
    global z_rot

    # Convert the rotations to degrees
    x_rot = np.degrees(x_rotation)
    y_rot = np.degrees(y_rotation)
    z_rot = np.degrees(z_rotation)

    print(x_rot, y_rot, z_rot, angle)
    x_rot = -180
    if which_way == "straight-right": #x and y can be solidsmall side if x and y are constant and z is decreasing variable, x is less than 1300. 
        if side == "solidbigside":
            if y_rot > 180:
                y_rot = ((360 - y_rot)/2) + 45
            elif y_rot < 0:
                y_rot = (-y_rot)
            z_rot = angle/2
        elif side == "solidsmallside":
            if y_rot > 180:
                y_rot = (-y_rot)
            elif y_rot < 0:
                y_rot = (360 + y_rot) - 45
            z_rot = (angle/2)+90
    elif which_way == "straight-left": #x and y constant and z increasing variable
        if side == "solidbigside":
            if y_rot > 180:
                y_rot = ((360 - y_rot)/2) + 45
            elif y_rot < 0:
                y_rot = (-y_rot)
            z_rot = -(angle/2)
        elif side == "solidsmallside":
            if 90 > y_rot > 180:
                y_rot = (-y_rot)
            elif y_rot < 0:
                y_rot = (360 + y_rot)/2 
            z_rot = -(angle/2)-90
    elif which_way == "straight-bottom": #x decreasing variable y and z constant
        y_rot = 90
        z_rot = 0
    elif which_way == "bottom-left": #x and z constant y increasing variable
        if 75 < angle <105:
            y_rot = angle
        elif angle > 90:
            y_rot = angle/2
        elif angle < 90:
            y_rot = angle*2
        else:
            y_rot = angle
        
        z_rot = -90
    elif which_way == "bottom-right": #x and z constant y decreasing variable
        if 75 < angle < 105:
            y_rot = angle
        elif angle > 90:
            y_rot = angle/2
        elif angle < 90:
            y_rot = angle*2
        else:
            y_rot = angle
        z_rot = 90

    print(which_way)

    return x_rot, y_rot, z_rot

def testCollision(x_range, y_range, z_range, x_ref, y_ref, z_ref, rx, ry, rz, step):
    for x in np.arange(x_range[0], x_range[1], step):
        for y in np.arange(y_range[0], y_range[1], step):
            for z in np.arange(z_range[0], z_range[1], step):
                # Perform collision check using the combination x, y, z
                # If no collision, add coordinates to the list
                tmpx = x_ref + x
                tmpy = y_ref + y
                tmpz = z_ref + z 
                
                try:
                    pose_point_in_reference = KUKA_2_Pose([tmpx, tmpy, tmpz, rz, ry, rx])
                    robot.MoveL(pose_point_in_reference) 
                except TargetReachError as e:
                    print("testCollision Error")
                    continue

                RDK.setCollisionActive(COLLISION_ON)
                if(RDK.Collisions() > 0):
                    RDK.setCollisionActive(COLLISION_OFF)
                else: #collision yok
                    RDK.setCollisionActive(COLLISION_OFF)
                    liste = [x, y, z, rx, ry, rz]
                    return liste
    
    return -1