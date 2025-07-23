# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).
It gathers all visualizations of the investigated and explained planning algorithms.
License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from IPBenchmark import Benchmark 
from IPEnvironment import CollisionChecker
from shapely.geometry import Point, Polygon, LineString
import shapely.affinity
import math
import numpy as np

from IPPlanarManipulator import PlanarJoint, PlanarRobot
from IPEnvironmentKin import planarRobotVisualize
from IPEnvironmentKin import KinChainCollisionChecker

benchList = list()


"""
# -----------------------------------------
trapField = dict()
trapField["obs1"] =   LineString([(6, 18), (6, 8), (16, 8), (16,18)]).buffer(1.0)
description = "Following the direct connection from goal to start would lead the algorithm into a trap."
benchList.append(Benchmark("Trap", CollisionChecker(trapField), [[10,15]], [[1,1], [10,1]], description, 2))

# -----------------------------------------
bottleNeckField = dict()
bottleNeckField["obs1"] = LineString([(0, 13), (11, 13)]).buffer(.5)
bottleNeckField["obs2"] = LineString([(13, 13), (23,13)]).buffer(.5)
description = "Planer has to find a narrow passage."
benchList.append(Benchmark("Bottleneck", CollisionChecker(bottleNeckField), [[4,15]], [[1,1], [18,1]], description, 2))

# -----------------------------------------
fatBottleNeckField = dict()
fatBottleNeckField["obs1"] = Polygon([(0, 8), (11, 8),(11, 15), (0, 15)]).buffer(.5)
fatBottleNeckField["obs2"] = Polygon([(13, 8), (24, 8),(24, 15), (13, 15)]).buffer(.5)
description = "Planer has to find a narrow passage with a significant extend."
benchList.append(Benchmark("Fat bottleneck", CollisionChecker(fatBottleNeckField), [[4,21]], [[1,1], [18,1]], description, 2))

# -----------------------------------------

myField = dict()
myField["L"] = Polygon([(10, 16), (10, 11), (13, 11), (13,12), (11,12), (11,16)])
myField["T"] = Polygon([(14,16), (14, 15), (15, 15),(15,11), (16,11), (16,15), (17, 15), (17, 16)])
myField["C"] = Polygon([(19, 16), (19, 11), (22, 11), (22, 12), (20, 12), (20, 15), (22, 15), (22, 16)])

myField["Antenna_L"] = Polygon([(3, 12), (1, 16), (2, 16), (4, 12)])
myField["Antenna_Head_L"] = Point(1.5, 16).buffer(1)

myField["Antenna_R"] = Polygon([(7, 12), (9, 16), (8, 16), (6, 12)])
myField["Antenna_Head_R"] = Point(8.5, 16).buffer(1)

myField["Rob_Head"] = Polygon([(2, 13), (2, 8), (8, 8), (8, 13)])
description = "Planer has to find a passage past a robot head and the print of the LTC."
benchList.append(Benchmark("MyField", CollisionChecker(myField), [[4,21]], [[1,1], [18,1]], description, 2))

"""
# -----------------------------------------
# Traveling Salesman Problem (TSP) Benchmarks
# -----------------------------------------


cross = dict()
offset = 5
cross["vertical"] = LineString([(15, 5), (15, 25)]).buffer(1)
cross["horizontal"] = LineString([(5, 15), (25, 15)]).buffer(1)
description = "Simple cross shape with 4 goals."
benchList.append(Benchmark(
    name="Plus Sign", 
    collisionChecker=CollisionChecker(cross), 
    goalList=[[10, 20], [20, 20], [20, 10]], 
    startList=[[10, 10]], 
    description=description, 
    level=1))



# Grid of squares
square_grid = dict()
scale = 4
offset = 0.5
for x in range(8):
    for y in range(8):
        # Each square is a Polygon with side length 2, centered at (x*scale, y*scale)
        cx, cy = x * scale, y * scale
        square_grid[f"square_{x}_{y}"] = Polygon([
            (cx - 1, cy - 1),
            (cx + 1, cy - 1),
            (cx + 1, cy + 1),
            (cx - 1, cy + 1)
        ])

description = "Grid of squares."
benchList.append(Benchmark(
    name="Square Grid", 
    collisionChecker=CollisionChecker(square_grid), 
    goalList=[[2, 2], [26, 5], [22, 15], [15, 26]], 
    startList=[[6, 10]], 
    description=description, 
    level=2))

# -------------------------------------------

# Grid of squares
square_grid = dict()
scale = 4
side_length = 2
offset_odd_y = 2
offset_even_y = 0.0
for x in range(8):
    for y in range(8):
        if y%2 == 0:
            offset = offset_even_y
        else:
            offset = offset_odd_y
        # Each square is a Polygon with side length side_length, centered at (x*scale, y*scale)
        cx, cy = x * scale + offset, y * scale
        square_grid[f"square_{x}_{y}"] = Polygon([
            (cx - side_length/2, cy - side_length/2),
            (cx + side_length/2, cy - side_length/2),
            (cx + side_length/2, cy + side_length/2),
            (cx - side_length/2, cy + side_length/2)
        ])

description = "Grid of squares."
benchList.append(Benchmark(
    name="Square Grid", 
    collisionChecker=CollisionChecker(square_grid), 
    goalList=[[2, 2], [26, 5.5], [22, 15], [15, 26]], 
    startList=[[6, 10]], 
    description=description, 
    level=2))


# 2-DoF planar robot
# -----------------------------------------
obst = dict()
obst["obs1"] = LineString([(-2, 0), (-0.8, 0)]).buffer(0.5)
r = PlanarRobot(n_joints=2)
environment = KinChainCollisionChecker(r, obst, fk_resolution=.2)
description = "Planar robot with two joints and 1 obstacle."
benchList.append(Benchmark("2-DoF planar Robot - 1 Obstacles", environment, [[2.0, 1.5]], [[-2.0,-1.5]], description, 1))

obst = dict()
obst["obs1"] = LineString([(-2, 0), (-0.8, 0)]).buffer(0.5)
obst["obs2"] = LineString([(2, 0), (2, 1)]).buffer(0.2)
r = PlanarRobot(n_joints=2)
environment = KinChainCollisionChecker(r, obst, fk_resolution=.2)
description = "Planar robot with two joints and 2 obstacles."
benchList.append(Benchmark("2-DoF planar Robot - 2 Obstacles", environment, [[2.0, 1.5]], [[-2.0,-1.5], [0.8, -0.5], [-0.3, 0.5]], description, 2))

obst = dict()
obst["obs1"] = LineString([(-2, 0), (-0.8, 0)]).buffer(0.5)
obst["obs2"] = LineString([(2, 0), (2, 1)]).buffer(0.2)
obst["obs3"] = LineString([(-0.5, -2.0), (-0.5, -2.5)]).buffer(0.5)
obst["obs4"] = LineString([(0.5, -2.0), (0.5, -2.5)]).buffer(0.5)
r = PlanarRobot(n_joints=2)
environment = KinChainCollisionChecker(r, obst, fk_resolution=.2)
description = "Planar robot with two joints and 4 obstacles."
benchList.append(Benchmark("2-DoF planar Robot - 4 Obstacles", environment, [[2.0, 1.5]], [[-2.3,-1.2], [0.8, -0.5], [-0.3, 0.5], [-1.5, 0.0]], description, 2))


# 3-DoF planar robot
# -----------------------------------------
#obst = dict()
#obst["obs1"] = LineString([(-2, 0), (-0.8, 0)]).buffer(0.5)
#obst["obs2"] = LineString([(2, 0), (2, 1)]).buffer(0.2)
#obst["obs3"] = LineString([(-1, 2), (1, 2)]).buffer(0.1)
#limits = [[-3.14,3.14],[-3.14,3.14],[-3.14,3.14]]
#r = PlanarRobot(n_joints=3)
#environment = KinChainCollisionChecker(r, obst, limits=limits, fk_resolution=.2)
#description = "Planar robot with three joints and obstacles."
#benchList.append(Benchmark("3-DoF planar Robot - 3 Obstacles", environment, [[2.0, 0.5, 0.0]], [[-2.0,-0.4, 0.0], [-2.0, -0.5, 0.0]], description, 1))

# 4-DoF planar robot
# -----------------------------------------
#obst = dict()
#obst["obs1"] = LineString([(-2, 0), (-0.8, 0)]).buffer(0.5)
#obst["obs2"] = LineString([(2, 0), (2, 1)]).buffer(0.2)
#obst["obs3"] = LineString([(-1, 2), (1, 2)]).buffer(0.1)
#limits = [[-3.14,3.14],[-3.14,3.14],[-3.14,3.14],[-3.14,3.14]]
#r = PlanarRobot(n_joints=4)
#environment = KinChainCollisionChecker(r, obst, limits=limits, fk_resolution=.2)
#description = "Planar robot with four joints and obstacles."
#benchList.append(Benchmark("4-DoF planar Robot - 3 Obstacles", environment, [[2.0, 0.5, 0.0, 0.0]], [[-2.0,-0.4, 0.0, 0.0], [-2.0, -0.5, 0.0, 0.0]], description, 1))
