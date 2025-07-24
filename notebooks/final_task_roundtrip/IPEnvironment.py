# coding: utf-8

"""
This code is part of a series of notebooks regarding  "Introduction to robot path planning".

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from IPPerfMonitor import IPPerfMonitor

import matplotlib.pyplot as plt

from shapely.geometry import Point, Polygon, LineString
from shapely import plotting

import numpy as np

class CollisionChecker(object):

    def __init__(self, scene, limits=[[0.0, 22.0], [0.0, 22.0]], statistic=None):
        self.scene = scene
        self.limits = limits

    def getDim(self):
        """ Return dimension of Environment (Shapely should currently always be 2)"""
        return 2

    def getEnvironmentLimits(self):
        """ Return limits of Environment"""
        return list(self.limits)

    @IPPerfMonitor
    def pointInCollision(self, pos):
        """ Return whether a configuration is
        inCollision -> True
        Free -> False """
        assert (len(pos) == self.getDim())
        for key, value in self.scene.items():
            if value.intersects(Point(pos[0], pos[1])):
                return True
        return False

    @IPPerfMonitor
    def lineInCollision(self, startPos, endPos):
        """ Check whether a line from startPos to endPos is colliding"""
        assert (len(startPos) == self.getDim())
        assert (len(endPos) == self.getDim())
        
        p1 = np.array(startPos)
        p2 = np.array(endPos)
        p12 = p2-p1
        k = 40
        #print("testing")
        for i in range(k):
            testPoint = p1 + (i+1)/k*p12
            if self.pointInCollision(testPoint)==True:
                return True
        
        return False
                

#        for key, value in self.scene.items():
#            if value.intersects(LineString([(startPos[0], startPos[1]), (endPos[0], endPos[1])])):
 #               return True
#        return False

    def drawObstacles(self, ax):
        for key, value in self.scene.items():
            plotting.plot_polygon(value, add_points=False, ax=ax, color='red')

from IPEnvironmentKin import interpolate_line
from IPEnvironmentKin import planarRobotVisualize
import matplotlib.pyplot as plt              
import matplotlib.animation
from IPython.display import HTML

matplotlib.rcParams['animation.embed_limit'] = 64
def animatePointRobotSolution(planner, environment, benchmark, solution, visualizer, step, workSpaceLimits=[[-3,3],[-3,3]]):
    _planner = planner
    _environment = environment
    _solution = solution
    _prmVisualizer = visualizer
    _step = step

    fig_local = plt.figure(figsize=(7, 7))
    ax1 = fig_local.add_subplot(1, 1, 1)
    ## get positions for solution
    solution_pos = [_planner.graph.nodes[node]['pos'] for node in _solution]
    ## interpolate to obtain a smoother movement
    i_solution_pos = [solution_pos[0]]
    for i in range(1, len(solution_pos)):
        segment_s = solution_pos[i-1]
        segment_e = solution_pos[i]
        i_solution_pos = i_solution_pos + interpolate_line(segment_s, segment_e, _step)[1:]
    ## animate
    frames = len(i_solution_pos)
        
    def animate(t):
        ## clear taks space figure
        ax1.cla()
        title = planner.plannerClass.__name__
        title += " " + benchmark.name
        ax1.set_title(title)
        ## fix figure size
        # ax1.set_xlim(workSpaceLimits[0])
        # ax1.set_ylim(workSpaceLimits[1])
        ## draw obstacles
        _environment.drawObstacles(ax1)
        # Redraw the entire PRM graph and the final solution path.
        _prmVisualizer(_planner, solution, ax1)
        # Draw a large red dot on the graph to show the robot's current progress.
        ax1.scatter(i_solution_pos[t][0], i_solution_pos[t][1], color='r', zorder=10, s=250)
        
        
    ani = matplotlib.animation.FuncAnimation(fig_local, animate, frames=frames)
    html = HTML(ani.to_jshtml())
    display(html)
    plt.close()


