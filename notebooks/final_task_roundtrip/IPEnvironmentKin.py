
"""
This code is part of a series of notebooks regarding  "Introduction to robot path planning".

Author: Gergely Soti, adapted by Bjoern Hein

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""
from IPPlanarManipulator import PlanarJoint, PlanarRobot
from IPEnvironment import CollisionChecker
from shapely.geometry import Point, Polygon, LineString
from shapely import plotting
import numpy as np
import copy

def interpolate_line(startPos, endPos, step_l):
    steps = []
    line = np.array(endPos) - np.array(startPos)
    line_l = np.linalg.norm(line)
    step = line / line_l * step_l
    n_steps = np.floor(line_l / step_l).astype(np.int32)
    c_step = np.array(startPos, dtype=float)
    for i in range(n_steps):
        steps.append(copy.deepcopy(c_step))
        c_step += step
    if not (c_step == np.array(endPos)).all():
        steps.append(np.array(endPos))
    return steps


class KinChainCollisionChecker(CollisionChecker):
    def __init__(self, kin_chain, scene, limits=[[-3.0, 3.0], [-3.0, 3.0]], statistic=None, fk_resolution=0.1):
        super(KinChainCollisionChecker, self).__init__(scene, limits, statistic)
        if len(limits) != kin_chain.dim:
            raise ValueError("Limits must match the dimension of the kinematic chain. Default values are for a 2-dof planar manipulator. If you use dof>2 you have to specify the limits explicitly")
        self.kin_chain = kin_chain
        self.fk_resolution = fk_resolution
        self.dim = self.kin_chain.dim

    def getDim(self):
        return self.dim
    
    def pointInCollision(self, pos):
        self.kin_chain.move(pos)
        joint_positions = self.kin_chain.get_transforms()
        for i in range(1, len(joint_positions)):
            if self.segmentInCollision(joint_positions[i-1], joint_positions[i]):
                return True
        return False
    
    def lineInCollision(self, startPos, endPos):
        assert (len(startPos) == self.getDim())
        assert (len(endPos) == self.getDim())
        steps = interpolate_line(startPos, endPos, self.fk_resolution)
        for pos in steps:
            if self.pointInCollision(pos):
                return True
        return False
    
    def segmentInCollision(self, startPos, endPos):
        for key, value in self.scene.items():
            if value.intersects(LineString([(startPos[0], startPos[1]), (endPos[0], endPos[1])])):
                return True
        return False
    
    def drawObstacles(self, ax, inWorkspace=False):
        if inWorkspace:
            for key, value in self.scene.items():
                plotting.plot_polygon(value, add_points=False, color='red', ax=ax)



def planarRobotVisualize(kin_chain, ax):
    joint_positions = kin_chain.get_transforms()
    for i in range(1, len(joint_positions)):
        xs = [joint_positions[i-1][0], joint_positions[i][0]]
        ys = [joint_positions[i-1][1], joint_positions[i][1]]
        ax.plot(xs, ys, color='g')
        
  
import matplotlib.pyplot as plt              
import matplotlib.animation
from IPython.display import HTML

matplotlib.rcParams['animation.embed_limit'] = 64
def animateSolution(planner, environment, benchmark, solution, visualizer, step, workSpaceLimits=[[-3,3],[-3,3]]):
    """
    Animates a robot's path found by a planner.
    
    For 2D environments, it creates a side-by-side animation of:
    1. The robot's movement in its physical workspace (left).
    2. Its progress along the pathfinding graph in configuration space (right).

    Parameters:
    - planner: The path planning object containing the graph.
    - environment: The environment object with obstacles and robot kinematics.
    - solution: A list of nodes from start to goal representing the path.
    - visualizer: A function to draw the planner's graph.
    - workSpaceLimits: Optional argument to define the plot boundaries.
    """
    _planner = planner
    _environment = environment
    _solution = solution
    _prmVisualizer = visualizer
    _step = step
    
    # --- Check if the environment is 2D to determine the plot layout ---
    if _environment.getDim() == 2:
    
        # 1. SETUP AND FIGURE INITIALIZATION
        # Create a wide figure and add two subplots, side-by-side.
        fig_local = plt.figure(figsize=(14, 7))
        # ax1 will show the robot's physical "workspace".
        ax1 = fig_local.add_subplot(1, 2, 1)
        # ax2 will show the planner's "configuration space" graph.
        ax2 = fig_local.add_subplot(1, 2, 2)


        # 2. PATH SMOOTHING (INTERPOLATION)
        # The planner's solution is just a few waypoints. To make the animation
        # smooth, we first extract the (x,y) coordinates for each waypoint.
        solution_pos = [_planner.graph.nodes[node]['pos'] for node in _solution]
        
        # Now, generate many small intermediate points between each waypoint.
        i_solution_pos = [solution_pos[0]]
        for i in range(1, len(solution_pos)):
            segment_s = solution_pos[i-1]
            segment_e = solution_pos[i]
            # The result is a long list of points for a fluid animation.
            i_solution_pos = i_solution_pos + interpolate_line(segment_s, segment_e, _step)[1:]

        # 3. THE ANIMATION LOOP
        # Get the total number of frames from our smoothed path.
        frames = len(i_solution_pos)
        # Get the robot's kinematic model from the environment.
        r = environment.kin_chain
        
        # This nested function draws every single frame of the animation.
        # Matplotlib will call it repeatedly, with `t` being the frame number.
        def animate(t):
            # --- Update the left plot (Workspace) ---
            ax1.cla()  # Clear the previous frame.
            ax1.set_xlim(workSpaceLimits[0]) # Reset the plot boundaries.
            ax1.set_ylim(workSpaceLimits[1])

            title = planner.plannerClass.__name__
            title += " " + benchmark.name
            ax1.set_title(title)

            _environment.drawObstacles(ax1, inWorkspace = True) # Redraw static obstacles.
            
            # Get the robot's position for the current frame `t`.
            pos = i_solution_pos[t]
            r.move(pos) # Update the robot's internal state (e.g., joint angles).
            planarRobotVisualize(r, ax1) # Draw the robot's physical shape.
        
            # --- Update the right plot (Configuration Space) ---
            ax2.cla() # Clear the previous frame.
            # Redraw the entire PRM graph and the final solution path.
            _prmVisualizer(_planner, solution, ax2)
            # Draw a large red dot on the graph to show the robot's current progress.
            ax2.scatter(i_solution_pos[t][0], i_solution_pos[t][1], color='r', zorder=10, s=250)

        # 4. GENERATING AND DISPLAYING THE ANIMATION
        # Create the animation object. This orchestrates the repeated calls to animate().
        ani = matplotlib.animation.FuncAnimation(fig_local, animate, frames=frames)
        # Convert the animation to JS/HTML for display in a Jupyter Notebook.
        html = HTML(ani.to_jshtml())
        display(html) # Show the animation.
        plt.close() # Prevent a static image from also being displayed.

    else:
        # This is a simplified version for non-2D environments.
        # It creates only one plot for the workspace.
        fig_local = plt.figure(figsize=(7, 7))
        ax1 = fig_local.add_subplot(1, 1, 1)
        
        # Get and interpolate the solution path, same as in the 2D case.
        solution_pos = [_planner.graph.nodes[node]['pos'] for node in _solution]
        i_solution_pos = [solution_pos[0]]
        for i in range(1, len(solution_pos)):
            segment_s = solution_pos[i-1]
            segment_e = solution_pos[i]
            i_solution_pos = i_solution_pos + interpolate_line(segment_s, segment_e, _step)[1:]
        
        frames = len(i_solution_pos)
        r = environment.kin_chain
        
        # The animate function here only updates the single workspace plot.
        def animate(t):
            ax1.cla()
            ax1.set_xlim(workSpaceLimits[0])
            ax1.set_ylim(workSpaceLimits[1])

            title = planner.plannerClass.__name__
            title += " " + benchmark.name
            ax1.set_title(title)

            # _environment.drawObstacles(ax1, inWorkspace = True)
            pos = i_solution_pos[t]
            r.move(pos)
            planarRobotVisualize(r, ax1)
        
        # Generate and display the animation.
        ani = matplotlib.animation.FuncAnimation(fig_local, animate, frames=frames)
        html = HTML(ani.to_jshtml())
        display(html)
        plt.close()