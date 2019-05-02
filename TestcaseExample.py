#* --------------------------------------------------------------------------
#  * File: Testcases.py
#  * ---------------------------------------------------------------------------
#  * Copyright (c) 2016 The Regents of the University of California.
#  * All rights reserved.
#  *
#  * Redistribution and use in source and binary forms, with or without
#  * modification, are permitted provided that the following conditions
#  * are met:
#  * 1. Redistributions of source code must retain the above copyright
#  *    notice, this list of conditions and the following disclaimer.
#  * 2. Redistributions in binary form must reproduce the above
#  *    copyright notice, this list of conditions and the following
#  *    disclaimer in the documentation and/or other materials provided
#  *    with the distribution.
#  * 3. All advertising materials mentioning features or use of this
#  *    software must display the following acknowledgement:
#  *       This product includes software developed by Cyber-Physical
#  *       Systems Lab at UCLA and UC Berkeley.
#  * 4. Neither the name of the University nor that of the Laboratory
#  *    may be used to endorse or promote products derived from this
#  *    software without specific prior written permission.
#  *
#  * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS''
#  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
#  * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
#  * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS
#  * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
#  * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#  * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
#  * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
#  * SUCH DAMAGE.
#  *
#  * Developed by: Yasser Shoukry
#  */

import argparse
from MultiRobotMotionPlanner import *


def motionPlanning_test1():
	"""Reach-Avoid motion planning testcase

	Two robots travelling through a cross intersection. Check Wiki for
	detailed explanation. 

	Arguments:
		None

	Returns:
		None 

	"""
    # Test case parameters
	numberOfRobots          = 2
	safetyLimit             = 0.5
	dwell                   = 20
	inputLimit              = 0.5
	maxHorizon              = 1000
	Ts                      = 0.2
	numberOfIntegrators     = 2

	# Workspace parameters
	nrows                   = 3
	ncols                   = 3
	xmin                    = 0.0
	xmax                    = 5.0
	ymin                    = 0.0
	ymax                    = 5.0

	# Obstacle regions (index starts from 0)
	obstacleRegions         = [True,False,True,False,False,False,True,False,True]

	A_square                = np.array([[-1.0, 0.0],
	                                    [1.0, 0.0],
	                                    [0.0, -1.0],
	                                    [0.0, 1.0]])

	# Region corners stores the dimension of each region in the map 
	regionCorners           = []

	# Sample map notation. 
	#   2   5   8
	#   1   4   7
	#   0   3   6
	regionCorners.append({'xmin': 0.0, 'xmax': 1.0, 'ymin': 0.0, 'ymax': 2.0})      # region 0
	regionCorners.append({'xmin': 0.0, 'xmax': 1.0, 'ymin': 2.0, 'ymax': 3.0})      # region 1
	regionCorners.append({'xmin': 0.0, 'xmax': 1.0, 'ymin': 3.0, 'ymax': 5.0})      # region 2

	regionCorners.append({'xmin': 1.0, 'xmax': 3.0, 'ymin': 0.0, 'ymax': 2.0})      # region 3
	regionCorners.append({'xmin': 1.0, 'xmax': 3.0, 'ymin': 2.0, 'ymax': 3.0})      # region 4
	regionCorners.append({'xmin': 1.0, 'xmax': 3.0, 'ymin': 3.0, 'ymax': 5.0})      # region 5

	regionCorners.append({'xmin': 3.0, 'xmax': 5.0, 'ymin': 0.0, 'ymax': 2.0})      # region 6
	regionCorners.append({'xmin': 3.0, 'xmax': 5.0, 'ymin': 2.0, 'ymax': 3.0})      # region 7
	regionCorners.append({'xmin': 3.0, 'xmax': 5.0, 'ymin': 3.0, 'ymax': 5.0})      # region 8

	# Define adjacent regions 
	adjacents                = []
	adjacents.append([0, 1, 3])
	adjacents.append([1, 0, 2, 4])
	adjacents.append([2, 1, 5])
	adjacents.append([3, 0, 4, 6])
	adjacents.append([4, 1, 3, 5, 7])
	adjacents.append([5, 2, 4, 8])
	adjacents.append([6, 3, 7])
	adjacents.append([7, 4, 6, 8])
	adjacents.append([8, 5, 7])

	regions = []    
	for counter in range(0,nrows*ncols):
	    b = np.array([-1 * regionCorners[counter]['xmin'], regionCorners[counter]['xmax'],
	                  -1 * regionCorners[counter]['ymin'], regionCorners[counter]['ymax']])
	    regions.append({'A': A_square, 'b':b, 'isObstacle': obstacleRegions[counter], 'adjacents':adjacents[counter]})

	workspace = {'xmin': xmin, 'xmax': xmax, 'ymin': ymin, 'ymax': ymax, 'regions':regions}

	# Add robot initial state ie. position
	robotsInitialState      = []
	robotsInitialState.append({'x0': 0.0, 'y0': 2.5, 'region': 1})   # first robot starts at (2.5,0)
	robotsInitialState.append({'x0': 2.0, 'y0': 0.5, 'region': 3})   # second robot starts at (0,2.5)

	# Add robot goal state ie. position
	robotsGoalState          = []
	robotsGoalState.append({'xf': 4.0, 'yf': 2.5, 'region': 7})      # first robot goal is (2.5, 5.0)
	robotsGoalState.append({'xf': 2.0, 'yf': 4.5, 'region': 5})      # second robot goal is (2.5, 5.0)

	# Add more constraints for each robot
	inputConstraints        = []
	inputConstraints.append({'uxMax': inputLimit, 'uxMin': -1 * inputLimit,
	                         'uyMax': inputLimit, 'uyMin': -1 * inputLimit})   # input constraints for the first robot
	inputConstraints.append({'uxMax': inputLimit, 'uxMin': -1 * inputLimit,
	                         'uyMax': inputLimit, 'uyMin': -1 * inputLimit})    # input constraints for the second robot

	if (len(adjacents) == len(regionCorners) == len(obstacleRegions) == nrows*ncols):
	    pass
	else:
	    print("Number of adjacent regions, region corners, obstacle regions do not match.")
	    exit()

	start                   = timeit.default_timer()
	for horizon in range(3,maxHorizon):
	    print '\n=============================================='
	    print '         Horizon = ', horizon
	    print '==============================================\n'
	    solver = MultiRobotMotionPlanner(horizon, numberOfRobots, workspace, numberOfIntegrators)
	    robotsTrajectory, loopIndex, counter_examples = solver.solve(
	        robotsInitialState, robotsGoalState, inputConstraints, Ts, safetyLimit, dwell)
	    
	    if robotsTrajectory:
	        break


	end = timeit.default_timer()
	time_smt = end - start
	print 'Exuection time = ', time_smt
	print 'Number of Robots = ', numberOfRobots
	print 'Safety Limit = ', safetyLimit
	print 'Trajectory length = ', len(robotsTrajectory[0]['x'])

	__animateTrajectories(robotsTrajectory, loopIndex, safetyLimit, workspace)


def motionPlanning_test2():
	"""LTL motion planning testcase

	Single robot moving in a complex environment. 

	Arguments:
		None

	Returns:
		None 

	"""
	# Testcase parameters
	maxHorizon = 1000
	numberOfRobots  = 1
	safetyLimit     = 0.5
	dwell           = 4
	inputLimit      = 0.2
	Ts              = 0.5
	numberOfIntegrators = 4


	A_square = np.array([[-1.0, 0.0],
	                     [1.0, 0.0],
	                     [0.0, -1.0],
	                     [0.0, 1.0]])

	# Region corners stores the dimension of each region in the map 
	regionCorners = []

	regionCorners.append({'xmin': 0.0, 'xmax': 1.0, 'ymin': 0.0, 'ymax': 2.5})  # region 0
	regionCorners.append({'xmin': 0.0, 'xmax': 1.0, 'ymin': 2.5, 'ymax': 3.5})  # region 1
	regionCorners.append({'xmin': 0.0, 'xmax': 1.5, 'ymin': 3.5, 'ymax': 4.0})  # region 2
	regionCorners.append({'xmin': 0.0, 'xmax': 1.5, 'ymin': 4.0, 'ymax': 6.0})  # region 3

	regionCorners.append({'xmin': 1.0, 'xmax': 1.5, 'ymin': 0.0, 'ymax': 2.5})  # region 4
	regionCorners.append({'xmin': 1.0, 'xmax': 1.5, 'ymin': 2.5, 'ymax': 3.5})  # region 5


	regionCorners.append({'xmin': 1.5, 'xmax': 3.0, 'ymin': 0.0, 'ymax': 0.5})  # region 6
	regionCorners.append({'xmin': 1.5, 'xmax': 3.0, 'ymin': 0.5, 'ymax': 2.5})  # region 7
	regionCorners.append({'xmin': 1.5, 'xmax': 3.0, 'ymin': 2.5, 'ymax': 3.5})  # region 8
	regionCorners.append({'xmin': 1.5, 'xmax': 2.0, 'ymin': 3.5, 'ymax': 4.0})  # region 9
	regionCorners.append({'xmin': 2.0, 'xmax': 2.5, 'ymin': 3.5, 'ymax': 4.0})  # region 10
	regionCorners.append({'xmin': 2.5, 'xmax': 3.0, 'ymin': 3.5, 'ymax': 4.0})  # region 11
	regionCorners.append({'xmin': 1.5, 'xmax': 2.0, 'ymin': 4.0, 'ymax': 6.0})  # region 12
	regionCorners.append({'xmin': 2.0, 'xmax': 2.5, 'ymin': 4.0, 'ymax': 6.0})  # region 13
	regionCorners.append({'xmin': 2.5, 'xmax': 3.0, 'ymin': 4.0, 'ymax': 5.5})  # region 14
	regionCorners.append({'xmin': 2.5, 'xmax': 3.0, 'ymin': 5.5, 'ymax': 6.0})  # region 15

	regionCorners.append({'xmin': 3.0, 'xmax': 3.5, 'ymin': 0.0, 'ymax': 0.5})  # region 16
	regionCorners.append({'xmin': 3.0, 'xmax': 3.5, 'ymin': 0.5, 'ymax': 5.5})  # region 17
	regionCorners.append({'xmin': 3.0, 'xmax': 3.5, 'ymin': 5.5, 'ymax': 6.0})  # region 18

	regionCorners.append({'xmin': 3.5, 'xmax': 6.0, 'ymin': 0.0, 'ymax': 0.5})  # region 19
	regionCorners.append({'xmin': 3.5, 'xmax': 4.0, 'ymin': 0.5, 'ymax': 2.5})  # region 20
	regionCorners.append({'xmin': 4.0, 'xmax': 6.0, 'ymin': 0.5, 'ymax': 2.5})  # region 21
	regionCorners.append({'xmin': 3.5, 'xmax': 4.0, 'ymin': 2.5, 'ymax': 3.0})  # region 22
	regionCorners.append({'xmin': 4.0, 'xmax': 6.0, 'ymin': 2.5, 'ymax': 3.0})  # region 23
	regionCorners.append({'xmin': 3.5, 'xmax': 4.0, 'ymin': 3.0, 'ymax': 5.5})  # region 24
	regionCorners.append({'xmin': 4.0, 'xmax': 6.0, 'ymin': 3.0, 'ymax': 6.0})  # region 25
	regionCorners.append({'xmin': 3.5, 'xmax': 4.0, 'ymin': 5.5, 'ymax': 6.0})  # region 26

	# Define adjacent regions 
	adjacents = []
	adjacents.append([0, 1, 4])                     # region 0
	adjacents.append([1, 0, 2, 5])                  # region 1
	adjacents.append([2, 1, 3, 5, 9])               # region 2
	adjacents.append([3, 2, 12])                    # region 3
	adjacents.append([4, 0, 6, 7, 5])               # region 4
	adjacents.append([5, 1, 8, 4, 2])               # region 5
	adjacents.append([6, 4, 16, 7])                 # region 6
	adjacents.append([7, 6, 8, 4, 17])              # region 7
	adjacents.append([8, 5, 17, 7, 9, 10, 11])      # region 8

	adjacents.append([9, 2, 10, 12])                # region 9
	adjacents.append([10, 9, 8, 13, 11])            # region 10
	adjacents.append([11, 8, 14, 17])               # region 11
	adjacents.append([12, 3, 9, 13])                # region 12
	adjacents.append([13, 12, 15, 14, 10])          # region 13
	adjacents.append([14, 15, 11, 13, 17])          # region 14
	adjacents.append([15, 14, 13, 18])              # region 15
	adjacents.append([16, 6, 19, 17])               # region 16
	adjacents.append([17, 7, 16, 8, 11, 14, 16, 24, 22, 20])  # region 17
	adjacents.append([18, 15, 17, 26])              # region 18
	adjacents.append([19, 16, 20, 21])              # region 19
	adjacents.append([20, 17, 22, 21, 19])          # region 20
	adjacents.append([21, 20, 19, 23])              # region 21
	adjacents.append([22, 24, 23, 20, 17])          # region 22
	adjacents.append([23, 25, 21, 22])              # region 23
	adjacents.append([24, 25, 26, 17, 20])          # region 24
	adjacents.append([25, 26, 24, 23])              # region 25
	adjacents.append([26, 18, 24, 25])              # region 26

	regions = []
	numberOfRegions     = 27
	obstacleREgionIndex = [2, 4, 13, 17, 23]
	obstacleRegions     = [False] * numberOfRegions
	for index in obstacleREgionIndex:
	    obstacleRegions[index] = True

	for counter in range(0, numberOfRegions):
	    b = np.array([-1 * regionCorners[counter]['xmin'], regionCorners[counter]['xmax'],
	                  -1 * regionCorners[counter]['ymin'], regionCorners[counter]['ymax']])
	    regions.append({'A': A_square, 'b': b, 'isObstacle': obstacleRegions[counter], 'adjacents': adjacents[counter]})


	workspace = {'xmin': 0.0, 'xmax': 6.0, 'ymin': 0.0, 'ymax': 6.0, 'regions': regions}

	# Add robot initial state ie. position
	robotsInitialState      = []
	robotsInitialState.append({'x0': 0.5, 'y0': 0.5, 'region':0})   # first robot starts at (2.5,0)
	#robotsInitialState.append({'x0': 5.0, 'y0': 2.5})   # second robot starts at (0,2.5)
	#robotsInitialState.append({'x0': 2.0, 'y0': 5.0})   # second robot starts at (0,2.5)
	#robotsInitialState.append({'x0': 2.0, 'y0': 0.0})  # second robot starts at (0,2.5)


	# Add robot goal state ie. position
	robotsGoalState          = []
	#robotsGoalState.append({'xf': 5.5, 'yf': 2.0, 'region': 21})      # first robot goal is (2.5, 5.0)
	#robotsGoalState.append({'xf': 0.0, 'yf': 2.5})      # second robot goal is (2.5, 5.0)
	#robotsGoalState.append({'xf': 2.0, 'yf': 0.0})  # second robot goal is (2.5, 5.0)
	#robotsGoalState.append({'xf': 2.0, 'yf': 5.0})  # second robot goal is (2.5, 5.0)


	inputConstraints = []
	inputConstraints.append({'uxMax': inputLimit, 'uxMin': -1 * inputLimit,
	                         'uyMax': inputLimit, 'uyMin': -1 * inputLimit})  # input constraints for the first robot
	inputConstraints.append({'uxMax': inputLimit, 'uxMin': -1 * inputLimit,
	                         'uyMax': inputLimit, 'uyMin': -1 * inputLimit})  # input constraints for the second robot
	inputConstraints.append({'uxMax': inputLimit, 'uxMin': -1 * inputLimit,
	                         'uyMax': inputLimit, 'uyMin': -1 * inputLimit})  # input constraints for the second robot
	inputConstraints.append({'uxMax': inputLimit, 'uxMin': -1 * inputLimit,
	                         'uyMax': inputLimit, 'uyMin': -1 * inputLimit})  # input constraints for the second robot


	'''
	# compute best dwell time
	maxFacet = 0
	for counter in range(0, numberOfRegions):
	    if obstacleRegions[counter] == False:
	        length = regionCorners[counter]['xmax'] - regionCorners[counter]['xmin']
	        if maxFacet < length:
	            maxFacet = length

	        width = regionCorners[counter]['ymax'] - regionCorners[counter]['ymin']
	        if maxFacet < width:
	            maxFacet = width


	dwell = int(0.1*maxFacet/(Ts * inputLimit))
	if dwell == 0:
	    dwell = 1

	print "dwell = ", dwell
	'''
	start = timeit.default_timer()
	for horizon in range(30, maxHorizon):
	    print '\n=============================================='
	    print '         Horizon = ', horizon
	    print '==============================================\n'
	    solver = MultiRobotMotionPlanner(horizon, numberOfRobots, workspace, numberOfIntegrators)


	    # At least one robot out of robots (0,1,2,3) to be at region 1
	    # r0 + r1 + r2 + r3 \ge 1
	    prop1 = solver.createAtomicProposition(21, [0], 'E', 1)
	    prop2 = solver.createAtomicProposition(3, [0], 'E', 1)
	    prop3 = solver.createAtomicProposition(25, [0], 'E', 1)


	    # Eventuality:
	    Eprop1 = solver.createCompoundProposition(prop1, [], 'E')
	    Eprop2 = solver.createCompoundProposition(prop2, [], 'E')
	    Eprop3 = solver.createCompoundProposition(prop3, [], 'E')

	    # AND
	    Eprop1AndEprop2 = solver.createCompoundProposition(Eprop1, Eprop2, 'AND')
	    Eprop1AndEprop2ANDEprop3 = solver.createCompoundProposition(Eprop1AndEprop2, Eprop3, 'AND')

	    solver.createLTLFormula(Eprop1AndEprop2ANDEprop3)


	    '''
	    # NOT prop1
	    NOTprop1 = solver.createCompoundProposition(prop1, [], 'NOT')

	    # Eventually (NOT prop1)
	    ENOTprop1 = solver.createCompoundProposition(NOTprop1, [], 'E')

	    # Globally Eventually (NOT prop1)
	    GENOTprop1 = solver.createCompoundProposition(ENOTprop1, [], 'G')

	    # Globally Eventually (prop1 AND prop2) AND Globally Eventually (NOT prop1)
	    GEprop1Andprop2ANDGENOTprop1 = solver.createCompoundProposition(GEprop1Andprop2, GENOTprop1, 'AND')
	    solver.createLTLFormula(GEprop1Andprop2ANDGENOTprop1)
	    '''

	    robotsTrajectory, loopIndex, counter_examples = solver.solve(
	        robotsInitialState, robotsGoalState, inputConstraints, Ts, safetyLimit, dwell
	    )

	    if robotsTrajectory:
	        break

	end                     = timeit.default_timer()
	time_smt                = end - start
	print 'Exuection time = ', time_smt
	print 'Number of Robots = ', numberOfRobots
	print 'Safety Limit = ', safetyLimit
	print 'Trajectory length = ', len(robotsTrajectory[0]['x'])

	__animateTrajectories(robotsTrajectory, loopIndex, safetyLimit, workspace)

	# [0, 0, 0, 0, 1, 5, 8, 7, 7, 6, 16, 19, 21, 21, 21]


def __animateTrajectories(robotsTrajectory, loopIndex, safetyLimit, workspace):
	"""Plots the trajectory and robots in the workspace
	
	Arguments:
		robotsTrajectory: Trajectory of the robots.
		loopIndex: Loop index for LTL case.
		safetyLimit: Safety limit between any two robots.
		workspace: Workspace which has the obstacles and free space information.
	
	Returns:
		None
	"""
	numberOfRobots      = len(robotsTrajectory)
	colors              = np.random.random((numberOfRobots, 3))

	if loopIndex > 0:
	    numberOfLoops = 5
	    loopPoints = range(loopIndex + 1, len(robotsTrajectory[0]['x']))

	    for loop in range(0, numberOfLoops):
	        for robotIndex in range(0, numberOfRobots):
	            robotsTrajectory[robotIndex]['x'] += [robotsTrajectory[robotIndex]['x'][i] for i in loopPoints]
	            robotsTrajectory[robotIndex]['y'] += [robotsTrajectory[robotIndex]['y'][i] for i in loopPoints]


	# Animate the trajectory
	fig                 = plt.figure(figsize=(7, 7))
	titleText           = 'Number of Robots = %d, Safety limit = %s' % (numberOfRobots, safetyLimit)
	plt.title(titleText)

	ax = fig.add_subplot(111, autoscale_on=False, xlim=(workspace['xmin'], workspace['xmax']), ylim=(workspace['ymin'], workspace['ymax']))
	ax.grid()
	ax.set_xlim(workspace['xmin'], workspace['xmax']), ax.set_xticks([])
	ax.set_ylim(workspace['ymin'], workspace['ymax']), ax.set_yticks([])

	def animationUpdate(framenumber):
	    thisx               = []
	    thisy               = []
	    trajX				= np.zeros((framenumber,numberOfRobots))
	    trajY 				= np.zeros((framenumber,numberOfRobots))

	    ax.clear()
	    for region in workspace['regions']:
	        if region['isObstacle']:
	            xmin    = -1*region['b'][0]
	            xmax    = region['b'][1]
	            ymin    = -1*region['b'][2]
	            ymax    = region['b'][3]
	            height  = ymax - ymin
	            width   = xmax - xmin
	            # Add obstacle patches
	            ax.add_patch(patches.Rectangle((xmin, ymin), width, height))

	    for robotIndex in range(0, numberOfRobots):
	        thisx.append(robotsTrajectory[robotIndex]['x'][framenumber])
	        thisy.append(robotsTrajectory[robotIndex]['y'][framenumber])

	    # Plot trajectory
	    if show_trajectory:
	        for robotIndex in range(0, numberOfRobots):
	        	for frameNumberIter in range(0, framenumber):
	        		trajX[frameNumberIter, robotIndex] = robotsTrajectory[robotIndex]['x'][frameNumberIter]
	        		trajY[frameNumberIter, robotIndex] = robotsTrajectory[robotIndex]['y'][frameNumberIter]

	        	ax.scatter(trajX[:,robotIndex], trajY[:,robotIndex], c=colors[robotIndex], s=2)

	    # Plot robots
	    ax.scatter(thisx, thisy, c=colors, s=200)
	    ax.set_xlim(workspace['xmin'], workspace['xmax']), ax.set_xticks([])
	    ax.set_ylim(workspace['ymin'], workspace['ymax']), ax.set_yticks([])

	animation = FuncAnimation(fig, animationUpdate, np.arange(1, len(robotsTrajectory[0]['x'])), interval=50)

	# Save animation
	if save_animation:
		animation.save(animation_name)

	plt.show()


if __name__ == "__main__":
	Parser = argparse.ArgumentParser()
	Parser.add_argument('--mode', default=0, help='0: Reach-Avoid, 1: LTL')
	Parser.add_argument('--save_animation',  type=bool, default=False, help='Save Animation. Default False.')
	Parser.add_argument('--show_trajectory',  type=bool, default=False, help='Show robot trajectory. Default True.')
	Parser.add_argument('--animation_name', default="animation.mp4", help='Animation video name')

	Args = Parser.parse_args()
	mode = Args.mode
	save_animation = Args.save_animation
	animation_name = Args.animation_name
	show_trajectory = Args.show_trajectory

	np.random.seed(0)
	if mode == 0:
		# Reach Avoid 
		motionPlanning_test1()
	else:
		# LTL
		motionPlanning_test2()


