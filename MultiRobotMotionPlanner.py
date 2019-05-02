#  * --------------------------------------------------------------------------
#  * File: MultiRobotMotionPlanner.py
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

import os
from solver import SMConvexSolver
import timeit
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import numpy as np

# TODO: Fix the PREFIX based counterexample

# TODO: add state constraints

#***************************************************************************************************
#***************************************************************************************************
#
#         CLASS SMConvexSolver
#
#***************************************************************************************************
#***************************************************************************************************

class MultiRobotMotionPlanner:
    # ========================================================
    #       Constructor
    # ========================================================
    def __init__(self, horizon, numberOfRobots, workspace, numberOfIntegrators):
        print ''
        self.horizon                    = horizon
        self.numberOfLTLBoolVars        = 0
        self.LTL                        = []
        self.numberOfRobots             = numberOfRobots

        self.workspace                  = workspace
        regions                         = workspace['regions']
        self.numberOfRegions            = len(regions)
        self.topLevelFormula            = []
        self.numberOfIntegrators        = numberOfIntegrators


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         Solve For A Particular Horizon
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def solve(self, robotsInitialState, robotsGoalState, inputConstraints, Ts,
              safetyLimit, dwell):

        numberOfSafetyConstraints = 0
        for robotIndex in range(0, self.numberOfRobots):
            for robotIndex2 in range(robotIndex+1, self.numberOfRobots):
                numberOfSafetyConstraints += 4

        numberOfStates          = 2 * self.numberOfIntegrators
        numberOfInputs          = 2
        numberOfBoolVars        = self.numberOfLTLBoolVars
        numOfRealVars           = (numberOfStates + 2 * numberOfInputs) * self.horizon * dwell * self.numberOfRobots

        robots, numOfConvIFClauses = self.__createsRobotDataStructure(self.numberOfRobots, dwell, self.horizon, numberOfStates, numberOfInputs, self.workspace)

        solver                  = SMConvexSolver.SMConvexSolver(numberOfBoolVars, numOfRealVars, numOfConvIFClauses,
                                    maxNumberOfIterations=10000,
                                    verbose='OFF',
                                    profiling='false',
                                    numberOfCores=8,
                                    counterExampleStrategy='IIS',
                                    slackTolerance=1E-3)

        # Add initial state constraints
        start = timeit.default_timer()
        self.__addInitialStateConstraints(solver, self.numberOfRobots, robots, robotsInitialState)

        # Add Goal Constraints
        start2 = timeit.default_timer()
        if not self.LTL:
            self.__addGoalStateConstraints(solver, self.numberOfRobots, robots, robotsGoalState)
        else:
            self.__LTLParser(solver, robots)

        # Add Workspace constraints
        start3 = timeit.default_timer()
        self.__addWorkspaceConstraints(solver, self.numberOfRobots, robots, dwell, self.horizon, self.workspace)

        # Add dynamics Constraints
        start4 = timeit.default_timer()
        self.__addDynamicsConstraints(solver, self.numberOfRobots, robots, dwell, self.horizon, Ts)

        # Add input constraints
        self.__addInputConstraints(solver, self.numberOfRobots, robots, dwell, self.horizon, inputConstraints)

        # Add Safety Constraints
        start5 = timeit.default_timer()
        self.__addSafetyConstraints(solver, self.numberOfRobots, robots, dwell, self.horizon, safetyLimit, self.workspace)

        end = timeit.default_timer()

        print 'Feeding constraints time = ', end - start, start2 - start, start3 - start2, start4 - start3, start5 - start4, end - start5

        '''
        # Add previous Counter Examples
        for counterExample in previousCounterExamples:
            if not counterExample:
                continue
            constraint                  = [ SMConvexSolver.NOT(solver.convIFClauses[counter]) for counter in counterExample ]
            solver.addBoolConstraint(SMConvexSolver.OR(*constraint))
        '''

        stateTraj, booleanTraj, convIFModel = solver.solve()
        counter_examples = solver.counterExamples

        '''
        for counterExample in counter_examples:
            ceStr       = 'CE: '
            for ceElement in counterExample:
                for horizonIndex in range(0, self.horizon):
                    convIFList = robots[0][horizonIndex]['regionsConstriantIndex']
                    if ceElement in convIFList:
                        ceStr = ceStr + str(convIFList.index(ceElement)) + ', '
                        break
            print ceStr

        print 'counterExamples', counter_examples
        '''

        activeIFClauses = [i for i, x in enumerate(convIFModel) if x == True]

        if not stateTraj:
            return [], 0, counter_examples

        robotsTraj                      = []
        for robotIndex in range(0, self.numberOfRobots):
            xTraj                       = []
            yTraj                       = []
            uxTraj                      = []
            uyTraj                      = []
            regionTraj                  = []
            for horizonIndex in range(0, self.horizon):
                regionTraj.append(activeIFClauses[horizonIndex] - horizonIndex * self.numberOfRegions)
                for dwellIndex in range(0, dwell):
                    xTraj.append(stateTraj[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['x']])
                    yTraj.append(stateTraj[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['y']])
                    uxTraj.append(stateTraj[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['ux']])
                    uyTraj.append(stateTraj[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['uy']])
            trajectory                  = {'x': xTraj, 'y': yTraj, 'ux': uxTraj, 'uy': uyTraj}
            robotsTraj.append(trajectory)
            print '\n======== Trajectory for Robot #', robotIndex, ' =================='
            print xTraj
            print yTraj
            print uxTraj
            print uyTraj
            print 'RegionTraj', regionTraj



        if self.LTL:
            #if self.topLevelFormula['loopExists']:
            startLoopPointers   = [booleanTraj[i] for i in self.topLevelFormula['loopStart']]
            loopIndexTrue       = [i for i, x in enumerate(startLoopPointers) if x == True]
            loopIndex           = loopIndexTrue[0] * dwell
        else:
            loopIndex           = -1
        return robotsTraj, loopIndex, counter_examples


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         Create "Robots" Data structure
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def __createsRobotDataStructure(self, numberOfRobots, dwell, horizon, numberOfStates, numberOfInputs, workspace):
        robots                          = []
        robotStateShiftIndex            = 0
        convexIFShiftIndex              = 0     # safety and region constraints
        regions                         = workspace['regions']
        numberOfRegions                 = len(regions)

        # ---------- STATE VAR INDEX ----------------------------------------------------------------
        for robotIndex in range(0, numberOfRobots):
            robots.append([])
            for horizonIndex in range(0, horizon):
                robots[robotIndex].append([])
                statesVarIndex                      = []

                safetyConstraints                   = []
                regionsConstraints                  = []

                for dwellIndex in range(0, dwell):
                    indexInputX                     = 0 + robotStateShiftIndex
                    indexInputY                     = 1 + robotStateShiftIndex
                    indexInputNormX                 = 2 + robotStateShiftIndex
                    indexInputNormY                 = 3 + robotStateShiftIndex
                    U                               = [indexInputX, indexInputY]

                    indexX                          = 4 + robotStateShiftIndex
                    indexY                          = 5 + robotStateShiftIndex
                    derivativeShift                 = 6 + robotStateShiftIndex
                    derivativesX                    = range(derivativeShift, derivativeShift + self.numberOfIntegrators -1)

                    derivativeShift                 = derivativesX[-1] + 1
                    #print 'derivativeShift', derivativeShift
                    derivativesY                    = range(derivativeShift, derivativeShift + self.numberOfIntegrators - 1)
                    #indexXDot                       = 6 + robotStateShiftIndex
                    #indexYDot                       = 7 + robotStateShiftIndex

                    states                          = [indexX, indexY] + derivativesX + derivativesY

                    #print states

                    indecies                        = { 'ux': indexInputX,
                                                        'uy': indexInputY,
                                                        'inputs': U,
                                                        'uxNorm': indexInputNormX,
                                                        'uyNorm': indexInputNormY,
                                                        'x': indexX,
                                                        #'vx': indexXDot,
                                                        'y': indexY,
                                                        #'vy': indexYDot,
                                                        'states': states,
                                                        'derivatives': derivativesX + derivativesY,
                                                        'integratorChainX': [indexX] + derivativesX,
                                                        'integratorChainY': [indexY] + derivativesY
                                                    }
                    robotStateShiftIndex = robotStateShiftIndex + (numberOfStates + 2 * numberOfInputs)
                    statesVarIndex.append(indecies)
                    #robots[robotIndex][horizonIndex].append(statesVarIndex)


                # Add indices for safety constraints (only for robots with index greater than current index)
                for robotIndex2 in range(0, numberOfRobots):
                    if robotIndex2 <= robotIndex:
                        safetyConstraints.append([])
                    else:
                        safetyConstraints.append(range(convexIFShiftIndex, convexIFShiftIndex+4))
                        convexIFShiftIndex              += 4

                # Add indices for region constraints
                regionsConstraints              = range(convexIFShiftIndex, convexIFShiftIndex + numberOfRegions)
                convexIFShiftIndex              += numberOfRegions

                if self.LTL and horizonIndex > 0:
                    consistencyConstraints          = convexIFShiftIndex
                    convexIFShiftIndex              = convexIFShiftIndex + 1
                else:
                    consistencyConstraints          = []

                robots[robotIndex][horizonIndex] = {'stateIndex': statesVarIndex,
                                                    'safetyConstraintsIndex': safetyConstraints,
                                                    'regionsConstriantIndex': regionsConstraints,
                                                    'consistencyConstraintIndex': consistencyConstraints
                                                    }


        #robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]
        return robots, convexIFShiftIndex


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         Add Initial Constraints
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def __addInitialStateConstraints(self, solver, numberOfRobots, robots, robotsInitialState):
        for robotIndex in range(0, numberOfRobots):
            initialStateConstraint = SMConvexSolver.LPClause(np.array([[1.0]]), [robotsInitialState[robotIndex]['x0']],
                                                             [solver.rVars[robots[robotIndex][0]['stateIndex'][0]['x']]],
                                                             sense="E")
            solver.addConvConstraint(initialStateConstraint)

            initialStateConstraint = SMConvexSolver.LPClause(np.array([[1.0]]), [robotsInitialState[robotIndex]['y0']],
                                                         [solver.rVars[robots[robotIndex][0]['stateIndex'][0]['y']]],
                                                         sense="E")
            solver.addConvConstraint(initialStateConstraint)

            # all higher derivatives are set to zero
            derivatives            = robots[robotIndex][0]['stateIndex'][0]['derivatives']
            for derivative in derivatives:
                initialStateConstraint = SMConvexSolver.LPClause(np.array([[1.0]]), [0.0],[solver.rVars[derivative]],sense="E")
                solver.addConvConstraint(initialStateConstraint)

            # region constraint
            initialRegion = robotsInitialState[robotIndex]['region']
            solver.addBoolConstraint(
                solver.convIFClauses[robots[robotIndex][0]['regionsConstriantIndex'][initialRegion]])


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         Add Goal Constraints
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def __addGoalStateConstraints(self, solver, numberOfRobots, robots, robotsGoalState):
        for robotIndex in range(0, numberOfRobots):
            goalStateConstraint = SMConvexSolver.LPClause(np.array([[1.0]]), [robotsGoalState[robotIndex]['xf']],
                                                             [solver.rVars[robots[robotIndex][-1]['stateIndex'][-1]['x']]],
                                                             sense="E")
            solver.addConvConstraint(goalStateConstraint)

            goalStateConstraint = SMConvexSolver.LPClause(np.array([[1.0]]), [robotsGoalState[robotIndex]['yf']],
                                                         [solver.rVars[robots[robotIndex][-1]['stateIndex'][-1]['y']]],
                                                         sense="E")
            solver.addConvConstraint(goalStateConstraint)

            # all higher derivatives are set to zero
            derivatives = robots[robotIndex][-1]['stateIndex'][-1]['derivatives']
            for derivative in derivatives:
                goalStateConstraint = SMConvexSolver.LPClause(np.array([[1.0]]), [0.0], [solver.rVars[derivative]],
                                                                 sense="E")
                solver.addConvConstraint(goalStateConstraint)

            # region constraint
            goalRegion = robotsGoalState[robotIndex]['region']
            solver.addBoolConstraint(
                solver.convIFClauses[robots[robotIndex][-1]['regionsConstriantIndex'][goalRegion]]
            )


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         Add Workspace Constraints
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def __addWorkspaceConstraints(self, solver, numberOfRobots, robots, dwell, horizon, workspace):
        # if no regions are specified, then put only workspace constraints
        if not workspace['regions']:
            for robotIndex in range(0, numberOfRobots):
                for horizonIndex in range(0, horizon):
                    for dwellIndex in range(0, dwell):
                        workspaceConstraint = SMConvexSolver.LPClause(np.array([[1]]), [workspace['xmin']],
                                                                      [solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['x']]],
                                                                      sense="G")
                        solver.addConvConstraint(workspaceConstraint)

                        workspaceConstraint = SMConvexSolver.LPClause(np.array([[1]]), [workspace['xmax']],
                                                                      [solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['x']]],
                                                                      sense="L")
                        solver.addConvConstraint(workspaceConstraint)

                        workspaceConstraint = SMConvexSolver.LPClause(np.array([[1]]), [workspace['ymin']],
                                                                      [solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['y']]],
                                                                      sense="G")
                        solver.addConvConstraint(workspaceConstraint)

                        workspaceConstraint = SMConvexSolver.LPClause(np.array([[1]]), [workspace['ymax']],
                                                                      [solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['y']]],
                                                                      sense="L")
                        solver.addConvConstraint(workspaceConstraint)

        else:
            regions                         = workspace['regions']
            numberOfRegions                 = len(regions)
            for robotIndex in range(0, numberOfRobots):
                for horizonIndex in range(0, horizon):
                    for regionIndex in range(0, numberOfRegions):
                        rVars               = []

                        # create the A matrix by rolling forward the regions[regionIndex]['A']
                        A_region            = regions[regionIndex]['A']
                        A                   = np.zeros([dwell * np.shape(A_region)[0], dwell * np.shape(A_region)[1]])
                        b                   = []

                        rowIndex            = 0
                        columnIndex         = 0
                        for dwellIndex in range(0, dwell):
                            rVars.append(solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['x']])
                            rVars.append(solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['y']])
                            rows        = range(rowIndex, rowIndex + np.shape(A_region)[0])
                            columns     = range(columnIndex, columnIndex + np.shape(A_region)[1])
                            for rowCounter in range(0, len(rows)):
                                A[rows[rowCounter],:][columns] = A_region[rowCounter,:]
                                b.append(regions[regionIndex]['b'][rowCounter])
                            rowIndex    += np.shape(A_region)[0]
                            columnIndex += np.shape(A_region)[1]


                        regionConstraint    = SMConvexSolver.LPClause(
                                                A,
                                                b,
                                                rVars,
                                                sense="L")

                        regionsCosntraintsIndex = robots[robotIndex][horizonIndex]['regionsConstriantIndex'][regionIndex]
                        solver.setConvIFClause(regionConstraint, regionsCosntraintsIndex)


                        # Rule #1: You can not visit an obstacle region
                        if regions[regionIndex]['isObstacle']:
                            solver.addBoolConstraint(SMConvexSolver.NOT(solver.convIFClauses[regionsCosntraintsIndex]))

                        """
                        Rule #2: You can move only through adjcaent regions:
                        unroll the state machine representing the workspace adjacency.
                           
                        For the $i$ th partition, generate the follwoing clause
                        P_{i,k} => P_{adj1,k+1} OR P_{adj2,k+1} OR ... P_{adjn,k+1}
                        """
                        if horizonIndex < horizon - 1:
                            adjacents                   = regions[regionIndex]['adjacents']
                            nextRegionsCosntraintsIndex = [robots[robotIndex][horizonIndex + 1]['regionsConstriantIndex'][i] for i in adjacents]
                            antecedent                  = solver.convIFClauses[regionsCosntraintsIndex]
                            consequent                  = [solver.convIFClauses[i] for i in nextRegionsCosntraintsIndex]
                            solver.addBoolConstraint(
                                SMConvexSolver.IMPLIES(antecedent, SMConvexSolver.OR(*consequent)))

                    
                    """
                    # ONLY ONE REGION AT A TIME #
                      for the $k$th time step , generate clause ensuring that only one partition is active
                      P_{1,k} + P_{2,k} + P_{p,k} = 1
                      where $p$ = number of partitions
                    """
                    solver.addBoolConstraint(
                        sum([SMConvexSolver.BoolVar2Int(solver.convIFClauses[i]) for i in robots[robotIndex][horizonIndex]['regionsConstriantIndex']]) == 1)


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         Add Input Constraints
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def __addInputConstraints(self, solver, numberOfRobots, robots, dwell, horizon, inputConstraints):

        # To minimize over L1 norm of inputs, we minimize over the auxiliary variables and bound the input from above
        # and below with the auxiliary variables

        for robotIndex in range(0, numberOfRobots):
            for horizonIndex in range(0, horizon):
                for dwellIndex in range(0, dwell):
                    solver.ConvSolver.objective.set_linear(
                        solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['uxNorm']], 1.0)
                    solver.ConvSolver.objective.set_linear(
                        solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['uyNorm']], 1.0)

                    rVars             = [solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['ux']],
                                         solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['uxNorm']]]

                    # ux \le uxNorm   <==>  ux - uxNorm \le 0
                    L1NormConstraint = SMConvexSolver.LPClause(np.array([[1.0, -1.0]]), [0.0], rVars, sense="L")
                    solver.addConvConstraint(L1NormConstraint)

                    # ux >= -1 * uxNorm  <==>  -uxNorm - ux \le 0
                    L1NormConstraint = SMConvexSolver.LPClause(np.array([[-1.0, -1.0]]), [0.0], rVars, sense="L")
                    solver.addConvConstraint(L1NormConstraint)


                    rVars             = [solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['uy']],
                                         solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['uyNorm']]]

                    # uy \le uyNorm   <==>  uy - uyNorm \le 0
                    L1NormConstraint = SMConvexSolver.LPClause(np.array([[1.0, -1.0]]), [0.0], rVars, sense="L")
                    solver.addConvConstraint(L1NormConstraint)

                    # uy >= -1 * uyNorm  <==>  -uyNorm - uy \le 0
                    L1NormConstraint = SMConvexSolver.LPClause(np.array([[-1.0, -1.0]]), [0.0], rVars, sense="L")
                    solver.addConvConstraint(L1NormConstraint)

        for robotIndex in range(0, numberOfRobots):
            for horizonIndex in range(0, horizon):
                for dwellIndex in range(0, dwell):
                    # max limit on ux
                    inputConstraint = SMConvexSolver.LPClause(np.array([[1.0]]), [inputConstraints[robotIndex]['uxMax']],
                                                                        [solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['ux']]],
                                                                         sense="L")
                    solver.addConvConstraint(inputConstraint)

                    # min limit on ux
                    inputConstraint = SMConvexSolver.LPClause(np.array([[1.0]]), [inputConstraints[robotIndex]['uxMin']],
                                                              [solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['ux']]],
                                                              sense="G")
                    solver.addConvConstraint(inputConstraint)

                    # max limit on uy
                    inputConstraint = SMConvexSolver.LPClause(np.array([[1.0]]), [inputConstraints[robotIndex]['uyMax']],
                                                              [solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['uy']]],
                                                              sense="L")
                    solver.addConvConstraint(inputConstraint)

                    # min limit on uy
                    inputConstraint = SMConvexSolver.LPClause(np.array([[1.0]]), [inputConstraints[robotIndex]['uyMin']],
                                                              [solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['uy']]],
                                                              sense="G")
                    solver.addConvConstraint(inputConstraint)


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         Add Dynamics Constraints
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def __addDynamicsConstraints(self, solver, numberOfRobots, robots, dwell, horizon, Ts):
        for robotIndex in range(0, numberOfRobots):
            for horizonIndex in range(0, horizon):
                for dwellIndex in range(0, dwell):

                    if dwellIndex < dwell -1:
                        dwellIndexNext      = dwellIndex + 1
                        horizonIndexNext    = horizonIndex
                    else:
                        dwellIndexNext      = 0
                        horizonIndexNext    = horizonIndex + 1

                    if horizonIndex == horizon -1 and dwellIndex == dwell -1:
                        break


                    integratorChainX = robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['integratorChainX']
                    integratorChainY = robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['integratorChainY']

                    integratorChainXNext = robots[robotIndex][horizonIndexNext]['stateIndex'][dwellIndexNext][
                        'integratorChainX']
                    integratorChainYNext = robots[robotIndex][horizonIndexNext]['stateIndex'][dwellIndexNext][
                        'integratorChainY']

                    for integratorIndex in range(0, self.numberOfIntegrators - 1):
                        # state_i(t+1) = state_i(t) + Ts * state_{i + 1}(t)
                        vars = [
                            solver.rVars[integratorChainXNext[integratorIndex]],
                            solver.rVars[integratorChainX[integratorIndex]],
                            solver.rVars[integratorChainX[integratorIndex + 1]]
                        ]

                        #print 'chainX', vars

                        dynamicsConstraint = SMConvexSolver.LPClause(np.array([[1.0, -1.0, -1 * Ts]]), [0.0], vars,
                                                                 sense="E")
                        solver.addConvConstraint(dynamicsConstraint)

                        vars = [
                            solver.rVars[integratorChainYNext[integratorIndex]],
                            solver.rVars[integratorChainY[integratorIndex]],
                            solver.rVars[integratorChainY[integratorIndex + 1]]
                        ]
                        #print 'chainY', vars

                        dynamicsConstraint = SMConvexSolver.LPClause(np.array([[1.0, -1.0, -1 * Ts]]), [0.0], vars,
                                                                     sense="E")
                        solver.addConvConstraint(dynamicsConstraint)



                    vars = [solver.rVars[integratorChainXNext[-1]],
                            solver.rVars[integratorChainX[-1]],
                            solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['ux']]
                            ]
                    dynamicsConstraint = SMConvexSolver.LPClause(np.array([[1.0, -1.0, -1 * Ts]]), [0.0], vars,
                                                                 sense="E")
                    solver.addConvConstraint(dynamicsConstraint)

                    vars = [solver.rVars[integratorChainYNext[-1]],
                            solver.rVars[integratorChainY[-1]],
                            solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['uy']]
                            ]
                    dynamicsConstraint = SMConvexSolver.LPClause(np.array([[1.0, -1.0, -1 * Ts]]), [0.0], vars,
                                                                 sense="E")
                    solver.addConvConstraint(dynamicsConstraint)


                    '''
                    # x(t+1) = x(t) + Ts * vx(t)
                    vars                = [solver.rVars[robots[robotIndex][horizonIndexNext]['stateIndex'][dwellIndexNext]['x']],
                                           solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['x']],
                                           solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['vx']]
                                           ]
                    dynamicsConstraint  = SMConvexSolver.LPClause(np.array([[1.0, -1.0, -1 * Ts]]), [0.0], vars, sense="E")
                    solver.addConvConstraint(dynamicsConstraint)

                    # y(t+1) = y(t) + Ts * vy(t)
                    vars                = [solver.rVars[robots[robotIndex][horizonIndexNext]['stateIndex'][dwellIndexNext]['y']],
                                           solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['y']],
                                           solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['vy']]
                                            ]
                    dynamicsConstraint  = SMConvexSolver.LPClause(np.array([[1.0, -1.0, -1 * Ts]]), [0.0], vars, sense="E")
                    solver.addConvConstraint(dynamicsConstraint)

                    # vx(t+1) = vx(t) + Ts * ux(t)
                    vars                = [solver.rVars[robots[robotIndex][horizonIndexNext]['stateIndex'][dwellIndexNext]['vx']],
                                           solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['vx']],
                                           solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['ux']]
                                          ]
                    dynamicsConstraint = SMConvexSolver.LPClause(np.array([[1.0, -1.0, -1 * Ts]]), [0.0], vars, sense="E")
                    solver.addConvConstraint(dynamicsConstraint)

                    # vy(t+1) = v(y) + Ts * uy(t)
                    vars                = [solver.rVars[robots[robotIndex][horizonIndexNext]['stateIndex'][dwellIndexNext]['vy']],
                                           solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['vy']],
                                           solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['uy']]
                                        ]
                    dynamicsConstraint = SMConvexSolver.LPClause(np.array([[1.0, -1.0, -1 * Ts]]), [0.0], vars, sense="E")
                    solver.addConvConstraint(dynamicsConstraint)
                    '''


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         Add Safety Constraints
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def __addSafetyConstraints(self, solver, numberOfRobots, robots, dwell, horizon, safetyLimit, workspace):
        for horizonIndex in range(0, horizon):
            for robotIndex in range(0, numberOfRobots):
                for robotIndex2 in range(robotIndex+1, numberOfRobots):
                    safetyConstraintsIndex = robots[robotIndex][horizonIndex]['safetyConstraintsIndex'][robotIndex2]

                    # x0 - x1 > epsilon
                    rVars               = []
                    A                   = np.zeros([dwell, 2*dwell])
                    b                   = []
                    sense               = "G"
                    for dwellIndex in range(0, dwell):
                        rVars.append(solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['x']])
                        rVars.append(solver.rVars[robots[robotIndex2][horizonIndex]['stateIndex'][dwellIndex]['x']])
                        A[dwellIndex,:][2*dwellIndex: 2*dwellIndex + 2] = [1.0, -1.0]
                        b.append(safetyLimit)


                    safetyConstraint = SMConvexSolver.LPClause(A, b, rVars, sense)
                    solver.setConvIFClause(safetyConstraint, safetyConstraintsIndex[0])

                    # x1 - x0 > epsilon
                    rVars               = []
                    A = np.zeros([dwell, 2 * dwell])
                    b                   = []
                    sense               = "G"
                    for dwellIndex in range(0, dwell):
                        rVars.append(solver.rVars[robots[robotIndex2][horizonIndex]['stateIndex'][dwellIndex]['x']])
                        rVars.append(solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['x']])
                        A[dwellIndex, :][2 * dwellIndex: 2 * dwellIndex + 2] = [1.0, -1.0]
                        b.append(safetyLimit)


                    safetyConstraint = SMConvexSolver.LPClause(A, b, rVars, sense)
                    solver.setConvIFClause(safetyConstraint, safetyConstraintsIndex[1])

                    # y0 - y1 > epsilon
                    rVars               = []
                    A = np.zeros([dwell, 2 * dwell])
                    b                   = []
                    sense               = "G"
                    for dwellIndex in range(0, dwell):
                        rVars.append(solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['y']])
                        rVars.append(solver.rVars[robots[robotIndex2][horizonIndex]['stateIndex'][dwellIndex]['y']])
                        A[dwellIndex, :][2 * dwellIndex: 2 * dwellIndex + 2] = [1.0, -1.0]
                        b.append(safetyLimit)


                    safetyConstraint = SMConvexSolver.LPClause(A, b, rVars, sense)
                    solver.setConvIFClause(safetyConstraint, safetyConstraintsIndex[2])

                    # y1 - y0 > epsilon
                    rVars               = []
                    A = np.zeros([dwell, 2 * dwell])
                    b                   = []
                    sense               = "G"
                    for dwellIndex in range(0, dwell):
                        rVars.append(solver.rVars[robots[robotIndex2][horizonIndex]['stateIndex'][dwellIndex]['y']])
                        rVars.append(solver.rVars[robots[robotIndex][horizonIndex]['stateIndex'][dwellIndex]['y']])
                        A[dwellIndex, :][2 * dwellIndex: 2 * dwellIndex + 2] = [1.0, -1.0]
                        b.append(safetyLimit)


                    safetyConstraint = SMConvexSolver.LPClause(A, b, rVars, sense)
                    solver.setConvIFClause(safetyConstraint, safetyConstraintsIndex[3])


                    '''
                    # If robot1 is on the right, it can not be on the left at the same time
                    solver.addBoolConstraint(
                        SMConvexSolver.EQUIV(
                            solver.convIFClauses[safetyConstraintsIndex[0]],
                            SMConvexSolver.NOT(
                                solver.convIFClauses[safetyConstraintsIndex[1]]
                            )
                        )
                    )

                    solver.addBoolConstraint(
                        SMConvexSolver.EQUIV(
                            solver.convIFClauses[safetyConstraintsIndex[2]],
                            SMConvexSolver.NOT(
                                solver.convIFClauses[safetyConstraintsIndex[3]]
                            )
                        )
                    )
                    '''

                    # if robots are in two adjacent regions THEN:
                    #               at least one of the safety conditions must be enforced (encoded using disjunction)


                    # The antecedent is encoded as (robot_1_at_region_1 AND  robot_1_at_adj(region)_1) OR ( ) OR ( ) ...
                    regionConstraintsFirstRobot     = robots[robotIndex][horizonIndex]['regionsConstriantIndex']
                    regionConstraintsSecondRobot    = robots[robotIndex2][horizonIndex]['regionsConstriantIndex']

                    regionConstraints               = []
                    for regioncounter in range(0, len(regionConstraintsFirstRobot)):
                        adjacents = workspace['regions'][regioncounter]['adjacents']
                        for adjacentCounter in range(0, len(adjacents)):
                            regionConstraints.append(SMConvexSolver.AND(solver.convIFClauses[regionConstraintsFirstRobot[regioncounter]],
                                           solver.convIFClauses[regionConstraintsSecondRobot[adjacents[adjacentCounter]]]))

                    antecedent      = SMConvexSolver.OR(*regionConstraints)

                    # if no regions are specified, then all robots are in the same region
                    if not workspace['regions']:
                        antecedent = True

                    constraint      = [solver.convIFClauses[i] for i in safetyConstraintsIndex]
                    consequent      = SMConvexSolver.OR(*constraint)
                    solver.addBoolConstraint(SMConvexSolver.IMPLIES(antecedent, consequent))


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         LTL Parser
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def __LTLParser(self, solver, robots):
        # get the top level formula
        for formula in self.LTL:
            if formula['type'] == 'LTL':
                topLevelFormula         = formula
                self.topLevelFormula    = formula
                self.__generateLTLBooleanConstraints(solver, formula, robots)

        for formula in self.LTL:
            if formula['type'] == 'atomic':
                self.__generateAtomicBooleanConstraints(solver, formula, topLevelFormula, robots)

            elif formula['type'] == 'compound1':
                if formula['operator'] == 'G':
                    self.__generateCompoundGLOBALLYBooleanConstraints(solver, formula, topLevelFormula)
                elif formula['operator'] == 'E':
                    self.__generateCompoundEVENTUALLYBooleanConstraints(solver, formula, topLevelFormula)
                elif formula['operator'] == 'NOT':
                    self.__generateCompoundNOTBooleanConstraints(solver, formula, topLevelFormula)

            elif formula['type'] == 'compound2':
                if formula['operator'] == 'AND':
                    self.__generateCompoundANDBooleanConstraints(solver, formula, topLevelFormula)
                elif formula['operator'] == 'OR':
                    self.__generateCompoundORBooleanConstraints(solver, formula, topLevelFormula)
                elif formula['operator'] == 'RELEASE':
                    self.__generateCompoundRELEASEBooleanConstraints(solver, formula, topLevelFormula)


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         TOP LEVEL LTL CONSTRAINTS
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def __generateLTLBooleanConstraints(self, solver, formula, robots):
        prop1 = self.LTL[formula['argument1']]

        # LTLbVar <=> formulabVar(t)
        #    for all time t \in {0, L}
        for horizonCounter in range(0, self.horizon):
            solver.addBoolConstraint(
                SMConvexSolver.EQUIV(
                    solver.bVars[formula['formulaSatisfied']],
                    solver.bVars[prop1['boolVarsIndex'][horizonCounter]]
                )
            )

        # the LTL boolean variables must be satisfied
        solver.addBoolConstraint(solver.bVars[formula['formulaSatisfied']])

        # ------------ Loop Constraints ---------------------
        # loopStart_0 <=> false
        solver.addBoolConstraint(
            SMConvexSolver.EQUIV(
                solver.bVars[formula['loopStart'][0]],
                False
            )
        )

        # inLoop_0 <=> false
        solver.addBoolConstraint(
            SMConvexSolver.EQUIV(
                solver.bVars[formula['inLoop'][0]],
                False
            )
        )

        # 1 \le i \le k
        for horizonCounter in range(1, self.horizon):
            # inLoop_i <=> inLoop_{i -1 } OR loopStart_i
            solver.addBoolConstraint(
                SMConvexSolver.EQUIV(
                    solver.bVars[formula['inLoop'][horizonCounter]],
                    SMConvexSolver.OR(
                        *[
                            solver.bVars[formula['loopStart'][horizonCounter]],
                            solver.bVars[formula['inLoop'][horizonCounter-1]],
                        ]
                    )
                )
            )

            # inLoop_{i-1} => NOT loopStart_{i}
            solver.addBoolConstraint(
                SMConvexSolver.IMPLIES(
                    solver.bVars[formula['inLoop'][horizonCounter-1]],
                    SMConvexSolver.NOT(
                        solver.bVars[formula['loopStart'][horizonCounter]]
                    )
                )
            )

            # loopStart_i => (s_{i-1} = s_{horizon})  region consistency
            for robotIndex in range(0, self.numberOfRobots):
                for regionIndex in range(0, self.numberOfRegions):
                    solver.addBoolConstraint(
                        SMConvexSolver.IMPLIES(
                            solver.bVars[formula['loopStart'][horizonCounter]],
                            SMConvexSolver.EQUIV(
                                solver.convIFClauses[robots[robotIndex][horizonCounter - 1]['regionsConstriantIndex'][regionIndex]],
                                solver.convIFClauses[robots[robotIndex][-1]['regionsConstriantIndex'][regionIndex]],
                            )
                        )
                    )

                # robot state constraints
                stateStartLoop  = robots[robotIndex][horizonCounter]['stateIndex'][0]['states']
                stateEndLoop    = robots[robotIndex][-1]['stateIndex'][-1]['states']
                numberOFstates  = len(stateStartLoop)
                A               = np.concatenate( (np.identity(numberOFstates), -1 *np.identity(numberOFstates)), axis = 1)
                b               = [0.0]*numberOFstates


                #print 'consistency', robots[robotIndex][horizonCounter]['consistencyConstraintIndex'], stateStartLoop + stateEndLoop
                consistencyConstraint = SMConvexSolver.LPClause(A, b, stateStartLoop + stateEndLoop, 'E')
                solver.setConvIFClause(
                    consistencyConstraint,
                    robots[robotIndex][horizonCounter]['consistencyConstraintIndex']
                )

                solver.addBoolConstraint(
                    SMConvexSolver.IMPLIES(
                        solver.bVars[formula['loopStart'][horizonCounter]],
                        solver.convIFClauses[robots[robotIndex][horizonCounter]['consistencyConstraintIndex']]
                    )
                )


        # loopexists <=> inloop_{horizon}
        solver.addBoolConstraint(
            SMConvexSolver.EQUIV(
                solver.bVars[formula['loopExists']],
                solver.bVars[formula['inLoop'][-1]]
            )
        )


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         Encode Constraints for ATOMIC Propositions
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def __generateAtomicBooleanConstraints(self, solver, formula, topLevelFormula, robots):
        robotsIndex         = formula['robots']
        sense               = formula['sense']
        rhs                 = formula['rhs']
        regionIndex         = formula['region']

        for horizonCounter in range(0, self.horizon):
            ifVars    = [robots[j][horizonCounter]['regionsConstriantIndex'][regionIndex] for j in robotsIndex]

            if sense == 'GE':
                solver.addBoolConstraint(
                    SMConvexSolver.EQUIV(
                        solver.bVars[formula['boolVarsIndex'][horizonCounter]],
                        sum([SMConvexSolver.BoolVar2Int(solver.convIFClauses[j]) for j in ifVars]) >= rhs
                    )
                )
            elif sense == 'LE':
                solver.addBoolConstraint(
                    SMConvexSolver.EQUIV(
                        solver.bVars[formula['boolVarsIndex'][horizonCounter]],
                        sum([SMConvexSolver.BoolVar2Int(solver.convIFClauses[j]) for j in ifVars]) <= rhs
                    )
                )
            elif sense == 'E':
                solver.addBoolConstraint(
                    SMConvexSolver.EQUIV(
                        solver.bVars[formula['boolVarsIndex'][horizonCounter]],
                        sum([SMConvexSolver.BoolVar2Int(solver.convIFClauses[j]) for j in ifVars]) == rhs
                    )
                )


        # ----- Formula Consistency -----------------------------
        for horizonCounter in range(1, self.horizon):
            # loop consistency
            # loopStart_i => (b_i <=> l_horizon)
            solver.addBoolConstraint(
                SMConvexSolver.IMPLIES(
                    solver.bVars[topLevelFormula['loopStart'][horizonCounter]],
                    SMConvexSolver.EQUIV(
                        solver.bVars[formula['boolVarsIndex'][horizonCounter-1]],
                        solver.bVars[formula['boolVarsIndex'][-1]]
                    )
                )
            )


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         Encode: Constraints of AND
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def __generateCompoundANDBooleanConstraints(self, solver, formula, topLevelFormula):
        prop1 = self.LTL[formula['argument1']]
        prop2 = self.LTL[formula['argument2']]

        for horizonCounter in range(0, self.horizon):
            solver.addBoolConstraint(
                SMConvexSolver.EQUIV(
                    solver.bVars[formula['boolVarsIndex'][horizonCounter]],
                    SMConvexSolver.AND(
                        *[
                            solver.bVars[prop1['boolVarsIndex'][horizonCounter]],
                            solver.bVars[prop2['boolVarsIndex'][horizonCounter]]
                        ]
                    )
                )
            )

        # ------------ Formula Consistency ---------------------
        for horizonCounter in range(1, self.horizon):
            # loop consistency
            # loopStart_i => (b_i <=> l_horizon)
            solver.addBoolConstraint(
                SMConvexSolver.IMPLIES(
                    solver.bVars[topLevelFormula['loopStart'][horizonCounter]],
                    SMConvexSolver.EQUIV(
                        solver.bVars[formula['boolVarsIndex'][horizonCounter-1]],
                        solver.bVars[formula['boolVarsIndex'][-1]]
                    )
                )
            )


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         Encode: Constraints of OR
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def __generateCompoundORBooleanConstraints(self, solver, formula, topLevelFormula):
        prop1 = self.LTL[formula['argument1']]
        prop2 = self.LTL[formula['argument2']]

        for horizonCounter in range(0, self.horizon):
            solver.addBoolConstraint(
                SMConvexSolver.EQUIV(
                    solver.bVars[formula['boolVarsIndex'][horizonCounter]],
                    SMConvexSolver.OR(
                        *[
                            solver.bVars[prop1['boolVarsIndex'][horizonCounter]],
                            solver.bVars[prop2['boolVarsIndex'][horizonCounter]]
                        ]
                    )
                )
            )

        # ------------ Formula Consistency ---------------------
        for horizonCounter in range(1, self.horizon):
            # loop consistency
            # loopStart_i => (b_i <=> l_horizon)
            solver.addBoolConstraint(
                SMConvexSolver.IMPLIES(
                    solver.bVars[topLevelFormula['loopStart'][horizonCounter]],
                    SMConvexSolver.EQUIV(
                        solver.bVars[formula['boolVarsIndex'][horizonCounter-1]],
                        solver.bVars[formula['boolVarsIndex'][-1]]
                    )
                )
            )


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         Encode: Constraints of NOT
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def __generateCompoundNOTBooleanConstraints(self, solver, formula, topLevelFormula):
        prop1 = self.LTL[formula['argument1']]

        for horizonCounter in range(0, self.horizon):
            solver.addBoolConstraint(
                SMConvexSolver.EQUIV(
                    solver.bVars[formula['boolVarsIndex'][horizonCounter]],
                    SMConvexSolver.NOT(
                        solver.bVars[prop1['boolVarsIndex'][horizonCounter]]
                    )
                )
            )

        # ------------ Formula Consistency ---------------------
        for horizonCounter in range(1, self.horizon):
            # loop consistency
            # loopStart_i => (b_i <=> l_horizon)
            solver.addBoolConstraint(
                SMConvexSolver.IMPLIES(
                    solver.bVars[topLevelFormula['loopStart'][horizonCounter]],
                    SMConvexSolver.EQUIV(
                        solver.bVars[formula['boolVarsIndex'][horizonCounter-1]],
                        solver.bVars[formula['boolVarsIndex'][-1]]
                    )
                )
            )


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         Encode: Constraints of RELEASE
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def __generateCompoundRELEASEBooleanConstraints(self, solver, formula, topLevelFormula):
        prop1 = self.LTL[formula['argument1']]
        prop2 = self.LTL[formula['argument2']]

        # (p1 R p2)_i <=> p2_i AND (p1_i OR (p1 U p2)_{i+1} )
        for horizonCounter in range(0, self.horizon - 1):
            solver.addBoolConstraint(
                SMConvexSolver.EQUIV(
                    solver.bVars[formula['boolVarsIndex'][horizonCounter]],
                    SMConvexSolver.AND(
                        *[
                            solver.bVars[prop2['boolVarsIndex'][horizonCounter]],
                            SMConvexSolver.OR(
                                *[
                                    solver.bVars[prop1['boolVarsIndex'][horizonCounter]],
                                    solver.bVars[formula['boolVarsIndex'][horizonCounter + 1]]
                                ]
                            )
                        ]
                    )
                )
            )

        # for i = horizon
        solver.addBoolConstraint(
            SMConvexSolver.EQUIV(
                solver.bVars[formula['boolVarsIndex'][-1]],
                SMConvexSolver.AND(
                    *[
                        solver.bVars[prop2['boolVarsIndex'][-1]],
                        SMConvexSolver.OR(
                            *[
                                solver.bVars[prop1['boolVarsIndex'][-1]],
                                SMConvexSolver.OR(
                                    *[
                                        SMConvexSolver.AND(
                                            solver.bVars[topLevelFormula['loopStart'][j]],
                                            solver.bVars[formula['boolVarsIndex2'][j]],
                                        ) for j in range(0, self.horizon)
                                    ]

                                )
                            ]
                        )
                    ]
                )
            )
        )

        # ------------- Auxiliary encoding for RELEASE ---------------
        for horizonCounter in range(0, self.horizon - 1):
            solver.addBoolConstraint(
                SMConvexSolver.EQUIV(
                    solver.bVars[formula['boolVarsIndex2'][horizonCounter]],
                    SMConvexSolver.AND(
                        *[
                            solver.bVars[prop2['boolVarsIndex'][horizonCounter]],
                            SMConvexSolver.OR(
                                *[
                                    solver.bVars[prop1['boolVarsIndex'][horizonCounter]],
                                    solver.bVars[formula['boolVarsIndex2'][horizonCounter + 1]]
                                ]
                            )
                        ]
                    )
                )
            )

        # for i = horizon
        solver.addBoolConstraint(
            SMConvexSolver.EQUIV(
                solver.bVars[formula['boolVarsIndex2'][-1]],
                solver.bVars[prop2['boolVarsIndex'][-1]]
            )
        )

        # ------------ Formula Consistency ---------------------
        for horizonCounter in range(1, self.horizon):
            # loop consistency
            # loopStart_i => (b_i <=> l_horizon)
            solver.addBoolConstraint(
                SMConvexSolver.IMPLIES(
                    solver.bVars[topLevelFormula['loopStart'][horizonCounter]],
                    SMConvexSolver.EQUIV(
                        solver.bVars[formula['boolVarsIndex'][horizonCounter-1]],
                        solver.bVars[formula['boolVarsIndex'][-1]]
                    )
                )
            )


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         Encode: Constraints of GLOBALLY
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def __generateCompoundGLOBALLYBooleanConstraints(self, solver, formula, topLevelFormula):
        # G phi = false R phi

        prop2 = self.LTL[formula['argument1']]

        # (p1 R p2)_i <=> p2_i AND (p1_i OR (p1 U p2)_{i+1} )
        for horizonCounter in range(0, self.horizon - 1):
            solver.addBoolConstraint(
                SMConvexSolver.EQUIV(
                    solver.bVars[formula['boolVarsIndex'][horizonCounter]],
                    SMConvexSolver.AND(
                        *[
                            solver.bVars[prop2['boolVarsIndex'][horizonCounter]],
                            SMConvexSolver.OR(
                                *[
                                    False,
                                    solver.bVars[formula['boolVarsIndex'][horizonCounter + 1]]
                                ]
                            )
                        ]
                    )
                )
            )

        # for i = horizon
        solver.addBoolConstraint(
            SMConvexSolver.EQUIV(
                solver.bVars[formula['boolVarsIndex'][-1]],
                SMConvexSolver.AND(
                    *[
                        solver.bVars[prop2['boolVarsIndex'][-1]],
                        SMConvexSolver.OR(
                            *[
                                False,
                                SMConvexSolver.OR(
                                    *[
                                        SMConvexSolver.AND(
                                            solver.bVars[topLevelFormula['loopStart'][j]],
                                            solver.bVars[formula['boolVarsIndex2'][j]],
                                        ) for j in range(0, self.horizon)
                                    ]

                                )
                            ]
                        )
                    ]
                )
            )
        )

        # ------------- Auxiliary encoding for RELEASE ---------------
        for horizonCounter in range(0, self.horizon - 1):
            solver.addBoolConstraint(
                SMConvexSolver.EQUIV(
                    solver.bVars[formula['boolVarsIndex2'][horizonCounter]],
                    SMConvexSolver.AND(
                        *[
                            solver.bVars[prop2['boolVarsIndex'][horizonCounter]],
                            SMConvexSolver.OR(
                                *[
                                    False,
                                    solver.bVars[formula['boolVarsIndex2'][horizonCounter + 1]]
                                ]
                            )
                        ]
                    )
                )
            )

        # for i = horizon
        solver.addBoolConstraint(
            SMConvexSolver.EQUIV(
                solver.bVars[formula['boolVarsIndex2'][-1]],
                solver.bVars[prop2['boolVarsIndex'][-1]]
            )
        )

        # ------------ Formula Consistency ---------------------
        for horizonCounter in range(1, self.horizon):
            # loop consistency
            # loopStart_i => (b_i <=> l_horizon)
            solver.addBoolConstraint(
                SMConvexSolver.IMPLIES(
                    solver.bVars[topLevelFormula['loopStart'][horizonCounter]],
                    SMConvexSolver.EQUIV(
                        solver.bVars[formula['boolVarsIndex'][horizonCounter-1]],
                        solver.bVars[formula['boolVarsIndex'][-1]]
                    )
                )
            )


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         Encode: Constraints of UNTIL
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def __generateCompoundUNTILBooleanConstraints(self, solver, formula, topLevelFormula):
        prop1 = self.LTL[formula['argument1']]
        prop2 = self.LTL[formula['argument2']]

        # (p1 U p2)_i <=> p2_i OR (p1_i AND (p1 U p2)_{i+1} )
        for horizonCounter in range(0, self.horizon - 1):
            solver.addBoolConstraint(
                SMConvexSolver.EQUIV(
                    solver.bVars[formula['boolVarsIndex'][horizonCounter]],
                    SMConvexSolver.OR(
                        *[
                            solver.bVars[prop2['boolVarsIndex'][horizonCounter]],
                            SMConvexSolver.AND(
                                *[
                                    solver.bVars[prop1['boolVarsIndex'][horizonCounter]],
                                    solver.bVars[formula['boolVarsIndex'][horizonCounter+1]]
                                ]
                            )
                        ]
                    )
                )
            )

        # for i = horizon
        solver.addBoolConstraint(
            SMConvexSolver.EQUIV(
                solver.bVars[formula['boolVarsIndex'][-1]],
                SMConvexSolver.OR(
                    *[
                        solver.bVars[prop2['boolVarsIndex'][-1]],
                        SMConvexSolver.AND(
                            *[
                                solver.bVars[prop1['boolVarsIndex'][-1]],
                                SMConvexSolver.OR(
                                    *[
                                        SMConvexSolver.AND(
                                                solver.bVars[topLevelFormula['loopStart'][j]],
                                                solver.bVars[formula['boolVarsIndex2'][j]],
                                        )   for j in range(0, self.horizon)
                                    ]

                                )
                            ]
                        )
                    ]
                )
            )
        )

        # ------------- Auxiliary encoding for UNTIL ---------------
        for horizonCounter in range(0, self.horizon - 1):
            solver.addBoolConstraint(
                SMConvexSolver.EQUIV(
                    solver.bVars[formula['boolVarsIndex2'][horizonCounter]],
                    SMConvexSolver.OR(
                        *[
                            solver.bVars[prop2['boolVarsIndex'][horizonCounter]],
                            SMConvexSolver.AND(
                                *[
                                    solver.bVars[prop1['boolVarsIndex'][horizonCounter]],
                                    solver.bVars[formula['boolVarsIndex2'][horizonCounter+1]]
                                ]
                            )
                        ]
                    )
                )
            )

        # for i = horizon
        solver.addBoolConstraint(
            SMConvexSolver.EQUIV(
                solver.bVars[formula['boolVarsIndex2'][-1]],
                solver.bVars[prop2['boolVarsIndex'][-1]]
            )
        )

        # ------------ Formula Consistency ---------------------
        for horizonCounter in range(1, self.horizon):
            # loop consistency
            # loopStart_i => (b_i <=> l_horizon)
            solver.addBoolConstraint(
                SMConvexSolver.IMPLIES(
                    solver.bVars[topLevelFormula['loopStart'][horizonCounter]],
                    SMConvexSolver.EQUIV(
                        solver.bVars[formula['boolVarsIndex'][horizonCounter-1]],
                        solver.bVars[formula['boolVarsIndex'][-1]]
                    )
                )
            )


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         Encode: Constraints of Eventually
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def __generateCompoundEVENTUALLYBooleanConstraints(self, solver, formula, topLevelFormula):
        # E phi = true U phi
        prop2 = self.LTL[formula['argument1']]

        # (p1 U p2)_i <=> p2_i OR (p1_i AND (p1 U p2)_{i+1} )
        for horizonCounter in range(0, self.horizon - 1):
            solver.addBoolConstraint(
                SMConvexSolver.EQUIV(
                    solver.bVars[formula['boolVarsIndex'][horizonCounter]],
                    SMConvexSolver.OR(
                        *[
                            solver.bVars[prop2['boolVarsIndex'][horizonCounter]],
                            SMConvexSolver.AND(
                                *[
                                    True,
                                    solver.bVars[formula['boolVarsIndex'][horizonCounter + 1]]
                                ]
                            )
                        ]
                    )
                )
            )

        # for i = horizon
        solver.addBoolConstraint(
            SMConvexSolver.EQUIV(
                solver.bVars[formula['boolVarsIndex'][-1]],
                SMConvexSolver.OR(
                    *[
                        solver.bVars[prop2['boolVarsIndex'][-1]],
                        SMConvexSolver.AND(
                            *[
                                True,
                                SMConvexSolver.OR(
                                    *[
                                        SMConvexSolver.AND(
                                            solver.bVars[topLevelFormula['loopStart'][j]],
                                            solver.bVars[formula['boolVarsIndex2'][j]],
                                        ) for j in range(0, self.horizon)
                                    ]

                                )
                            ]
                        )
                    ]
                )
            )
        )

        # ------------- Auxiliary encoding for UNTIL ---------------
        for horizonCounter in range(0, self.horizon - 1):
            solver.addBoolConstraint(
                SMConvexSolver.EQUIV(
                    solver.bVars[formula['boolVarsIndex2'][horizonCounter]],
                    SMConvexSolver.OR(
                        *[
                            solver.bVars[prop2['boolVarsIndex'][horizonCounter]],
                            SMConvexSolver.AND(
                                *[
                                    True,
                                    solver.bVars[formula['boolVarsIndex2'][horizonCounter + 1]]
                                ]
                            )
                        ]
                    )
                )
            )

        # for i = horizon
        solver.addBoolConstraint(
            SMConvexSolver.EQUIV(
                solver.bVars[formula['boolVarsIndex2'][-1]],
                solver.bVars[prop2['boolVarsIndex'][-1]]
            )
        )

        # ------------ Formula Consistency ---------------------
        for horizonCounter in range(1, self.horizon):
            # loop consistency
            # loopStart_i => (b_i <=> l_horizon)
            solver.addBoolConstraint(
                SMConvexSolver.IMPLIES(
                    solver.bVars[topLevelFormula['loopStart'][horizonCounter]],
                    SMConvexSolver.EQUIV(
                        solver.bVars[formula['boolVarsIndex'][horizonCounter-1]],
                        solver.bVars[formula['boolVarsIndex'][-1]]
                    )
                )
            )


    # ***************************************************************************************************
    # ***************************************************************************************************
    #
    #         LTL Parser: Public APIs
    #
    # ***************************************************************************************************
    # ***************************************************************************************************
    def createAtomicProposition(self, region, robots, sense, rhs):
        proposition = {'type': 'atomic',
                 'robots': robots,
                 'sense': sense,
                 'rhs': rhs,
                 'region': region,
                 'boolVarsIndex': range(self.numberOfLTLBoolVars, self.numberOfLTLBoolVars + self.horizon)
                 }

        self.LTL.append(proposition)
        self.numberOfLTLBoolVars    += self.horizon

        return len(self.LTL) - 1        # return index of the added proposition


    def createCompoundProposition(self, prop1, prop2, operator):
        # ----------------------------------------------------------------------
        if operator == 'AND' or operator == 'OR' or operator == 'RELEASE':
            formula     = {'type': 'compound2',
                               'operator': operator,
                               'argument1': prop1,
                               'argument2': prop2,
                               'boolVarsIndex': range(self.numberOfLTLBoolVars, self.numberOfLTLBoolVars + self.horizon)
                               }
            self.LTL.append(formula)
            self.numberOfLTLBoolVars += self.horizon
            return len(self.LTL) - 1  # return index of the added proposition

        #----------------------------------------------------------------------
        if operator == 'NOT' or operator == 'NEXT':
            formula = {'type': 'compound1',
                           'operator': operator,
                           'argument1': prop1,
                           'boolVarsIndex': range(self.numberOfLTLBoolVars, self.numberOfLTLBoolVars + self.horizon)
                           }
            self.LTL.append(formula)
            self.numberOfLTLBoolVars += self.horizon
            return len(self.LTL) - 1  # return index of the added proposition
        # ----------------------------------------------------------------------
        elif operator == 'E' or operator == 'G':      # Eventually or Globally (always)
            formula = {'type': 'compound1',
                            'operator': operator,
                            'argument1': prop1,
                            'boolVarsIndex': range(self.numberOfLTLBoolVars, self.numberOfLTLBoolVars +  self.horizon),
                            'boolVarsIndex2': range(self.numberOfLTLBoolVars +  self.horizon, self.numberOfLTLBoolVars + 2 * self.horizon),
                           }
            self.LTL.append(formula)
            self.numberOfLTLBoolVars += 2 * self.horizon
            return len(self.LTL) - 1  # return index of the added proposition


        # ----------------------------------------------------------------------
        elif operator == 'UNTIL':      # Until
            print 'U'



    def createLTLFormula(self, prop):
        formula = {'type': 'LTL',
                   'argument1': prop,
                   'formulaSatisfied': self.numberOfLTLBoolVars,        # one variable to indicate the satisfiaction of the formula
                   'loopExists': self.numberOfLTLBoolVars + 1,          # one variable to indicate that loop exists
                   'inLoop': range(self.numberOfLTLBoolVars + 2, self.numberOfLTLBoolVars + 2 + self.horizon),
                    'loopStart': range(self.numberOfLTLBoolVars + 2 + self.horizon, self.numberOfLTLBoolVars + 2 + 2 * self.horizon)
                   }
        # only one variable that indicates that the LTL formula is satisfied
        self.LTL.append(formula)
        self.numberOfLTLBoolVars += (2 + 2 * self.horizon)
