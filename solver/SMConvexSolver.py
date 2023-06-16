#  * --------------------------------------------------------------------------
#  * File: SMConvexSolver.py
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
#  *       This product includes software developed by Cyber-Physical &
#  *       Systems Lab at UCLA.
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

import sys
import os 
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path+'/z3/z3-4.4.1-x64-osx-10.11/bin/')
import z3 as z3


import numpy as np

import cplex as cplex
from cplex.exceptions import CplexError

# from multiprocessing import Pool

import multiprocessing

import timeit



#***************************************************************************************************
#***************************************************************************************************
#
#         CLASS SMConvexSolver
#
#***************************************************************************************************
#***************************************************************************************************

class SMConvexSolver:
    
    # ========================================================
    #       Constructor
    # ========================================================
    def __init__(self, numOfBoolVars, numOfRealVars, numOfConvexIFClauses,
                 maxNumberOfIterations = 1000, slackTolerance = 1E-6, counterExampleStrategy = 'IIS', verbose = 'OFF', profiling = 'true', numberOfCores = 2):
        # TODO: check that numOfConvexIFConstraints > 0

        # ------------ Initialize SAT Solvers   ---------------------------------------
        self.SATsolver                  = z3.Solver()
        self.SATsolver.reset()
        
        # ------------ Initialize Public Variables ------------------------------------
        self.bVars                      = z3.BoolVector('b', numOfBoolVars)
        self.rVars                      = ['x'+str(i) for i in range(0, numOfRealVars)]
        self.convIFClauses              = z3.BoolVector('bConv', numOfConvexIFClauses) # Boolean abstraction of ConvIFClauses
        
        self.counterExamples            = list()
        
        # ------------ Initialize Private Variables ------------------------------------
        self.__convIFClauses             = [None] * numOfConvexIFClauses   # the actual ConvIFClauses
        self.__slackIFVars               = ['s'+str(i)+'_' for i in range(0, numOfConvexIFClauses)] # list to hold "real-valued" slack variables
        self.__slackIFVarsBound          = ['t'+str(i) for i in range(0, numOfConvexIFClauses)] # list to hold "real-valued" slack
        # ------------ Initialize Solver Parameters ------------------------------------
        self.maxNumberOfIterations      = maxNumberOfIterations
        self.slackTolerance             = slackTolerance
        self.counterExampleStrategy     = counterExampleStrategy
        self.verbose                    = verbose
        self.profiling                  = profiling
        self.numberOfCores              = numberOfCores

        self.slackRatio                 = 20
        
        # ------------ Initialize Convex Solver  ---------------------------------------
        self.ConvSolver                 = cplex.Cplex()
        self.ConvSolver.parameters.threads.set(numberOfCores)
        self.ConvSolver.parameters.barrier.convergetol.set(self.slackTolerance)
        self.ConvSolver.parameters.simplex.tolerances.optimality.set(self.slackTolerance)

        if self.verbose == 'OFF':
            self.ConvSolver.set_results_stream(None) # set verbose  = 0
            self.ConvSolver.set_log_stream(None)
        
        
        self.ConvSolver.objective.set_sense(self.ConvSolver.objective.sense.minimize)      # we minimize the slack

        # CPLEX default LB is 0.0, we change it to -infinty. These variables do not appear in the optimization objective (only slack variables appear there, so we omit the "obj" parameter)
        self.ConvSolver.variables.add(
                            names       =   self.rVars,
                            ub          =   [cplex.infinity] * numOfRealVars,
                            lb          =   [-cplex.infinity] * numOfRealVars
                        )

        # add slack variabels
        self.ConvSolver.variables.add(
                            names       =   self.__slackIFVars,
                            ub          =   [cplex.infinity] * numOfConvexIFClauses,
                            lb          =   [-cplex.infinity] * numOfConvexIFClauses
                        )
        if self.counterExampleStrategy == 'PREFIX' or self.counterExampleStrategy == 'SMC':
            # our objective is to minimize the sum of absolute value of slack variabeles ,i.e., min |s_0| + |s_1| + ....
            # the trick is to define new "bounding" variables t_0, t_1, .... and then minimze t_0 + t_1 + ...
            # subject to |s_i| \le t_i which translates to s_i \le t_i and -s_i \le t_i
            self.ConvSolver.variables.add(
                            obj         =   [1.0] * numOfConvexIFClauses,
                            names       =   self.__slackIFVarsBound,
                            ub          =   [cplex.infinity] * numOfConvexIFClauses,
                            lb          =   [-cplex.infinity] * numOfConvexIFClauses
                        )
        else:
            # in IIS and trivial we do not minimize the sum of slack, it is just feasability.
            # When using the IIS-based strategy, we would like to force infeasibility and leave the solver to relax the slack variables
            # in order to discover conflicts. Thus, we put the upper bound equal to 0.0.
            self.ConvSolver.variables.add(
                        names       =   self.__slackIFVarsBound,
                        ub          =   [0.0] * numOfConvexIFClauses,
                        lb          =   [0.0] * numOfConvexIFClauses
            )

        
        for slackIFcounter in range(0, numOfConvexIFClauses):
            self.ConvSolver.linear_constraints.add(
                lin_expr    = [cplex.SparsePair(ind = [self.__slackIFVars[slackIFcounter], self.__slackIFVarsBound[slackIFcounter]], val = [1.0, -1.0])],
                senses      = ['L'],
                rhs         = [0.0],
            )
            self.ConvSolver.linear_constraints.add(
                lin_expr    = [cplex.SparsePair(ind = [self.__slackIFVars[slackIFcounter], self.__slackIFVarsBound[slackIFcounter]], val = [-1.0, -1.0])],
                senses      = ['L'],
                rhs         = [0.0],
            )

    # ========================================================
    #       Set upper bound on real variables
    # ========================================================
    def setUpperBound(self, rVar, bound): # if you want an upper bound that is not infinity
        # when IIS is used, the solver is going to relax the upper bound of all variables to reason about infeasibility.
        # However, we would like to restrict the analysis to the slack variables of the interface clauses.
        # Thus, we enforce upper bounds of rVars as a linear constraint instead of "explicit" upper bound,
        # and we leave the "explicit" upper bound as infinity
        if self.counterExampleStrategy == 'IIS':
            self.ConvSolver.linear_constraints.add(
                        lin_expr    = [cplex.SparsePair(ind = [rVar], val = [1.0])],
                        senses      = ['L'],
                        rhs         = [bound],
                    )
        else:
            self.ConvSolver.variables.set_upper_bounds(rVar, bound)

    # ========================================================
    #       Set Lower bound on real variabeles
    # ========================================================

    def setLowerBound(self, rVar, bound): #if you want a lower bound that is not -infinity
        self.ConvSolver.variables.set_lower_bounds(rVar, bound)

    # ========================================================
    #               Add Boolean Constraints
    # ========================================================
    def addBoolConstraint(self, constraint):
        self.SATsolver.add(constraint)


    # ========================================================
    #               Add Interface Constraints
    # ========================================================
    def addInterfaceConstraint(self, constraint):
        self.SATsolver.add(constraint)
        

    # ========================================================
    #               Add Convex Constraints
    # ========================================================
    def addConvConstraint(self, constraint):
        if constraint['type'] == 'LP':
            self.ConvSolver.linear_constraints.add(
                          lin_expr    = constraint['lin_expr'],
                          senses      = constraint['senses'],
                          rhs         = constraint['rhs'],
                )
        elif constraint['type'] == 'QP':
            self.ConvSolver.quadratic_constraints.add(
                            quad_expr   = constraint['quad_expr'],
                            lin_expr    = constraint['lin_expr'],
                            sense       = constraint['sense'],
                            rhs         = constraint['rhs'],
                )
    
    # ========================================================
    #               Set Convex Interface Clause
    # ========================================================
    def setConvIFClause(self, clause, index):
        # Add slack variable
        slack_variable                  = 's'+str(index)+'_'
        slackedClause                   =   self.__slackConstraint(clause, slack_variable)
        
        # Add the constraints to the convex solver. We will delte the unncessary later on
        if slackedClause['type'] == 'LP':
            self.ConvSolver.linear_constraints.add(
                        lin_expr        = slackedClause['lin_expr'],
                        senses          = slackedClause['senses'],
                        rhs             = slackedClause['rhs'],
                        names           = slackedClause['name']
                    )
        elif slackedClause['type'] == 'QP':
            self.ConvSolver.quadratic_constraints.add(
                        quad_expr       = slackedClause['quad_expr'],
                        lin_expr        = slackedClause['lin_expr'],
                        sense           = slackedClause['sense'],
                        rhs             = slackedClause['rhs'],
                        name            = slackedClause['name']
                    )
        
        #print slackedClause
        self.__convIFClauses[index]     =   slackedClause
    
    # ========================================================
    #               Main Solver Function
    # ========================================================
    def solve(self):
        # TODO: check that all interface constraints are set
        
        
        solutionFound                       = False
        iterationsCounter                   = 0
        # ------------ Main Loop ------------------------------------------------------
        while solutionFound == False and iterationsCounter < self.maxNumberOfIterations:
            iterationsCounter               = iterationsCounter + 1
            if self.verbose == 'ON':
                print ('******** SMConv Solver, iteration = ', iterationsCounter, '********')

            if self.profiling == 'true':
                start = timeit.default_timer()
            SATcheck                        = self.SATsolver.check()
            if self.profiling == 'true':
                end = timeit.default_timer()
                print('SAT time', end-start)
        # ------------ Call SAT solver -------------------------------------------------
            if  SATcheck == z3.unsat:
                print ('========== ERROR: Problem is UNSAT ==========')
                return list(), list(), list()
            else:
        # ------------ Extract Boolean Models ------------------------------------------
                convIFModel, bModel         = self.__extractSATModel()
                #print 'ConvIfModel = ', [i for i, x in enumerate(convIFModel) if x == True]
        # ------------ Prepare the convex problem --------------------------------------
                if self.profiling == 'true':
                    start                       = timeit.default_timer()
                constrainedConvSolver       = self.__prepareConvexProblem(convIFModel)
                if self.profiling == 'true':
                    end                         = timeit.default_timer()
                    print('prepare conv solver', end - start)
        # ------------ Solve convex problem --------------------------------------------
                if self.profiling == 'true':
                    start                       = timeit.default_timer()
                convSolnFound               = self.__solveConvexProblem(constrainedConvSolver, convIFModel)
                if self.profiling == 'true':
                    end                         = timeit.default_timer()
                    print('solve conv problem', end - start)
                
                #print('stat ', convStatus, constrainedConvSolver.solution.get_status_string())
                if convSolnFound == -1 :
                    print( '========== ERROR: Problem is INFEASIBLE ==========')
                    return list(), list(), list()
                            
                if convSolnFound == 1 :
                    rVarsModel          = constrainedConvSolver.solution.get_values(self.rVars)
                    solutionFound       = 1
                    if self.verbose == 'ON':
                        print ('========== Solution Found =========')
                    return rVarsModel, bModel, convIFModel
                else:
        # ------------ Find counterexample----------------------------------------------
                    if self.profiling == 'true':
                        start = timeit.default_timer()
                    counterExamples         = self.__generateCounterExample(constrainedConvSolver, convIFModel)
                    if not counterExamples: # no counter example can be found .. something is wrong
                        print ('========== ERROR: Problem is UNSAT ==========')
                        return list(), list(), list()
                    if self.profiling == 'true':
                        end = timeit.default_timer()
                        print('gen counterexample', end - start)
        # ------------ Add counter examples to SAT solver --------------------------------
                    for counterExample in counterExamples:
                        self.SATsolver.add(counterExample)
        # ------------ END OF MAIN LOOP -------------------------------------------------

        return list(), list(), list()
    
    # ========================================================
    #               Extract SAT Model
    # ========================================================
    def __extractSATModel(self):
        z3Model                 = self.SATsolver.model()
        convIFModel             = [z3.is_true(z3Model[bConv])   for bConv   in self.convIFClauses]
        bModel                  = [z3.is_true(z3Model[b])       for b       in self.bVars]
        return convIFModel, bModel



    # ========================================================
    #               Prepare Convex Problem
    # ========================================================
    def __prepareConvexProblem(self, convIFModel):
        # Create a new instance of the convex solver. Initial solver contains the original convex constraints.
        # Add names to these constraints and then use their names to the delete some constraints.

        # Create a new CPLEX solver object from the origional data
        constrainedConvSolver           = cplex.Cplex(self.ConvSolver)
        constrainedConvSolver.parameters.threads.set(self.numberOfCores)

        if self.verbose == 'OFF':
            constrainedConvSolver.set_results_stream(None)
            constrainedConvSolver.set_log_stream(None)

        end = timeit.default_timer()


        # Extract only those constraints that are assigned to false
        inactiveIFClauses               = [i for i, x in enumerate(convIFModel) if x != True]
        #activeIFClauses                 = [i for i, x in enumerate(convIFModel) if x == True]

        # Delete the corresponding "slacked" constraints
        for clauseIndex in inactiveIFClauses:
            clause                      = self.__convIFClauses[clauseIndex]
            if clause['type'] == 'LP':
                constrainedConvSolver.linear_constraints.delete(clause['name'])
            elif clause['type'] == 'QP':
                constrainedConvSolver.quadratic_constraints.delete(clause['name'])
                
        #print constrainedConvSolver.linear_constraints.get_senses()
        #print constrainedConvSolver.linear_constraints.get_rhs()
        #print constrainedConvSolver.linear_constraints.get_rows()
        
        
        if self.counterExampleStrategy == 'PREFIX':
            activeIFClauses                 = [i for i, x in enumerate(convIFModel) if x == True]

            print ('activeIFClauses = ', activeIFClauses)
            ii = [i * 27 for i in range(0, len(activeIFClauses))]
            print ([x - y for x, y in zip(activeIFClauses, ii)])

            print ('activeIFClauses = ', activeIFClauses)
            #for activeIfClausecounter in range(1, len(activeIFClauses)):
            pastSlackIndecies                      = list()
            for activeIfClause in activeIFClauses:
                if not pastSlackIndecies: # if is empty
                    pastSlackIndecies.append(activeIfClause)
                    continue
                
                
                pastSlacks                  = [self.__slackIFVarsBound[i] for i in pastSlackIndecies]
                currentSlack                = [self.__slackIFVarsBound[activeIfClause]]

                #print currentSlack, [pastSlacks[-1]]
                #constrainedConvSolver.linear_constraints.add(
                #                       lin_expr = [cplex.SparsePair(ind = currentSlack + pastSlacks,
                #                        val =  [-1] + [self.slackRatio]*len(pastSlacks))],
                #                        senses          = ['L'],
                #                        rhs             = [0.0],
                #        )
                #print 'pastSlacks', pastSlacks, 'currentSlack', currentSlack, currentSlack + pastSlacks,[pastSlacks[-1]]

                constrainedConvSolver.linear_constraints.add(
                                       lin_expr = [cplex.SparsePair(ind = currentSlack + [pastSlacks[-1]],
                                        val =  [-1] + [1])],
                                        senses          = ['L'],
                                        rhs             = [0.0],
                        )
        
                pastSlackIndecies.append(activeIfClause)
                
        #print constrainedConvSolver.linear_constraints.get_senses()
        #print constrainedConvSolver.linear_constraints.get_rhs()
        #print constrainedConvSolver.linear_constraints.get_rows()
        
        return constrainedConvSolver

    # ========================================================
    #               Solve Convex Problem
    # ========================================================
    def __solveConvexProblem(self, constrainedConvSolver, convIFModel):
        if self.counterExampleStrategy == 'SMC':
            #constrainedConvSolver.feasopt(constrainedConvSolver.feasopt.upper_bound_constraints())
            constrainedConvSolver.solve()
            if self.verbose == 'ON':
                print ('SMC status = ', constrainedConvSolver.solution.get_status(), constrainedConvSolver.solution.get_status_string())
            
            slackVarsModel                      =   np.array(constrainedConvSolver.solution.get_values(self.__slackIFVarsBound))
            #problemIFClauses                    =   [i for i, x in enumerate(slackVarsModel) if x > self.slackTolerance]
            #print 'slack =', sum(slackVarsModel), self.__slackIFVarsBound, constrainedConvSolver.solution.get_values(self.__slackIFVarsBound)
            
            #for clauseIndex in problemIFClauses:
            #    clause                              = self.__convIFClauses[clauseIndex]
                #print constrainedConvSolver.linear_constraints.get_rhs(clause['name'])
                #print constrainedConvSolver.linear_constraints.get_rows(clause['name'])
            
            

            #if constrainedConvSolver.solution.get_status() == 23: # 'feasible solution from infeasib analysis'
            #    convSolnFound       = 1
            #elif constrainedConvSolver.solution.get_status() == 14:
            #    convSolnFound       = 0
            #else:
            #    convSolnFound       = -1 # something went wrong
            #    print 'infeasible'
                
            if abs(sum(slackVarsModel)) < self.slackTolerance:
                convSolnFound       = 1
            else:
                convSolnFound       = 0
    
        elif self.counterExampleStrategy == 'IIS':
            constrainedConvSolver.solve()
            if self.verbose == 'ON':
                print ('IIS status = ', constrainedConvSolver.solution.get_status(), constrainedConvSolver.solution.get_status_string())
            
            #print constrainedConvSolver.variables.get_upper_bounds(self.__slackIFVarsBound)
            #print constrainedConvSolver.linear_constraints.get_rhs()
            #print constrainedConvSolver.linear_constraints.get_rows()


            #slackVarsModel                      =   np.array(constrainedConvSolver.solution.get_values(self.__slackIFVarsBound))
            #problemIFClauses                    =   [i for i, x in enumerate(slackVarsModel) if x > self.slackTolerance]

            #print 'slackVarsModel = ', slackVarsModel, 'problemIFClauses = ', problemIFClauses

            
            if constrainedConvSolver.solution.get_status() == 1:
                convSolnFound = 1
            elif constrainedConvSolver.solution.get_status() != 1:
                convSolnFound = 0
            else:
                convSolnFound       = -1 # something went wrong

        elif self.counterExampleStrategy == 'PREFIX':
            constrainedConvSolver.solve()
            if self.verbose == 'ON':
                print ('PREFIX status = ', constrainedConvSolver.solution.get_status(), constrainedConvSolver.solution.get_status_string())
            
            #constrainedConvSolver.conflict.refine(constrainedConvSolver.conflict.upper_bound_constraints(1.0, self.__slackIFVars))
            #conflictMembers          = constrainedConvSolver.conflict.get()
            #print [i for i, x in enumerate(conflictMembers) if x > 0]
            #print conflictMembers

            activeIFClauses                     = [i for i, x in enumerate(convIFModel) if x == True]
            slackVarsModel                      = np.array(constrainedConvSolver.solution.get_values(self.__slackIFVarsBound))
            activeSlackVarsModel                = [slackVarsModel[i] for i in activeIFClauses]
            print ('activeSlackVarsModel', activeSlackVarsModel)
            problemIFClauses                    = [i for i, x in enumerate(activeSlackVarsModel) if x > self.slackTolerance]

            #print 'slackVarsModel = ', slackVarsModel, 'problemIFClauses = ', problemIFClauses, 'sum = ', sum(slackVarsModel), 'self.slackTolerance', self.slackTolerance
            if not problemIFClauses: #max(abs(slackVarsModel)) < self.slackTolerance:
                convSolnFound       = 1
            else:
                convSolnFound = 0
        
                # generate counter example directly
                #slackVarsModelList                      = slackVarsModel.tolist()
            
                convIFModelIndex                        = [i for i, x in enumerate(convIFModel) if x == True]
                print( convIFModelIndex, problemIFClauses)
                #indexOfFirstNonZeroSlack                = convIFModelIndex.index(problemIFClauses[0])
                counterExample                          = [activeIFClauses[i] for i in range(0, problemIFClauses[0]+1)]
                #counterExample                          = [convIFModelIndex[i] for i in range(indexOfFirstNonZeroSlack,indexOfFirstNonZeroSlack+1)]
                #print "indexOfFirstNonZeroSlack", indexOfFirstNonZeroSlack, convIFModelIndex[0:indexOfFirstNonZeroSlack]
                print (counterExample)
                self.counterExamples.append(counterExample)
                #print "solve = ", self.counterExamples, indexOfFirstNonZeroSlack
            
        elif self.counterExampleStrategy == 'Trivial':
            constrainedConvSolver.solve()
            if constrainedConvSolver.solution.get_status() == 1:
                convSolnFound       = -1 # something went wrong
            # the problem is always feasible since slack variables are upper bounded by infinity
            elif sum(constrainedConvSolver.solution.get_values(self.__slackIFVars)) < self.slackTolerance:
                convSolnFound = 1
            else:
                convSolnFound = 0
        return convSolnFound

    # ========================================================
    #               Generate Counterexample
    # ========================================================
    def __generateCounterExample(self, constrainedConvSolver, convIFModel):
        counterExamples                         = list()
        if self.verbose == 'ON':
            print ('********* Generating Counterexample *********')
        if self.counterExampleStrategy == 'SMC':
            inactiveIFClauses                   = [i for i, x in enumerate(convIFModel) if x != True]
            slackVarsModel                      = np.array(constrainedConvSolver.solution.get_values(self.__slackIFVarsBound))

            # all slacks are positive, therefore assigning -1 to inactive slacks forces them to be at the beginning
            # of the sorted list
            slackVarsModel[inactiveIFClauses]   = -1.0
            
            # Now, sort the slacks and get rid of the inactive slacks
            sortedIndices                      = sorted(range(len(slackVarsModel)), key=lambda k: slackVarsModel[k])
            sortedIndices                      = sortedIndices[len(inactiveIFClauses):]
            
            counterExampleFound                 = 0
            
            #print "sortedIndices",sortedIndices, "slackVarsModel", slackVarsModel[sortedIndices]
            
            counterExampleIFModel                                   = [False] * len(convIFModel)
            # assign the largest slack to true
            counterExampleIFModel[sortedIndices[-1]]                = True
            
            
            counter                                                 = 0
            #print "counterExampleIFModel", counterExampleIFModel
            
            while counterExampleFound == 0:
                counterExampleIFModel[sortedIndices[counter]]       = True
                counterExampleConvSolver  = self.__prepareConvexProblem(counterExampleIFModel)
                counterExampleConvSolver.solve()
                slackVarsModel                      =   np.array(counterExampleConvSolver.solution.get_values(self.__slackIFVarsBound))
                #print "counter example slacks", slackVarsModel
                
                if sum(slackVarsModel) > self.slackTolerance:
                    counterExampleFound         = 1
                    activeIFClauses             = [i for i, x in enumerate(counterExampleIFModel) if x == True]
                    counterExample              = z3.Or([ self.convIFClauses[counter] != convIFModel[counter] for counter in activeIFClauses ])
                    self.counterExamples.append(activeIFClauses)
                    counterExamples.append(counterExample)
                    #print "counter Example", activeIFClauses, "full model", sortedIndices
                else:
                    counter                                         = counter + 1
            
        
            '''
            # just for one core right now
            counterExampleFound                 = 0
            counterExampleIFModel               = [[] for i in range(self.numberOfCores)]
            
            
            for coreCounter in range(0,self.numberOfCores):
                counterExampleIFModel[coreCounter]                                  = [False] * len(convIFModel)
                counterExampleIFModel[coreCounter][sortedIndices[-(coreCounter+1)]]= True
            
            counter                             = 0
            
            
            
            
            while counterExampleFound == 0:
                return_queue                    = multiprocessing.Queue()
                jobs                            = []
                
                counterExampleConvSolver        = [None]*self.numberOfCores
                for coreCounter in range(0,self.numberOfCores):
                    print 'counterExampleIFModel', counterExampleIFModel, 'sortedIndices', sortedIndices, 'counter', counter
                    
                    counterExampleIFModel[coreCounter][sortedIndices[counter]]  = True
                    counterExampleConvSolver[coreCounter]  = self.__prepareConvexProblem(counterExampleIFModel[coreCounter])
                
                    p = multiprocessing.Process(target = _generateCounterExampleThreadRun,
                                                args= (counterExampleConvSolver[coreCounter], coreCounter, return_queue))
                    jobs.append(p)
                    p.start()
            
                for proc in jobs:
                    proc.join()
            
                for coreCounter in range(0,self.numberOfCores):
                    return_queue_instant            = return_queue.get()
                    counterExampleFoundCore         = return_queue_instant[1]
                    coreNumber                      = return_queue_instant[0]
                    if counterExampleFoundCore == 1:
                        counterExampleFound         = 1
                        activeIFClauses             = [i for i, x in enumerate(counterExampleIFModel[coreNumber]) if x == True]
                        counterExample              = sum([BoolVar2Int(self.convIFClauses[i]) for i in activeIFClauses] ) <= len(activeIFClauses) - 1
                        self.counterExamples.append(activeIFClauses)
                        counterExamples.append(counterExample)
                
                # check infeasibility
                # counterExampleConvSolver.feasopt(counterExampleConvSolver.feasopt.upper_bound_constraints())
                
                # if infeasible, then we found a counter example
                # if counterExampleConvSolver.solution.get_status() == 14:
                #    counterExampleFound         = 1
                # else:
                #    counter                     = counter + 1
            
                counter                             = counter + 1
            '''
        elif self.counterExampleStrategy == 'IIS':
            # find a feasible relaxation of a problem by relaxing the upper bounds of the slack variables
            # (all other vars have upper bound equal to infinity)

            try:
                constrainedConvSolver.conflict.refine(constrainedConvSolver.conflict.upper_bound_constraints(1.0, self.__slackIFVarsBound))
                conflictMembers          = constrainedConvSolver.conflict.get()
                #print 'status code =', constrainedConvSolver.conflict.group_status[3], constrainedConvSolver.conflict.group_status[-1], constrainedConvSolver.conflict.group_status[5]
                #if self.verbose == 'ON':
                #print('refiner', conflictMembers)
                activeIFClauses             = [i for i, x in enumerate(conflictMembers) if x > 0]
                counterExample              = z3.Or([ self.convIFClauses[counter] != convIFModel[counter] for counter in activeIFClauses ])
                self.counterExamples.append(activeIFClauses)
                counterExamples.append(counterExample)

                #print 'IIS CE = ', activeIFClauses
            except:
                counterExample              = []
            
            """
            constrainedConvSolver.feasopt(constrainedConvSolver.feasopt.upper_bound_constraints())
            slackVarsModel          = constrainedConvSolver.solution.get_values(self.__slackIFVars)

            activeIFClauses         = [i for i, x in enumerate(convIFModel) if x == True]
            activeSlackVars         = np.array(slackVarsModel)[activeIFClauses]
            print('infeasb ', activeIFClauses, activeSlackVars)
            # extract the constraints that have high slack variabeles
            activeIFClauses         = [i for i, x in enumerate(slackVarsModel) if x > self.slackTolerance]
            """
        elif self.counterExampleStrategy == 'PREFIX':
            # counter example was generated while solving the convex problem
            # reterieve the last one in the counter example list
            activeIFClauses         = self.counterExamples[-1]
            #print "activeIFClauses", activeIFClauses, self.counterExamples
            #counterExample          = z3.Or([ self.convIFClauses[counter] != True for counter in activeIFClauses ])
            antecedent              = sum([BoolVar2Int(self.convIFClauses[i]) for i in activeIFClauses[0:-1]] ) <= len(activeIFClauses[0:-1]) - 1
            consequent              = self.convIFClauses[activeIFClauses[-1]] != True
            #print 'activeIFClauses',activeIFClauses, 'antecedent', [self.convIFClauses[i] for i in activeIFClauses[0:-1]], 'consequent', consequent
            counterExample          = z3.Or(antecedent, consequent)
            counterExamples.append(counterExample)
        
        
        elif self.counterExampleStrategy == 'Trivial':
            # trivial counterexample: extract all constraints that are assigned to true
            activeIFClauses         = [i for i, x in enumerate(convIFModel) if x == True]

            counterExample              = z3.Or([ self.convIFClauses[counter] != convIFModel[counter] for counter in activeIFClauses ])
            self.counterExamples.append(activeIFClauses)
            counterExamples.append(counterExample)
        else:
            # extract only those constraints that are assigned to true
            activeIFClauses         = [i for i, x in enumerate(convIFModel) if x == True]
            slackVarsModel          = constrainedConvSolver.solution.get_values(self.__slackIFVars)
            activeSlackVars         = np.array(slackVarsModel)[activeIFClauses]


            counterExample          = z3.Or([ self.convIFClauses[counter] != convIFModel[counter] for counter in activeIFClauses ])
            counterExamples.append(counterExample)
            self.counterExamples.append(activeIFClauses)
        
        #print "CE", activeIFClauses
        return counterExamples



    # ========================================================
    #               Add Slack to Convex Clauses
    # ========================================================
    def __slackConstraint(self, constraint, slackVariable):
        if constraint['type'] == 'LP':
            return self.__slackLPConstraint(constraint, slackVariable)
        elif constraint['type'] == 'QP':
            return self.__slackQPConstraint(constraint, slackVariable)

    # ========================================================
    #               Add Slack to Linear Clauses
    # ========================================================
    def __slackLPConstraint(self, constraint, slackVariable):
        slacked_lin_expr            = list()
        numOfRows                   = len(constraint['rhs'])
        # to slack this constraint, we edit the linear constraint as:
        # Ax \le b + s *1 (where 1 is a vector of all ones)
        # [A -1] [x s]^T \le b
    
        slacked_lin_expr            = list()
        A                           = constraint['A']
        b                           = constraint['rhs']
        newRhs                      = list()

        for counter in range(0, numOfRows):
            A_row                   = A[counter,:]
            rVars                   = constraint['x']
            slacked_lin_expr.append(cplex.SparsePair(
                    ind             = constraint['x'] + [slackVariable],
                    val             = np.append(A_row, -1.0)
                    ))
            newRhs.append(b[counter])
        
        '''
        # Second we create the constraint Ax \le b - s *1
        for counter in range(0, numOfRows):
            A_row                   = A[counter,:]
            rVars                   = constraint['x']
            slacked_lin_expr.append(cplex.SparsePair(
                    ind             = constraint['x'] + [slackVariable],
                    val             = np.append(A_row, 1.0)
                    ))
            newRhs.append(b[counter])
        '''
        
        #newRhs                      = np.concatenate(constraint['rhs'], constraint['rhs'])
        slackedConstraint           = {
                    'type'          :constraint['type'],
                    'lin_expr'      :slacked_lin_expr,
                    'rhs'           :newRhs,
                    'x'             :constraint['x'],
                    's'             :slackVariable,
                    'senses'        :constraint['senses'], #+ constraint['senses'],
                    'A'             :constraint['A'],
                    'name'          :['slckd'+slackVariable+str(i) for i in range(0,len(newRhs))]
                    }

        return slackedConstraint

    # ========================================================
    #               Add Slack to Quadratic Clauses
    # ========================================================
    def __slackQPConstraint(self, constraint, slackVariable):
        # to slack this constraint, we edit the linear constraint as:
        # x^T Q x + c^T x \le b + s
        # x^T Q x + (c^T x - s) \le b
        # x^T Q x + [c^T -1] [x s] \le b
        
        slacked_lin_expr        = cplex.SparsePair(
                                    ind = constraint['x'] + [slackVariable],
                                    val = np.append(constraint['c'], -1.0)
                                )
        slackedConstraint       = {
                    'type'      :constraint['type'],
                    'quad_expr' :constraint['quad_expr'],
                    'lin_expr'  :slacked_lin_expr,
                    'rhs'       :constraint['rhs'],
                    'x'         :constraint['x'],
                    's'         :slackVariable,
                    'sense'     :constraint['sense'],
                    'Q'         :constraint['Q'],
                    'c'         :constraint['c'],
                    'name'      :'slckd'+slackVariable
                }
        
        return slackedConstraint



#=========================================================
#       Public Helper APIs
#=========================================================

# ============ Helper Functions to construct Boolean clauses ======================
def AND(*b):
    return z3.And(b)

def OR(*b):
    return z3.Or(b)

def NOT(b):
    return z3.Not(b)

def IMPLIES(b1, b2):
    return z3.Implies(b1, b2)

def EQUIV(b1, b2):
    return b1 == b2
    #return AND(IMPLIES(b1, b2), IMPLIES(b2, b1))

# ============ Helper Functions to construct pseudo Boolean clauses ===============
def BoolVar2Int(b):
    return  z3.If(b, 1, 0)



# ============ Define Quadratic Clause ======================
def QPClause(Q, c, b, rVars, sense = "L"):
    # x = rVars
    # sense = "L" (less than), "G" (greater than), "E" (equal) for
    #x^T Q x + c^T x {sense} b
    
    
    # TODO: add a dimension check
    
    
    # Put the constraint in CPLEX format, example below:
    # l = cplex.SparsePair(ind = ['x'], val = [1.0])
    # q = cplex.SparseTriple(ind1 = ['x'], ind2 = ['y'], val = [1.0])
    # c.quadratic_constraints.add(name = "my_quad",
    #                             lin_expr = l,
    #                             quad_expr = q,
    #                             rhs = 1.0,
    #                             sense = "G")
    
    ind1            = list()
    ind2            = list()
    val             = list()
    matrixSize      = len(rVars)
    
    for counter in range(0, matrixSize):
        for counter2 in range(0, matrixSize):
            ind1.append(rVars[counter])
            ind2.append(rVars[counter2])
            val.append(Q[counter,counter2])
    
    lin_expr        = cplex.SparsePair(ind = rVars, val = c)
    quad_expr       = cplex.SparseTriple(ind1 = ind1, ind2 = ind2, val = val)
    rhs             = b
    
    
    constraint      = { 'type':'QP', 'quad_expr':quad_expr, 'lin_expr':lin_expr, 'rhs':rhs, 'x':rVars, 'sense':sense,
                        'Q':Q,  'c':c}
    return constraint

# ============ Define Linear Clause ======================
def LPClause(A, b, rVars, sense = "L"):
    # x = rVars
    # sense = "L", "G", "E"
    #A x {sense} b, A is a matrix and b is a vector with same dimension
    
    
    # TODO: add a dimension check
    
    
    # Put the constraint in CPLEX format, example below:
    # lin_expr = [cplex.SparsePair(ind = ["x1", "x3"], val = [1.0, -1.0]),\
    #    cplex.SparsePair(ind = ["x1", "x2"], val = [1.0, 1.0]),\
    #    cplex.SparsePair(ind = ["x1", "x2", "x3"], val = [-1.0] * 3),\
    #    cplex.SparsePair(ind = ["x2", "x3"], val = [10.0, -2.0])],\
    # senses = ["E", "L", "G", "R"],\
    # rhs = [0.0, 1.0, -1.0, 2.0],\
    
    numOfRows       = len(b)
    lin_expr        = list()
    
    for counter in range(0, numOfRows):
        lin_expr.append(cplex.SparsePair(ind = rVars, val = A[counter,:]))
    
    rhs             = b
    senses          = [sense] * numOfRows

    constraint  = {'type':'LP', 'lin_expr':lin_expr, 'rhs':rhs, 'x':rVars, 'senses':senses, 'A':A}
    return constraint






def _generateCounterExampleThreadRun(counterExampleConvSolver, instanceindex, return_q):
    counterExampleConvSolver.parameters.threads.set(1)
    # check infeasibility
    counterExampleConvSolver.feasopt(counterExampleConvSolver.feasopt.upper_bound_constraints())
        
    # if infeasible, then we find a counter example
    if counterExampleConvSolver.solution.get_status() == 14:
        return_q.put([instanceindex, 1])
    else:
        return_q.put([instanceindex, 0])



