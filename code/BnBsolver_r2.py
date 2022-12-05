#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 11 2022
@author: Avery Bodenstein
Branch and Bound Vertex Cover Solver

original (stable) single threaded BnB solver
"""

#import numpy as np
import time
import multiprocessing
import heapdict as hd
from commonLib import parse_graph_hd, write_sol ,log_sol
from ApproxSolver import _solveApprox_nofile
import math
import os

# main solver function called from template.py launches a thread to initiate solver
# with a maximum time limit
def solveBnB(inputFile,outputFile,maxTime):
    # Start solution as a process
    manager = multiprocessing.Manager()
    returnDict = manager.dict()
    p = multiprocessing.Process(target=_solveBnB, name="SolveBnB", args=(inputFile,outputFile,returnDict))
    p.start()

    # Wait a maximum of maxTime seconds for solution to complete (if function returns it passes)
    p.join(maxTime)
    
    # If thread is still active
    if p.is_alive():
        print("Maximum Computation Time Exceeded")
    
        # Terminate solver
        p.terminate()
        p.join()
    
    # Write final value to solution file
    if os.path.exists(f"{outputFile}.trace"):
        os.remove(f"{outputFile}.trace")
    write_sol(outputFile,returnDict['coverSet'][-1])
    for (soln,solTime) in zip(returnDict['coverSet'],returnDict['solTime']):
        log_sol(outputFile,len(soln),solTime)
    
# actual solver function that implements single-threaded branch and bound solver
def _solveBnB(inputFile,outputFile,returnDict):
    
    # log start time
    startTime = time.perf_counter()
    # parse graph
    subproblem, nNodes, nEdges = parse_graph_hd(inputFile)
    
    # get approximation via heuristic
    initSol = _solveApprox_nofile(subproblem.copy(),nNodes,nEdges)
    returnDict['coverSet'] = [initSol]
    returnDict['solTime'] = [time.perf_counter()-startTime]
    
    #print(f"Heuristic Solution: {initSol}")
    
    # Create empty priority queue of partial solutions
    q = hd.heapdict()
    q[0] = [lowerBound(subproblem,nEdges),[subproblem,[],0]]
    #print(f"Initial Heuristic Lower Bound: {q[0][0]}")
    # while queue is not empty
    nIterations = 0
    bestCover = 0
    bestCost = len(initSol)
    while q:
        # Choose current candidate with best cost to go
        u = q.popitem()

        #if (nIterations % 1000) == 0:
        #    print(nIterations)
        #    print(len(q))
        nIterations = nIterations + 1
        
        # Branch by either adding, or not adding best node
        withNode, withoutNode = splitNode(u[1],nEdges)
        
        # DEBUG
        if withNode[1][2] > bestCover:
            bestCover = withNode[1][2]
            #print(f"Best Cover: {withNode[1][2]} : {withNode[1][1]}")

        # Check to see if all edges are now covered
        if withNode[1][2] == nEdges:
            # If so, check cost vs current best
            if len(withNode[1][1]) < bestCost:
                bestCost = len(withNode[1][1])
                #print(f"NEW BEST COST FOUND!!! {len(withNode[1][1])}")
                returnDict['coverSet'].append(withNode[1][1])
                returnDict['solTime'].append(time.perf_counter()-startTime)
        elif withNode[1][2] > nEdges:
            raise
        else:
            # If not check lower bound on cost to go
            if withNode[0] < bestCost:
                # make sure it's not empty
                if len(withNode[1][0]) != 0:
                    # if minimum cost to go is less than current best add to queue
                    q[nIterations] = withNode
                    
        # check lower bound on cost to go without node:
        if withoutNode[0] < bestCost:
            # make sure it's not empty
            if len(withoutNode[1][0]) != 0:
                # if minimum cost to go is less than current best add to queue
                q[nIterations+0.5] = withoutNode
        
    #print('Done!')
    #print(f"{nIterations} Iterations")

# get lower bound on cost to go
def lowerBound(q,nEdges):
    # get lower bound
    coverSet = edgeDelete(q,nEdges)
    if coverSet == -1:
        #print("trimming...")
        return float('inf')
        
    bound = math.floor(0.5*len(coverSet))
    return bound
    
# get bounded heuristic solution to vertex cover problem
def edgeDelete(q,nEdges):
    tempQ = q.copy()
    coverSet = []
    coverNo = 0
    while coverNo < nEdges:
        if len(tempQ) == 0:
            coverSet = -1
            break
        # select vertex
        testVertex = tempQ.popitem()
        # get edge (if there is one)
        if testVertex[1][0] > 0:
            vertex2Label = []
            for vertLabel in testVertex[1][1]:
                if vertLabel in tempQ:
                    vertex2Label = vertLabel
                    break
            if vertex2Label:
                vertex2 = tempQ.pop(vertex2Label)
                # add both to cover set
                coverSet.append(testVertex[0])
                coverNo = coverNo + testVertex[1][0]
                coverSet.append(vertex2Label)
                coverNo = coverNo + vertex2[0]-1
                # update edges in q
                for toNode in testVertex[1][1][1:]:
                    # probably kind of slow. Could track "disallowed vertices" seperately for speedup?
                    if toNode in tempQ:
                        # subtract 1 from number of unique edges
                        tempQ[toNode][0] = tempQ[toNode][0] - 1
                        # remove source node
                        tempQ[toNode][1].remove(testVertex[0])
                vertex2[1].remove(testVertex[0])
                for toNode in vertex2[1]:
                    # probably kind of slow. Could track "disallowed vertices" seperately for speedup?
                    if toNode in tempQ:
                        # subtract 1 from number of unique edges
                        tempQ[toNode][0] = tempQ[toNode][0] - 1
                        # remove source node
                        tempQ[toNode][1].remove(vertex2Label)
            else:
                #print(f"vertex {testVertex[0]} is isolated! {testVertex[1][0]} edges")
                coverSet.append(testVertex[0])
                coverNo = coverNo + testVertex[1][0]
    return coverSet
                
# debug function to print contents of heapdict object        
def printHD(inDict):
    for key in inDict.d.keys():
        print(f"{key}: {inDict[key]}")

# split on node with least adjacent edges
def splitNode(inputSubproblem,nEdges):
    # extract values for clarity
    edgesToGo = nEdges - inputSubproblem[1][2]
    # Create output with node selected
    withNode = inputSubproblem
    # get node with least adjacent edges
    curNode = withNode[1][0].popitem()
    # Calculate best cost to go without that node
    lowerBoundWithout = lowerBound(withNode[1][0],edgesToGo)
    # Create output without node selected
    withoutNode = [lowerBoundWithout,[withNode[1][0].copy(),withNode[1][1].copy(),withNode[1][2]]]
    # add current node to withNode partial solution
    withNode[1][1].append(curNode[0])
    withNode[1][2] = withNode[1][2] + curNode[1][0]
    # update edges
    for toNode in curNode[1][1]:
        # probably kind of slow. Could track "disallowed vertices" seperately for speedup?
        if toNode in withNode[1][0]:
            withNode[1][0][toNode][0] = withNode[1][0][toNode][0] - 1
            withNode[1][0][toNode][1].remove(curNode[0])
        else:
            pass
    lowerBoundWith = lowerBound(withNode[1][0],nEdges - withNode[1][2])
    # Calculate lower bound of node with
    withNode[0] = lowerBoundWith
    # Return candidates
    return withNode,withoutNode

# main (for debug)
if __name__ == '__main__':
    testDict = dict()
    #_solveBnB('./DATA-1/dummy1.graph','./test.out',testDict)
    _solveBnB('./DATA-1/karate.graph','./test.out',testDict)
    #_solveBnB('./DATA-1/jazz.graph','./test.out',testDict)
    # _solveBnB('./DATA-1/email.graph','./test.out',testDict)
    # _solveBnB('./DATA-1/star2.graph','./test.out',testDict)