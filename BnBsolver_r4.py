#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 11 2022
@author: Avery Bodenstein
Branch and Bound Vertex Cover Solver
"""

#import numpy as np
import time
import multiprocessing
import heapdict as hd
from commonLib import parse_graph_hd, write_sol ,log_sol
from ApproxSolver import _solveApprox_nofile
import math
import os

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
    nThreads = 12
    maxIter= 1000 # maximum iterations per thread before returning
    nIterations = 1
    nRounds = 0
    bestCost = 2*len(initSol)+5
    # Start solution as a process
    manager = multiprocessing.Manager()
    retSol = [manager.dict()]*nThreads
    while q:
        # run single threaded until at least nThread candidates are found
        while len(q) < nThreads:
            tDict = {
                'coverSet': [],
                'solTime' : 0,
                'q': []
                }
            _BnBWorker(q,nEdges,bestCost,nIterations,10,tDict)
            if tDict['coverSet']:
                print(f"NEW BEST! {len(tDict['coverSet'])}")
                bestCost = len(tDict['coverSet'])
                for ii in range(0,len(tDict['coverSet'])):
                    returnDict['coverSet'].append(tDict['coverSet'][ii])
                    returnDict['solTime'] = [tDict['solTime'][ii]-startTime]
            q = hd.heapdict()
            q = tDict['q']
            nIterations = tDict['nIter']
            #for subprob in tDict['q']:
            #    q[nIterations] = tDict['q'][subprob]
            #    nIterations = nIterations + 1
        if nRounds < 1:
            # divide candidates into threads
            workerCandidates = [hd.heapdict()]*nThreads
            curIter = [nIterations]*nThreads
            ii = 0
            while q:
                temp = q.popitem()
                workerCandidates[ii][temp[0]] = temp[1]
                ii = (ii + 1) % nThreads
        # start each thread
        p = [0]*nThreads
        for ii in range(0,nThreads):
            p[ii] = multiprocessing.Process(target=_BnBWorker, name="BnBWorker", args=(workerCandidates[ii],nEdges,bestCost,curIter[ii],maxIter,retSol[ii]))
            p[ii].start()
        
        # wait for all threads to be done
        join_all(p,600)
        
        # create new worker candidate queues (if necessary)
        workerCandidates = []
        
        nSubproblems = 0
        for ii in range(0,nThreads):
            workerCandidates.append(retSol[ii]['q'])
            nSubproblems = nSubproblems + len(workerCandidates[ii])
            curIter[ii] = curIter[ii] + retSol[ii]['nIter']
            # check for new best cost
            if retSol[ii]['coverSet']:
                if type(retSol[ii]['coverSet'][0]) is list:
                    for jj in range(0,len(retSol[ii]['coverSet'])):
                        if len(retSol[ii]['coverSet'][jj]) < bestCost:
                            #print(f"NEW BEST COST: {len(retSol[ii]['coverSet'][jj])}")
                            bestCost = len(retSol[ii]['coverSet'][jj])
                            returnDict['coverSet'].append(retSol[ii]['coverSet'][jj])
                            returnDict['solTime'] = [retSol[ii]['solTime'][jj]-startTime]
                else:
                    if len(retSol[ii]['coverSet']) < bestCost:
                        #print(f"NEW BEST COST: {len(retSol[ii]['coverSet'])}")
                        bestCost = len(retSol[ii]['coverSet'])
                        returnDict['coverSet'].append(retSol[ii]['coverSet'])
                        returnDict['solTime'] = [retSol[ii]['solTime'][0]-startTime]
        
        q = [0]*nSubproblems
            
        # combine queues from each worker: (DONT DO THIS! IT'S SUPER SLOW!!!)
        # just extract bestCost from each.
        # feed each thread back it's own list as long as it's not length 0
        #q = hd.heapdict()
        #for ii in range(0,nThreads):
        #    for subprob in retSol[ii]['q']:
        #        q[nIterations] = retSol[ii]['q'][subprob]
        #        nIterations = nIterations + 1
        
        # debug
        nRounds = nRounds + 1
        #print(f"{nRounds*nThreads*maxIter}:{nSubproblems}")
            
        if nRounds > 100:
            return
        
    #print('Done!')
    #print(f"{nIterations} Iterations")

def _BnBWorker(q,nEdges,bestCost,curIter,maxIter,returnDict):
    
    returnDict['coverSet'] = []
    returnDict['solTime'] = []
    nIterations = 0
    while q and nIterations < maxIter:
        u = q.popitem()
        solStruct = _BnBSubprocess(u,nEdges,bestCost)
        # extract new candidates
        if len(solStruct['candidates']) == 2:
            q[curIter + nIterations+ 0.25] = solStruct['candidates'][0]
            q[curIter + nIterations+ 0.5] = solStruct['candidates'][1]
        elif len(solStruct['candidates']) == 1:
            q[curIter + nIterations+ 0.25] = solStruct['candidates'][0]
        # check for new best solution
        if solStruct['coverSet']:
            # check for extremely unlikely case that two threads find new bests
            if len(solStruct['coverSet']) < bestCost:
                # update best cost
                bestCost = len(solStruct['coverSet'])
                #temp1 = solStruct['coverSet'].copy()
                #temp2 = solStruct['solTime']
                #print(bestCost)
                #print(temp1)
                #print(temp2)
                # update return struct
                
                #returnDict['butt'] = bestCost
                #returnDict['coverSet'].append(temp1)
                #print(type(solStruct['solTime']))
                #returnDict['solTime'].append(temp2)
                
                #retSet.append(solStruct['coverSet'])
                #retTime.append(solStruct['solTime'])
                if not returnDict['coverSet']:
                    returnDict['coverSet'] = solStruct['coverSet'].copy()
                    returnDict['solTime'] = [solStruct['solTime']]
                else:
                    returnDict['coverSet'].append(solStruct['coverSet'])
                    returnDict['solTime'].append(solStruct['solTime'])
                
        nIterations = nIterations + 1
    returnDict['q'] = q
    returnDict['nIter'] = nIterations
        

def _BnBSubprocess(u,nEdges,bestCost):
    
    returnDict = {
        'coverSet': [],
        'solTime' : 0,
        'candidates': [],
            }
    
    # Branch by either adding, or not adding best node
    withNode, withoutNode = splitNode(u[1],nEdges)

    # Check to see if all edges are now covered
    if withNode[1][2] == nEdges:
        # If so, check cost vs current best
        if len(withNode[1][1]) < bestCost:
            bestCost = len(withNode[1][1])
            #print(f"NEW BEST COST FOUND!!! {len(withNode[1][1])}")
            returnDict['coverSet'] = withNode[1][1]
            returnDict['solTime'] = time.perf_counter()
    elif withNode[1][2] > nEdges:
        raise
    else:
        # If not check lower bound on cost to go
        if withNode[0] < bestCost:
            # make sure it's not empty
            if len(withNode[1][0]) != 0:
                # if minimum cost to go is less than current best add to queue
                returnDict['candidates'].append(withNode)
                
    # check lower bound on cost to go without node:
    if withoutNode[0] < bestCost:
        # make sure it's not empty
        if len(withoutNode[1][0]) != 0:
            # if minimum cost to go is less than current best add to queue
            returnDict['candidates'].append(withoutNode)
    return returnDict

def lowerBound(q,nEdges):
    # get lower bound
    coverSet = edgeDelete(q,nEdges)
    if coverSet == -1:
        #print("trimming...")
        return float('inf')
        
    bound = math.ceil(0.5*len(coverSet))
    return bound
    
# I guess this doesn't work???
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
                
# perform depth first search in q
def dfs(q):
    pass
    
        
def printHD(inDict):
    for key in inDict.d.keys():
        print(f"{key}: {inDict[key]}")

def join_all(threads, timeout):
    """
    Args:
        threads: a list of thread objects to join
        timeout: the maximum time to wait for the threads to finish
    Raises:
        RuntimeError: is not all the threads have finished by the timeout
    """
    start = cur_time = time.time()
    while cur_time <= (start + timeout):
        for thread in threads:
            if thread.is_alive():
                thread.join(timeout=0)
        if all(not t.is_alive() for t in threads):
            break
        time.sleep(0.001)
        cur_time = time.time()
    else:
        still_running = [t for t in threads if t.is_alive()]
        num = len(still_running)
        names = [t.name for t in still_running]
        raise RuntimeError('Timeout on {0} threads: {1}'.format(num, names))

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

if __name__ == '__main__':
    testDict = dict()
    #_solveBnB('./DATA-1/dummy1.graph','./test.out',testDict)
    #_solveBnB('./DATA-1/karate.graph','./test.out',testDict)
    _solveBnB('./DATA-1/jazz.graph','./test.out',testDict)
    # _solveBnB('./DATA-1/email.graph','./test.out',testDict)
    # _solveBnB('./DATA-1/star2.graph','./test.out',testDict)