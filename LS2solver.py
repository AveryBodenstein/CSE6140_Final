#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import copy
import random
import time
import warnings

import numpy
from commonLib import parse_graph
from deap import base, creator, tools, algorithms

# Build the adjacency list of the graph.
def getAdjacencyList(nodeLabel, adjacent):
    adjDict = {}
    for i in range(len(nodeLabel)):
        adjDict[nodeLabel[i]] = adjacent[i]
    return adjDict

# Create the edge list of the graph.
def getEdgeList(adjacencyList, numberofNodes):
    edgeList = []
    for i in range(1, numberofNodes + 1):
        for j in range(i+1, numberofNodes + 1):
            if j in adjacencyList[i]:
                edgeList.append((i, j))
    return edgeList

# The cost function, calculates the number of edges not present in the solution.
def trueCost(currentSolution, edgeList, numNodes):
    if(len(currentSolution) == 0):
        return len(edgeList)
    cost = 0
    setSoln = set(currentSolution)
    for edge in edgeList:
        if edge[0] not in setSoln and edge[1] not in setSoln:
            cost = cost + 1
    return (cost, )

# Function to build the starting population.
def initiate(container, func, n):
    starting_set = set()
    for i in range(n):
        while True:
            candidate = func()
            if candidate not in starting_set:
                starting_set.add(candidate)
                break
    return container(starting_set)

# Mutation function.
# Changes one element of the individual.
def fixedSizeMutate(parent, low, up):
    if len(parent) == 0:
        return (parent,)
    child = copy.deepcopy(parent)
    child.remove(child[random.randint(0, len(child)-1)])
    while True:
        candidate = random.randint(low, up)
        if candidate not in child:
            child.append(candidate)
            break
    return (child,)

# A modified version of the DEAP library's easimple algorithm.
def custom_eaSimple(population, toolbox, cxpb, mutpb, ngen, timeLimit=None,
            verbose=__debug__):
    start_time = None
    if timeLimit is not None:
        start_time = time.perf_counter()
    # Evaluate the individuals with an invalid fitness.
    invalid_ind = [ind for ind in population if not ind.fitness.valid]
    fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)

    best_fit = 10000000
    gen_best_fit = 0
    for ind, fit in zip(invalid_ind, fitnesses):
        ind.fitness.values = fit
        if fit[0] < best_fit:
            best_fit = fit[0]

    if verbose:
        print("0 " + str(len(invalid_ind)) + " " + str(best_fit))

    # Begin the generational process.
    for gen in range(1, ngen + 1):
        if timeLimit is not None:
            cur_time = time.perf_counter()
            if (cur_time - start_time) > timeLimit:
                print("Out of time")
                break

        if best_fit == 0:
            return True, population

        if gen - gen_best_fit > 30:
            return False, population

        # Select the next generation individuals.
        offspring = toolbox.select(population, len(population))

        # Vary the pool of individuals
        offspring = algorithms.varAnd(offspring, toolbox, cxpb, mutpb)

        # Evaluate the individuals with an invalid fitness.
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit
            if fit[0] < best_fit:
                best_fit = fit[0]
                gen_best_fit = gen

        # Replace the current population by the offspring.
        population[:] = offspring

        if verbose:
            print(str(gen) + " " + str(len(invalid_ind)) + " " + str(best_fit))

    return False, population

# Build the genetic algorithm components and run it.
def geneticBinary(numNodes, indSize, edgeList, timeLimit=None):
    creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
    creator.create("Individual", list, fitness=creator.FitnessMin)

    toolbox = base.Toolbox()
    toolbox.register("node", random.randint, 1, numNodes)
    toolbox.register("individual", initiate, creator.Individual, toolbox.node, n=indSize)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    toolbox.register("evaluate", trueCost, edgeList=edgeList, numNodes=numNodes)
    toolbox.register("mate", tools.cxTwoPoint)
    toolbox.register("mutate", fixedSizeMutate, low=1, up=numNodes)
    toolbox.register("select", tools.selTournament, tournsize=4)

    pop_size = 1000
    num_gens = 1000
    if numNodes > 2000:
        pop_size = 100
        num_gens = 200
    pop = toolbox.population(n=pop_size)

    result, pop = custom_eaSimple(pop, toolbox, cxpb=0.5, mutpb=0.5, ngen=num_gens, timeLimit=timeLimit, verbose=False)

    return result, pop

def getNodeDegrees(nodeLabel, nodeEdges):
    degree = {}
    for i in range(len(nodeLabel)):
        degree[nodeLabel[i]] = nodeEdges[i]
    return degree

# Implements a binary search decision problem genetic algorithm.
def solveLS2(inputFile,outputFile=None,maxTime=600,initSeed=100):
    warnings.filterwarnings("ignore")
    random.seed(initSeed)

    if outputFile is not None:
        sol_file = open(outputFile + ".sol", "w")
        trace_file = open(outputFile + ".trace", "w")

    adjacent,nodeEdges,nodeLabel,nNodes,nEdges = parse_graph(inputFile)
    print(nNodes)
    print(nEdges)
    adjacencyList = getAdjacencyList(nodeLabel, adjacent)
    edgeList = getEdgeList(adjacencyList, nNodes)
    print("start")

    best_pop = None
    best_sol = nNodes

    start_time = time.perf_counter()
    
    high = nNodes
    low = 0

    if nNodes > 4000:
        result, best_pop = geneticBinary(numNodes=nNodes, indSize=nNodes-10, edgeList=edgeList, timeLimit=maxTime)
        high = nNodes - 11

    while low < high:
        cur_time = time.perf_counter()
        if (cur_time - start_time > maxTime):
            print("Out of time")
            break
        mid = int((high + low) / 2)
        result, pop = geneticBinary(nNodes, indSize=mid, edgeList=edgeList, timeLimit=(start_time + maxTime - cur_time))
        print("Size:" + str(mid) + " Result:" + str(result))
        if result:
            high = mid-1
            write_time = time.perf_counter()
            if outputFile is not None:
                trace_file.write(str(write_time - start_time) + " " + str(mid) + "\n")
            if mid < best_sol:
                best_sol = mid 
                best_pop = pop
        else:
            low = mid+1
    
    if outputFile is not None:
        sol_file.write(str(best_sol) + "\n")
        if best_pop is not None:
            pop.sort(key=lambda x: x.fitness, reverse=True)
            for item in pop[0]:
                sol_file.write(str(item) + " ")
        
        sol_file.close()
        trace_file.close()



if __name__ == "__main__":   

    inputFile = './DATA-1/jazz.graph'
    # inputFile = './DATA-1/karate.graph'
    # inputFile = './DATA-1/football.graph'
    # inputFile = './DATA-1/as-22july06.graph'
    # inputFile = './DATA-1/star2.graph'    
    # inputFile = './DATA-1/netscience.graph'
    # inputFile = './DATA-1/delaunay_n10.graph'

    # solveLS2(inputFile, outputFile="test_output")
    solveLS2(inputFile, outputFile=None)
    
    # result, pop = geneticBinary(nNodes, indSize=int(899), edgeList=edgeList, seed=100)
    # pop.sort(key=lambda x: x.fitness, reverse=True)
    # print(pop[0].fitness)
    # print(len(pop[0]))

    