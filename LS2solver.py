#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 11 14:39:05 2022

@author: Avery Bodenstein

Local Search Vertex Cover Solver (2)
Genetic Algorithms
"""

import copy
import random
import time

import numpy
from commonLib import parse_graph
from deap import base, creator, tools, algorithms

def getAdjacencyList(nodeLabel, adjacent):
    adjDict = {}
    for i in range(len(nodeLabel)):
        adjDict[nodeLabel[i]] = adjacent[i]
    return adjDict

def getEdgeList(adjacencyList, numberofNodes):
    edgeList = []
    for i in range(1, numberofNodes + 1):
        for j in range(i+1, numberofNodes + 1):
            if j in adjacencyList[i]:
                edgeList.append((i, j))
    return edgeList

def trueCost(currentSolution, edgeList, numNodes):
    if(len(currentSolution) == 0):
        return len(edgeList)
    cost = 0
    setSoln = set(currentSolution)
    for edge in edgeList:
        if edge[0] not in setSoln and edge[1] not in setSoln:
            cost = cost + 1
    return (cost, )

def initiate(container, func, n):
    starting_set = set()
    for i in range(n):
        while True:
            candidate = func()
            if candidate not in starting_set:
                starting_set.add(candidate)
                break
    return container(starting_set)

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

def custom_eaSimple(population, toolbox, cxpb, mutpb, ngen, stats=None,
             halloffame=None, verbose=__debug__):
    # Evaluate the individuals with an invalid fitness
    invalid_ind = [ind for ind in population if not ind.fitness.valid]
    fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)

    best_fit = 10000000
    gen_best_fit = 0
    for ind, fit in zip(invalid_ind, fitnesses):
        ind.fitness.values = fit
        if fit[0] < best_fit:
            best_fit = fit[0]

    if halloffame is not None:
        halloffame.update(population)

    if verbose:
        print("0 " + str(len(invalid_ind)) + " " + str(best_fit))

    # Begin the generational process
    for gen in range(1, ngen + 1):
        if best_fit == 0:
            return True, population

        if gen - gen_best_fit > 30:
            # print("Returning False")
            return False, population

        # Select the next generation individuals
        offspring = toolbox.select(population, len(population))

        # Vary the pool of individuals
        offspring = algorithms.varAnd(offspring, toolbox, cxpb, mutpb)

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit
            if fit[0] < best_fit:
                best_fit = fit[0]
                gen_best_fit = gen

        # Replace the current population by the offspring
        population[:] = offspring

        if verbose:
            print(str(gen) + " " + str(len(invalid_ind)) + " " + str(best_fit))

    print("best fit:" + str(best_fit))
    return False, population

def geneticBinary(numNodes, indSize, edgeList, seed, timeLimit=None):
    random.seed(seed)
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

    pop = toolbox.population(n=1000)

    # print("setup done")
    result, pop = custom_eaSimple(pop, toolbox, cxpb=0.5, mutpb=0.5, ngen=1000, verbose=False)

    return result, pop

def getNodeDegrees(nodeLabel, nodeEdges):
    degree = {}
    for i in range(len(nodeLabel)):
        degree[nodeLabel[i]] = nodeEdges[i]
    return degree


def solveLS2(inputFile,outputFile,maxTime,initSeed):
    pass

import warnings

if __name__ == "__main__":
    warnings.filterwarnings("ignore")

    inputFile = './DATA-1/jazz.graph'
    # inputFile = './DATA-1/karate.graph'    
    # inputFile = './DATA-1/star2.graph'    
    # inputFile = './DATA-1/netscience.graph'
    adjacent,nodeEdges,nodeLabel,nNodes,nEdges = parse_graph(inputFile)
    print(nNodes)
    print(nEdges)
    adjacencyList = getAdjacencyList(nodeLabel, adjacent)
    edgeList = getEdgeList(adjacencyList, nNodes)
    print("start")
    # result, pop = geneticBinary(nNodes, indSize=int(899), edgeList=edgeList, seed=100)
    # pop.sort(key=lambda x: x.fitness, reverse=True)
    # print(pop[0].fitness)
    # print(len(pop[0]))
    high = nNodes
    low = 0
    while low < high:
        mid = int((high + low) / 2)
        result, pop = geneticBinary(nNodes, indSize=mid, edgeList=edgeList, seed=100)
        print("Size:" + str(mid) + " Result:" + str(result))
        if result:
            high = mid-1
        else:
            low = mid+1