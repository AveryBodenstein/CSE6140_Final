#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 11 14:39:05 2022

@author: Avery Bodenstein

Local Search Vertex Cover Solver (2)
Genetic Algorithms
"""

from commonLib import parse_graph

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

# def cost()



def solveLS2(inputFile,outputFile,maxTime,initSeed):
    pass


if __name__ == "__main__":
    # inputFile = './DATA-1/star2.graph'
    inputFile = './DATA-1/karate.graph'
    # testDict = dict()
    # coverSet = _solveApprox('./DATA-1/star2.graph','./test.out',testDict)
    
    # print(len(coverSet)
    adjacent,nodeEdges,nodeLabel,nNodes,nEdges = parse_graph(inputFile)
    # print(nNodes)
    print(nEdges)
    # print("adjacent")
    # print(adjacent)
    # print("nodeEdges")
    # print(nodeEdges)
    # print("nodeLabel")
    # print(nodeLabel)
    # print(len(nodeEdges) == len(nodeLabel))
    adjacencyList = getAdjacencyList(nodeLabel, adjacent)
    # print(adjacencyList)
    edgeList = getEdgeList(adjacencyList, nNodes)
    print(len(edgeList))
    # print((edgeList))
    print(edgeList[10][1])

    # print(adjDict)
    # print(adjDict[33])