#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 11 2022

@author: Avery Bodenstein

Local Search Vertex Cover Solver (1)
"""

import random
import math
import time


def init(V, E, neighbors):
    '''
    find an initial valid solution
    :param V: number of vertices
    :param E: number of edges
    :param neighbors: adjacency list, list of list, 1-indexed
    :return: initial solution, list of vertices(integers)
    '''
    order = list(range(1, V+1))
    random.shuffle(order)
    order = [-1] + order
    ans = []
    covered = set()
    i = 1
    while len(covered) < E:
        curr = order[i]
        for nei in neighbors[curr]:
            edge = (curr, nei)
            if curr > nei:
                edge = (nei, curr)
            covered.add(edge)
        i += 1
    return order[1:i]

def solveLS1(inputFile,outputFile,maxTime,initSeed):
    '''
    solve using simulated annealing, save the result to .sol file and trace to .trace file
    :param inputFile: input file path with suffix,e.g., DATA-1/power.graph
    :param outputFile: output file path without suffix,e.g., OUTPUT/power
    :param maxTime: maximum time in seconds
    :param initSeed: fixed random seed, integer
    :return: No return
    '''
    random.seed(initSeed)
    V = -1
    E = -1
    T_MAX = 10
    k = 10000
    EDGE_LOSS_MAX = 100
    edge_loss = 30
    neighbors = [-1]
    best = (int(1e9), [])
    start_time = time.time()
    trace_str = ''

    with open(inputFile, 'r') as f:
        first = f.readline()
        values = first.split()
        values = [int(value) for value in values]
        if len(values) != 3:
            raise ValueError('First line of input should be |V| |E| 0')
        V = values[0]
        E = values[1]
        for i in range(1, 1+V):
            neighbors.append(list(map(int, f.readline().split())))

    while time.time()-start_time < maxTime:
        init_sol = init(V, E, neighbors)
        v_set = set(init_sol)
        if len(v_set) < best[0]:
            best = (len(v_set), list(v_set))    # (best size, best solution of that size)
        #print(len(v_set))
        edges = set()
        for i in v_set:
            for j in neighbors[i]:
                edges.add((min(i, j), max(i, j)))

        T = T_MAX
        while T > 0:
            next = random.randint(1, V)
            gain = 0
            if next in v_set:   # remove one vertex
                gain += 1
                for j in neighbors[next]:
                    if j not in v_set:  # lost cover to one edge
                        gain -= edge_loss
            else:
                gain -= 1
                for j in neighbors[next]:
                    if j not in v_set:  # got cover to one edge
                        gain += edge_loss

            if gain >= 0:
                p = 1
            else:
                p = math.exp(gain/T)
            if random.random() <= p:
                if next in v_set:
                    v_set.remove(next)
                    for j in neighbors[next]:
                        if j not in v_set:
                            edges.remove((min(next,j), max(next,j)))
                else:
                    v_set.add(next)
                    for j in neighbors[next]:
                        if j not in v_set:
                            edges.add((min(next,j), max(next,j)))

            if len(edges) == E and len(v_set) < best[0]:
                best = (len(v_set), list(v_set))
                trace_str += f'{time.time()-start_time}, {best[0]}'
                trace_str += '\n'
            #else:
            #    print(len(edges)-E, len(v_set))

            T -= T_MAX/k

    output_path = outputFile+'_LS1_'+str(maxTime)+'_'+str(initSeed)+'.sol'
    with open(output_path, 'w') as file:
        file.write(str(best[0])+'\n')
        file.write(','.join(map(str, best[1])))
    trace_path = outputFile+'_LS1_'+str(maxTime)+'_'+str(initSeed)+'.trace'
    with open(trace_path, 'w') as file:
        file.write(trace_str)
    print(best)

    '''
    # check the final solution is valid
    edges = set()
    for i in best[1]:
        for j in neighbors[i]:
            edges.add((min(i, j), max(i, j)))
    print(len(edges) - E)
    '''


solveLS1('DATA-1/power.graph', 'OUTPUT/power', 10, 666)