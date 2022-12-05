#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 27 18:13:26 2022

@author: avery
"""

import heapdict as hd

q = hd.heapdict()
q['one'] = [1,[2,3]]
q['two'] = [2,[3,4]]
q['three'] = [7,[8,9]]

h  = hd.heapdict()
for key in q.d:
    #test = q.d[key]
    #h.d[test[1]] = [test[0].copy(),test[1],test[2]]
    h.d[q.d[key][1]] = [q.d[key][0].copy(),q.d[key][1],q.d[key][2]]
h.heap = q.heap.copy()

j = q.copy()


h['one'][0] = 0
j['one'][0] = 2

print(h['one'])
print(q['one'])
print(j['one'])