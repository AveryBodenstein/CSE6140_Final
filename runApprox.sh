#!/bin/bash


graphFiles=`ls ./DATA-1/ | grep .graph`

for graph in ${graphFiles}
do
	echo ${graph}
	python template.py -inst ./DATA-1/${graph} -alg Approx -time 10 -seed 7 -out ./OUTPUT

done
