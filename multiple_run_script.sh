#!/bin/sh

# for i in `seq 1000 1000000 10001000 `;
for i in `10 1000 10000 100000 `;
# for i in `seq 10 10 100 `;
do
         ./build/main_mykdtree $i
done
