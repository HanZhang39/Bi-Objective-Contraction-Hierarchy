# Contraction Hierarchies (CHs) for Bi-Objective Search

This repo contains the C++ implementations for our ICAPS 2023 paper [1] on CHs for bi-objective search.
This project is still under active development.


## Requirements

+ CMake
+ C++14
+ Boost 

We tested our code only on a MacBook with CMake 3.22 and Boost 1.81. This code might be able to work with CMake and Boost of earlier versions.

## Instructions

``` shell
cmake .
make
```
After typing the above commands. Cmake will generate three executables in the `bin` folder:

+ `./bin/boa` contains the BOA* solver we used as a baseline algorithm.
+ `./bin/contraction` construct a CH for a given map.
+ `./bin/ch_solver` solves queries on a given CH.

For each executable, you can invoke it with `--help` to see the expected input arguments.

Example usage:

``` shell


# build a CH for the NE map with 99.95% of nodes contracted 
./bin/contraction --map USA-road-d.NE.gr USA-road-t.NE.gr  --output ch_NE_9995.txt -l 0.9995

# evaluate the CH for the NE map with BOA+CH+partial_expansion
./bin/ch_solver -i ch_NE_9995.txt -q instances/instances_NE.txt -o NE_9995_BOA_CH_p.txt  -t 1800  --partial True

# or (without partial expansion)
# ./bin/ch_solver -i ch_NE_9995.txt -q instances/instances_NE.txt -o NE_9995_BOA_CH.txt  -t 1800  --partial False

# Run BOA*
# ./bin/boa --map USA-road-d.NE.gr USA-road-t.NE.gr -q instances/instances_NE.txt -o NE_BOA.txt  -t 1800 
```

You can download other road networks we used in the paper from [here](http://www.diag.uniroma1.it/~challenge9/download.shtml).
The `instances` folder contains the instance files we used in our paper.

## References
[1] H. Zhang, O. Salzman, A. Felner, S. Kumar, C. Hernandez and S. Koenig. [Efficient Multi-Query Bi-Objective Search via Contraction Hierarchies](http://idm-lab.org/bib/abstracts/Koen23b.html). In International Conference on Automated Planning and Scheduling (ICAPS), 2023.
