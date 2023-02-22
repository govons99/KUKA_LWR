# KUKA LWR-4

## Compiling procedure

For compiling use the standard procedure:

    - mkdir build

    - cd build

    - cmake ..

    - make 


## Simulation

The simulations are implemented in C++ and the robot is commanded with a fedback linearitazion control scheme.

1. _main_sim.cpp_

## Experiment

The code running on the real KUKA LWR 4+ is implemented in C++.

1. _main_pos.cpp_ regards the implementation of a position control on the real KUKA LWR 4+

In order to connect to the robot and run the code, use the following procedure:

    - sudo ./main_pos
