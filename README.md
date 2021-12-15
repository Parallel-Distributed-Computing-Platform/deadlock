# Solve Gridlock and Deadlock with sumo simulator

It is a program that solves deadlock and gridlock when there is no traffic light in Sumo simulator.

## How to use

There are a total of 4 files.

1. none.py When executing, the number of cars can be determined, and whether to ignore or keep the signal can be determined (0=keep the signal, 1=ignore the signal).
2. File deadlock for reproducing deadlock.py. When executed, the number of vehicles can be determined, and whether to control may be set. (0=no control, 1=with control)
3. File gridlock for reproducing gridlock.py. In the same manner as above, the number of vehicles and whether to control may be set. (0=no control, 1=with control)
4. When running randomScenario.py, which solves both deadlock and gridlock, a car is created at each end. In the same manner as above, the number of vehicles and whether to control may be set. (0=no control, 1=with control)
