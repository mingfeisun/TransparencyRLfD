# Expressive RLfD
[![Demo](http://img.youtube.com/vi/U4c8gebIX8c/0.jpg)](https://youtu.be/U4c8gebIX8c) 

Robot motions in Learning from Demonstration

* To find out what kind of robot motions will affect future demonstrations

Related robot motion types: attentional motions? uncertainty motions? predicatable motions?

* To know how those robot motions influence the demonstrations (positive or negative?)

Possible influences: how the learning process evolve over time? the state visited, the values updated


# Overview
<img src="docs/framework.png" alt="drawing" width="600px"/>

## Tentative schedule: 

### 1. Demonstration (Jun 15th)

<img src="docs/milestone-1-1.png" alt="drawing" width="600px"/>

<img src="docs/milestone-1-2.png" alt="drawing" width="600px"/>

<img src="docs/teleop.gif" alt="drawing" width="600px"/>

### 2. Model-free RL (Jul 1st)

**Single state-action update**
* [Q-Learning](https://github.com/mingfeisun/matlab-reinforcement-learning/tree/master/RL-Q-Learning/Reinforcement%20Learning(Q-Learning))
<img src="docs/q-learning.bmp" alt="drawing" width="600px"/>

**Autonomous Q-learning** test on the simulation:

<img src="docs/q-learning-autonomous.gif" alt="drawing" width="600px"/>

* [SARSA](https://github.com/mingfeisun/matlab-reinforcement-learning/tree/master/SARSA)
<img src="docs/sarsa.bmp" alt="drawing" width="600px"/>

**Multiple state-action update**: update whole trajectory
* TD(lambda)
* SARSA(lambda)

**To be decided**:

To-do list:
* ~~State: pose (position & orientation), velocity, physical properties etc.~~
* ~~Reward: -1 for each movment until reach final goal, -2 for collision, 10 for reach final goal, object pose~~

**Two different algorithms**: to compare these two types of algorithms
* Inverse Reinforcement Learning 
* * Reference: [Showing versus Doing: Teaching by Demonstration](https://papers.nips.cc/paper/6413-showing-versus-doing-teaching-by-demonstration.pdf)
* * Exp 1: Teaching Goal-based Reward function
* * Exp 2: Teaching Feature-based Reward function
* Model-free Reinforcement Learning

### 3. Learning process evaluation (Jul 15th)

**Q-learning**:
* ~~Q function update (update for current state and action only)~~
* ~~Predicted action vs. human input action~~

### 4. Motion planning (Jul 25th)
* Testing on PR2 robot: robot right arm follows cup movement: (speed: x10)
<img src="docs/robot-follow-cup.gif" alt="drawing" width="600px"/>

* Testing on [UR10 robot](https://github.com/ros-industrial/universal_robot): (speed: x5)
<img src="docs/arm-move-cup.gif" alt="drawing" width="600px"/>


## To-do lists

1. ~~Need to find out the possible robot motions that would affect the human perception~~
2. ~~Need to implement the related motions~~
* ~~Gesturing motions: policy-based~~
<img src="docs/arm-gesturing-x2.gif" alt="drawing" width="600px"/>

* ~~Pausing motions: state-uncertainty-based~~
<img src="docs/arm-pausing-x2.gif" alt="drawing" width="600px"/>

3. ~~Need to determine the learning model and evaluate the model effects towards the learning~~

## Expected outcome
<img src="docs/doing-showing.png" alt="drawing" width="600px"/>

## Final outcome
<img src="docs/adaptive-gesturing.gif" alt="drawing" width="600px"/>

## Related source code
[Matlab Reinforcement Learning](https://github.com/mingfeisun/matlab-reinforcement-learning)
