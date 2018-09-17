# pddlstream

An implementation of STRIPStream that uses PDDL for the specifiation of actions and streams.

This repository is the "third version" of the STRIPStream framework, intended to replace the previous versions:

1) https://github.com/caelan/stripstream
2) https://github.com/caelan/ss

## Installation

```
$ git clone https://github.com/caelan/pddlstream.git
$ cd pddlstream
$ git submodule update --init --recursive
$ ./FastDownward/build.py
```

If `./FastDownward/build.py` fails, install FastDownward's dependencies using your package manager:
* APT (Linux): `$ sudo apt-get install cmake g++ g++-multilib make python`
<!--* Homebrew (OS X): TBD
* MacPorts (OS X): TBD
* N/A (Windows): install each dependency manually-->

If necessary, see FastDownward's documentation for more detailed installation instructions:

http://www.fast-downward.org/ObtainingAndRunningFastDownward

## Resources

* Planning Domain Description Language (PDDL): http://users.cecs.anu.edu.au/~patrik/pddlman/writing.html
* Derived Predicates: https://www.cs.cmu.edu/afs/cs/project/jair/pub/volume28/coles07a-html/node18.html

## Examples

### Pure Python

These are simple examples that can be run without additional depedencies:
* Blocksworld: `$ python -m examples.blocksworld.blocksworld`
* Blocksworld with Derived Predicates: `$ python -m examples.blocksworld.blocksworld_derived`
* Discrete Belief Space: `$ python -m examples.discrete_belief.run`
* Rovers: `$ python -m examples.ipc.rovers.run`
* Satellites: `$ python -m examples.ipc.satellites.run`

### Python TKinter

Install numpy and Python TKinter on Linux using: 
```
$ pip install numpy
$ sudo apt-get install python-tk
```

Examples:
* Discrete TAMP: `$ python -m examples.discrete_tamp.run`
* 1D Continuous TAMP: `$ python -m examples.continuous_tamp.run`
* 2D Motion Planning: `$ python -m examples.motion.run`

<img src="images/discrete_tamp.png" height="100">&emsp;<img src="images/continuous_tamp.png" height="100">&emsp;<img src="images/motion.png" height="100">

### PyBullet

Install PyBullet on OS X or Linux using: 
```
$ pip install numpy pybullet
```

Examples:
* Kuka IIWA task and motion planning - ```python -m examples.pybullet.kuka.run```
* PR2 task and motion planning - ```python -m examples.pybullet.pr2.run```
* PR2 planning and execution - ```python -m examples.pybullet.pr2_belief.run```
<!--[![Kuka IIWA](https://img.youtube.com/vi/3HJrkgIGK7c/0.jpg)](https://www.youtube.com/watch?v=3HJrkgIGK7c)-->
[<img src="https://img.youtube.com/vi/3HJrkgIGK7c/0.jpg" height="150">](https://www.youtube.com/watch?v=3HJrkgIGK7c)
&emsp;[<img src="https://img.youtube.com/vi/oWr6m12nXcM/0.jpg" height="150">](https://www.youtube.com/watch?v=oWr6m12nXcM)
&emsp;<img src="images/pybullet_belief.png" height="150">

See https://github.com/caelan/ss-pybullet for more information.

## Publications

STRIPStream: Integrating Symbolic Planners and Blackbox Samplers
https://arxiv.org/abs/1802.08705

STRIPStream: Planning In Infinite Domains
https://arxiv.org/abs/1701.00287

## Citation

Caelan R. Garrett, Tomas Lozano-Perez, Leslie P. Kaelbling. STRIPStream: Integrating Symbolic Planners and Blackbox Samplers, 2018.
