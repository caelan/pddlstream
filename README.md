# pddlstream

An implementation of STRIPStream that uses PDDL for the specifiation of actions and streams.

This repository is the "third version" of the STRIPStream framework, intended to replace the previous versions:

1) https://github.com/caelan/stripstream
2) https://github.com/caelan/ss

## Installation

Clone PDDLStream, and add the repository to your PYTHONPATH.
```
$ git clone https://github.com/caelan/pddlstream.git
$ cd pddlstream
$ export PYTHONPATH=${PWD}:${PYTHONPATH}
```

Install the FastDownward Planning System by using the following instructions:
```
$ git clone https://github.com/caelan/FastDownward.git
$ cd FastDownward
$ ./build.py
$ export FD_PATH=${PWD}/builds/release32/
```

Add each `export ...` command with the appropriate paths to your `~/.bashrc` (Linux) or `~/.bash_profile` (OS X) file for future use.

If `./build.py` fails, install FastDownward's dependencies using your package manager:
* APT (Linux): `$ sudo apt-get install cmake g++ g++-multilib make python`
* Homebrew (OS X): TBD
* MacPorts (OS X): TBD
* N/A (Windows): install each dependency manually

If necessary, see FastDownward's documentation for more detailed installation instructions:

http://www.fast-downward.org/ObtainingAndRunningFastDownward

## Examples

### Pure Python

These are simple examples that can be run without additional depedencies:
* Blocksworld: `$ python examples/pddl/blocksworld.py` (alternatively `python -m examples.pddl.blocksworld`)
* Blocksworld with Derived Predicates: `$ python examples/pddl/blocksworld_derived.py`
* Discrete TAMP: `$ python examples/discrete_tamp/run.py`
* 1D Continuous TAMP: `$ python examples/continuous_tamp/run.py`
* 2D Motion Planning: `$ python examples/motion/run.py`

<img src="images/discrete_tamp.png" height="100">&emsp;<img src="images/continuous_tamp.png" height="100">&emsp;<img src="images/motion.png" height="100">

### PyBullet

These examples require installation of following library:
https://github.com/caelan/ss-pybullet

* TBD
