# pddlstream

PDDLStream is a planning framework comprised of an action language and suite of algorithms for Artificial Intelligence (AI) planning in the presence of sampling procedures.
PDDLStream extends Planning Domain Definition Language (PDDL) by introducing streams, declarative specifications of sampling procedures.
PDDLStream algorithms are domain independent and solve PDDLStream problems with only a blackbox description of each sampler.
The original application of PDDLStream was for general-purpose robot Task and Motion Planning (TAMP). 

The [default](https://github.com/caelan/pddlstream) **pddlstream** branch ([stable](https://github.com/caelan/pddlstream/tree/stable)) is the newest stable "release" of **pddlstream**.
The [master](https://github.com/caelan/pddlstream/tree/master) **pddlstream** branch is the most recent and advanced version of **pddlstream** but also is somewhat experimental.

<!--https://www.markdownguide.org/basic-syntax/-->
<!--https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet-->

## Publications

* [PDDLStream: Integrating Symbolic Planners and Blackbox Samplers via Optimistic Adaptive Planning](https://arxiv.org/abs/1802.08705
)
<!--* [STRIPStream: Planning In Infinite Domains](https://arxiv.org/abs/1701.00287)-->

## Citation

Caelan R. Garrett, Tomás Lozano-Pérez, Leslie P. Kaelbling. PDDLStream: Integrating Symbolic Planners and Blackbox Samplers via Optimistic Adaptive Planning, International Conference on Automated Planning and Scheduling (ICAPS), 2020.

## Contact

Caelan Garrett: [username]@mit.edu

## History

PDDLStream is the "third version" of the PDDLStream/STRIPStream planning framework, intended to supersede previous versions:

1) https://github.com/caelan/stripstream
2) https://github.com/caelan/ss

PDDLStream makes several representational and algorithmic improvements over these versions.
Most notably, it adheres to PDDL conventions and syntax whenever possible and contains several new algorithms. 
<!--An implementation of STRIPStream that uses PDDL for the specifiation of actions and streams.-->
<!--https://github.com/caelan/pddlstream/compare/master...stable-->

## Installation

```
$ git clone https://github.com/caelan/pddlstream.git
$ cd pddlstream
$ git submodule update --init --recursive
$ ./FastDownward/build.py release64
```

If building fails, install FastDownward's dependencies using your package manager:
* APT (Linux): `$ sudo apt-get install cmake g++ g++-multilib make python`
<!--* Homebrew (OS X): TBD
* MacPorts (OS X): TBD
* N/A (Windows): install each dependency manually-->

If necessary, see FastDownward's [documentation](http://www.fast-downward.org/ObtainingAndRunningFastDownward) for more detailed installation instructions.

My FastDownward "fork" is several years old. If you have trouble installing FastDownward, try installing the following branch: [downward](https://github.com/caelan/pddlstream/tree/downward).

## Examples

### PyBullet

Install PyBullet on OS X or Linux using: 
```
$ pip install numpy pybullet
```

Examples:
* PR2 TAMP - ```python -m examples.pybullet.tamp.run```
* PR2 Cleaning and Cooking - ```python -m examples.pybullet.pr2.run```
* Turtlebot Rovers - ```python -m examples.pybullet.turtlebot_rovers.run```
* PR2 Rovers - ```python -m examples.pybullet.pr2_rovers.run```
* PR2 Planning and Execution - ```python -m examples.pybullet.pr2_belief.run```
* Kuka Cleaning and Cooking - ```python -m examples.pybullet.kuka.run```
* Turtlebot NAMO - ```python -m examples.pybullet.namo.run```
* Turtlebot Multi-Robot - ```python -m examples.pybullet.turtlebots.run```

<!--[![Kuka IIWA](https://img.youtube.com/vi/3HJrkgIGK7c/0.jpg)](https://www.youtube.com/watch?v=3HJrkgIGK7c)-->
[<img src="https://img.youtube.com/vi/Uc0fogLsPMI/0.jpg" height="200">](https://www.youtube.com/watch?v=Uc0fogLsPMI)
&emsp;[<img src="https://img.youtube.com/vi/HVD8SpmguYs/0.jpg" height="200">](https://www.youtube.com/watch?v=HVD8SpmguYs)
&emsp;[<img src="https://img.youtube.com/vi/oWr6m12nXcM/0.jpg" height="200">](https://www.youtube.com/watch?v=oWr6m12nXcM)
<img src="images/pybullet_belief.png" height="150">
&emsp;[<img src="https://img.youtube.com/vi/XcxsU0VuRUI/0.jpg" height="200">](https://www.youtube.com/watch?v=XcxsU0VuRUI)
&emsp;[<img src="https://img.youtube.com/vi/3HJrkgIGK7c/0.jpg" height="200">](https://www.youtube.com/watch?v=3HJrkgIGK7c)

See https://github.com/caelan/pybullet-planning for more information about my PyBullet primitives.

### Python TKinter

Install numpy and Python TKinter on Linux using: 
```
$ pip install numpy
$ sudo apt-get install python-tk
```

Examples:
* 1D Continuous TAMP: `$ python -m examples.continuous_tamp.run`
* 2D Motion Planning: `$ python -m examples.motion.run`
* Discrete TAMP: `$ python -m examples.discrete_tamp.run`
* Discrete TAMP with pushing: `$ python -m examples.discrete_tamp.run`

<img src="images/discrete_tamp.png" height="100">&emsp;<img src="images/continuous_tamp.png" height="100">&emsp;<img src="images/motion.png" height="100">

### Pure Python

Simple examples that can be run without additional dependencies:
* Blocksworld: `$ python -m examples.blocksworld.blocksworld`
* Blocksworld with Derived Predicates: `$ python -m examples.blocksworld.blocksworld_derived`
* Discrete Belief Space: `$ python -m examples.discrete_belief.run`
* Kitchen (debug streams): `python -m examples.kitchen.run`
<!--* Discrete Belief: `python -m examples.table_obs.run`-->

### Advanced Functionality

Test cases or advanced (and undocumented) functionality:
* Action Description Language (ADL): `$ python -m examples.adl.run`
* Deferred streams (postponed evaluation): `$ python -m examples.defer.run`
* Exogenous streams (observations): `$ python -m examples.exogenous.run`
* Fluent streams (state constraints): `$ python -m examples.fluent.run`
* Constraint satisfaction: `$ python -m examples.satisfy.run`
* Wild streams (ad hoc certification): `$ python -m examples.wild.run`

### International Planning Competition (IPC)

Unmodified PDDL IPC examples solved using PDDLStream's modified translator:
* Rovers: `$ python -m examples.ipc.rovers.run`
* Satellites: `$ python -m examples.ipc.satellites.run`

## Applications

External projects that make use of PDDLStream:
* Online TAMP - https://github.com/caelan/SS-Replan
* Automated Construction - https://github.com/caelan/pb-construction
* Learning + TAMP (LTAMP) - https://github.com/caelan/LTAMP
<!--https://github.com/rachelholladay/ftamp-->

## Resources

* [Recent Talk](https://youtu.be/YfCh5qMYGuQ)
* [Recent Overview](https://youtu.be/7xLPogztQss)
* [Recent Tutorial](https://youtu.be/JNOk1rylDpU)
* [PDDLStream Tutorial](https://web.mit.edu/caelan/www/presentations/6.881_19-11-12.pdf)
* [Planning Domain Definition Language (PDDL)](http://users.cecs.anu.edu.au/~patrik/pddlman/writing.html)
* [Derived Predicates](https://www.cs.cmu.edu/afs/cs/project/jair/pub/volume28/coles07a-html/node18.html)
<!--* [PDDLStream Tutorial](https://web.mit.edu/caelan/www/presentations/6.881_TAMP.pdf)-->

## Retired

"Retired" folders indicate code that no longer is continuously supported and thus is likely outdated.

### Drake

Install Drake on OS X or Ubuntu by following the following instructions: http://drake.mit.edu/installation.html.

Alternatively, install Drake through docker by following the following instructions: http://manipulation.csail.mit.edu/install_drake_docker.html. Use the appropriate docker_run_bash script with docker tag drake-20181128.

Examples:
* Kuka IIWA task and motion planning - ```python -m examples.drake.run```
<!--[![Kuka IIWA](https://img.youtube.com/vi/3HJrkgIGK7c/0.jpg)](https://www.youtube.com/watch?v=3HJrkgIGK7c)-->
<img src="images/drake_kuka.png" height="150">

Additional PDDLStream + Drake examples can be found at: https://github.com/RobotLocomotion/6-881-examples.

<!--https://drake.mit.edu/gallery.html#task-and-motion-planning-->
