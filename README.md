# task-planner

This is the code from the Special Problem (Spring 2020) aimed at surveing task planning frameworks and thair compatibility with the ros task manager. 

## The framework used

The  framework used is the meta-csp-framework that can be found at [metacsp.org](http://metacsp.org). It uses Constraint Satisfaction Problems (CSP) to decide the task scheduling.

You should follow the instructions to build the project on Eclipse, then add it to the Java Build Path of your project to access the classes of the framework.

## My code

meta-csp-planner is a Java project containing two example of use of the framework with and without domain files. Both are based on the planning of a robot that visits multiple places in a map. 

However, _PlannerTurtleWithoutDomain.java_ was designed to make a planning of the turtleBot in the scene _gtl\_wifi\_planner.ttt_. _PlannerTurtleWithoutDomain.java_ creates an output file that can be read by _toMission.py_ to generate a mission readable by the ros task manager. An example of mission generated can be seen in _mission.py_.
