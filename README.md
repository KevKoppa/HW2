# HW2
HW2


Your task for this homework is to extend the base functionality provided, to generate results on a more interesting scenario.

The code provided on the main branch will generate a video identical to the one seen below, when running HW2.simulate() (after instantiating package, etc.).

<img src="https://github.com/av-course/HW2/blob/main/hw2.gif" />

Your task is to work on the curved-track branch, and recreate the trajectory optimization procedue in a manner that adheres to the curved lane boundaries. You will have to think about how a circular lane-boundary constraint can be expressed, and incorporate it into the generation of motion planning probelms. You will be using the main branch as your template in this process. I have already provided you an immense amount of starter code, so your task will mainly involve reading the code, understanding what it does, and making minor modificaitons. 


Deliverables:

There is no autograder for this homework. Instead, generate a video demonstrating your resulting trajectory planner working on the cirlce track (like I have provided for you here). That is all! 

Tips: Unlike the straight-lane version, in the curved lane world the other vehicles on the road will deviate from the constant-velocity predictions that your plan assumes. You may need to generate more intelligent predictions for what other vehicles are doing, and you may need to find a way to keep an extra buffer away from other vehicles so as to not get into situations where your planner cannot find a feasible motion plan. 
