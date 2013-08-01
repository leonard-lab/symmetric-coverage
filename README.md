Symmetric coverage algorithm code for DCSL at Princeton University

Description:
------------

This code allows Miabots to run the voronoi and gradient-based control laws described in the paper "Symmetric coverage of dynamic mapping error for mobile sensor networks" by Carlos H. Caicedo-Nunez and Naomi Ehrich Leonard.  The regions being surveyed, number and positions of robots, and various coefficients for the robots surveying and nature of the field are customizeable.  Due to the costliness of calculations, the program differs slightly from the algorithm. The memory of past measurements and number of points being examined are also adjustable, rather than viewing every point and remembering every previous position, so as to allow the program to calculate quickly.

Dependencies:
-------------

These programs require Miabots.m and its dependencies, which can be downloaded at https://github.com/leonard-lab/dcsl-matlab-api.git. 

Running: 
--------

To run on Miabots: 1. Open a terminal and run command %roslaunch dcsl_miabot_main miabot_main.launch (miabot_main3.launch for 3 robots) 2. One by one, turn on each robot and connect it using the GUI 3. Put robots into view of camera and ensure that each is properly tracked 4. Use a preset run script, runScript, twoRobotsCircle, twoRobotsSquare, or threeRobotsCircle, or create a field or streamedField object with desired variables. 5. Create a miabots object using the control law from the field or streamedField object, and run Miabots.start To shutdown system: 6. run Miabots.shutdown, then control-c out of the terminal used to open the ROS GUI
