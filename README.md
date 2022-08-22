# FSAE-lapsim

This is a simple lap sim I made for Formula SAE.

The target of the project was to create a simulation that allowed the team 
to evaluate tradeoffs not capable of being evaluated with a point mass lap 
simulation. This includes variables such as aero balance, weight 
distribution, center of gravity height, and yaw inertia.

The simulation uses simple physics principles and 

Files:

test - shows various usages of the below functions and classes

solve_track - function that solves for laptime given a car and track
Car - class for a given car, storing vehicle properties such as mass
Track - class for tracks, storing track data in the forms of segments with 
provided curvature
dynamic_pts - function to calculate overall dynamics event points for a 
provided vehicle

and various csv/text files for vehicle and track data