The point odometry algorithm in it's current form is based on 
physical impulce model for rotation and an iterative search algorithm for location.

There was attempts to utilize gradient decent for location and rotation simultaniously
but it proved to be difficult as the rotation uses trigonometry k*sin(x) 
which has the problem of guranteed overshootting for sufficently large scale
x_(n+1) = x_n + const \* d/dx( k\*sin(x_n) ) =  x_n + const \* k\*cos(x_n) ~= x_n + const \* k
for any choosen const there exist some sample data scale k that will make const \* k > 2 PI 
which greatly overshoots and randomizes the rotation
k*sin(x + const \* k) would be the next step.
Way that was attempted to adress is scale every point into the unit sphere so that k becomes constrained.
Trying gradient decent with the normalized points at the unit sphere movement was smooth, 
but it had one fatal flaw. 
When location and rotation has arrived at their local optimums, both have derivative of 0 haltting all progress.
In reality it ends up making a pseudo random walk when both are very close to 0.

The way it was addressed was to treat optimalRotation as a part of the fitness funcion, 
but it has the draw back of no longer being pure gradient decent.
