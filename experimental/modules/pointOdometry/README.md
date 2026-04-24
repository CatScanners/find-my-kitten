The point odometry algorithm in it's current form is based on 
physical impulce model for rotation and an iterative search algorithm for location.

There was attempts to utilize gradient decent for location and rotation simultaniously
but it proved to be difficult as the rotation uses trigonometry k\*sin(x) 
which has the problem of guranteed overshootting for sufficently large scale
x_(n+1) = x_n + const \* d/dx( k\*sin(x_n) ) =  x_n + const \* k\*cos(x_n) ~= x_n + const \* k
for any choosen const there exist some sample data scale k that will make const \* k > 2 PI 
which greatly overshoots and randomizes the rotation
k\*sin(x + const \* k) would be the next step.
Way that was attempted to adress is scale every point into the unit sphere so that k becomes constrained.
Trying gradient decent with the normalized points at the unit sphere movement was smooth, 
but it had one fatal flaw. 
When location and rotation has arrived at their local optimums, both have derivative of 0 haltting all progress.
In reality it ends up making a pseudo random walk when both are very close to 0.

The way it was addressed was to treat optimalRotation as a part of the fitness funcion, 
but it has the draw back of no longer being pure gradient decent.

Currently at the core of point odometry there is a physics inspired algorithm 
that uses sphere impulse model to compute the gradient that tells
in which direction the camera should rotate to approach the optimal rotation.

For optimal locaiton gradient decent is used in combination with solved location on the display screen
Set of equations for the dispaly creen with 3D point in space.

x1+w\*x2+h\*x3=t\*(x-locx)+locx

y1+w\*y2+h\*y3=t\*(y-locy)+locy

z1+w\*z2+h\*z3=t\*(z-locz)+locz

solution to it:
solving for w,h the scaled x,y of the image position is obtained. 
gradient fitness function decent minimizes the squared distance error.

raw solutions:

w=((−((y1\*z3-y3\*z1)\*(x-x1)-(x1\*z3-x3\*z1)\*(y-y1)+(x1\*y3-x3\*y1)\*(z-z1)))/((y2\*z3-y3\*z2)\*(x-x1)-(x2\*z3-x3\*z2)\*(y-y1)+(x2\*y3-x3\*y2)\*(z-z1)))

h=(((y1\*z2-y2\*z1)\*(x-x1)-(x1\*z2-x2\*z1)\*(y-y1)+(x1\*y2-x2\*y1)\*(z-z1))/((y2\*z3-y3\*z2)\*(x-x1)-(x2\*z3-x3\*z2)\*(y-y1)+(x2\*y3-x3\*y2)\*(z-z1)))

derivatve of x on (w-w2D)^2+(h-h2D)^2

derivatve of y on (w-w2D)^2+(h-h2D)^2

derivatve of z on (w-w2D)^2+(h-h2D)^2

gives the gradinet decent.
Current implementation did a bit of a math hack where the the division was gotten rid of first before derivatives were solved.

(w-w2D)^2+(h-h2D)^2

became

(w-w2D)^2\*divisor_w2D^2+(h-h2D)^2\*divisor_h2D^2

notes about w,h:
(x,y) would virtually be at (x,y,0) with camera at (0,0,1) facing at (0,0,0).
