## Chipmunk

[Chipmunk 6.1.1](http://chipmunk-physics.net/ "Chipmunk") bindings for 
[Nimrod 0.9.0](http://nimrod-code.org "Nimrod")

## Using DebugDraw

DebugDraw is a library for easily visualizing your simulation.
Using it is simple, there is an example in the examples dir :)

#### Overridden functions
DebugDraw purposefully clashes with these functions:
 
* debugDraw.addShape() - also creates the proper SFML shape for this shape, if you 
want to store user data pass it as an extra argument
* debugDraw.removeShape() - destroys the sfml shape 
* debugDraw.get/setUserData() - handles the shape userdata correctly, do not call these before the shape is initialized

Call debugDrawInit() after you add statics!
