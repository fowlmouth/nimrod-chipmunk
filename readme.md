## Chipmunk2D

[Chipmunk 6.1.*](http://chipmunk-physics.net/ "Chipmunk") bindings for 
[Nim 0.13.0](http://nim-lang.org "Nimrod") or higher.<br>
Tested with Chipmunk 6.1.5

### Installation: ###
[![nimble](https://raw.githubusercontent.com/yglukhov/nimble-tag/master/nimble.png)](https://github.com/yglukhov/nimble-tag)
<br>
Installation can be done using the Nimble package manager from the shell/command line (Nimble has to be installed):
```sh 
$ nimble install chipmunk
```

## Notes on some examples:

The **examples/planets.nim** and **examples/debugdraw_text.nim** examples need to be compiled with
the switch: ```-d:csfmlNoDestructors```<br>
This disables destructors in the Nim-CSFML library, otherwise you will get a lot of segfaults!<br>
Thanks to BlaXpirit for pointing this out.

## Using DebugDraw:

DebugDraw is a library for easily visualizing your simulation.
Using it is simple, there is an example in the examples dir :)

#### Overridden functions
DebugDraw purposefully clashes with these functions:
 
* debugDraw.addShape() - also creates the proper SFML shape for this shape, if you 
want to store user data pass it as an extra argument
* debugDraw.removeShape() - destroys the sfml shape 
* debugDraw.get/setUserData() - handles the shape userdata correctly, do not call these before the shape is initialized

Call debugDrawInit() after you add statics!
