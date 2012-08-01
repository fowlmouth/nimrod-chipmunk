
const Lib = "libchipmunk.so.6.0.3"
type 
  TVector* {.final, pure.} = object
    x*, y*: cdouble
  TTimestamp* = cuint
  TBodyVelocityFunc* = proc(body: PBody, gravity: TVector,
                            damping: cdouble; dt: cdouble)
  TBodyPositionFunc* = proc(body: PBody; dt: cdouble)
  TComponentNode*{.pure, final.} = object 
    root*: PBody
    next*: PBody
    idleTime*: cdouble
  
  PArray = ptr TArray
  TArray = object
  PHashSet = ptr THashSet
  THashSet = object
  
  #/ Spatial index bounding box callback function type.
  #/ The spatial index calls this function and passes you a pointer to an object you added
  #/ when it needs to get the bounding box associated with that object.
  TSpatialIndexBBFunc* = proc (obj: pointer): BB{.cdecl.}
  #/ Spatial index/object iterator callback function type.
  TSpatialIndexIteratorFunc* = proc (obj: pointer; data: pointer){.cdecl.}
  #/ Spatial query callback function type. 
  TSpatialIndexQueryFunc* = proc (obj1: pointer; obj2: pointer; data: pointer){.
      cdecl.}
  #/ Spatial segment query callback function type.
  TSpatialIndexSegmentQueryFunc* = proc (obj1: pointer; obj2: pointer; 
      data: pointer): Float{.cdecl.}
  #/ private
  TSpatialIndex{.pure, final.} = object 
    klass: ptr TSpatialIndexClass
    bbfunc: TSpatialIndexBBFunc
    staticIndex: ptr TSpatialIndex
    dynamicIndex: ptr TSpatialIndex

  
  PSpace* = ptr TSpace
  TSpace* {.pure, final.} = object
    iterations*: cint 
    gravity*: TVector
    damping*: cdouble
    idleSpeedThreshold*: cdouble 
    sleepTimeThreshold*: cdouble 
    collisionSlop*: cdouble 
    collisionBias*: cdouble
    collisionPersistence*: TTimestamp        
    enableContactGraph*: cint ##BOOL
    data*: pointer
    staticBody*: PBody
    stamp: TTimestamp
    currDT: cdouble
    bodies: PArray
    rousedBodies: PArray
    sleepingComponents: PArray
    staticShapes: ptr cpSpatialIndex
    activeShapes: ptr cpSpatialIndex
    arbiters: PArray
    contactBuffersHead: ptr cpContactBufferHeader
    cachedArbiters: ptr cpHashSet
    pooledArbiters: PArray
    constraints: PArray
    allocatedBuffers: PArray
    locked: cint
    collisionHandlers: ptr cpHashSet
    defaultHandler: cpCollisionHandler
    postStepCallbacks: ptr cpHashSet
    arbiterApplyImpulse: cpSpaceArbiterApplyImpulseFunc
    staticBody: cpBody
  PBody* = ptr TBody
  TBody*{.pure, final.} = object 
    velocityFunc*: TBodyVelocityFunc 
    positionFunc*: TBodyPositionFunc                                       
    m*: cdouble           
    mInv*: cdouble       
    i*: cdouble           
    iInv*: cdouble       
    p*: TVector            
    v*: TVector            
    f*: TVector 
    a*: cdouble 
    w*: cdouble 
    t*: cdouble 
    rot*: TVector 
    data*: pointer
    vLimit*: cdouble   
    wLimit*: cdouble
    vBias*: TVector
    wBias*: cdouble
    space*: PSpace
    shapeList*: PShape
    arbiterList*: PArbity
    constraintList*: PConstraint
    node*: cpComponentNode

const 
  CP_BUFFER_BYTES* = (32 * 1024)  

#/ Version string.
var VersionString*{.importc: "cpVersionString", dynlib: Lib.}: cstring
#/ Calculate the moment of inertia for a circle.
  #/ @c r1 and @c r2 are the inner and outer diameters. A solid circle has an inner diameter of 0.
proc MomentForCircle*(m: cdouble; r1: cdouble; r2: cdouble; offset: TVectoror): cdouble {.
  cdecl, importc: "cpMomentForCircle", dynlib: Lib.}

#/ Calculate area of a hollow circle.
  #/ @c r1 and @c r2 are the inner and outer diameters. A solid circle has an inner diameter of 0.
proc AreaForCircle*(r1: cdouble; r2: cdouble): cdouble {.cdecl, 
      importc: "cpAreaForCircle", dynlib: Lib.}
  #/ Calculate the moment of inertia for a line segment.
  #/ Beveling radius is not supported.
  proc MomentForSegment*(m: Float; a: Vect; b: Vect): Float{.cdecl, 
      importc: "cpMomentForSegment", dynlib: Lib.}
  #/ Calculate the area of a fattened (capsule shaped) line segment.
  proc AreaForSegment*(a: Vect; b: Vect; r: Float): Float{.cdecl, 
      importc: "cpAreaForSegment", dynlib: Lib.}
  #/ Calculate the moment of inertia for a solid polygon shape assuming it's center of gravity is at it's centroid. The offset is added to each vertex.
  proc MomentForPoly*(m: Float; numVerts: cint; verts: ptr Vect; offset: Vect): Float{.
      cdecl, importc: "cpMomentForPoly", dynlib: Lib.}
  #/ Calculate the signed area of a polygon. A Clockwise winding gives positive area.
  #/ This is probably backwards from what you expect, but matches Chipmunk's the winding for poly shapes.
  proc AreaForPoly*(numVerts: cint; verts: ptr Vect): Float{.cdecl, 
      importc: "cpAreaForPoly", dynlib: Lib.}
  #/ Calculate the natural centroid of a polygon.
  proc CentroidForPoly*(numVerts: cint; verts: ptr Vect): Vect{.cdecl, 
      importc: "cpCentroidForPoly", dynlib: Lib.}
  #/ Center the polygon on the origin. (Subtracts the centroid of the polygon from each vertex)
  proc RecenterPoly*(numVerts: cint; verts: ptr Vect){.cdecl, 
      importc: "cpRecenterPoly", dynlib: Lib.}
  #/ Calculate the moment of inertia for a solid box.
  proc MomentForBox*(m: Float; width: Float; height: Float): Float{.cdecl, 
      importc: "cpMomentForBox", dynlib: Lib.}
  #/ Calculate the moment of inertia for a solid box.
  proc MomentForBox2*(m: Float; box: BB): Float{.cdecl, 
      importc: "cpMomentForBox2", dynlib: Lib.}

##cp property emulators
template defGetter(otype: typedesc, memberType: typedesc, memberName: expr, procName: expr): stmt {.immediate.} =
  proc `get procName`*(obj: otype): memberType =
    return obj.memberName
template defSetter(otype: typedesc, memberType: typedesc, memberName: expr, procName: expr): stmt {.immediate.} =
  proc `set procName`*(obj: otype, value: memberType) =
    obj.memberName = value
template defProp(otype: typedesc, memberType: typedesc, memberName: expr, procName: expr): stmt {.immediate.} =
  defGetter(otype, memberType, memberName, procName)
  defSetter(otype, memberType, memberName, procName)


##cpspace.h
proc allocSpace*(): PSpace {.
  importc: "cpSpaceAlloc", dynlib: Lib.}
proc Init*(space: PSpace): PSpace {.
  importc: "cpSpaceInit", dynlib: Lib.}
proc newSpace*(): PSpace {.
  importc: "cpSpaceNew", dynlib: Lib.}
proc destroy*(space: PSpace) {.
  importc: "cpSpaceDestroy", dynlib: Lib.}
proc free*(space: PSpace) {.
  importc: "cpSpaceFree", dynlib: Lib.}

defProp(PSpace, member: expr, name: Radius)
defProp(PSpace, 
proc getRadius*(o: PSpace): TVector =
  return o.member

##cpBody.h
proc allocBody*(): PBody {.importc: "cpBodyAlloc", dynlib: Lib.}
proc init*(body: PBody; m: cdouble; i: cdouble): PBody {.
  importc: "cpBodyInit", dynlib: Lib.}
proc newBody*(m: cdouble; i: cdouble): PBody {.
  importc: "cpBodyNew", dynlib: Lib.}

proc initStaticBody*(body: PBody): PBody{.
  importc: "cpBodyInitStatic", dynlib: Lib.}
#/ Allocate and initialize a static cpBody.
proc newStatic*(): PBody{.importc: "cpBodyNewStatic", dynlib: Lib.}
#/ Destroy a cpBody.
proc destroy*(body: PBody){.importc: "cpBodyDestroy", dynlib: Lib.}
#/ Destroy and free a cpBody.
proc free*(body: PBody){.importc: "cpBodyFree", dynlib: Lib.}

#/ Wake up a sleeping or idle body.
proc activate*(body: PBody){.importc: "cpBodyActivate", dynlib: Lib.}
#/ Wake up any sleeping or idle bodies touching a static body.
proc activateStatic*(body: PBody; filter: PShape){.
    importc: "cpBodyActivateStatic", dynlib: Lib.}
#/ Force a body to fall asleep immediately.
proc Sleep*(body: PBody){.importc: "cpBodySleep", dynlib: Lib.}
#/ Force a body to fall asleep immediately along with other bodies in a group.
proc SleepWithGroup*(body: PBody; group: PBody){.
    importc: "cpBodySleepWithGroup", dynlib: Lib.}
#/ Returns true if the body is sleeping.
proc isSleeping*(body: PBody): cpBool{.inline.} = 
  return body.node.root != nil
#/ Returns true if the body is static.
proc isStatic*(body: PBody): cpBool{.inline.} = 
  return body.node.idleTime == INFINITY
#/ Returns true if the body has not been added to a space.
proc isRogue*(body: PBody): cpBool{.inline.} = 
  return body.space == nil

# #define CP_DefineBodyStructGetter(type, member, name) \
# static inline type cpBodyGet##name(const cpBody *body){return body->member;}
# #define CP_DefineBodyStructSetter(type, member, name) \
# static inline void cpBodySet##name(cpBody *body, const type value){ \
# 	cpBodyActivate(body); \
# 	cpBodyAssertSane(body); \
# 	body->member = value; \
# }
# #define CP_DefineBodyStructProperty(type, member, name) \
# CP_DefineBodyStructGetter(type, member, name) \
# CP_DefineBodyStructSetter(type, member, name)

defGetter(PBody, m, cdouble, Mass)
#/ Set the mass of a body.
proc setMass*(body: PBody; m: cdouble){.
  importc: "cpBodySetMass", dynlib: Lib.}

#/ Get the moment of a body.
defGetter(PBody, i, cdouble, Moment)
#/ Set the moment of a body.
proc SetMoment*(body: PBody; i: cdouble) {.
  importc: "cpBodySetMoment", dynlib: Lib.}

#/ Get the position of a body.
defGetter(PBody, TVector, p, Pos)
#/ Set the position of a body.
proc setPos*(body: PBody; pos: TVector){.
  importc: "cpBodySetPos", dynlib: Lib.}

defProp(PBody, TVector, v, Vel)
defProp(PBody, TVector, f, Force)

#/ Get the angle of a body.
defGetter(PBody, cdouble, a, Angle)
#/ Set the angle of a body.
proc setAngle*(body: PBody; a: cdouble){.
  importc: "cpBodySetAngle", dynlib: Lib.}

defProp(PBody, cdouble, w, AngVel)
defProp(PBody, cdouble, t, Torque)
defGetter(PBody, TVector, rot, Rot)
defProp(PBody, cdouble, v_limit, VelLimit)
defProp(PBody, cdouble, w_limit, AngVelLimit)
defProp(PBody, pointer, data, UserData)

#/ Default Integration functions.
proc UpdateVelocity*(body: PBody; gravity: TVector; damping: cdouble; dt: cdouble){.
  importc: "cpBodyUpdateVelocity", dynlib: Lib.}
proc UpdatePosition*(body: PBody; dt: cdouble){.
  importc: "cpBodyUpdatePosition", dynlib: Lib.}
#/ Convert body relative/local coordinates to absolute/world coordinates.
proc Local2World*(body: PBody; v: TVector): TVector{.inline.} = 
  return cpvadd(body.p, cpvrotate(v, body.rot))

#/ Convert body absolute/world coordinates to  relative/local coordinates.
proc cpBodyWorld2Local*(body: PBody; v: TVector): TVector{.inline.} = 
  return cpvunrotate(cpvsub(v, body.p), body.rot)

#/ Set the forces and torque or a body to zero.

proc cpBodyResetForces*(body: PBody){.importc: "cpBodyResetForces", 
    dynlib: Lib.}
#/ Apply an force (in world coordinates) to the body at a point relative to the center of gravity (also in world coordinates).

proc cpBodyApplyForce*(body: PBody; f: TVector; r: TVector){.
    importc: "cpBodyApplyForce", dynlib: Lib.}
#/ Apply an impulse (in world coordinates) to the body at a point relative to the center of gravity (also in world coordinates).

proc cpBodyApplyImpulse*(body: PBody; j: TVector; r: TVector){.
    importc: "cpBodyApplyImpulse", dynlib: Lib.}
#/ Get the velocity on a body (in world units) at a point on the body in world coordinates.

proc cpBodyGetVelAtWorldPoint*(body: PBody; point: TVector): TVector{.
    importc: "cpBodyGetVelAtWorldPoint", dynlib: Lib.}
#/ Get the velocity on a body (in world units) at a point on the body in local coordinates.

proc cpBodyGetVelAtLocalPoint*(body: PBody; point: TVector): TVector{.
    importc: "cpBodyGetVelAtLocalPoint", dynlib: Lib.}
#/ Get the kinetic energy of a body.
# static inline cdouble cpBodyKineticEnergy(const cpBody *body)
# {
# 	// Need to do some fudging to avoid NaNs
# 	cpFloat vsq = cpvdot(body->v, body->v);
# 	cpFloat wsq = body->w*body->w;
# 	return (vsq ? vsq*body->m : 0.0f) + (wsq ? wsq*body->i : 0.0f);
# }
#/ Body/shape iterator callback function type. 

type 
  cpBodyShapeIteratorFunc* = proc (body: PBody; shape: ptr cpShape; 
                                   data: pointer)

#/ Call @c func once for each shape attached to @c body and added to the space.

proc cpBodyEachShape*(body: PBody; func: cpBodyShapeIteratorFunc; 
                      data: pointer){.importc: "cpBodyEachShape", dynlib: Lib.}
#/ Body/constraint iterator callback function type. 

type 
  cpBodyConstraintIteratorFunc* = proc (body: PBody; 
                                        constraint: ptr cpConstraint; 
                                        data: pointer)

#/ Call @c func once for each constraint attached to @c body and added to the space.

proc cpBodyEachConstraint*(body: PBody; func: cpBodyConstraintIteratorFunc; 
                           data: pointer){.importc: "cpBodyEachConstraint", 
    dynlib: Lib.}
#/ Body/arbiter iterator callback function type. 

type 
  cpBodyArbiterIteratorFunc* = proc (body: PBody; arbiter: ptr cpArbiter; 
                                     data: pointer)

#/ Call @c func once for each arbiter that is currently active on the body.

proc cpBodyEachArbiter*(body: PBody; func: cpBodyArbiterIteratorFunc; 
                        data: pointer){.importc: "cpBodyEachArbiter", 
                                        dynlib: Lib.}
#/@}


#/ Allocate a spatial hash.

proc SpaceHashAlloc*(): ptr TSpaceHash{.cdecl, importc: "cpSpaceHashAlloc", 
                                        dynlib: Lib.}
#/ Initialize a spatial hash. 

proc SpaceHashInit*(hash: ptr TSpaceHash; celldim: Float; numcells: cint; 
                    bbfunc: TSpatialIndexBBFunc; staticIndex: ptr TSpatialIndex): ptr TSpatialIndex{.
    cdecl, importc: "cpSpaceHashInit", dynlib: Lib.}
#/ Allocate and initialize a spatial hash.

proc SpaceHashNew*(celldim: Float; cells: cint; bbfunc: TSpatialIndexBBFunc; 
                   staticIndex: ptr TSpatialIndex): ptr TSpatialIndex{.cdecl, 
    importc: "cpSpaceHashNew", dynlib: Lib.}
#/ Change the cell dimensions and table size of the spatial hash to tune it.
#/ The cell dimensions should roughly match the average size of your objects
#/ and the table size should be ~10 larger than the number of objects inserted.
#/ Some trial and error is required to find the optimum numbers for efficiency.

proc SpaceHashResize*(hash: ptr TSpaceHash; celldim: Float; numcells: cint){.
    cdecl, importc: "cpSpaceHashResize", dynlib: Lib.}
#MARK: AABB Tree


#/ Allocate a bounding box tree.

proc BBTreeAlloc*(): ptr TBBTree{.cdecl, importc: "cpBBTreeAlloc", dynlib: Lib.}
#/ Initialize a bounding box tree.

proc BBTreeInit*(tree: ptr TBBTree; bbfunc: TSpatialIndexBBFunc; 
                 staticIndex: ptr TSpatialIndex): ptr TSpatialIndex{.cdecl, 
    importc: "cpBBTreeInit", dynlib: Lib.}
#/ Allocate and initialize a bounding box tree.

proc BBTreeNew*(bbfunc: TSpatialIndexBBFunc; staticIndex: ptr TSpatialIndex): ptr TSpatialIndex{.
    cdecl, importc: "cpBBTreeNew", dynlib: Lib.}
#/ Perform a static top down optimization of the tree.

proc BBTreeOptimize*(index: ptr TSpatialIndex){.cdecl, 
    importc: "cpBBTreeOptimize", dynlib: Lib.}
#/ Bounding box tree velocity callback function.
#/ This function should return an estimate for the object's velocity.

type 
  TBBTreeVelocityFunc* = proc (obj: pointer): Vect{.cdecl.}

#/ Set the velocity function for the bounding box tree to enable temporal coherence.

proc BBTreeSetVelocityFunc*(index: ptr TSpatialIndex; func: TBBTreeVelocityFunc){.
    cdecl, importc: "cpBBTreeSetVelocityFunc", dynlib: Lib.}
#MARK: Single Axis Sweep


#/ Allocate a 1D sort and sweep broadphase.

proc Sweep1DAlloc*(): ptr TSweep1D{.cdecl, importc: "cpSweep1DAlloc", 
                                    dynlib: Lib.}
#/ Initialize a 1D sort and sweep broadphase.

proc Sweep1DInit*(sweep: ptr TSweep1D; bbfunc: TSpatialIndexBBFunc; 
                  staticIndex: ptr TSpatialIndex): ptr TSpatialIndex{.cdecl, 
    importc: "cpSweep1DInit", dynlib: Lib.}
#/ Allocate and initialize a 1D sort and sweep broadphase.

proc Sweep1DNew*(bbfunc: TSpatialIndexBBFunc; staticIndex: ptr TSpatialIndex): ptr TSpatialIndex{.
    cdecl, importc: "cpSweep1DNew", dynlib: Lib.}
