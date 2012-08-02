when defined(Linux):
  const Lib = "libchipmunk.so.6.0.3"
else:
  echo("Platform unsupported")
  quit(1)    
const 
  CP_BUFFER_BYTES* = (32 * 1024)  
  CP_MAX_CONTACTS_PER_ARBITER* = 4
type 
  Bool32* = cint  #replace one day with cint-compatible bool
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
  
  THashValue = uint  # uintptr_t 
  TCollisionType = uint #uintptr_t
  TGroup = uint #uintptr_t
  TLayers = uint 
  PArray = ptr TArray
  TArray{.pure,final.} = object
  PHashSet = ptr THashSet
  THashSet{.pure, final.} = object
  PContact* = ptr TContact
  TContact*{.pure,final.} = object
  PConstraint* = ptr TConstraint
  TConstraint*{.pure, final.} = object
  PArbiter* = ptr TArbiter
  TArbiter*{.pure, final.} = object 
    e*: cdouble
    u*: cdouble 
    surface_vr*: TVector
    a*: PShape
    b*: PShape
    body_a*: PBody
    body_b*: PBody
    thread_a*: TArbiterThread
    thread_b*: TArbiterThread
    numContacts*: cint
    contacts*: PContact
    stamp*: TTimestamp
    handler*: PCollisionHandler
    swappedColl*: bool32
    state*: TArbiterState
  PCollisionHandler* = ptr TCollisionHandler
  TCollisionHandler*{.pure, final.} = object 
    a*: TCollisionType
    b*: TCollisionType
    begin*: TCollisionBeginFunc
    preSolve*: TCollisionPreSolveFunc
    postSolve*: TCollisionPostSolveFunc
    separate*: TCollisionSeparateFunc
    data*: pointer
  TArbiterState*{.size: sizeof(cint).} = enum 
    ArbiterStateFirstColl,    # Arbiter is active and its not the first collision.
    ArbiterStateNormal,       # Collision has been explicitly ignored.
                              # Either by returning false from a begin collision handler or calling cpArbiterIgnore().
    ArbiterStateIgnore,       # Collison is no longer active. A space will cache an arbiter for up to cpSpace.collisionPersistence more steps.
    ArbiterStateCached
  TArbiterThread*{.pure, final.} = object 
    next*: PArbiter        # Links to next and previous arbiters in the contact graph.
    prev*: PArbiter
  
  TContactPoint*{.pure, final.} = object 
    point*: TVector    #/ The position of the contact point.
    normal*: TVector   #/ The normal of the contact point.
    dist*: cdouble     #/ The depth of the contact point.
  #/ A struct that wraps up the important collision data for an arbiter.
  TContactPointSet*{.pure, final.} = object 
    count*: cint              #/ The number of contact points in the set.
    points*: array[0..CP_MAX_CONTACTS_PER_ARBITER - 1, TContactPoint] #/ The array of contact points.
  
  #/ Collision begin event function callback type.
  #/ Returning false from a begin callback causes the collision to be ignored until
  #/ the the separate callback is called when the objects stop colliding.
  TCollisionBeginFunc* = proc (arb: PArbiter; space: PSpace; data: pointer): Bool{.
      cdecl.}
  #/ Collision pre-solve event function callback type.
  #/ Returning false from a pre-step callback causes the collision to be ignored until the next step.
  TCollisionPreSolveFunc* = proc (arb: PArbiter; space: PSpace; 
                                  data: pointer): bool {.cdecl.}
  #/ Collision post-solve event function callback type.
  TCollisionPostSolveFunc* = proc (arb: PArbiter; space: PSpace; 
                                   data: pointer){.cdecl.}
  #/ Collision separate event function callback type.
  TCollisionSeparateFunc* = proc (arb: PArbiter; space: PSpace; 
                                  data: pointer){.cdecl.}
  
  #/ Chipmunk's axis-aligned 2D bounding box type. (left, bottom, right, top)
  PBB* = ptr TBB
  TBB* {.pure, final.} = object 
    l*, b*, r*, t*: cdouble
  
  #/ Spatial index bounding box callback function type.
  #/ The spatial index calls this function and passes you a pointer to an object you added
  #/ when it needs to get the bounding box associated with that object.
  TSpatialIndexBBFunc* = proc (obj: pointer): TBB{.cdecl.}
  #/ Spatial index/object iterator callback function type.
  TSpatialIndexIteratorFunc* = proc (obj: pointer; data: pointer){.cdecl.}
  #/ Spatial query callback function type. 
  TSpatialIndexQueryFunc* = proc (obj1: pointer; obj2: pointer; data: pointer){.
      cdecl.}
  #/ Spatial segment query callback function type.
  TSpatialIndexSegmentQueryFunc* = proc (obj1: pointer; obj2: pointer; 
      data: pointer): Float{.cdecl.}
  #/ private
  PSpatialIndex = ptr TSpatialIndex
  TSpatialIndex{.pure, final.} = object 
    klass: ptr TSpatialIndexClass
    bbfunc: TSpatialIndexBBFunc
    staticIndex: ptr TSpatialIndex
    dynamicIndex: ptr TSpatialIndex

  TSpatialIndexDestroyImpl* = proc (index: ptr TSpatialIndex){.cdecl.}
  TSpatialIndexCountImpl* = proc (index: ptr TSpatialIndex): cint{.cdecl.}
  TSpatialIndexEachImpl* = proc (index: ptr TSpatialIndex; 
                                 func: TSpatialIndexIteratorFunc; data: pointer){.
      cdecl.}
  TSpatialIndexContainsImpl* = proc (index: ptr TSpatialIndex; obj: pointer; 
                                     hashid: THashValue): Bool32 {.cdecl.}
  TSpatialIndexInsertImpl* = proc (index: ptr TSpatialIndex; obj: pointer; 
                                   hashid: THashValue){.cdecl.}
  TSpatialIndexRemoveImpl* = proc (index: ptr TSpatialIndex; obj: pointer; 
                                   hashid: THashValue){.cdecl.}
  TSpatialIndexReindexImpl* = proc (index: ptr TSpatialIndex){.cdecl.}
  TSpatialIndexReindexObjectImpl* = proc (index: ptr TSpatialIndex; 
      obj: pointer; hashid: THashValue){.cdecl.}
  TSpatialIndexReindexQueryImpl* = proc (index: ptr TSpatialIndex; 
      func: TSpatialIndexQueryFunc; data: pointer){.cdecl.}
  TSpatialIndexPointQueryImpl* = proc (index: ptr TSpatialIndex; point: TVector; 
                                       func: TSpatialIndexQueryFunc; 
                                       data: pointer){.cdecl.}
  TSpatialIndexSegmentQueryImpl* = proc (index: ptr TSpatialIndex; obj: pointer; 
      a: TVector; b: TVector; t_exit: cdouble; func: TSpatialIndexSegmentQueryFunc; 
      data: pointer){.cdecl.}
  TSpatialIndexQueryImpl* = proc (index: ptr TSpatialIndex; obj: pointer; 
                                  bb: TBB; func: TSpatialIndexQueryFunc; 
                                  data: pointer){.cdecl.}
  TSpatialIndexClass*{.pure, final.} = object 
    destroy*: TSpatialIndexDestroyImpl
    count*: TSpatialIndexCountImpl
    each*: TSpatialIndexEachImpl
    contains*: TSpatialIndexContainsImpl
    insert*: TSpatialIndexInsertImpl
    remove*: TSpatialIndexRemoveImpl
    reindex*: TSpatialIndexReindexImpl
    reindexObject*: TSpatialIndexReindexObjectImpl
    reindexQuery*: TSpatialIndexReindexQueryImpl
    pointQuery*: TSpatialIndexPointQueryImpl
    segmentQuery*: TSpatialIndexSegmentQueryImpl
    query*: TSpatialIndexQueryImpl

  PContactBufferHeader* = ptr TContentBufferHeader
  TContentBufferHeader* {.pure, final.} = object
  TSpaceArbiterApplyImpulseFunc* = proc (arb: PArbiter){.cdecl.}
  
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
    staticShapes: PSpatialIndex
    activeShapes: PSpatialIndex
    arbiters: PArray
    contactBuffersHead: PContactBufferHeader
    cachedArbiters: PHashSet
    pooledArbiters: PArray
    constraints: PArray
    allocatedBuffers: PArray
    locked: cint
    collisionHandlers: PHashSet
    defaultHandler: TCollisionHandler
    postStepCallbacks: PHashSet
    arbiterApplyImpulse: TSpaceArbiterApplyImpulseFunc
    staticBody2: TBody  #_staticBody 
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
    arbiterList*: PArbiter
    constraintList*: PConstraint
    node*: TComponentNode
  #/ Segment query info struct.
  PSegmentQueryInfo* = ptr TSegmentQueryInfo
  TSegmentQueryInfo*{.pure, final.} = object 
    shape*: PShape         #/ The shape that was hit, NULL if no collision occured.
    t*: cdouble            #/ The normalized distance along the query segment in the range [0, 1].
    n*: TVector            #/ The normal of the surface hit.
  TShapeType*{.size: sizeof(cint).} = enum 
    CP_CIRCLE_SHAPE, CP_SEGMENT_SHAPE, CP_POLY_SHAPE, CP_NUM_SHAPES
  TShapeCacheDataImpl* = proc (shape: PShape; p: TVector; rot: TVector): TBB{.cdecl.}
  TShapeDestroyImpl* = proc (shape: PShape){.cdecl.}
  TShapePointQueryImpl* = proc (shape: PShape; p: TVector): bool32 {.cdecl.}
  TShapeSegmentQueryImpl* = proc (shape: PShape; a: TVector; b: TVector; 
                                  info: PSegmentQueryInfo){.cdecl.}
  PShapeClass* = ptr TShapeClass
  TShapeClass*{.pure, final.} = object 
    kind*: TShapeType
    cacheData*: TShapeCacheDataImpl
    destroy*: TShapeDestroyImpl
    pointQuery*: TShapePointQueryImpl
    segmentQuery*: TShapeSegmentQueryImpl
  PShape* = ptr TShape
  TShape*{.pure, final.} = object 
    klass: PShapeClass   #/ PRIVATE
    body*: PBody           #/ The rigid body this collision shape is attached to.
    bb*: TBB               #/ The current bounding box of the shape.   
    sensor*: Bool32        #/ Sensor flag.
                           #/ Sensor shapes call collision callbacks but don't produce collisions.  
    e*: cdouble            #/ Coefficient of restitution. (elasticity)
    u*: cdouble            #/ Coefficient of friction.
    surface_v*: TVector    #/ Surface velocity used when solving for friction.
    data*: pointer        #/ User definable data pointer. Generally this points to your the game object class so you can access it when given a cpShape reference in a callback.
    collision_type*: TCollisionType #/ Collision type of this shape used when picking collision handlers.
    group*: TGroup      #/ Group of this shape. Shapes in the same group don't collide.
    layers*: TLayers   #/ Layer bitmask for this shape. Shapes only collide if the bitwise and of their layers is non-zero.
    space: PSpace        #PRIVATE
    next: PShape         #PRIVATE
    prev: PShape         #PRIVATE
    hashid: THashValue  #PRIVATE
  PCircleShape* = ptr TCircleShape
  TCircleShape*{.pure, final.} = object

#/ Version string.
var VersionString*{.importc: "cpVersionString", dynlib: Lib.}: cstring
#/ Calculate the moment of inertia for a circle.
#/ @c r1 and @c r2 are the inner and outer diameters. A solid circle has an inner diameter of 0.
proc MomentForCircle*(m, r1, r2: cdouble; offset: TVector): cdouble {.
  cdecl, importc: "cpMomentForCircle", dynlib: Lib.}

#/ Calculate area of a hollow circle.
#/ @c r1 and @c r2 are the inner and outer diameters. A solid circle has an inner diameter of 0.
proc AreaForCircle*(r1: cdouble; r2: cdouble): cdouble {.
  cdecl, importc: "cpAreaForCircle", dynlib: Lib.}
#/ Calculate the moment of inertia for a line segment.
#/ Beveling radius is not supported.
proc MomentForSegment*(m: cdouble; a, b: TVector): cdouble {.
  cdecl, importc: "cpMomentForSegment", dynlib: Lib.}
#/ Calculate the area of a fattened (capsule shaped) line segment.
proc AreaForSegment*(a, b: TVector; r: cdouble): cdouble {.
  cdecl, importc: "cpAreaForSegment", dynlib: Lib.}
#/ Calculate the moment of inertia for a solid polygon shape assuming it's center of gravity is at it's centroid. The offset is added to each vertex.
proc MomentForPoly*(m: cdouble; numVerts: cint; verts: ptr TVector; offset: TVector): cdouble {.
  cdecl, importc: "cpMomentForPoly", dynlib: Lib.}
#/ Calculate the signed area of a polygon. A Clockwise winding gives positive area.
#/ This is probably backwards from what you expect, but matches Chipmunk's the winding for poly shapes.
proc AreaForPoly*(numVerts: cint; verts: ptr TVector): cdouble {.
  cdecl, importc: "cpAreaForPoly", dynlib: Lib.}
#/ Calculate the natural centroid of a polygon.
proc CentroidForPoly*(numVerts: cint; verts: ptr TVector): TVector {.
  cdecl, importc: "cpCentroidForPoly", dynlib: Lib.}
#/ Center the polygon on the origin. (Subtracts the centroid of the polygon from each vertex)
proc RecenterPoly*(numVerts: cint; verts: ptr TVector) {.
  cdecl, importc: "cpRecenterPoly", dynlib: Lib.}
#/ Calculate the moment of inertia for a solid box.
proc MomentForBox*(m, width, height: cdouble): cdouble {.
  cdecl, importc: "cpMomentForBox", dynlib: Lib.}
#/ Calculate the moment of inertia for a solid box.
proc MomentForBox2*(m: cdouble; box: TBB): cdouble {.
  cdecl, importc: "cpMomentForBox2", dynlib: Lib.}

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
proc isSleeping*(body: PBody): Bool {.inline.} = 
  return body.node.root != nil
#/ Returns true if the body is static.
proc isStatic*(body: PBody): bool {.inline.} = 
  return body.node.idleTime == INFINITY
#/ Returns true if the body has not been added to a space.
proc isRogue*(body: PBody): Bool {.inline.} = 
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



defProp(PArbiter, cdouble, e, Elasticity)
defProp(PArbiter, cdouble, u, Friction)
depProp(PArbiter, TVector, surface_vr, SurfaceVelocity)

#/ Calculate the total impulse that was applied by this 
#/ This function should only be called from a post-solve, post-step or cpBodyEachArbiter callback.
proc totalImpulse*(obj: PArbiter): TVector {.cdecl, importc: "cpArbiterTotalImpulse", dynlib: Lib.}

#/ Calculate the total impulse including the friction that was applied by this arbiter.
#/ This function should only be called from a post-solve, post-step or cpBodyEachArbiter callback.
proc totalImpulseWithFriction*(obj: PArbiter): TVector {.cdecl, importc: "cpArbiterTotalImpulseWithFriction", dynlib: Lib.}

#/ Calculate the amount of energy lost in a collision including static, but not dynamic friction.
#/ This function should only be called from a post-solve, post-step or cpBodyEachArbiter callback.
proc totalKE*(obj: PArbiter): cdouble {.cdecl, importc: "cpArbiterTotalKE", dynlib: Lib.}


#/ Causes a collision pair to be ignored as if you returned false from a begin callback.
#/ If called from a pre-step callback, you will still need to return false
#/ if you want it to be ignored in the current step.
proc ignore*(arb: PArbiter) {.cdecl, importc: "cpArbiterIgnore", dynlib: Lib.}

#/ Return the colliding shapes involved for this arbiter.
#/ The order of their cpSpace.collision_type values will match
#/ the order set when the collision handler was registered.
proc getShapes*(arb: PArbiter, a, b: var PShape) {.inline.} =
  if arb.swappedColl.bool:
    a = arb.b
    b = arb.a
  else:
    a = arb.a
    b = arb.b

#/ A macro shortcut for defining and retrieving the shapes from an arbiter.
#define CP_ARBITER_GET_SHAPES(arb, a, b) cpShape *a, *b; cpArbiterGetShapes(arb, &a, &b);
template getShapes*(arb: PArbiter, name1, name2: expr): stmt {.immediate.} =
  var name1, name2: PShape
  getShapes(arb, name1, name2)


#/ Return the colliding bodies involved for this arbiter.
#/ The order of the cpSpace.collision_type the bodies are associated with values will match
#/ the order set when the collision handler was registered.
proc getBodies*(arb: PArbiter, a, b: var PBody) {.inline.} = 
  getShapes(arb, shape1, shape2)
  a = shape1.body
  b = shape2.body

#/ A macro shortcut for defining and retrieving the bodies from an arbiter.
#define CP_ARBITER_GET_BODIES(arb, a, b) cpBody *a, *b; cpArbiterGetBodies(arb, &a, &b);
template getBodies*(arb: PArbiter, name1, name2: expr): stmt {.immediate.} =
  var name1, name2: PBOdy
  getBodies(arb, name1, name2)

proc isFirstContact*(arb: PArbiter): bool32 {.inline.} =
  result = arb.state == FirstColl

proc getCount*(arb: PArbiter): cint {.inline.} =
  result = arb.numContacts

#/ Return a contact set from an arbiter.
proc getContactPointSet*(arb: PArbiter): TContactPointSet {.
  cdecl, importc: "cpArbiterGetContactPointSet", dynlib: Lib.}
#/ Get the normal of the @c ith contact point.
proc getNormal*(arb: PArbiter; i: cint): TVector {.
  cdecl, importc: "cpArbiterGetNormal", dynlib: Lib.}
#/ Get the position of the @c ith contact point.
proc getPoint*(arb: PArbiter; i: cint): TVector {.
  cdecl, importc: "cpArbiterGetPoint", dynlib: Lib.}
#/ Get the depth of the @c ith contact point.
proc getDepth*(arb: PArbiter; i: cint): cdouble {.
  cdecl, importc: "cpArbiterGetDepth", dynlib: Lib.}

##Shapes
template defShapeSetter(memberType: typedesc, memberName: expr, procName: expr, activates: bool): stmt {.immediate.} =
  proc `set procName`*(obj: PShape, value: memberType) =
    if activates and obj.body != nil: obj.body.activate()
    obj.memberName = value
template defShapeProp(memberType: typedesc, memberName: expr, procName: expr, activates: bool): stmt {.immediate.} =
  defGetter(PShape, memberType, memberName, procName)
  defShapeSetter(memberType, memberName, procName, activates)

#/ Destroy a shape.
proc destroy*(shape: PShape) {.
  cdecl, importc: "cpShapeDestroy", dynlib: Lib.}
#/ Destroy and Free a shape.
proc free*(shape: PShape){.
  cdecl, importc: "cpShapeFree", dynlib: Lib.}
#/ Update, cache and return the bounding box of a shape based on the body it's attached to.
proc cacheBB*(shape: PShape): TBB{.
cdecl, importc: "cpShapeCacheBB", dynlib: Lib.}
#/ Update, cache and return the bounding box of a shape with an explicit transformation.
proc update*(shape: PShape; pos: TVector; rot: TVector): TBB {.
  cdecl, importc: "cpShapeUpdate", dynlib: Lib.}
#/ Test if a point lies within a shape.
proc pointQuery*(shape: PShape; p: TVector): Bool32 {.
  cdecl, importc: "cpShapePointQuery", dynlib: Lib.}

defGetter(PShape, PBody, body, Body)
proc setBody*(shape: PShape, value: PBody) {.
  cdecl, importc: "cpShapeSetBody", dynlib: Lib.}

defGetter(PShape, TBB, bb, BB)
defShapeProp(Bool32, sensor, Sensor, true)
defShapeProp(cdouble, e, Elasiticity, false)
defShapeProp(cdouble, u, Friction, true)
defShapeProp(TVector, surface_v, SurfaceVelocity, true)
defShapeProp(pointer, data, UserData, false)
defShapeProp(TCollisionType, collision_type, CollisionType, true)
defShapeProp(TGroup, group, Group, true)
defShapeProp(TLayers, layers, Layers, true)
