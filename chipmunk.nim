
when defined(Linux):
  const Lib = "libchipmunk.so.6.1.1"
else:
  {.error: "Platform unsupported".}
from math import sqrt, sin, cos, arctan2
const 
  CP_BUFFER_BYTES* = (32 * 1024)  
  CP_MAX_CONTACTS_PER_ARBITER* = 4
  CpInfinity: cfloat = 1.0/0
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
  
  THashValue = cuint  # uintptr_t 
  TCollisionType = cuint #uintptr_t
  TGroup * = cuint #uintptr_t
  TLayers* = cuint
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
  PContactPointSet* = ptr TContactPointSet
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
      data: pointer): cdouble {.cdecl.}
  #/ private
  PSpatialIndex = ptr TSpatialIndex
  TSpatialIndex{.pure, final.} = object 
    klass: PSpatialIndexClass
    bbfunc: TSpatialIndexBBFunc
    staticIndex: PSpatialIndex
    dynamicIndex: PSpatialIndex

  TSpatialIndexDestroyImpl* = proc (index: PSpatialIndex){.cdecl.}
  TSpatialIndexCountImpl* = proc (index: PSpatialIndex): cint{.cdecl.}
  TSpatialIndexEachImpl* = proc (index: PSpatialIndex; 
                                 func: TSpatialIndexIteratorFunc; data: pointer){.
      cdecl.}
  TSpatialIndexContainsImpl* = proc (index: PSpatialIndex; obj: pointer; 
                                     hashid: THashValue): Bool32 {.cdecl.}
  TSpatialIndexInsertImpl* = proc (index: PSpatialIndex; obj: pointer; 
                                   hashid: THashValue){.cdecl.}
  TSpatialIndexRemoveImpl* = proc (index: PSpatialIndex; obj: pointer; 
                                   hashid: THashValue){.cdecl.}
  TSpatialIndexReindexImpl* = proc (index: PSpatialIndex){.cdecl.}
  TSpatialIndexReindexObjectImpl* = proc (index: PSpatialIndex; 
      obj: pointer; hashid: THashValue){.cdecl.}
  TSpatialIndexReindexQueryImpl* = proc (index: PSpatialIndex; 
      func: TSpatialIndexQueryFunc; data: pointer){.cdecl.}
  TSpatialIndexPointQueryImpl* = proc (index: PSpatialIndex; point: TVector; 
                                       func: TSpatialIndexQueryFunc; 
                                       data: pointer){.cdecl.}
  TSpatialIndexSegmentQueryImpl* = proc (index: PSpatialIndex; obj: pointer; 
      a: TVector; b: TVector; t_exit: cdouble; func: TSpatialIndexSegmentQueryFunc; 
      data: pointer){.cdecl.}
  TSpatialIndexQueryImpl* = proc (index: PSpatialIndex; obj: pointer; 
                                  bb: TBB; func: TSpatialIndexQueryFunc; 
                                  data: pointer){.cdecl.}
  PSpatialIndexClass* = ptr TSpatialIndexClass
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
  
  PSpaceHash* = ptr TSpaceHash
  TSpaceHash* {.pure, final.} = object
  PBBTree* = ptr TBBTree
  TBBTree* {.pure, final.} = object
  PSweep1D* = ptr TSweep1D
  TSweep1D* {.pure, final.} = object

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
  #/ Body/shape iterator callback function type. 
  TBodyShapeIteratorFunc* = proc (body: PBody; shape: PShape; 
                                   data: pointer) {.cdecl.}
  #/ Body/constraint iterator callback function type. 
  TBodyConstraintIteratorFunc* = proc (body: PBody; 
                                        constraint: PConstraint; 
                                        data: pointer) {.cdecl.}
  #/ Body/arbiter iterator callback function type. 
  TBodyArbiterIteratorFunc* = proc (body: PBody; arbiter: PArbiter; 
                                     data: pointer)
  
  PNearestPointQueryInfo* = ptr TNearestPointQueryInfo
  #/ Nearest point query info struct.
  TNearestPointQueryInfo*{.pure, final.} = object
    shape: PShape  #/ The nearest shape, NULL if no shape was within range.
    p: TVector     #/ The closest point on the shape's surface. (in world space coordinates)
    d: cdouble      #/ The distance to the point. The distance is negative if the point is inside the shape.
  
  PSegmentQueryInfo* = ptr TSegmentQueryInfo
  #/ Segment query info struct.
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
    shape: PShape
    c, tc: TVector
    r: cdouble
  PSegmentShape* = ptr TSegmentShape
  TSegmentShape*{.pure, final.} = object
    shape: PShape
    a, b, n: TVector
    ta, tb, tn: TVector
    r: cdouble
    aTangent, bTangent: TVector
  
  #/ Post Step callback function type.
  TPostStepFunc* = proc (space: PSpace; obj: pointer; data: pointer){.cdecl.}
  #/ Point query callback function type.
  TSpacePointQueryFunc* = proc (shape: PShape; data: pointer){.cdecl.}
  #/ Segment query callback function type.
  TSpaceSegmentQueryFunc* = proc (shape: PShape; t: cdouble; n: TVector; 
                                  data: pointer){.cdecl.}
  #/ Rectangle Query callback function type.
  TSpaceBBQueryFunc* = proc (shape: PShape; data: pointer){.cdecl.}
  #/ Shape query callback function type.
  TSpaceShapeQueryFunc* = proc (shape: PShape; points: PContactPointSet; 
                                data: pointer){.cdecl.}
  #/ Space/body iterator callback function type.
  TSpaceBodyIteratorFunc* = proc (body: PBody; data: pointer){.cdecl.}
  #/ Space/body iterator callback function type.
  TSpaceShapeIteratorFunc* = proc (shape: PShape; data: pointer){.cdecl.}
  #/ Space/constraint iterator callback function type.
  TSpaceConstraintIteratorFunc* = proc (constraint: PConstraint; 
                                        data: pointer){.cdecl.}

#/ Version string.
#var VersionString*{.importc: "cpVersionString", dynlib: Lib.}: cstring
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
  proc `get procName`*(obj: otype): memberType {.cdecl.} =
    return obj.memberName
template defSetter(otype: typedesc, memberType: typedesc, memberName: expr, procName: expr): stmt {.immediate.} =
  proc `set procName`*(obj: otype, value: memberType) {.cdecl.} =
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

defProp(PSpace, cint, iterations, Iterations)
defProp(PSpace, TVector, gravity, Gravity)
defProp(PSpace, cdouble, damping, Damping)
defProp(PSpace, cdouble, idleSpeedThreshold, IdleSpeedThreshold)
defProp(PSpace, cdouble, sleepTimeThreshold, SleepTimeThreshold)
defProp(PSpace, cdouble, collisionSlop, CollisionSlop)
defProp(PSpace, cdouble, collisionBias, CollisionBias)
defProp(PSpace, TTimestamp, collisionPersistence, CollisionPersistence)
defProp(PSpace, Bool32, enableContactGraph, EnableContactGraph)
defProp(PSpace, pointer, data, UserData)
defGetter(PSpace, PBody, staticBody, StaticBody)
defGetter(PSpace, cdouble, currDt, CurrentTimeStep)


#/ returns true from inside a callback and objects cannot be added/removed.
proc isLocked*(space: PSpace): Bool{.inline.} = 
  result = space.locked.bool

#/ Set a default collision handler for this space.
#/ The default collision handler is invoked for each colliding pair of shapes
#/ that isn't explicitly handled by a specific collision handler.
#/ You can pass NULL for any function you don't want to implement.
proc setDefaultCollisionHandler*(space: PSpace; begin: TCollisionBeginFunc; 
                                  preSolve: TCollisionPreSolveFunc; 
                                  postSolve: TCollisionPostSolveFunc; 
                                  separate: TCollisionSeparateFunc; 
                                  data: pointer){.
  cdecl, importc: "cpSpaceSetDefaultCollisionHandler", dynlib: Lib.}
#/ Set a collision handler to be used whenever the two shapes with the given collision types collide.
#/ You can pass NULL for any function you don't want to implement.
proc addCollisionHandler*(space: PSpace; a: TCollisionType; 
                               b: TCollisionType; begin: TCollisionBeginFunc; 
                               preSolve: TCollisionPreSolveFunc; 
                               postSolve: TCollisionPostSolveFunc; 
                               separate: TCollisionSeparateFunc; data: pointer){.
  cdecl, importc: "cpSpaceAddCollisionHandler", dynlib: Lib.}
#/ Unset a collision handler.
proc removeCollisionHandler*(space: PSpace; a: TCollisionType; 
                                  b: TCollisionType){.
  cdecl, importc: "cpSpaceRemoveCollisionHandler", dynlib: Lib.}
#/ Add a collision shape to the simulation.
#/ If the shape is attached to a static body, it will be added as a static shape.
proc addShape*(space: PSpace; shape: PShape): PShape{.
  cdecl, importc: "cpSpaceAddShape", dynlib: Lib.}
#/ Explicity add a shape as a static shape to the simulation.
proc addStaticShape*(space: PSpace; shape: PShape): PShape{.
  cdecl, importc: "cpSpaceAddStaticShape", dynlib: Lib.}
#/ Add a rigid body to the simulation.
proc addBody*(space: PSpace; body: PBody): PBody{.
  cdecl, importc: "cpSpaceAddBody", dynlib: Lib.}
#/ Add a constraint to the simulation.
proc addConstraint*(space: PSpace; constraint: PConstraint): PConstraint{.
    cdecl, importc: "cpSpaceAddConstraint", dynlib: Lib.}
#/ Remove a collision shape from the simulation.
proc removeShape*(space: PSpace; shape: PShape){.
  cdecl, importc: "cpSpaceRemoveShape", dynlib: Lib.}
#/ Remove a collision shape added using cpSpaceAddStaticShape() from the simulation.
proc removeStaticShape*(space: PSpace; shape: PShape){.
  cdecl, importc: "cpSpaceRemoveStaticShape", dynlib: Lib.}
#/ Remove a rigid body from the simulation.
proc removeBody*(space: PSpace; body: PBody){.
  cdecl, importc: "cpSpaceRemoveBody", dynlib: Lib.}
#/ Remove a constraint from the simulation.
proc RemoveConstraint*(space: PSpace; constraint: PConstraint){.
  cdecl, importc: "cpSpaceRemoveConstraint", dynlib: Lib.}
#/ Test if a collision shape has been added to the space.
proc containsShape*(space: PSpace; shape: PShape): Bool{.
  cdecl, importc: "cpSpaceContainsShape", dynlib: Lib.}
#/ Test if a rigid body has been added to the space.
proc containsBody*(space: PSpace; body: PBody): Bool{.
  cdecl, importc: "cpSpaceContainsBody", dynlib: Lib.}
#/ Test if a constraint has been added to the space.

proc containsConstraint*(space: PSpace; constraint: PConstraint): Bool{.
  cdecl, importc: "cpSpaceContainsConstraint", dynlib: Lib.}
#/ Schedule a post-step callback to be called when cpSpaceStep() finishes.
#/ @c obj is used a key, you can only register one callback per unique value for @c obj
proc addPostStepCallback*(space: PSpace; func: TPostStepFunc; 
                               obj: pointer; data: pointer){.
  cdecl, importc: "cpSpaceAddPostStepCallback", dynlib: Lib.}
                                        
#/ Query the space at a point and call @c func for each shape found.
proc pointQuery*(space: PSpace; point: TVector; layers: TLayers; 
                      group: TGroup; func: TSpacePointQueryFunc; data: pointer){.
  cdecl, importc: "cpSpacePointQuery", dynlib: Lib.}

#/ Query the space at a point and return the first shape found. Returns NULL if no shapes were found.
proc pointQueryFirst*(space: PSpace; point: TVector; layers: TLayers; 
                       group: TGroup): PShape{.cdecl, 
    importc: "cpSpacePointQueryFirst", dynlib: Lib.}

#/ Perform a directed line segment query (like a raycast) against the space calling @c func for each shape intersected.
proc segmentQuery*(space: PSpace; start: TVector; to: TVector; 
                    layers: TLayers; group: TGroup; 
                    func: TSpaceSegmentQueryFunc; data: pointer){.
  cdecl, importc: "cpSpaceSegmentQuery", dynlib: Lib.}
#/ Perform a directed line segment query (like a raycast) against the space and return the first shape hit. Returns NULL if no shapes were hit.
proc segmentQueryFirst*(space: PSpace; start: TVector; to: TVector; 
                         layers: TLayers; group: TGroup; 
                         res: PSegmentQueryInfo): PShape{.
  cdecl, importc: "cpSpaceSegmentQueryFirst", dynlib: Lib.}

#/ Perform a fast rectangle query on the space calling @c func for each shape found.
#/ Only the shape's bounding boxes are checked for overlap, not their full shape.
proc BBQuery*(space: PSpace; bb: TBB; layers: TLayers; group: TGroup; 
                   func: TSpaceBBQueryFunc; data: pointer){.
  cdecl, importc: "cpSpaceBBQuery", dynlib: Lib.}

#/ Query a space for any shapes overlapping the given shape and call @c func for each shape found.
proc shapeQuery*(space: PSpace; shape: PShape; func: TSpaceShapeQueryFunc; data: pointer): Bool {.
  cdecl, importc: "cpSpaceShapeQuery", dynlib: Lib.}
#/ Call cpBodyActivate() for any shape that is overlaps the given shape.
proc activateShapesTouchingShape*(space: PSpace; shape: PShape){.
    cdecl, importc: "cpSpaceActivateShapesTouchingShape", dynlib: Lib.}

#/ Call @c func for each body in the space.
proc eachBody*(space: PSpace; func: TSpaceBodyIteratorFunc; data: pointer){.
  cdecl, importc: "cpSpaceEachBody", dynlib: Lib.}

#/ Call @c func for each shape in the space.
proc eachShape*(space: PSpace; func: TSpaceShapeIteratorFunc; 
                     data: pointer){.
  cdecl, importc: "cpSpaceEachShape", dynlib: Lib.}
#/ Call @c func for each shape in the space.
proc eachConstraint*(space: PSpace; func: TSpaceConstraintIteratorFunc; 
                          data: pointer){.
  cdecl, importc: "cpSpaceEachConstraint", dynlib: Lib.}
#/ Update the collision detection info for the static shapes in the space.
proc reindexStatic*(space: PSpace){.
  cdecl, importc: "cpSpaceReindexStatic", dynlib: Lib.}
#/ Update the collision detection data for a specific shape in the space.
proc reindexShape*(space: PSpace; shape: PShape){.
  cdecl, importc: "cpSpaceReindexShape", dynlib: Lib.}
#/ Update the collision detection data for all shapes attached to a body.
proc reindexShapesForBody*(space: PSpace; body: PBody){.
  cdecl, importc: "cpSpaceReindexShapesForBody", dynlib: Lib.}
#/ Switch the space to use a spatial has as it's spatial index.
proc SpaceUseSpatialHash*(space: PSpace; dim: cdouble; count: cint){.
  cdecl, importc: "cpSpaceUseSpatialHash", dynlib: Lib.}
#/ Step the space forward in time by @c dt.
proc step*(space: PSpace; dt: cdouble) {.
  cdecl, importc: "cpSpaceStep", dynlib: Lib.}


#/ Convenience constructor for cpVect structs.
proc newVector*(x, y: cdouble): TVector {.inline.} =
  result.x = x
  result.y = y
let VectorZero* = newVector(0.0, 0.0)


#/ Vector dot product.
proc dot*(v1, v2: TVector): cdouble {.inline.} = 
  result = v1.x * v2.x + v1.y * v2.y

#/ Returns the length of v.
#proc len*(v: TVector): cdouble {.
#  cdecl, importc: "cpvlength", dynlib: Lib.}
proc len*(v: TVector): cdouble {.inline.} =
  result = v.dot(v).sqrt
#/ Spherical linearly interpolate between v1 and v2.
proc slerp*(v1, v2: TVector; t: cdouble): TVector {.
  cdecl, importc: "cpvslerp", dynlib: Lib.}
#/ Spherical linearly interpolate between v1 towards v2 by no more than angle a radians
proc slerpconst*(v1, v2: TVector; a: cdouble): TVector {.
  cdecl, importc: "cpvslerpconst", dynlib: Lib.}
#/ Returns the unit length vector for the given angle (in radians).
#proc vectorForAngle*(a: cdouble): TVector {.
#  cdecl, importc: "cpvforangle", dynlib: Lib.}
proc vectorForAngle*(a: cdouble): TVector {.inline.} =
  result = newVector(math.cos(a), math.sin(a))
#/ Returns the angular direction v is pointing in (in radians).
proc toAngle*(v: TVector): cdouble {.inline.} =
  result = math.arctan2(v.y, v.x)
#/	Returns a string representation of v. Intended mostly for debugging purposes and not production use.
#/	@attention The string points to a static local and is reset every time the function is called.
#/	If you want to print more than one vector you will have to split up your printing onto separate lines.
proc `$`*(v: TVector): cstring {.cdecl, importc: "cpvstr", dynlib: Lib.}


#/ Check if two vectors are equal. (Be careful when comparing floating point numbers!)
proc `==`*(v1, v2: TVector): bool {.inline.} =
  result = v1.x == v2.x and v1.y == v2.y

#/ Add two vectors
proc `+`*(v1, v2: TVector): TVector {.inline.} =
  result = newVector(v1.x + v2.x, v1.y + v2.y)

#/ Subtract two vectors.
proc `-`*(v1, v2: TVector): TVector {.inline.} =
  result = newVector(v1.x - v2.x, v1.y - v2.y)

#/ Negate a vector.
proc `-`*(v: TVector): TVector {.inline.} = 
  result = newVector(- v.x, - v.y)

#/ Scalar multiplication.
proc `*`*(v: TVector, s: cdouble): TVector {.inline.} =
  result = newVector(v.x * s, v.y * s)

#/ 2D vector cross product analog.
#/ The cross product of 2D vectors results in a 3D vector with only a z component.
#/ This function returns the magnitude of the z value.
proc cross*(v1, v2: TVector): cdouble {.inline.} = 
  result = v1.x * v2.y - v1.y * v2.x

#/ Returns a perpendicular vector. (90 degree rotation)
proc perp*(v: TVector): TVector {.inline.} = 
  result = newVector(- v.y, v.x)

#/ Returns a perpendicular vector. (-90 degree rotation)
proc rperp*(v: TVector): TVector {.inline.} = 
  result = newVector(v.y, - v.x)

#/ Returns the vector projection of v1 onto v2.
proc project*(v1,v2: TVector): TVector {.inline.} = 
  result = v2 * (v1.dot(v2) / v2.dot(v2))

#/ Uses complex number multiplication to rotate v1 by v2. Scaling will occur if v1 is not a unit vector.

proc rotate*(v1, v2: TVector): TVector {.inline.} = 
  result = newVector(v1.x * v2.x - v1.y * v2.y, v1.x * v2.y + v1.y * v2.x)
#/ Inverse of cpvrotate().
proc unrotate*(v1, v2: TVector): TVector {.inline.} = 
  result = newVector(v1.x * v2.x + v1.y * v2.y, v1.y * v2.x - v1.x * v2.y)
#/ Returns the squared length of v. Faster than cpvlength() when you only need to compare lengths.
proc lenSq*(v: TVector): cdouble {.inline.} = 
  result = v.dot(v)
#/ Linearly interpolate between v1 and v2.
proc lerp*(v1, v2: TVector; t: cdouble): TVector {.inline.} = 
  result = (v1 * (1.0 - t)) + (v2 * t)
#/ Returns a normalized copy of v.
proc normalize*(v: TVector): TVector {.inline.} = 
  result = v * (1.0 / v.len)
#/ Returns a normalized copy of v or cpvzero if v was already cpvzero. Protects against divide by zero errors.
proc normalizeSafe*(v: TVector): TVector {.inline.} = 
  result = if v.x == 0.0 and v.y == 0.0: VectorZero else: v.normalize
#/ Clamp v to length len.
proc clamp*(v: TVector; len: cdouble): TVector {.inline.} = 
  result = if v.dot(v) > len * len: v.normalize * len else: v
#/ Linearly interpolate between v1 towards v2 by distance d.
proc lerpconst*(v1, v2: TVector; d: cdouble): TVector {.inline.} = 
  result = v1 + clamp(v2 - v1, d)             #vadd(v1 + vclamp(vsub(v2, v1), d))
#/ Returns the distance between v1 and v2.
proc dist*(v1, v2: TVector): cdouble {.inline.} = 
  result = (v1 - v2).len #vlength(vsub(v1, v2))
#/ Returns the squared distance between v1 and v2. Faster than cpvdist() when you only need to compare distances.
proc distsq*(v1, v2: TVector): cdouble {.inline.} = 
  result = (v1 - v2).lenSq  #vlengthsq(vsub(v1, v2))
#/ Returns true if the distance between v1 and v2 is less than dist.
proc near*(v1, v2: TVector; dist: cdouble): Bool{.inline.} = 
  result = v1.distSq(v2) < dist * dist





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
  return body.node.idleTime == CpInfinity
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

defGetter(PBody, cdouble, m, Mass)
#/ Set the mass of a body.
proc setMass*(body: PBody; m: cdouble){.
  cdecl, importc: "cpBodySetMass", dynlib: Lib.}

#/ Get the moment of a body.
defGetter(PBody, cdouble, i, Moment)
#/ Set the moment of a body.
proc SetMoment*(body: PBody; i: cdouble) {.
  cdecl, importc: "cpBodySetMoment", dynlib: Lib.}

#/ Get the position of a body.
defGetter(PBody, TVector, p, Pos)
#/ Set the position of a body.
proc setPos*(body: PBody; pos: TVector){.
  cdecl, importc: "cpBodySetPos", dynlib: Lib.}

defProp(PBody, TVector, v, Vel)
defProp(PBody, TVector, f, Force)

#/ Get the angle of a body.
defGetter(PBody, cdouble, a, Angle)
#/ Set the angle of a body.
proc setAngle*(body: PBody; a: cdouble){.
  cdecl, importc: "cpBodySetAngle", dynlib: Lib.}

defProp(PBody, cdouble, w, AngVel)
defProp(PBody, cdouble, t, Torque)
defGetter(PBody, TVector, rot, Rot)
defProp(PBody, cdouble, v_limit, VelLimit)
defProp(PBody, cdouble, w_limit, AngVelLimit)
defProp(PBody, pointer, data, UserData)

#/ Default Integration functions.
proc UpdateVelocity*(body: PBody; gravity: TVector; damping: cdouble; dt: cdouble){.
  cdecl, importc: "cpBodyUpdateVelocity", dynlib: Lib.}
proc UpdatePosition*(body: PBody; dt: cdouble){.
  cdecl, importc: "cpBodyUpdatePosition", dynlib: Lib.}
#/ Convert body relative/local coordinates to absolute/world coordinates.
proc Local2World*(body: PBody; v: TVector): TVector{.inline.} = 
  result = body.p + v.rotate(body.rot) ##return cpvadd(body.p, cpvrotate(v, body.rot))
#/ Convert body absolute/world coordinates to  relative/local coordinates.
proc cpBodyWorld2Local*(body: PBody; v: TVector): TVector{.inline.} = 
  result = (v - body.p).unrotate(body.rot)
#/ Set the forces and torque or a body to zero.
proc cpBodyResetForces*(body: PBody){.
  cdecl, importc: "cpBodyResetForces", dynlib: Lib.}
#/ Apply an force (in world coordinates) to the body at a point relative to the center of gravity (also in world coordinates).
proc cpBodyApplyForce*(body: PBody; f, r: TVector){.
  cdecl, importc: "cpBodyApplyForce", dynlib: Lib.}
#/ Apply an impulse (in world coordinates) to the body at a point relative to the center of gravity (also in world coordinates).
proc cpBodyApplyImpulse*(body: PBody; j, r: TVector){.
  cdecl, importc: "cpBodyApplyImpulse", dynlib: Lib.}
#/ Get the velocity on a body (in world units) at a point on the body in world coordinates.

proc cpBodyGetVelAtWorldPoint*(body: PBody; point: TVector): TVector{.
  cdecl, importc: "cpBodyGetVelAtWorldPoint", dynlib: Lib.}
#/ Get the velocity on a body (in world units) at a point on the body in local coordinates.

proc cpBodyGetVelAtLocalPoint*(body: PBody; point: TVector): TVector{.
  cdecl, importc: "cpBodyGetVelAtLocalPoint", dynlib: Lib.}
#/ Get the kinetic energy of a body.
# static inline cdouble cpBodyKineticEnergy(const cpBody *body)
# {
# 	// Need to do some fudging to avoid NaNs
# 	cpFloat vsq = cpvdot(body->v, body->v);
# 	cpFloat wsq = body->w*body->w;
# 	return (vsq ? vsq*body->m : 0.0f) + (wsq ? wsq*body->i : 0.0f);
# }
proc kineticEnergy*(body: PBOdy): cdouble =
  result = (body.v.dot(body.v) * body.m) + (body.w * body.w * body.i)

#/ Call @c func once for each shape attached to @c body and added to the space.
proc eachShape*(body: PBody; func: TBodyShapeIteratorFunc; 
                      data: pointer){.
  cdecl, importc: "cpBodyEachShape", dynlib: Lib.}
#/ Call @c func once for each constraint attached to @c body and added to the space.
proc eachConstraint*(body: PBody; func: TBodyConstraintIteratorFunc; 
                           data: pointer) {.
  cdecl, importc: "cpBodyEachConstraint", dynlib: Lib.}
#/ Call @c func once for each arbiter that is currently active on the body.
proc eachArbiter*(body: PBody; func: TBodyArbiterIteratorFunc; 
                        data: pointer){.
  cdecl, importc: "cpBodyEachArbiter", dynlib: Lib.}
#/ Allocate a spatial hash.
proc SpaceHashAlloc*(): PSpaceHash{.
  cdecl, importc: "cpSpaceHashAlloc", dynlib: Lib.}
#/ Initialize a spatial hash. 
proc SpaceHashInit*(hash: PSpaceHash; celldim: Float; numcells: cint; 
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
  TBBTreeVelocityFunc* = proc (obj: pointer): TVector {.cdecl.}

#/ Set the velocity function for the bounding box tree to enable temporal coherence.

proc BBTreeSetVelocityFunc*(index: PSpatialIndex; func: TBBTreeVelocityFunc){.
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
defProp(PArbiter, TVector, surface_vr, SurfaceVelocity)

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
#proc getBodies*(arb: PArbiter, a, b: var PBody) {.inline.} = 
#  getShapes(arb, shape1, shape2)
#  a = shape1.body
#  b = shape2.body

#/ A macro shortcut for defining and retrieving the bodies from an arbiter.
#define CP_ARBITER_GET_BODIES(arb, a, b) cpBody *a, *b; cpArbiterGetBodies(arb, &a, &b);
template getBodies*(arb: PArbiter, name1, name2: expr): stmt {.immediate.} =
  var name1, name2: PBOdy
  getBodies(arb, name1, name2)

proc isFirstContact*(arb: PArbiter): bool {.inline.} =
  result = arb.state == ArbiterStateFirstColl

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
  proc `set procName`*(obj: PShape, value: memberType) {.cdecl.} =
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

#/ Perform a nearest point query. It finds the closest point on the surface of shape to a specific point.
#/ The value returned is the distance between the points. A negative distance means the point is inside the shape.
proc nearestPointQuery*(shape: PShape; p: TVector; res: PNearestPointQueryInfo): cdouble {.
  cdecl, importc: "cpShapeNearestPointQuery", dynlib: Lib.}
#/ Perform a segment query against a shape. @c info must be a pointer to a valid cpSegmentQueryInfo structure.
proc segmentQuery*(shape: PShape, a, b: TVector, info: PSegmentQueryInfo): bool {.
  cdecl, importc: "cpShapeSegmentQuery", dynlib: Lib.}

#/ Get the hit point for a segment query.
## Possibly change; info to PSegmentQueryInfo 
proc queryHitPoint*(start, to: TVector, info: TSegmentQueryInfo): TVector {.inline.} =
  result = start.lerp(to, info.t)

#/ Get the hit distance for a segment query.
proc queryHitDist*(start, to: TVector, info: TSegmentQueryInfo): cdouble {.inline.} =
  result = start.dist(to) * info.t

defGetter(PShape, PSpace, space, Space)

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

#/ When initializing a shape, it's hash value comes from a counter.
#/ Because the hash value may affect iteration order, you can reset the shape ID counter
#/ when recreating a space. This will make the simulation be deterministic.
proc resetShapeIdCounter*(): void {.cdecl, importc: "cpResetShapeIdCounter", dynlib: Lib.}
#/ Allocate a circle shape.
proc CircleShapeAlloc*(): PCircleShape {.cdecl, importc: "cpCircleShapeAlloc", dynlib: Lib.}
#/ Initialize a circle shape.
proc init*(circle: PCircleShape, body: PBody, radius: cdouble, offset: TVector): PCircleShape {.
  cdecl, importc: "cpCircleShapeInit", dynlib: Lib.}
#/ Allocate and initialize a circle shape.
proc newCircleShape*(body: PBody, radius: cdouble, offset: TVector): PShape {.
  cdecl, importc: "cpCircleShapeNew", dynlib: Lib.}


proc getCircleOffset*(shape: PShape): TVector {.
  cdecl, importc: "cpCircleShapeGetOffset", dynlib: Lib.}
proc getCircleRadius*(shape: PShape): cdouble {.
  cdecl, importc: "cpCircleShapeGetRadius", dynlib: Lib.}

#/ Allocate a segment shape.
proc allocSegmentShape*(): PSegmentShape {.
  cdecl, importc: "cpSegmentShapeAlloc", dynlib: Lib.}
#/ Initialize a segment shape.
proc init*(seg: PSegmentShape, body: PBody, a, b: TVector, radius: cdouble): PSegmentShape {.
  cdecl, importc: "cpSegmentShapeInit", dynlib: Lib.}
#/ Allocate and initialize a segment shape.
proc newSegmentShape*(body: PBody, a, b: TVector, radius: cdouble): PShape {.
  cdecl, importc: "cpSegmentShapeNew", dynlib: Lib.}

proc setSegmentNeighbors*(shape: PShape, prev, next: TVector) {.
  cdecl, importc: "cpSegmentShapeSetNeighbors", dynlib: Lib.}
proc getSegmentA*(shape: PShape): TVector {.
  cdecl, importc: "cpSegmentShapeGetA", dynlib: Lib.}
proc getSegmentB*(shape: PShape): TVector {.
  cdecl, importc: "cpSegmentShapeGetB", dynlib: Lib.}
proc getSegmentNormal*(shape: PShape): TVector {.
  cdecl, importc: "cpSegmentShapeGetNormal", dynlib: Lib.}
proc getSegmentRadius*(shape: PShape): cdouble {.
  cdecl, importc: "cpSegmentShapeGetRadius", dynlib: Lib.}




