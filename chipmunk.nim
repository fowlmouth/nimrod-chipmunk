# Copyright (c) 2007 Scott Lembcke
#  
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#  
#  The above copyright notice and this permission notice shall be included in
#  all copies or substantial portions of the Software.
#  
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
# 
when defined(Linux):
  const Lib = "libchipmunk.so.6.(1|2).(0|1|2|3|4|5)"
elif defined(Windows):
  const Lib = "chipmunk.6.(1|2).(0|1|2|3|4|5).dll"
else:
  {.error: "Platform unsupported".}
when defined(MoreNim):
  {.hint: "MoreNim defined; some Chipmunk functions replaced in Nim".}
{.deadCodeElim: on.}
from math import sqrt, sin, cos, arctan2
when defined(CpUseFloat):
  {.hint: "CpUseFloat defined; using float32 as float".}
  type CpFloat* = cfloat
else:
  type CpFloat* = cdouble
const 
  CP_BUFFER_BYTES* = (32 * 1024)  
  CP_MAX_CONTACTS_PER_ARBITER* = 4
  CpInfinity*: CpFloat = 1.0/0

import basic2d
when CpFloat is float:
  type Vector* = Vector2d
else:
  type Vector* = object
    x*,y*: cpFloat
const VectorIsVector2d* = Vector is Vector2d

{.pragma: pf, pure, final.}
type 
  Bool32* = cint  #replace one day with cint-compatible bool
  CpDataPointer* = pointer
  Timestamp* = cuint
  BodyVelocityFunc* = proc(body: BodyPtr, gravity: Vector,
                           damping: CpFloat; dt: CpFloat){.cdecl.}
  BodyPositionFunc* = proc(body: BodyPtr; dt: CpFloat){.cdecl.}
  ComponentNode*{.pf.} = object 
    root*: BodyPtr
    next*: BodyPtr
    idleTime*: CpFloat
  
  HashValue = cuint  # uintptr_t 
  CollisionType* = cuint #uintptr_t
  Group * = cuint #uintptr_t
  Layers* = cuint
  ArrayPtr = ptr Array
  Array{.pure,final.} = object
  HashSetPtr = ptr HashSet
  HashSet{.pf.} = object
  ContactPtr* = ptr Contact
  Contact*{.pure,final.} = object
  ArbiterPtr* = ptr Arbiter
  Arbiter*{.pf.} = object 
    e*: CpFloat
    u*: CpFloat 
    surface_vr*: Vector
    data*: pointer
    a*: ShapePtr
    b*: ShapePtr
    body_a*: BodyPtr
    body_b*: BodyPtr
    thread_a*: ArbiterThread
    thread_b*: ArbiterThread
    numContacts*: cint
    contacts*: ContactPtr
    stamp*: Timestamp
    handler*: CollisionHandlerPtr
    swappedColl*: Bool32
    state*: ArbiterState
  CollisionHandlerPtr* = ptr CollisionHandler
  CollisionHandler*{.pf.} = object 
    a*: CollisionType
    b*: CollisionType
    begin*: CollisionBeginFunc
    preSolve*: CollisionPreSolveFunc
    postSolve*: CollisionPostSolveFunc
    separate*: CollisionSeparateFunc
    data*: pointer
  ArbiterState*{.size: sizeof(cint).} = enum 
    ArbiterStateFirstColl,    # Arbiter is active and its not the first collision.
    ArbiterStateNormal,       # Collision has been explicitly ignored.
                              # Either by returning false from a begin collision handler or calling cArbiterPtrIgnore().
    ArbiterStateIgnore,       # Collison is no longer active. A space will cache an arbiter for up to cpSpace.collisionPersistence more steps.
    ArbiterStateCached
  ArbiterThread*{.pf.} = object 
    next*: ArbiterPtr        # Links to next and previous arbiters in the contact graph.
    prev*: ArbiterPtr
  
  ContactPoint*{.pf.} = object 
    point*: Vector    #/ The position of the contact point.
    normal*: Vector   #/ The normal of the contact point.
    dist*: CpFloat     #/ The depth of the contact point.
  #/ A struct that wraps up the important collision data for an arbiter.
  ContactPtrPointSet* = ptr ContactPointSet
  ContactPointSet*{.pf.} = object 
    count*: cint              #/ The number of contact points in the set.
    points*: array[0..CP_MAX_CONTACTS_PER_ARBITER - 1, ContactPoint] #/ The array of contact points.
  
  #/ Collision begin event function callback type.
  #/ Returning false from a begin callback causes the collision to be ignored until
  #/ the the separate callback is called when the objects stop colliding.
  CollisionBeginFunc* = proc (arb: ArbiterPtr; space: SpacePtr; data: pointer): bool{.
      cdecl.}
  #/ Collision pre-solve event function callback type.
  #/ Returning false from a pre-step callback causes the collision to be ignored until the next step.
  CollisionPreSolveFunc* = proc (arb: ArbiterPtr; space: SpacePtr; 
                                  data: pointer): bool {.cdecl.}
  #/ Collision post-solve event function callback type.
  CollisionPostSolveFunc* = proc (arb: ArbiterPtr; space: SpacePtr; 
                                   data: pointer){.cdecl.}
  #/ Collision separate event function callback type.
  CollisionSeparateFunc* = proc (arb: ArbiterPtr; space: SpacePtr; 
                                  data: pointer){.cdecl.}
  
  #/ Chipmunk's axis-aligned 2D bounding box type. (left, bottom, right, top)
  BBPtr* = ptr BB
  BB* {.pf.} = object 
    l*, b*, r*, t*: CpFloat
  
  #/ Spatial index bounding box callback function type.
  #/ The spatial index calls this function and passes you a pointer to an object you added
  #/ when it needs to get the bounding box associated with that object.
  SpatialIndexBBFunc* = proc (obj: pointer): BB{.cdecl.}
  #/ Spatial index/object iterator callback function type.
  SpatialIndexIteratorFunc* = proc (obj: pointer; data: pointer){.cdecl.}
  #/ Spatial query callback function type. 
  SpatialIndexQueryFunc* = proc (obj1: pointer; obj2: pointer; data: pointer){.
      cdecl.}
  #/ Spatial segment query callback function type.
  SpatialIndexSegmentQueryFunc* = proc (obj1: pointer; obj2: pointer; 
      data: pointer): CpFloat {.cdecl.}
  #/ private
  SpatialIndexPtr = ptr SpatialIndex
  SpatialIndex{.pf.} = object 
    klass: SpatialIndexPtrClass
    bbfunc: SpatialIndexBBFunc
    staticIndex: SpatialIndexPtr
    dynamicIndex: SpatialIndexPtr

  SpatialIndexDestroyImpl* = proc (index: SpatialIndexPtr){.cdecl.}
  SpatialIndexCountImpl* = proc (index: SpatialIndexPtr): cint{.cdecl.}
  SpatialIndexEachImpl* = proc (index: SpatialIndexPtr; 
                                 fun: SpatialIndexIteratorFunc; data: pointer){.
      cdecl.}
  SpatialIndexContainsImpl* = proc (index: SpatialIndexPtr; obj: pointer; 
                                     hashid: HashValue): Bool32 {.cdecl.}
  SpatialIndexInsertImpl* = proc (index: SpatialIndexPtr; obj: pointer; 
                                   hashid: HashValue){.cdecl.}
  SpatialIndexRemoveImpl* = proc (index: SpatialIndexPtr; obj: pointer; 
                                   hashid: HashValue){.cdecl.}
  SpatialIndexReindexImpl* = proc (index: SpatialIndexPtr){.cdecl.}
  SpatialIndexReindexObjectImpl* = proc (index: SpatialIndexPtr; 
      obj: pointer; hashid: HashValue){.cdecl.}
  SpatialIndexReindexQueryImpl* = proc (index: SpatialIndexPtr; 
      fun: SpatialIndexQueryFunc; data: pointer){.cdecl.}
  SpatialIndexPointQueryImpl* = proc (index: SpatialIndexPtr; point: Vector; 
                                       fun: SpatialIndexQueryFunc; 
                                       data: pointer){.cdecl.}
  SpatialIndexSegmentQueryImpl* = proc (index: SpatialIndexPtr; obj: pointer; 
      a: Vector; b: Vector; t_exit: CpFloat; fun: SpatialIndexSegmentQueryFunc; 
      data: pointer){.cdecl.}
  SpatialIndexQueryImpl* = proc (index: SpatialIndexPtr; obj: pointer; 
                                  bb: BB; fun: SpatialIndexQueryFunc; 
                                  data: pointer){.cdecl.}
  SpatialIndexPtrClass* = ptr SpatialIndexClass
  SpatialIndexClass*{.pf.} = object 
    destroy*: SpatialIndexDestroyImpl
    count*: SpatialIndexCountImpl
    each*: SpatialIndexEachImpl
    contains*: SpatialIndexContainsImpl
    insert*: SpatialIndexInsertImpl
    remove*: SpatialIndexRemoveImpl
    reindex*: SpatialIndexReindexImpl
    reindexObject*: SpatialIndexReindexObjectImpl
    reindexQuery*: SpatialIndexReindexQueryImpl
    pointQuery*: SpatialIndexPointQueryImpl
    segmentQuery*: SpatialIndexSegmentQueryImpl
    query*: SpatialIndexQueryImpl
  
  SpaceHashPtr* = ptr SpaceHash
  SpaceHash* {.pf.} = object
  BBTreePtr* = ptr BBTree
  BBTree* {.pf.} = object
  Sweep1DPtr* = ptr Sweep1D
  Sweep1D* {.pf.} = object
  
  #/ Bounding box tree velocity callback function.
  #/ This function should return an estimate for the object's velocity.
  BBTreeVelocityFunc* = proc (obj: pointer): Vector {.cdecl.}
  
  ContactBufferHeaderPtr* = ptr ContentBufferHeader
  ContentBufferHeader* {.pf.} = object
  SpaceArbiterApplyImpulseFunc* = proc (arb: ArbiterPtr){.cdecl.}
  
  SpacePtr* = ptr Space
  Space* {.pf.} = object
    iterations*: cint 
    gravity*: Vector
    damping*: CpFloat
    idleSpeedThreshold*: CpFloat 
    sleepTimeThreshold*: CpFloat 
    collisionSlop*: CpFloat 
    collisionBias*: CpFloat
    collisionPersistence*: Timestamp        
    enableContactGraph*: cint ##BOOL
    data*: pointer
    staticBody*: BodyPtr
    stamp: Timestamp
    currDT: CpFloat
    bodies: ArrayPtr
    rousedBodies: ArrayPtr
    sleepingComponents: ArrayPtr
    staticShapes: SpatialIndexPtr
    activeShapes: SpatialIndexPtr
    arbiters: ArrayPtr
    contactBuffersHead: ContactBufferHeaderPtr
    cachedArbiters: HashSetPtr
    pooledArbiters: ArrayPtr
    constraints: ArrayPtr
    allocatedBuffers: ArrayPtr
    locked: cint
    collisionHandlers: HashSetPtr
    defaultHandler: CollisionHandler
    postStepCallbacks: HashSetPtr
    arbiterApplyImpulse: SpaceArbiterApplyImpulseFunc
    staticBody2: Body  #_staticBody 
  BodyPtr* = ptr Body
  Body*{.pf.} = object 
    velocityFunc*: BodyVelocityFunc 
    positionFunc*: BodyPositionFunc                                       
    m*: CpFloat           
    mInv*: CpFloat       
    i*: CpFloat           
    iInv*: CpFloat       
    p*: Vector            
    v*: Vector            
    f*: Vector 
    a*: CpFloat 
    w*: CpFloat 
    t*: CpFloat 
    rot*: Vector 
    data*: pointer
    vLimit*: CpFloat   
    wLimit*: CpFloat
    vBias*: Vector
    wBias*: CpFloat
    space*: SpacePtr
    shapeList*: ShapePtr
    arbiterList*: ArbiterPtr
    constraintList*: ConstraintPtr
    node*: ComponentNode
  #/ Body/shape iterator callback function type. 
  BodyShapeIteratorFunc* = proc (body: BodyPtr; shape: ShapePtr; 
                                 data: pointer) {.cdecl.}
  #/ Body/constraint iterator callback function type. 
  BodyConstraintIteratorFunc* = proc (body: BodyPtr; 
                                      constraint: ConstraintPtr; 
                                      data: pointer) {.cdecl.}
  #/ Body/arbiter iterator callback function type. 
  BodyArbiterIteratorFunc* = proc (body: BodyPtr; arbiter: ArbiterPtr; 
                                   data: pointer) {.cdecl.}
  
  NearestPointQueryInfoPtr* = ptr NearestPointQueryInfo
  #/ Nearest point query info struct.
  NearestPointQueryInfo*{.pf.} = object
    shape: ShapePtr  #/ The nearest shape, NULL if no shape was within range.
    p: Vector     #/ The closest point on the shape's surface. (in world space coordinates)
    d: CpFloat      #/ The distance to the point. The distance is negative if the point is inside the shape.
  
  SegmentQueryInfoPtr* = ptr SegmentQueryInfo
  #/ Segment query info struct.
  SegmentQueryInfo*{.pf.} = object 
    shape*: ShapePtr         #/ The shape that was hit, NULL if no collision occured.
    t*: CpFloat            #/ The normalized distance along the query segment in the range [0, 1].
    n*: Vector            #/ The normal of the surface hit.
  ShapeType*{.size: sizeof(cint).} = enum 
    CP_CIRCLE_SHAPE, CP_SEGMENT_SHAPE, CP_POLY_SHAPE, CP_NUM_SHAPES
  ShapeCacheDataImpl* = proc (shape: ShapePtr; p: Vector; rot: Vector): BB{.cdecl.}
  ShapeDestroyImpl* = proc (shape: ShapePtr){.cdecl.}
  ShapePointQueryImpl* = proc (shape: ShapePtr; p: Vector): Bool32 {.cdecl.}
  ShapeSegmentQueryImpl* = proc (shape: ShapePtr; a: Vector; b: Vector; 
                                  info: SegmentQueryInfoPtr){.cdecl.}
  ShapeClassPtr* = ptr ShapeClass
  ShapeClass*{.pf.} = object 
    kind*: ShapeType
    cacheData*: ShapeCacheDataImpl
    destroy*: ShapeDestroyImpl
    pointQuery*: ShapePointQueryImpl
    segmentQuery*: ShapeSegmentQueryImpl
  ShapePtr* = ptr Shape
  Shape*{.pf.} = object 
    klass*: ShapeClassPtr   #/ PRIVATE
    body*: BodyPtr           #/ The rigid body this collision shape is attached to.
    bb*: BB               #/ The current bounding box of the shape.   
    sensor*: Bool32        #/ Sensor flag.
                           #/ Sensor shapes call collision callbacks but don't produce collisions.  
    e*: CpFloat            #/ Coefficient of restitution. (elasticity)
    u*: CpFloat            #/ Coefficient of friction.
    surface_v*: Vector    #/ Surface velocity used when solving for friction.
    data*: pointer        #/ User definable data pointer. Generally this points to your the game object class so you can access it when given a cShapePtr reference in a callback.
    collision_type*: CollisionType #/ Collision type of this shape used when picking collision handlers.
    group*: Group      #/ Group of this shape. Shapes in the same group don't collide.
    layers*: Layers   #/ Layer bitmask for this shape. Shapes only collide if the bitwise and of their layers is non-zero.
    space: SpacePtr        #PRIVATE
    next: ShapePtr         #PRIVATE
    prev: ShapePtr         #PRIVATE
    hashid: HashValue  #PRIVATE
  CircleShapePtr* = ptr CircleShape
  CircleShape*{.pf.} = object
    shape: ShapePtr
    c, tc: Vector
    r: CpFloat
  PolyShapePtr* = ptr PolyShape
  PolyShape*{.pf.} = object
    shape: ShapePtr
    numVerts: cint
    verts, tVerts: Vector
    planes, tPlanes: SplittingPlanePtr
  SegmentShapePtr* = ptr SegmentShape
  SegmentShape*{.pf.} = object
    shape: ShapePtr
    a, b, n: Vector
    ta, tb, tn: Vector
    r: CpFloat
    aTangent, bTangent: Vector
  SplittingPlanePtr* = ptr SplittingPlane
  SplittingPlane*{.pf.} = object
    n: Vector
    d: CpFloat
  
  #/ Post Step callback function type.
  PostStepFunc* = proc (space: SpacePtr; obj: pointer; data: pointer){.cdecl.}
  #/ Point query callback function type.
  SpacePointQueryFunc* = proc (shape: ShapePtr; data: pointer){.cdecl.}
  #/ Segment query callback function type.
  SpaceSegmentQueryFunc* = proc (shape: ShapePtr; t: CpFloat; n: Vector; 
                                  data: pointer){.cdecl.}
  #/ Rectangle Query callback function type.
  SpaceBBQueryFunc* = proc (shape: ShapePtr; data: pointer){.cdecl.}
  #/ Shape query callback function type.
  SpaceShapeQueryFunc* = proc (shape: ShapePtr; points: ContactPtrPointSet; 
                                data: pointer){.cdecl.}
  #/ Space/body iterator callback function type.
  SpaceBodyIteratorFunc* = proc (body: BodyPtr; data: pointer){.cdecl.}
  #/ Space/body iterator callback function type.
  SpaceShapeIteratorFunc* = proc (shape: ShapePtr; data: pointer){.cdecl.}
  #/ Space/constraint iterator callback function type.
  SpaceConstraintIteratorFunc* = proc (constraint: ConstraintPtr; 
                                        data: pointer){.cdecl.}
  #/ Opaque cConstraintPtr struct.
  ConstraintPtr* = ptr Constraint
  Constraint*{.pf.} = object 
    klass: ConstraintPtrClass #/PRIVATE
    a*: BodyPtr            #/ The first body connected to this constraint.
    b*: BodyPtr              #/ The second body connected to this constraint.
    space: SpacePtr         #/PRIVATE
    next_a: ConstraintPtr  #/PRIVATE
    next_b: ConstraintPtr #/PRIVATE
    maxForce*: CpFloat  #/ The maximum force that this constraint is allowed to use. Defaults to infinity.
    errorBias*: CpFloat #/ The rate at which joint error is corrected. Defaults to pow(1.0 - 0.1, 60.0) meaning that it will correct 10% of the error every 1/60th of a second.
    maxBias*: CpFloat    #/ The maximum rate at which joint error is corrected. Defaults to infinity.       
    preSolve*: ConstraintPreSolveFunc  #/ Function called before the solver runs. Animate your joint anchors, update your motor torque, etc.
    postSolve*: ConstraintPostSolveFunc #/ Function called after the solver runs. Use the applied impulse to perform effects like breakable joints.
    data*: CpDataPointer  # User definable data pointer. Generally this points to your the game object class so you can access it when given a cConstraintPtr reference in a callback.
  ConstraintPreStepImpl = proc (constraint: ConstraintPtr; dt: CpFloat){.cdecl.}
  ConstraintApplyCachedImpulseImpl = proc (constraint: ConstraintPtr; dt_coef: CpFloat){.cdecl.}
  ConstraintApplyImpulseImpl = proc (constraint: ConstraintPtr){.cdecl.}
  ConstraintGetImpulseImpl = proc (constraint: ConstraintPtr): CpFloat{.cdecl.}
  ConstraintPtrClass = ptr ConstraintClass
  ConstraintClass{.pf.} = object 
    preStep*: ConstraintPreStepImpl
    applyCachedImpulse*: ConstraintApplyCachedImpulseImpl
    applyImpulse*: ConstraintApplyImpulseImpl
    getImpulse*: ConstraintGetImpulseImpl
  #/ Callback function type that gets called before solving a joint.
  ConstraintPreSolveFunc* = proc (constraint: ConstraintPtr; space: SpacePtr){.
    cdecl.}
  #/ Callback function type that gets called after solving a joint.
  ConstraintPostSolveFunc* = proc (constraint: ConstraintPtr; space: SpacePtr){.
    cdecl.}

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
proc allocSpace*(): SpacePtr {.
  importc: "cpSpaceAlloc", dynlib: Lib.}
proc Init*(space: SpacePtr): SpacePtr {.
  importc: "cpSpaceInit", dynlib: Lib.}
proc newSpace*(): SpacePtr {.
  importc: "cpSpaceNew", dynlib: Lib.}
proc destroy*(space: SpacePtr) {.
  importc: "cpSpaceDestroy", dynlib: Lib.}
proc free*(space: SpacePtr) {.
  importc: "cpSpaceFree", dynlib: Lib.}

defProp(SpacePtr, cint, iterations, Iterations)
defProp(SpacePtr, Vector, gravity, Gravity)
defProp(SpacePtr, CpFloat, damping, Damping)
defProp(SpacePtr, CpFloat, idleSpeedThreshold, IdleSpeedThreshold)
defProp(SpacePtr, CpFloat, sleepTimeThreshold, SleepTimeThreshold)
defProp(SpacePtr, CpFloat, collisionSlop, CollisionSlop)
defProp(SpacePtr, CpFloat, collisionBias, CollisionBias)
defProp(SpacePtr, Timestamp, collisionPersistence, CollisionPersistence)
defProp(SpacePtr, Bool32, enableContactGraph, EnableContactGraph)
defProp(SpacePtr, pointer, data, UserData)
defGetter(SpacePtr, BodyPtr, staticBody, StaticBody)
defGetter(SpacePtr, CpFloat, currDt, CurrentTimeStep)


#/ returns true from inside a callback and objects cannot be added/removed.
proc isLocked*(space: SpacePtr): bool{.inline.} = 
  result = space.locked.bool

#/ Set a default collision handler for this space.
#/ The default collision handler is invoked for each colliding pair of shapes
#/ that isn't explicitly handled by a specific collision handler.
#/ You can pass NULL for any function you don't want to implement.
proc setDefaulCollisionHandler*(space: SpacePtr; begin: CollisionBeginFunc; 
                                  preSolve: CollisionPreSolveFunc; 
                                  postSolve: CollisionPostSolveFunc; 
                                  separate: CollisionSeparateFunc; 
                                  data: pointer){.
  cdecl, importc: "cpSpaceSetDefaulCollisionHandler", dynlib: Lib.}
#/ Set a collision handler to be used whenever the two shapes with the given collision types collide.
#/ You can pass NULL for any function you don't want to implement.
proc addCollisionHandler*(space: SpacePtr; a, b: CollisionType; 
                           begin: CollisionBeginFunc = nil; 
                           preSolve: CollisionPreSolveFunc = nil; 
                           postSolve: CollisionPostSolveFunc = nil; 
                           separate: CollisionSeparateFunc = nil; 
                           data: pointer = nil){.
  cdecl, importc: "cpSpaceAddCollisionHandler", dynlib: Lib.}
#/ Unset a collision handler.
proc removeCollisionHandler*(space: SpacePtr; a: CollisionType; 
                                  b: CollisionType){.
  cdecl, importc: "cpSpaceRemoveCollisionHandler", dynlib: Lib.}
#/ Add a collision shape to the simulation.
#/ If the shape is attached to a static body, it will be added as a static shape.
proc addShape*(space: SpacePtr; shape: ShapePtr): ShapePtr{.
  cdecl, importc: "cpSpaceAddShape", dynlib: Lib.}
#/ Explicity add a shape as a static shape to the simulation.
proc addStaticShape*(space: SpacePtr; shape: ShapePtr): ShapePtr{.
  cdecl, importc: "cpSpaceAddStaticShape", dynlib: Lib.}
#/ Add a rigid body to the simulation.
proc addBody*(space: SpacePtr; body: BodyPtr): BodyPtr{.
  cdecl, importc: "cpSpaceAddBody", dynlib: Lib.}
#/ Add a constraint to the simulation.
proc addConstraint*(space: SpacePtr; constraint: ConstraintPtr): ConstraintPtr{.
    cdecl, importc: "cpSpaceAddConstraint", dynlib: Lib.}
#/ Remove a collision shape from the simulation.
proc removeShape*(space: SpacePtr; shape: ShapePtr){.
  cdecl, importc: "cpSpaceRemoveShape", dynlib: Lib.}
#/ Remove a collision shape added using cpSpaceAddStaticShape() from the simulation.
proc removeStaticShape*(space: SpacePtr; shape: ShapePtr){.
  cdecl, importc: "cpSpaceRemoveStaticShape", dynlib: Lib.}
#/ Remove a rigid body from the simulation.
proc removeBody*(space: SpacePtr; body: BodyPtr){.
  cdecl, importc: "cpSpaceRemoveBody", dynlib: Lib.}
#/ Remove a constraint from the simulation.
proc RemoveConstraint*(space: SpacePtr; constraint: ConstraintPtr){.
  cdecl, importc: "cpSpaceRemoveConstraint", dynlib: Lib.}
#/ Test if a collision shape has been added to the space.
proc containsShape*(space: SpacePtr; shape: ShapePtr): bool{.
  cdecl, importc: "cpSpaceContainsShape", dynlib: Lib.}
#/ Test if a rigid body has been added to the space.
proc containsBody*(space: SpacePtr; body: BodyPtr): bool{.
  cdecl, importc: "cpSpaceContainsBody", dynlib: Lib.}
#/ Test if a constraint has been added to the space.

proc containsConstraint*(space: SpacePtr; constraint: ConstraintPtr): bool{.
  cdecl, importc: "cpSpaceContainsConstraint", dynlib: Lib.}
#/ Schedule a post-step callback to be called when cpSpaceStep() finishes.
#/ @c obj is used a key, you can only register one callback per unique value for @c obj
proc addPostStepCallback*(space: SpacePtr; fun: PostStepFunc; 
                               obj: pointer; data: pointer){.
  cdecl, importc: "cpSpaceAddPostStepCallback", dynlib: Lib.}
                                        
#/ Query the space at a point and call @c func for each shape found.
proc pointQuery*(space: SpacePtr; point: Vector; layers: Layers; 
                      group: Group; fun: SpacePointQueryFunc; data: pointer){.
  cdecl, importc: "cpSpacePointQuery", dynlib: Lib.}

#/ Query the space at a point and return the first shape found. Returns NULL if no shapes were found.
proc pointQueryFirst*(space: SpacePtr; point: Vector; layers: Layers; 
                       group: Group): ShapePtr{.
  cdecl, importc: "cpSpacePointQueryFirst", dynlib: Lib.}

#/ Perform a directed line segment query (like a raycast) against the space calling @c func for each shape intersected.
proc segmentQuery*(space: SpacePtr; start: Vector; to: Vector; 
                    layers: Layers; group: Group; 
                    fun: SpaceSegmentQueryFunc; data: pointer){.
  cdecl, importc: "cpSpaceSegmentQuery", dynlib: Lib.}
#/ Perform a directed line segment query (like a raycast) against the space and return the first shape hit. Returns NULL if no shapes were hit.
proc segmentQueryFirst*(space: SpacePtr; start: Vector; to: Vector; 
                         layers: Layers; group: Group; 
                         res: SegmentQueryInfoPtr): ShapePtr{.
  cdecl, importc: "cpSpaceSegmentQueryFirst", dynlib: Lib.}

#/ Perform a fast rectangle query on the space calling @c func for each shape found.
#/ Only the shape's bounding boxes are checked for overlap, not their full shape.
proc BBQuery*(space: SpacePtr; bb: BB; layers: Layers; group: Group; 
                   fun: SpaceBBQueryFunc; data: pointer){.
  cdecl, importc: "cpSpaceBBQuery", dynlib: Lib.}

#/ Query a space for any shapes overlapping the given shape and call @c func for each shape found.
proc shapeQuery*(space: SpacePtr; shape: ShapePtr; fun: SpaceShapeQueryFunc; data: pointer): bool {.
  cdecl, importc: "cpSpaceShapeQuery", dynlib: Lib.}
#/ Call cpBodyActivate() for any shape that is overlaps the given shape.
proc activateShapesTouchingShape*(space: SpacePtr; shape: ShapePtr){.
    cdecl, importc: "cpSpaceActivateShapesTouchingShape", dynlib: Lib.}

#/ Call @c func for each body in the space.
proc eachBody*(space: SpacePtr; fun: SpaceBodyIteratorFunc; data: pointer){.
  cdecl, importc: "cpSpaceEachBody", dynlib: Lib.}

#/ Call @c func for each shape in the space.
proc eachShape*(space: SpacePtr; fun: SpaceShapeIteratorFunc; 
                     data: pointer){.
  cdecl, importc: "cpSpaceEachShape", dynlib: Lib.}
#/ Call @c func for each shape in the space.
proc eachConstraint*(space: SpacePtr; fun: SpaceConstraintIteratorFunc; 
                          data: pointer){.
  cdecl, importc: "cpSpaceEachConstraint", dynlib: Lib.}
#/ Update the collision detection info for the static shapes in the space.
proc reindexStatic*(space: SpacePtr){.
  cdecl, importc: "cpSpaceReindexStatic", dynlib: Lib.}
#/ Update the collision detection data for a specific shape in the space.
proc reindexShape*(space: SpacePtr; shape: ShapePtr){.
  cdecl, importc: "cpSpaceReindexShape", dynlib: Lib.}
#/ Update the collision detection data for all shapes attached to a body.
proc reindexShapesForBody*(space: SpacePtr; body: BodyPtr){.
  cdecl, importc: "cpSpaceReindexShapesForBody", dynlib: Lib.}
#/ Switch the space to use a spatial has as it's spatial index.
proc SpaceUseSpatialHash*(space: SpacePtr; dim: CpFloat; count: cint){.
  cdecl, importc: "cpSpaceUseSpatialHash", dynlib: Lib.}
#/ Step the space forward in time by @c dt.
proc step*(space: SpacePtr; dt: CpFloat) {.
  cdecl, importc: "cpSpaceStep", dynlib: Lib.}


#/ Convenience constructor for cpVect structs.
proc vector*(x, y: CpFloat): Vector {.inline.} =
  result.x = x
  result.y = y
proc newVector*(x, y: CpFloat): Vector {.inline.} =
  return vector(x, y)
#let VectorZero* = newVector(0.0, 0.0)
var VectorZero* = newVector(0.0, 0.0)

#/ Vector dot product.
proc dot*(v1, v2: Vector): CpFloat {.inline.} = 
  result = v1.x * v2.x + v1.y * v2.y

#/ Returns the length of v.
#proc len*(v: Vector): CpFloat {.
#  cdecl, importc: "cpvlength", dynlib: Lib.}
proc len*(v: Vector): CpFloat {.inline.} =
  result = v.dot(v).sqrt
#/ Spherical linearly interpolate between v1 and v2.
proc slerp*(v1, v2: Vector; t: CpFloat): Vector {.
  cdecl, importc: "cpvslerp", dynlib: Lib.}
#/ Spherical linearly interpolate between v1 towards v2 by no more than angle a radians
proc slerpconst*(v1, v2: Vector; a: CpFloat): Vector {.
  cdecl, importc: "cpvslerpconst", dynlib: Lib.}
#/ Returns the unit length vector for the given angle (in radians).
#proc vectorForAngle*(a: CpFloat): Vector {.
#  cdecl, importc: "cpvforangle", dynlib: Lib.}
proc vector*(a: CpFloat): Vector =
  result = newVector(math.cos(a), math.sin(a))
template vectorForAngle*(rads: CpFloat): Vector = vector(rads)
#/ Returns the angular direction v is pointing in (in radians).
proc toAngle*(v: Vector): CpFloat {.inline.} =
  result = math.arctan2(v.y, v.x)
#/	Returns a string representation of v. Intended mostly for debugging purposes and not production use.
#/	@attention The string points to a static local and is reset every time the function is called.
#/	If you want to print more than one vector you will have to split up your printing onto separate lines.
#proc `$`*(v: Vector): cstring {.cdecl, importc: "cpvstr", dynlib: Lib.}


#/ Check if two vectors are equal. (Be careful when comparing floating point numbers!)
#proc `==`*(v1, v2: Vector): bool {.inline.} =
#  result = v1.x == v2.x and v1.y == v2.y

when not VectorIsVector2d:

  #/ Add two vectors
  proc `+`*(v1, v2: Vector): Vector {.inline.} =
    result = newVector(v1.x + v2.x, v1.y + v2.y)
  proc `+=`*(v1: var Vector; v2: Vector) =
    v1.x = v1.x + v2.x
    v1.y = v1.y + v2.y

  #/ Subtract two vectors.
  proc `-`*(v1, v2: Vector): Vector {.inline.} =
    result = newVector(v1.x - v2.x, v1.y - v2.y)
  proc `-=`*(v1: var Vector; v2: Vector) =
    v1.x = v1.x - v2.x
    v1.y = v1.y - v2.y

  #/ Negate a vector.
  proc `-`*(v: Vector): Vector {.inline.} = 
    result = newVector(- v.x, - v.y)

  #/ Scalar multiplication.
  proc `*`*(v: Vector, s: CpFloat): Vector {.inline.} =
    result.x = v.x * s
    result.y = v.y * s
  proc `*=`*(v: var Vector; s: CpFloat) =
    v.x = v.x * s
    v.y = v.y * s
  proc normalize*(v: var Vector) {.inline.} = 
    #/ Normalizes vector v
    result *= (1.0 / result.len).CPfloat

#/ 2D vector cross product analog.
#/ The cross product of 2D vectors results in a 3D vector with only a z component.
#/ This function returns the magnitude of the z value.
proc cross*(v1, v2: Vector): CpFloat {.inline.} = 
  result = v1.x * v2.y - v1.y * v2.x

#/ Returns a perpendicular vector. (90 degree rotation)
proc perp*(v: Vector): Vector {.inline.} = 
  result = newVector(- v.y, v.x)

#/ Returns a perpendicular vector. (-90 degree rotation)
proc rperp*(v: Vector): Vector {.inline.} = 
  result = newVector(v.y, - v.x)

#/ Returns the vector projection of v1 onto v2.
proc project*(v1,v2: Vector): Vector {.inline.} = 
  result = v2 * (v1.dot(v2) / v2.dot(v2))

#/ Uses complex number multiplication to rotate v1 by v2. Scaling will occur if v1 is not a unit vector.

proc rotate*(v1, v2: Vector): Vector {.inline.} = 
  result = newVector(v1.x * v2.x - v1.y * v2.y, v1.x * v2.y + v1.y * v2.x)
#/ Inverse of cpvrotate().
proc unrotate*(v1, v2: Vector): Vector {.inline.} = 
  result = newVector(v1.x * v2.x + v1.y * v2.y, v1.y * v2.x - v1.x * v2.y)
#/ Returns the squared length of v. Faster than cpvlength() when you only need to compare lengths.
proc lenSq*(v: Vector): CpFloat {.inline.} = 
  result = v.dot(v)
#/ Linearly interpolate between v1 and v2.
proc lerp*(v1, v2: Vector; t: CpFloat): Vector {.inline.} = 
  result = (v1 * (1.0 - t)) + (v2 * t)

#/ Returns a normalized copy of v or cpvzero if v was already cpvzero. Protects against divide by zero errors.
proc normalizeSafe*(v: Vector): Vector {.inline.} = 
  if v.x == 0.0 and v.y == 0.0: 
    result = VectorZero 
  else:
    result = v
    result.normalize
#/ Clamp v to length len.
proc clamp*(v: Vector; len: CpFloat): Vector {.inline.} = 
  result = if v.dot(v) > len * len: v.normalizeSafe * len else: v
#/ Linearly interpolate between v1 towards v2 by distance d.
proc lerpconst*(v1, v2: Vector; d: CpFloat): Vector {.inline.} = 
  result = v1 + clamp(v2 - v1, d)             #vadd(v1 + vclamp(vsub(v2, v1), d))
#/ Returns the distance between v1 and v2.
proc dist*(v1, v2: Vector): CpFloat {.inline.} = 
  result = (v1 - v2).len #vlength(vsub(v1, v2))
#/ Returns the squared distance between v1 and v2. Faster than cpvdist() when you only need to compare distances.
proc distsq*(v1, v2: Vector): CpFloat {.inline.} = 
  result = (v1 - v2).lenSq  #vlengthsq(vsub(v1, v2))
#/ Returns true if the distance between v1 and v2 is less than dist.
proc near*(v1, v2: Vector; dist: CpFloat): bool{.inline.} = 
  result = v1.distSq(v2) < dist * dist



##cpBody.h
proc allocBody*(): BodyPtr {.importc: "cpBodyAlloc", dynlib: Lib.}
proc init*(body: BodyPtr; m: CpFloat; i: CpFloat): BodyPtr {.
  importc: "cpBodyInit", dynlib: Lib.}
proc newBody*(m: CpFloat; i: CpFloat): BodyPtr {.
  importc: "cpBodyNew", dynlib: Lib.}

proc initStaticBody*(body: BodyPtr): BodyPtr{.
  importc: "cpBodyInitStatic", dynlib: Lib.}
#/ Allocate and initialize a static cpBody.
proc newStatic*(): BodyPtr{.importc: "cpBodyNewStatic", dynlib: Lib.}
#/ Destroy a cpBody.
proc destroy*(body: BodyPtr){.importc: "cpBodyDestroy", dynlib: Lib.}
#/ Destroy and free a cpBody.
proc free*(body: BodyPtr){.importc: "cpBodyFree", dynlib: Lib.}

#/ Wake up a sleeping or idle body.
proc activate*(body: BodyPtr){.importc: "cpBodyActivate", dynlib: Lib.}
#/ Wake up any sleeping or idle bodies touching a static body.
proc activateStatic*(body: BodyPtr; filter: ShapePtr){.
    importc: "cpBodyActivateStatic", dynlib: Lib.}
#/ Force a body to fall asleep immediately.
proc Sleep*(body: BodyPtr){.importc: "cpBodySleep", dynlib: Lib.}
#/ Force a body to fall asleep immediately along with other bodies in a group.
proc SleepWithGroup*(body: BodyPtr; group: BodyPtr){.
    importc: "cpBodySleepWithGroup", dynlib: Lib.}
#/ Returns true if the body is sleeping.
proc isSleeping*(body: BodyPtr): bool {.inline.} = 
  return body.node.root != nil
#/ Returns true if the body is static.
proc isStatic*(body: BodyPtr): bool {.inline.} = 
  return body.node.idleTime == CpInfinity
#/ Returns true if the body has not been added to a space.
proc isRogue*(body: BodyPtr): bool {.inline.} = 
  return body.space == nil

# #define CP_DefineBodyStructGetter(type, member, name) \
# static inline type cpBodyGet##name(const cpBody *body){return body->member;}
# #define CP_DefineBodyStructSetter(type, member, name) \
# static inline void cpBodySet##name(cpBody *body, const type value){ \
# 	cpBodyActivate(body); \
# 	cpBodyAssertSane(body); \
# 	body->member = value; \
# }
template defBodySetter(fieldType: typedesc;
                            fieldName, procName: expr): stmt {.immediate.} =
  proc `set procName`*(body: BodyPtr; value: fieldType) =
    body.activate()
    ##assertsane(body) ##<-- implement this, one day..
    body.fieldName = value
# #define CP_DefineBodyStructProperty(type, member, name) \
# CP_DefineBodyStructGetter(type, member, name) \
# CP_DefineBodyStructSetter(type, member, name)
template defBodyProp(fieldType: typedesc; 
                      fieldName, procName: expr): stmt {.immediate.} =
  defBodySetter(fieldType, fieldName, procName)
  defGetter(BodyPtr, fieldType, fieldName, procName)


defGetter(BodyPtr, CpFloat, m, Mass)
#/ Set the mass of a body.
when defined(MoreNim):
  defBodySetter(CpFloat, m, Mass)
else:
  proc setMass*(body: BodyPtr; m: CpFloat){.
    cdecl, importc: "cpBodySetMass", dynlib: Lib.}

#/ Get the moment of a body.
defGetter(BodyPtr, CpFloat, i, Moment)
#/ Set the moment of a body.
when defined(MoreNim):
  defBodySetter(CpFloat, i, Moment)
else: 
  proc SetMoment*(body: BodyPtr; i: CpFloat) {.
    cdecl, importc: "cpBodySetMoment", dynlib: Lib.}

#/ Get the position of a body.
defGetter(BodyPtr, Vector, p, Pos)
#/ Set the position of a body.
when defined(MoreNim):
  defBodySetter(Vector, p, Pos)
else:
  proc setPos*(body: BodyPtr; pos: Vector) {.
    cdecl, importc: "cpBodySetPos", dynlib: Lib.}

defBodyProp(Vector, v, Vel)
defBodyProp(Vector, f, Force)

#/ Get the angle of a body.
defGetter(BodyPtr, CpFloat, a, Angle)
#/ Set the angle of a body.
proc setAngle*(body: BodyPtr; a: CpFloat){.
  cdecl, importc: "cpBodySetAngle", dynlib: Lib.}

defBodyProp(CpFloat, w, AngVel)
defBodyProp(CpFloat, t, Torque)
defGetter(BodyPtr, Vector, rot, Rot)
defBodyProp(CpFloat, v_limit, VelLimit)
defBodyProp(CpFloat, w_limit, AngVelLimit)
defBodyProp(pointer, data, UserData)

#/ Default Integration functions.
proc UpdateVelocity*(body: BodyPtr; gravity: Vector; damping: CpFloat; dt: CpFloat){.
  cdecl, importc: "cpBodyUpdateVelocity", dynlib: Lib.}
proc UpdatePosition*(body: BodyPtr; dt: CpFloat){.
  cdecl, importc: "cpBodyUpdatePosition", dynlib: Lib.}
#/ Convert body relative/local coordinates to absolute/world coordinates.
proc Local2World*(body: BodyPtr; v: Vector): Vector{.inline.} = 
  result = body.p + v.rotate(body.rot) ##return cpvadd(body.p, cpvrotate(v, body.rot))
#/ Convert body absolute/world coordinates to  relative/local coordinates.
proc world2Local*(body: BodyPtr; v: Vector): Vector{.inline.} = 
  result = (v - body.p).unrotate(body.rot)
#/ Set the forces and torque or a body to zero.
proc resetForces*(body: BodyPtr){.
  cdecl, importc: "cpBodyResetForces", dynlib: Lib.}
#/ Apply an force (in world coordinates) to the body at a point relative to the center of gravity (also in world coordinates).
proc applyForce*(body: BodyPtr; f, r: Vector){.
  cdecl, importc: "cpBodyApplyForce", dynlib: Lib.}
#/ Apply an impulse (in world coordinates) to the body at a point relative to the center of gravity (also in world coordinates).
proc applyImpulse*(body: BodyPtr; j, r: Vector){.
  cdecl, importc: "cpBodyApplyImpulse", dynlib: Lib.}
#/ Get the velocity on a body (in world units) at a point on the body in world coordinates.

proc getVelAtWorldPoint*(body: BodyPtr; point: Vector): Vector{.
  cdecl, importc: "cpBodyGetVelAtWorldPoint", dynlib: Lib.}
#/ Get the velocity on a body (in world units) at a point on the body in local coordinates.
proc getVelAtLocalPoint*(body: BodyPtr; point: Vector): Vector{.
  cdecl, importc: "cpBodyGetVelAtLocalPoint", dynlib: Lib.}
#/ Get the kinetic energy of a body.
# static inline CpFloat cpBodyKineticEnergy(const cpBody *body)
# {
# 	// Need to do some fudging to avoid NaNs
# 	cpFloat vsq = cpvdot(body->v, body->v);
# 	cpFloat wsq = body->w*body->w;
# 	return (vsq ? vsq*body->m : 0.0f) + (wsq ? wsq*body->i : 0.0f);
# }
proc kineticEnergy*(body: BodyPtr): CpFloat =
  result = (body.v.dot(body.v) * body.m) + (body.w * body.w * body.i)

#/ Call @c func once for each shape attached to @c body and added to the space.
proc eachShape*(body: BodyPtr; fun: BodyShapeIteratorFunc; 
                      data: pointer){.
  cdecl, importc: "cpBodyEachShape", dynlib: Lib.}
#/ Call @c func once for each constraint attached to @c body and added to the space.
proc eachConstraint*(body: BodyPtr; fun: BodyConstraintIteratorFunc; 
                           data: pointer) {.
  cdecl, importc: "cpBodyEachConstraint", dynlib: Lib.}
#/ Call @c func once for each arbiter that is currently active on the body.
proc eachArbiter*(body: BodyPtr; fun: BodyArbiterIteratorFunc; 
                        data: pointer){.
  cdecl, importc: "cpBodyEachArbiter", dynlib: Lib.}
#/ Allocate a spatial hash.
proc SpaceHashAlloc*(): SpaceHashPtr{.
  cdecl, importc: "cSpaceHashPtrAlloc", dynlib: Lib.}
#/ Initialize a spatial hash. 
proc SpaceHashInit*(hash: SpaceHashPtr; celldim: CpFloat; numcells: cint; 
                    bbfunc: SpatialIndexBBFunc; staticIndex: SpatialIndexPtr): SpatialIndexPtr{.
  cdecl, importc: "cSpaceHashPtrInit", dynlib: Lib.}
#/ Allocate and initialize a spatial hash.
proc SpaceHashNew*(celldim: CpFloat; cells: cint; bbfunc: SpatialIndexBBFunc; 
                   staticIndex: SpatialIndexPtr): SpatialIndexPtr{.
  cdecl, importc: "cSpaceHashPtrNew", dynlib: Lib.}
#/ Change the cell dimensions and table size of the spatial hash to tune it.
#/ The cell dimensions should roughly match the average size of your objects
#/ and the table size should be ~10 larger than the number of objects inserted.
#/ Some trial and error is required to find the optimum numbers for efficiency.
proc SpaceHashResize*(hash: SpaceHashPtr; celldim: CpFloat; numcells: cint){.
  cdecl, importc: "cSpaceHashPtrResize", dynlib: Lib.}
#MARK: AABB Tree


#/ Allocate a bounding box tree.
proc BBTreeAlloc*(): BBTreePtr{.cdecl, importc: "cBBTreePtrAlloc", dynlib: Lib.}
#/ Initialize a bounding box tree.
proc BBTreeInit*(tree: BBTreePtr; bbfunc: SpatialIndexBBFunc; 
                 staticIndex: ptr SpatialIndex): ptr SpatialIndex{.cdecl, 
    importc: "cBBTreePtrInit", dynlib: Lib.}
#/ Allocate and initialize a bounding box tree.
proc BBTreeNew*(bbfunc: SpatialIndexBBFunc; staticIndex: SpatialIndexPtr): SpatialIndexPtr{.
    cdecl, importc: "cBBTreePtrNew", dynlib: Lib.}
#/ Perform a static top down optimization of the tree.
proc BBTreeOptimize*(index: SpatialIndexPtr){.
  cdecl, importc: "cBBTreePtrOptimize", dynlib: Lib.}
#/ Set the velocity function for the bounding box tree to enable temporal coherence.

proc BBTreeSetVelocityFunc*(index: SpatialIndexPtr; fun: BBTreeVelocityFunc){.
    cdecl, importc: "cBBTreePtrSetVelocityFunc", dynlib: Lib.}
#MARK: Single Axis Sweep


#/ Allocate a 1D sort and sweep broadphase.

proc Sweep1DAlloc*(): ptr Sweep1D{.cdecl, importc: "cSweep1DPtrAlloc", 
                                    dynlib: Lib.}
#/ Initialize a 1D sort and sweep broadphase.

proc Sweep1DInit*(sweep: ptr Sweep1D; bbfunc: SpatialIndexBBFunc; 
                  staticIndex: ptr SpatialIndex): ptr SpatialIndex{.cdecl, 
    importc: "cSweep1DPtrInit", dynlib: Lib.}
#/ Allocate and initialize a 1D sort and sweep broadphase.

proc Sweep1DNew*(bbfunc: SpatialIndexBBFunc; staticIndex: ptr SpatialIndex): ptr SpatialIndex{.
    cdecl, importc: "cSweep1DPtrNew", dynlib: Lib.}



defProp(ArbiterPtr, CpFloat, e, Elasticity)
defProp(ArbiterPtr, CpFloat, u, Friction)
defProp(ArbiterPtr, Vector, surface_vr, SurfaceVelocity)

#/ Calculate the total impulse that was applied by this 
#/ This function should only be called from a post-solve, post-step or cpBodyEachArbiter callback.
proc totalImpulse*(obj: ArbiterPtr): Vector {.cdecl, importc: "cArbiterPtrTotalImpulse", dynlib: Lib.}

#/ Calculate the total impulse including the friction that was applied by this arbiter.
#/ This function should only be called from a post-solve, post-step or cpBodyEachArbiter callback.
proc totalImpulseWithFriction*(obj: ArbiterPtr): Vector {.cdecl, importc: "cArbiterPtrTotalImpulseWithFriction", dynlib: Lib.}

#/ Calculate the amount of energy lost in a collision including static, but not dynamic friction.
#/ This function should only be called from a post-solve, post-step or cpBodyEachArbiter callback.
proc totalKE*(obj: ArbiterPtr): CpFloat {.cdecl, importc: "cArbiterPtrTotalKE", dynlib: Lib.}


#/ Causes a collision pair to be ignored as if you returned false from a begin callback.
#/ If called from a pre-step callback, you will still need to return false
#/ if you want it to be ignored in the current step.
proc ignore*(arb: ArbiterPtr) {.cdecl, importc: "cArbiterPtrIgnore", dynlib: Lib.}

#/ Return the colliding shapes involved for this arbiter.
#/ The order of their cpSpace.collision_type values will match
#/ the order set when the collision handler was registered.
proc getShapes*(arb: ArbiterPtr, a, b: var ShapePtr) {.inline.} =
  if arb.swappedColl.bool:
    a = arb.b
    b = arb.a
  else:
    a = arb.a
    b = arb.b

#/ A macro shortcut for defining and retrieving the shapes from an arbiter.
#define CP_ARBITER_GET_SHAPES(arb, a, b) cShapePtr *a, *b; cArbiterPtrGetShapes(arb, &a, &b);
template getShapes*(arb: ArbiterPtr, name1, name2: expr): stmt {.immediate.} =
  var name1, name2: ShapePtr
  getShapes(arb, name1, name2)


#/ Return the colliding bodies involved for this arbiter.
#/ The order of the cpSpace.collision_type the bodies are associated with values will match
#/ the order set when the collision handler was registered.
#proc getBodies*(arb: ArbiterPtr, a, b: var BodyPtr) {.inline.} = 
#  getShapes(arb, shape1, shape2)
#  a = shape1.body
#  b = shape2.body

#/ A macro shortcut for defining and retrieving the bodies from an arbiter.
#define CP_ARBITER_GET_BODIES(arb, a, b) cpBody *a, *b; cArbiterPtrGetBodies(arb, &a, &b);
template getBodies*(arb: ArbiterPtr, name1, name2: expr): stmt {.immediate.} =
  var name1, name2: PBOdy
  getBodies(arb, name1, name2)

proc isFirsContact*(arb: ArbiterPtr): bool {.inline.} =
  result = arb.state == ArbiterStateFirstColl

proc getCount*(arb: ArbiterPtr): cint {.inline.} =
  result = arb.numContacts

#/ Return a contact set from an arbiter.
proc geContactPointSet*(arb: ArbiterPtr): ContactPointSet {.
  cdecl, importc: "cArbiterPtrGeContactPointSet", dynlib: Lib.}
#/ Get the normal of the @c ith contact point.
proc getNormal*(arb: ArbiterPtr; i: cint): Vector {.
  cdecl, importc: "cArbiterPtrGetNormal", dynlib: Lib.}
#/ Get the position of the @c ith contact point.
proc getPoint*(arb: ArbiterPtr; i: cint): Vector {.
  cdecl, importc: "cArbiterPtrGetPoint", dynlib: Lib.}
#/ Get the depth of the @c ith contact point.
proc getDepth*(arb: ArbiterPtr; i: cint): CpFloat {.
  cdecl, importc: "cArbiterPtrGetDepth", dynlib: Lib.}

##Shapes
template defShapeSetter(memberType: typedesc, memberName: expr, procName: expr, activates: bool): stmt {.immediate.} =
  proc `set procName`*(obj: ShapePtr, value: memberType) {.cdecl.} =
    if activates and obj.body != nil: obj.body.activate()
    obj.memberName = value
template defShapeProp(memberType: typedesc, memberName: expr, procName: expr, activates: bool): stmt {.immediate.} =
  defGetter(ShapePtr, memberType, memberName, procName)
  defShapeSetter(memberType, memberName, procName, activates)

#/ Destroy a shape.
proc destroy*(shape: ShapePtr) {.
  cdecl, importc: "cShapePtrDestroy", dynlib: Lib.}
#/ Destroy and Free a shape.
proc free*(shape: ShapePtr){.
  cdecl, importc: "cShapePtrFree", dynlib: Lib.}
#/ Update, cache and return the bounding box of a shape based on the body it's attached to.
proc cacheBB*(shape: ShapePtr): BB{.
  cdecl, importc: "cShapePtrCacheBB", dynlib: Lib.}
#/ Update, cache and return the bounding box of a shape with an explicit transformation.
proc update*(shape: ShapePtr; pos: Vector; rot: Vector): BB {.
  cdecl, importc: "cShapePtrUpdate", dynlib: Lib.}
#/ Test if a point lies within a shape.
proc pointQuery*(shape: ShapePtr; p: Vector): Bool32 {.
  cdecl, importc: "cShapePtrPointQuery", dynlib: Lib.}

#/ Perform a nearest point query. It finds the closest point on the surface of shape to a specific point.
#/ The value returned is the distance between the points. A negative distance means the point is inside the shape.
proc nearestPointQuery*(shape: ShapePtr; p: Vector; res: NearestPointQueryInfoPtr): CpFloat {.
  cdecl, importc: "cShapePtrNearestPointQuery", dynlib: Lib.}
#/ Perform a segment query against a shape. @c info must be a pointer to a valid cSegmentQueryInfoPtr structure.
proc segmentQuery*(shape: ShapePtr, a, b: Vector, info: SegmentQueryInfoPtr): bool {.
  cdecl, importc: "cShapePtrSegmentQuery", dynlib: Lib.}

#/ Get the hit point for a segment query.
## Possibly change; info to SegmentQueryInfoPtr 
proc queryHitPoint*(start, to: Vector, info: SegmentQueryInfo): Vector {.inline.} =
  result = start.lerp(to, info.t)

#/ Get the hit distance for a segment query.
proc queryHitDist*(start, to: Vector, info: SegmentQueryInfo): CpFloat {.inline.} =
  result = start.dist(to) * info.t

defGetter(ShapePtr, SpacePtr, space, Space)

defGetter(ShapePtr, BodyPtr, body, Body)
proc seBody*(shape: ShapePtr, value: BodyPtr) {.
  cdecl, importc: "cShapePtrSeBody", dynlib: Lib.}


defGetter(ShapePtr, BB, bb, BB)
#defShapeProp(Bool32, sensor, Sensor, true)
proc setSensor*(obj: ShapePtr, value: bool) {.cdecl.} =
  if obj.body != nil: obj.body.activate()
  obj.sensor = value.Bool32
proc getSensor*(obj: ShapePtr): bool {.cdecl.} = return obj.sensor.bool

defShapeProp(CpFloat, e, Elasticity, false)
defShapeProp(CpFloat, u, Friction, true)
defShapeProp(Vector, surface_v, SurfaceVelocity, true)
defShapeProp(pointer, data, UserData, false)
defShapeProp(CollisionType, collision_type, CollisionType, true)
defShapeProp(Group, group, Group, true)
defShapeProp(Layers, layers, Layers, true)

#/ When initializing a shape, it's hash value comes from a counter.
#/ Because the hash value may affect iteration order, you can reset the shape ID counter
#/ when recreating a space. This will make the simulation be deterministic.
proc resetShapeIdCounter*(): void {.cdecl, importc: "cpResetShapeIdCounter", dynlib: Lib.}
#/ Allocate a circle shape.
proc CircleShapeAlloc*(): CircleShapePtr {.cdecl, importc: "cpCircleShapeAlloc", dynlib: Lib.}
#/ Initialize a circle shape.
proc init*(circle: CircleShapePtr, body: BodyPtr, radius: CpFloat, offset: Vector): CircleShapePtr {.
  cdecl, importc: "cpCircleShapeInit", dynlib: Lib.}
#/ Allocate and initialize a circle shape.
proc newCircleShape*(body: BodyPtr, radius: CpFloat, offset: Vector): ShapePtr {.
  cdecl, importc: "cpCircleShapeNew", dynlib: Lib.}

proc getCircleOffset*(shape: ShapePtr): Vector {.
  cdecl, importc: "cpCircleShapeGetOffset", dynlib: Lib.}
proc getCircleRadius*(shape: ShapePtr): CpFloat {.
  cdecl, importc: "cpCircleShapeGetRadius", dynlib: Lib.}


#/ Allocate a polygon shape.
proc allocPolyShape*(): PolyShapePtr {.
  cdecl, importc: "cpPolyShapeAlloc", dynlib: Lib.}
#/ Initialize a polygon shape.
#/ A convex hull will be created from the vertexes.
proc init*(poly: PolyShapePtr; body: BodyPtr, numVerts: cint;
            verts: ptr Vector; offset: Vector): PolyShapePtr {.
  cdecl, importc: "cpPolyShapeInit", dynlib: Lib.}
#/ Allocate and initialize a polygon shape.
#/ A convex hull will be created from the vertexes.
proc newPolyShape*(body: BodyPtr; numVerts: cint; verts: ptr Vector; 
                    offset: Vector): ShapePtr {.
  cdecl, importc: "cpPolyShapeNew", dynlib: Lib.}
#/ Initialize a box shaped polygon shape.
proc init*(poly: PolyShapePtr; body: BodyPtr; width, height: CpFloat): PolyShapePtr {.
  cdecl, importc: "cpBoxShapeInit", dynlib: Lib.}
#/ Initialize an offset box shaped polygon shape.
proc init*(poly: PolyShapePtr; body: BodyPtr; box: BB): PolyShapePtr {.
  cdecl, importc: "cpBoxShapeInit2", dynlib: Lib.}
#/ Allocate and initialize a box shaped polygon shape.
proc newBoxShape*(body: BodyPtr; width, height: CpFloat): ShapePtr {.
  cdecl, importc: "cpBoxShapeNew", dynlib: Lib.}
#/ Allocate and initialize an offset box shaped polygon shape.
proc newBoxShape*(body: BodyPtr; box: BB): ShapePtr {.
  cdecl, importc: "cpBoxShapeNew2", dynlib: Lib.}

#/ Check that a set of vertexes is convex and has a clockwise winding.
#/ NOTE: Due to floating point precision issues, hulls created with cpQuickHull() are not guaranteed to validate!
proc validatePoly*(verts: ptr Vector; numVerts: cint): bool {.
  cdecl, importc: "cpPolyValidate", dynlib: Lib.}
#/ Get the number of verts in a polygon shape.
proc getNumVerts*(shape: ShapePtr): cint {.
  cdecl, importc: "cpPolyShapeGetNumVerts", dynlib: Lib.}
#/ Get the @c ith vertex of a polygon shape.
proc getVert*(shape: ShapePtr; index: cint): Vector {.
  cdecl, importc: "cpPolyShapeGetVert", dynlib: Lib.}

#/ Allocate a segment shape.
proc allocSegmentShape*(): SegmentShapePtr {.
  cdecl, importc: "cpSegmentShapeAlloc", dynlib: Lib.}
#/ Initialize a segment shape.
proc init*(seg: SegmentShapePtr, body: BodyPtr, a, b: Vector, radius: CpFloat): SegmentShapePtr {.
  cdecl, importc: "cpSegmentShapeInit", dynlib: Lib.}
#/ Allocate and initialize a segment shape.
proc newSegmentShape*(body: BodyPtr, a, b: Vector, radius: CpFloat): ShapePtr {.
  cdecl, importc: "cpSegmentShapeNew", dynlib: Lib.}

proc setSegmentNeighbors*(shape: ShapePtr, prev, next: Vector) {.
  cdecl, importc: "cpSegmentShapeSetNeighbors", dynlib: Lib.}
proc getSegmentA*(shape: ShapePtr): Vector {.
  cdecl, importc: "cpSegmentShapeGetA", dynlib: Lib.}
proc getSegmentB*(shape: ShapePtr): Vector {.
  cdecl, importc: "cpSegmentShapeGetB", dynlib: Lib.}
proc getSegmentNormal*(shape: ShapePtr): Vector {.
  cdecl, importc: "cpSegmentShapeGetNormal", dynlib: Lib.}
proc getSegmentRadius*(shape: ShapePtr): CpFloat {.
  cdecl, importc: "cpSegmentShapeGetRadius", dynlib: Lib.}


#/ Version string.
#var VersionString*{.importc: "cpVersionString", dynlib: Lib.}: cstring
#/ Calculate the moment of inertia for a circle.
#/ @c r1 and @c r2 are the inner and outer diameters. A solid circle has an inner diameter of 0.
when defined(MoreNim):
  proc momentForCircle*(m, r1, r2: CpFloat; offset: Vector): CpFloat {.cdecl.} =
    result = m * (0.5 * (r1 * r1 + r2 * r2) + lenSq(offset))
else:
  proc momentForCircle*(m, r1, r2: CpFloat; offset: Vector): CpFloat {.
    cdecl, importc: "cpMomentForCircle", dynlib: Lib.}

#/ Calculate area of a hollow circle.
#/ @c r1 and @c r2 are the inner and outer diameters. A solid circle has an inner diameter of 0.
proc AreaForCircle*(r1: CpFloat; r2: CpFloat): CpFloat {.
  cdecl, importc: "cpAreaForCircle", dynlib: Lib.}
#/ Calculate the moment of inertia for a line segment.
#/ Beveling radius is not supported.
proc MomentForSegment*(m: CpFloat; a, b: Vector): CpFloat {.
  cdecl, importc: "cpMomentForSegment", dynlib: Lib.}
#/ Calculate the area of a fattened (capsule shaped) line segment.
proc AreaForSegment*(a, b: Vector; r: CpFloat): CpFloat {.
  cdecl, importc: "cpAreaForSegment", dynlib: Lib.}
#/ Calculate the moment of inertia for a solid polygon shape assuming it's center of gravity is at it's centroid. The offset is added to each vertex.
proc MomentForPoly*(m: CpFloat; numVerts: cint; verts: ptr Vector; offset: Vector): CpFloat {.
  cdecl, importc: "cpMomentForPoly", dynlib: Lib.}
#/ Calculate the signed area of a polygon. A Clockwise winding gives positive area.
#/ This is probably backwards from what you expect, but matches Chipmunk's the winding for poly shapes.
proc AreaForPoly*(numVerts: cint; verts: ptr Vector): CpFloat {.
  cdecl, importc: "cpAreaForPoly", dynlib: Lib.}
#/ Calculate the natural centroid of a polygon.
proc CentroidForPoly*(numVerts: cint; verts: ptr Vector): Vector {.
  cdecl, importc: "cpCentroidForPoly", dynlib: Lib.}
#/ Center the polygon on the origin. (Subtracts the centroid of the polygon from each vertex)
proc RecenterPoly*(numVerts: cint; verts: ptr Vector) {.
  cdecl, importc: "cpRecenterPoly", dynlib: Lib.}
#/ Calculate the moment of inertia for a solid box.
proc MomentForBox*(m, width, height: CpFloat): CpFloat {.
  cdecl, importc: "cpMomentForBox", dynlib: Lib.}
#/ Calculate the moment of inertia for a solid box.
proc MomentForBox2*(m: CpFloat; box: BB): CpFloat {.
  cdecl, importc: "cpMomentForBox2", dynlib: Lib.}



##constraints
type 
  #TODO: all these are private
  #TODO: defConstraintProp()
  PinJointPtr = ptr PinJoint
  PinJoint{.pf.} = object 
    constraint: ConstraintPtr
    anchr1: Vector
    anchr2: Vector
    dist: CpFloat
    r1: Vector
    r2: Vector
    n: Vector
    nMass: CpFloat
    jnAcc: CpFloat
    jnMax: CpFloat
    bias: CpFloat
  SlideJointPtr = ptr SlideJoint
  SlideJoint{.pf.} = object 
    constraint: ConstraintPtr
    anchr1: Vector
    anchr2: Vector
    min: CpFloat
    max: CpFloat
    r1: Vector
    r2: Vector
    n: Vector
    nMass: CpFloat
    jnAcc: CpFloat
    jnMax: CpFloat
    bias: CpFloat
  PivotJointPtr = ptr PivotJoint
  PivotJoint{.pf.} = object 
    constraint: ConstraintPtr
    anchr1: Vector
    anchr2: Vector
    r1: Vector
    r2: Vector
    k1: Vector
    k2: Vector
    jAcc: Vector
    jMaxLen: CpFloat
    bias: Vector
  GrooveJointPtr = ptr GrooveJoint
  GrooveJoint{.pf.} = object 
    constraint: ConstraintPtr
    grv_n: Vector
    grv_a: Vector
    grv_b: Vector
    anchr2: Vector
    grv_tn: Vector
    clamp: CpFloat
    r1: Vector
    r2: Vector
    k1: Vector
    k2: Vector
    jAcc: Vector
    jMaxLen: CpFloat
    bias: Vector
  DampedSpringPtr = ptr DampedSpring
  DampedSpring{.pf.} = object 
    constraint: ConstraintPtr
    anchr1: Vector
    anchr2: Vector
    restLength: CpFloat
    stiffness: CpFloat
    damping: CpFloat
    springForceFunc: DampedSpringForceFunc
    target_vrn: CpFloat
    v_coef: CpFloat
    r1: Vector
    r2: Vector
    nMass: CpFloat
    n: Vector
  DampedRotarySpringPtr = ptr DampedRotarySpring
  DampedRotarySpring{.pf.} = object 
    constraint: ConstraintPtr
    restAngle: CpFloat
    stiffness: CpFloat
    damping: CpFloat
    springTorqueFunc: DampedRotarySpringTorqueFunc
    target_wrn: CpFloat
    w_coef: CpFloat
    iSum: CpFloat
  RotaryLimitJointPtr = ptr RotaryLimitJoint
  RotaryLimitJoint{.pf.} = object 
    constraint: ConstraintPtr
    min: CpFloat
    max: CpFloat
    iSum: CpFloat
    bias: CpFloat
    jAcc: CpFloat
    jMax: CpFloat
  RatchetJointPtr = ptr RatchetJoint
  RatchetJoint{.pf.} = object 
    constraint: ConstraintPtr
    angle: CpFloat
    phase: CpFloat
    ratchet: CpFloat
    iSum: CpFloat
    bias: CpFloat
    jAcc: CpFloat
    jMax: CpFloat
  GearJointPtr = ptr GearJoint
  GearJoint{.pf.} = object 
    constraint: ConstraintPtr
    phase: CpFloat
    ratio: CpFloat
    ratio_inv: CpFloat
    iSum: CpFloat
    bias: CpFloat
    jAcc: CpFloat
    jMax: CpFloat
  SimpleMotorPtr = ptr SimpleMotor
  SimpleMotor{.pf.} = object 
    constraint: ConstraintPtr
    rate: CpFloat
    iSum: CpFloat
    jAcc: CpFloat
    jMax: CpFloat
  DampedSpringForceFunc* = proc (spring: ConstraintPtr; dist: CpFloat): CpFloat{.
    cdecl.}
  DampedRotarySpringTorqueFunc* = proc (spring: ConstraintPtr; 
      relativeAngle: CpFloat): CpFloat {.cdecl.}
#/ Destroy a constraint.
proc destroy*(constraint: ConstraintPtr){.
  cdecl, importc: "cConstraintPtrDestroy", dynlib: Lib.}
#/ Destroy and free a constraint.111
proc free*(constraint: ConstraintPtr){.
  cdecl, importc: "cConstraintPtrFree", dynlib: Lib.}

#/ @private
proc activateBodies(constraint: ConstraintPtr) {.inline.} = 
  if not constraint.a.isNil: constraint.a.activate()
  if not constraint.b.isNil: constraint.b.activate()

# /// @private
# #define CP_DefineConstraintStructGetter(type, member, name) \
# static inline type cpConstraint##Get##name(const cpConstraint *constraint){return constraint->member;}
# /// @private
# #define CP_DefineConstraintStructSetter(type, member, name) \
# static inline void cpConstraint##Set##name(cpConstraint *constraint, type value){ \
# 	cpConstraintActivateBodies(constraint); \
# 	constraint->member = value; \
# }
template defConstraintSetter(memberType: typedesc, member: expr, name: expr): stmt {.immediate.} =
  proc `set name`*(constraint: ConstraintPtr, value: memberType) {.cdecl.} =
    activateBodies(constraint)
    constraint.member = value
template defConstraintProp(memberType: typedesc, member: expr, name: expr): stmt {.immediate.} =
  defGetter(ConstraintPtr, memberType, member, name)
  defConstraintSetter(memberType, member, name)
# CP_DefineConstraintStructGetter(cpSpace*, CP_PRIVATE(space), Space)
defGetter(ConstraintPtr, SpacePtr, space, Space)
defGetter(ConstraintPtr, BodyPtr, a, A)
defGetter(ConstraintPtr, BodyPtr, a, B)
defGetter(ConstraintPtr, CpFloat, maxForce, MaxForce)
defGetter(ConstraintPtr, CpFloat, errorBias, ErrorBias)
defGetter(ConstraintPtr, CpFloat, maxBias, MaxBias)
defGetter(ConstraintPtr, ConstraintPreSolveFunc, preSolve, PreSolveFunc)
defGetter(ConstraintPtr, ConstraintPostSolveFunc, postSolve, PostSolveFunc)
defGetter(ConstraintPtr, CpDataPointer, data, UserData)
# Get the last impulse applied by this constraint.
proc getImpulse*(constraint: ConstraintPtr): CpFloat {.inline.} = 
  return constraint.klass.getImpulse(constraint)

# #define cpConstraintCheckCast(constraint, struct) \
# 	cpAssertHard(constraint->CP_PRIVATE(klass) == struct##GetClass(), "Constraint is not a "#struct)
# #define CP_DefineConstraintGetter(struct, type, member, name) \
# static inline type struct##Get##name(const cpConstraint *constraint){ \
# 	cpConstraintCheckCast(constraint, struct); \
# 	return ((struct *)constraint)->member; \
# }
# #define CP_DefineConstraintSetter(struct, type, member, name) \
# static inline void struct##Set##name(cpConstraint *constraint, type value){ \
# 	cpConstraintCheckCast(constraint, struct); \
# 	cpConstraintActivateBodies(constraint); \
# 	((struct *)constraint)->member = value; \
# }
template constraintCheckCast(constraint: ConstraintPtr, ctype: expr): stmt {.immediate.} =
  assert(constraint.klass == `ctype GetClass`(), "Constraint is the wrong class")
template defCGetter(ctype: expr, memberType: typedesc, member: expr, name: expr): stmt {.immediate.} = 
  proc `get ctype name`*(constraint: ConstraintPtr): memberType {.cdecl.} =
    constraintCheckCast(constraint, ctype)
    result = cast[`ctype Ptr`](constraint).member
template defCSetter(ctype: expr, memberType: typedesc, member: expr, name: expr): stmt {.immediate.} =
  proc `set ctype name`*(constraint: ConstraintPtr, value: memberType) {.cdecl.} =
    constraintCheckCast(constraint, ctype)
    activateBodies(constraint)
    cast[`ctype Ptr`](constraint).member = value
template defCProp(ctype: expr, memberType: typedesc, member: expr, name: expr): stmt {.immediate.} =
  defCGetter(ctype, memberType, member, name)
  defCSetter(ctype, memberType, member, name)

proc PinJointGetClass*(): ConstraintPtrClass{.
  cdecl, importc: "cpPinJointGetClass", dynlib: Lib.}
#/ @private

#/ Allocate a pin joint.
proc AllocPinJoint*(): PinJointPtr {.cdecl, importc: "cpPinJointAlloc", dynlib: Lib.}
#/ Initialize a pin joint.
proc PinJointInit*(joint: PinJointPtr; a: BodyPtr; b: BodyPtr; anchr1: Vector; 
                   anchr2: Vector): PinJointPtr{.
  cdecl, importc: "cpPinJointInit", dynlib: Lib.}
#/ Allocate and initialize a pin joint.
proc newPinJoint*(a: BodyPtr; b: BodyPtr; anchr1: Vector; anchr2: Vector): ConstraintPtr{.
  cdecl, importc: "cpPinJointNew", dynlib: Lib.}
# CP_DefineConstraintProperty(cpPinJoint, cpVect, anchr1, Anchr1)
defCProp(PinJoint, Vector, anchr1, Anchr1)
defCProp(PinJoint, Vector, anchr2, Anchr2)
defCProp(PinJoint, CpFloat, dist, Dist)

proc SlideJointGetClass*(): ConstraintPtrClass{.
  cdecl, importc: "cpSlideJointGetClass", dynlib: Lib.}
#/ Allocate a slide joint.
proc AllocSlideJoint*(): SlideJointPtr{.
  cdecl, importc: "cpSlideJointAlloc", dynlib: Lib.}
#/ Initialize a slide joint.
proc init*(joint: SlideJointPtr; a, b: BodyPtr; anchr1, anchr2: Vector;
            min, max: CpFloat): SlideJointPtr{.
  cdecl, importc: "cpSlideJointInit", dynlib: Lib.}
#/ Allocate and initialize a slide joint.
proc newSlideJoint*(a, b: BodyPtr; anchr1, anchr2: Vector; min, max: CpFloat): ConstraintPtr{.
  cdecl, importc: "cpSlideJointNew", dynlib: Lib.}

defCProp(SlideJoint, Vector, anchr1, Anchr1)
defCProp(SlideJoint, Vector, anchr2, Anchr2)
defCProp(SlideJoint, CpFloat, min, Min)
defCProp(SlideJoint, CpFloat, max, Max)

proc PivotJointGetClass*(): ConstraintPtrClass {.
  cdecl, importc: "cpPivotJointGetClass", dynlib: Lib.}

#/ Allocate a pivot joint
proc allocPivotJoint*(): PivotJointPtr{.
  cdecl, importc: "cpPivotJointAlloc", dynlib: Lib.}
#/ Initialize a pivot joint.
proc init*(joint: PivotJointPtr; a, b: BodyPtr; anchr1, anchr2: Vector): PivotJointPtr{.
  cdecl, importc: "cpPivotJointInit", dynlib: Lib.}
#/ Allocate and initialize a pivot joint.
proc newPivotJoint*(a, b: BodyPtr; pivot: Vector): ConstraintPtr{.
  cdecl, importc: "cpPivotJointNew", dynlib: Lib.}
#/ Allocate and initialize a pivot joint with specific anchors.
proc newPivotJoint*(a, b: BodyPtr; anchr1, anchr2: Vector): ConstraintPtr{.
  cdecl, importc: "cpPivotJointNew2", dynlib: Lib.}

defCProp(PivotJoint, Vector, anchr1, Anchr1)
defCProp(PivotJoint, Vector, anchr2, Anchr2)


proc GrooveJointGetClass*(): ConstraintPtrClass{.
  cdecl, importc: "cpGrooveJointGetClass", dynlib: Lib.}
#/ Allocate a groove joint.
proc GrooveJointAlloc*(): ptr GrooveJoint{.
  cdecl, importc: "cpGrooveJointAlloc", dynlib: Lib.}
#/ Initialize a groove joint.
proc Init*(joint: GrooveJointPtr; a, b: BodyPtr; groove_a, groove_b, anchr2: Vector): GrooveJointPtr{.
  cdecl, importc: "cpGrooveJointInit", dynlib: Lib.}
#/ Allocate and initialize a groove joint.
proc newGrooveJoint*(a, b: BodyPtr; groove_a, groove_b, anchr2: Vector): ConstraintPtr{.
  cdecl, importc: "cpGrooveJointNew", dynlib: Lib.}

defCGetter(GrooveJoint, Vector, grv_a, GrooveA)
defCGetter(GrooveJoint, Vector, grv_b, GrooveB)
# /// Set endpoint a of a groove joint's groove
proc SetGrooveA*(constraint: ConstraintPtr, value: Vector) {.
  cdecl, importc: "cpGrooveJointSetGrooveA", dynlib: Lib.}
# /// Set endpoint b of a groove joint's groove
proc SetGrooveB*(constraint: ConstraintPtr, value: Vector) {.
  cdecl, importc: "cpGrooveJointSetGrooveB", dynlib: Lib.}
defCProp(GrooveJoint, Vector, anchr2, Anchr2)

proc DampedSpringGetClass*(): ConstraintPtrClass{.
  cdecl, importc: "cpDampedSpringGetClass", dynlib: Lib.}
#/ Allocate a damped spring.
proc AllocDampedSpring*(): DampedSpringPtr{.
  cdecl, importc: "cpDampedSpringAlloc", dynlib: Lib.}
#/ Initialize a damped spring.
proc init*(joint: DampedSpringPtr; a, b: BodyPtr; anchr1, anchr2: Vector;
            restLength, stiffness, damping: CpFloat): DampedSpringPtr{.
  cdecl, importc: "cpDampedSpringInit", dynlib: Lib.}
#/ Allocate and initialize a damped spring.
proc newDampedSpring*(a, b: BodyPtr; anchr1, anchr2: Vector; 
                      restLength, stiffness, damping: CpFloat): ConstraintPtr{.
  cdecl, importc: "cpDampedSpringNew", dynlib: Lib.}

# CP_DefineConstraintProperty(cpDampedSpring, cpVect, anchr1, Anchr1)
defCProp(DampedSpring, Vector, anchr1, Anchr1)
defCProp(DampedSpring, Vector, anchr2, Anchr2)
defCProp(DampedSpring, CpFloat, restLength, RestLength)
defCProp(DampedSpring, CpFloat, stiffness, Stiffness)
defCProp(DampedSpring, CpFloat, damping, Damping)
defCProp(DampedSpring, DampedSpringForceFunc, springForceFunc, SpringForceFunc)


proc DampedRotarySpringGetClass*(): ConstraintPtrClass{.
  cdecl, importc: "cpDampedRotarySpringGetClass", dynlib: Lib.}

#/ Allocate a damped rotary spring.
proc DampedRotarySpringAlloc*(): DampedRotarySpringPtr{.
  cdecl, importc: "cpDampedRotarySpringAlloc", dynlib: Lib.}
#/ Initialize a damped rotary spring.
proc init*(joint: DampedRotarySpringPtr; a, b: BodyPtr; 
            restAngle, stiffness, damping: CpFloat): DampedRotarySpringPtr{.
  cdecl, importc: "cpDampedRotarySpringInit", dynlib: Lib.}
#/ Allocate and initialize a damped rotary spring.
proc DampedRotarySpringNew*(a, b: BodyPtr; restAngle, stiffness, damping: CpFloat): ConstraintPtr{.
  cdecl, importc: "cpDampedRotarySpringNew", dynlib: Lib.}

defCProp(DampedRotarySpring, CpFloat, restAngle, RestAngle)
defCProp(DampedRotarySpring, CpFloat, stiffness, Stiffness)
defCProp(DampedRotarySpring, CpFloat, damping, Damping)
defCProp(DampedRotarySpring, DampedRotarySpringTorqueFunc, springTorqueFunc, SpringTorqueFunc)


proc RotaryLimitJointGetClass*(): ConstraintPtrClass{.
  cdecl, importc: "cpRotaryLimitJointGetClass", dynlib: Lib.}
#/ Allocate a damped rotary limit joint.
proc allocRotaryLimitJoint*(): RotaryLimitJointPtr{.
  cdecl, importc: "cpRotaryLimitJointAlloc", dynlib: Lib.}
#/ Initialize a damped rotary limit joint.
proc init*(joint: RotaryLimitJointPtr; a, b: BodyPtr; min, max: CpFloat): RotaryLimitJointPtr{.
  cdecl, importc: "cpRotaryLimitJointInit", dynlib: Lib.}
#/ Allocate and initialize a damped rotary limit joint.
proc newRotaryLimitJoint*(a, b: BodyPtr; min, max: CpFloat): ConstraintPtr{.
  cdecl, importc: "cpRotaryLimitJointNew", dynlib: Lib.}

defCProp(RotaryLimitJoint, CpFloat, min, Min)
defCProp(RotaryLimitJoint, CpFloat, max, Max)


proc RatchetJointGetClass*(): ConstraintPtrClass{.
  cdecl, importc: "cpRatchetJointGetClass", dynlib: Lib.}
#/ Allocate a ratchet joint.
proc AllocRatchetJoint*(): RatchetJointPtr{.
  cdecl, importc: "cpRatchetJointAlloc", dynlib: Lib.}
#/ Initialize a ratched joint.
proc init*(joint: RatchetJointPtr; a, b: BodyPtr; phase, ratchet: CpFloat): RatchetJointPtr{.
  cdecl, importc: "cpRatchetJointInit", dynlib: Lib.}
#/ Allocate and initialize a ratchet joint.
proc NewRatchetJoint*(a, b: BodyPtr; phase, ratchet: CpFloat): ConstraintPtr{.
  cdecl, importc: "cpRatchetJointNew", dynlib: Lib.}

defCProp(RatchetJoint, CpFloat, angle, Angle)
defCProp(RatchetJoint, CpFloat, phase, Phase)
defCProp(RatchetJoint, CpFloat, ratchet, Ratchet)


proc GearJointGetClass*(): ConstraintPtrClass{.cdecl, 
    importc: "cpGearJointGetClass", dynlib: Lib.}
#/ Allocate a gear joint.
proc AllocGearJoint*(): GearJointPtr{.
  cdecl, importc: "cpGearJointAlloc", dynlib: Lib.}
#/ Initialize a gear joint.
proc init*(joint: GearJointPtr; a, b: BodyPtr, phase, ratio: CpFloat): GearJointPtr{.
  cdecl, importc: "cpGearJointInit", dynlib: Lib.}
#/ Allocate and initialize a gear joint.
proc NewGearJoint*(a, b: BodyPtr; phase, ratio: CpFloat): ConstraintPtr{.
  cdecl, importc: "cpGearJointNew", dynlib: Lib.}

defCProp(GearJoint, CpFloat, phase, Phase)
defCGetter(GearJoint, CpFloat, ratio, Ratio)
#/ Set the ratio of a gear joint.
proc GearJointSetRatio*(constraint: ConstraintPtr; value: CpFloat){.
  cdecl, importc: "cpGearJointSetRatio", dynlib: Lib.}


proc SimpleMotorGetClass*(): ConstraintPtrClass{.
  cdecl, importc: "cpSimpleMotorGetClass", dynlib: Lib.}
#/ Allocate a simple motor.
proc AllocSimpleMotor*(): SimpleMotorPtr{.
  cdecl, importc: "cpSimpleMotorAlloc", dynlib: Lib.}
#/ initialize a simple motor.
proc init*(joint: SimpleMotorPtr; a, b: BodyPtr; 
                      rate: CpFloat): SimpleMotorPtr{.
  cdecl, importc: "cpSimpleMotorInit", dynlib: Lib.}
#/ Allocate and initialize a simple motor.
proc newSimpleMotor*(a, b: BodyPtr; rate: CpFloat): ConstraintPtr{.
  cdecl, importc: "cpSimpleMotorNew", dynlib: Lib.}

defCProp(SimpleMotor, CpFloat, rate, Rate)



