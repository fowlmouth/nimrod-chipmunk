import sfml, sfml_colors, chipmunk, math
type
  ## TShapeData is created when the shape is initialized or added to the space
  ## field[0] is whatever userdata you pass to it in addShape()
  ## field[1] is the sfml sprite for the shape
  PShapeData* = ptr TShapeData
  TShapeData* = array[0..1, pointer] ##tuple[data: pointer; sprite: pointer]
let
  Colors: array[TShapeType, TColor] = [Green, Blue, Red, Yellow]

proc cp2sfml(vec: TVector): TVector2f =
  result.x = vec.x
  result.y = vec.y
proc floor(vec: TVector): TVector2f =
  result.x = vec.x.floor
  result.y = vec.y.floor
proc degrees(rad: float): float = rad * PI / 180.0

template WINDOW(a: pointer): PRenderWindow = cast[PRenderWindow](a)
template TOSPRITE(a: chipmunk.PShape, to: typedesc): expr =
  cast[to](cast[PShapeData](a.data)[1])
 
proc drawShape(shape: chipmunk.PShape, winda: pointer) {.cdecl.} =
  case shape.klass.kind
  of CP_CIRCLE_SHAPE:
    let 
      circ = TOSPRITE(shape, sfml.PCircleShape)
      body = shape.getBody()
    circ.setPosition body.getPos.floor()
    WINDOW(winda).draw circ
  of CP_SEGMENT_SHAPE:
    WINDOW(winda).draw TOSPRITE(shape, sfml.PVertexArray)
  of CP_POLY_SHAPE:
    let 
      poly = TOSPRITE(shape, sfml.PConvexShape)
      body = shape.getBody()
    poly.setPosition body.getPos.floor
    poly.setRotation body.getAngle.degrees
    WINDOW(winda).draw poly
  else:
    discard


proc initializeShape(shape: chipmunk.PShape; userData: pointer = nil) {.cdecl.} =
  if not shape.data.isNil:
    return
  
  var data = cast[PShapeData](alloc0(sizeof(TShapeData)))
  data[0] = userData
  
  case shape.klass.kind
  of CP_CIRCLE_SHAPE:
    var circ = sfml.newCircleShape(shape.getCircleRadius(), 30)
    let radius = shape.getCircleRadius()
    circ.setOrigin vec2f(radius, radius)
    circ.setFillColor colors[CP_CIRCLE_SHAPE]
    data[1] = circ
  of CP_SEGMENT_SHAPE:
    var seg = sfml.newVertexArray(sfml.Lines, 2)
    seg[0].position = shape.getSegmentA.cp2sfml()
    seg[1].position = shape.getSegmentB.cp2sfml()
    seg[0].color = colors[CP_SEGMENT_SHAPE]
    seg[1].color = colors[CP_SEGMENT_SHAPE]
    data[1] = seg
  of CP_POLY_SHAPE:
    var poly = sfml.newConvexShape(shape.getNumVerts())
    for i in 0.. <shape.getNumVerts():
      poly.setPoint i.cuint, shape.getVert(i).cp2sfml()
    poly.setFillColor colors[CP_POLY_SHAPE]
    data[1] = poly
  else: 
    echo "Unknown shape type! ", repr(shape.klass.kind)
    return
  
  shape.setUserData data

proc draw*(window: PRenderWindow; space: PSpace) {.inline.} =
  space.eachShape(drawShape, cast[pointer](window))

proc debugDrawInit*(space: PSpace) = 
  space.eachShape(initializeShape, nil)

proc addShape*(space: PSpace; shape: chipmunk.PShape; 
      userData: pointer = nil): chipmunk.PShape {.discardable.} =
  result = chipmunk.addShape(space, shape)
  initializeShape result, userData

proc removeShape*(space: PSpace; shape: chipmunk.PShape, deallocData = true) =
  chipmunk.removeShape space, shape
  if not shape.data.isNil:
    case shape.klass.kind
    of CP_SEGMENT_SHAPE: TOSPRITE(shape, PVertexArray).destroy()
    of CP_POLY_SHAPE: TOSPRITE(shape, sfml.PRectangleShape).destroy()
    of CP_CIRCLE_SHAPE: TOSPRITE(shape, sfml.PCircleShape).destroy()
    else: discard
    if deallocData:
      dealloc(shape.data)

proc getUserData*(shape: chipmunk.PShape): pointer {.inline.} = 
  return cast[PShapeData](shape.data)[0]
proc setUserData*(shape: chipmunk.PShape; data: pointer) {.inline.} = 
  cast[PShapeData](shape.data)[0] = data
