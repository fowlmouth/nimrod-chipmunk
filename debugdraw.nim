import csfml, chipmunk, math
type
  ## ShapeData is created when the shape is initialized or added to the space
  ## field[0] is whatever userdata you pass to it in addShape()
  ## field[1] is the sfml sprite for the shape
  ShapeDataPtr* = ptr ShapeData
  ShapeData* = array[0..1, pointer] ##tuple[data: pointer; sprite: pointer]
let
  colors: array[ShapeType, Color] = [Green, Blue, Red, Yellow]
  
proc cp2sfml(vec: Vector): Vector2f =
  result.x = vec.x
  result.y = vec.y

proc floor(vec: Vector): Vector2f =
  result.x = vec.x.floor
  result.y = vec.y.floor

proc degrees(rad: float): float = rad * PI / 180.0

template WINDOW(a: pointer): RenderWindow = cast[RenderWindow](a)
template TOSPRITE*(a: chipmunk.ShapePtr, to: typedesc): expr =
  cast[to](cast[ShapeDataPtr](a.data)[1])

proc drawShape(shape: chipmunk.ShapePtr, winda: pointer) {.cdecl.} =
  case shape.klass.kind
  of CP_CIRCLE_SHAPE:
    TOSPRITE(shape, csfml.CircleShape).position = shape.getBody().getPos.floor()
    WINDOW(winda).draw TOSPRITE(shape, csfml.CircleShape)
  of CP_SEGMENT_SHAPE:
    WINDOW(winda).draw TOSPRITE(shape, csfml.VertexArray)
  of CP_POLY_SHAPE:
    let 
      poly = TOSPRITE(shape, csfml.ConvexShape)
      body = shape.getBody()
    poly.position = body.getPos.floor
    poly.rotation = body.getAngle.degrees
    WINDOW(winda).draw poly
  else:
    discard

proc initializeShape(shape: chipmunk.ShapePtr; userData: pointer = nil) {.cdecl.} =
  if not shape.data.isNil:
    return
  # Initialize the chipmunk shape data attribute
  shape.data = cast[ShapeDataPtr](alloc0(sizeof(ShapeData)))
  # Create a data variable on the heap
  var data = cast[ShapeDataPtr](alloc0(sizeof(ShapeData)))
  data[0] = userData
  # Add the shape to the to the data[1] field
  case shape.klass.kind
  of CP_CIRCLE_SHAPE:
    data[1] = csfml.newCircleShape(shape.getCircleRadius(), 30)
    let radius = shape.getCircleRadius()
    cast[csfml.CircleShape](data[1]).origin = Vector2f(x:radius, y:radius)
    cast[csfml.CircleShape](data[1]).fillColor = colors[CP_CIRCLE_SHAPE]
  of CP_SEGMENT_SHAPE:
    ## VertexArray == array[x, ptr Vertex]
    data[1] = csfml.newVertexArray(PrimitiveType.Lines, 2)
    cast[VertexArray](data[1])[0].position = shape.getSegmentA.cp2sfml()
    cast[VertexArray](data[1])[1].position = shape.getSegmentB.cp2sfml()
    cast[VertexArray](data[1])[0].color = colors[CP_SEGMENT_SHAPE]
    cast[VertexArray](data[1])[1].color = colors[CP_SEGMENT_SHAPE]
  of CP_POLY_SHAPE:
    var poly = csfml.newConvexShape(shape.getNumVerts())
    for i in 0.. <shape.getNumVerts():
      poly.setPoint i, shape.getVert(i).cp2sfml()
    poly.fillColor = colors[CP_POLY_SHAPE]
    data[1] = poly
    cast[ShapeDataPtr](shape.data)[1] = poly
  else: 
    echo "Unknown shape type! ", repr(shape.klass.kind)
    return
  # Write the data variable to the shape's data attribute
  shape.data = data

proc draw*(window: RenderWindow; space: SpacePtr) {.inline.} =
  space.eachShape(drawShape, cast[pointer](window))

proc debugDrawInit*(space: SpacePtr) = 
  space.eachShape(initializeShape, nil)

proc addShape*(space: SpacePtr; shape: chipmunk.ShapePtr; 
               userData: pointer = nil): chipmunk.ShapePtr {.discardable.} =
  result = chipmunk.addShape(space, shape)
  initializeShape result, userData

proc removeShape*(space: SpacePtr; shape: chipmunk.ShapePtr, deallocData = true) =
  chipmunk.removeShape space, shape
  if not shape.data.isNil:
    case shape.klass.kind
    of CP_SEGMENT_SHAPE: TOSPRITE(shape, VertexArray).destroy()
    of CP_POLY_SHAPE: TOSPRITE(shape, csfml.RectangleShape).destroy()
    of CP_CIRCLE_SHAPE: TOSPRITE(shape, csfml.CircleShape).destroy()
    else: discard
    if deallocData:
      dealloc(shape.data)

