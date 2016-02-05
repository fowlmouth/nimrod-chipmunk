import csfml, chipmunk, math

let
  colors: array[ShapeType, Color] = [Green, Blue, Red, Yellow]
  
proc cp2sfml(vec: Vector): Vector2f =
  result.x = vec.x
  result.y = vec.y

proc floor(vec: Vector): Vector2f =
  result.x = vec.x.floor
  result.y = vec.y.floor

template WINDOW(shape: pointer): RenderWindow = cast[RenderWindow](shape)
template TOSPRITE*(shape: chipmunk.ShapePtr, to: typedesc): expr =
  cast[to](shape.data)

proc drawShape(shape: chipmunk.ShapePtr, winda: pointer) {.cdecl.} =
  case shape.klass.kind
  of CP_CIRCLE_SHAPE:
    TOSPRITE(shape, csfml.CircleShape).position = shape.getBody().getPos.floor()
    WINDOW(winda).draw TOSPRITE(shape, csfml.CircleShape)
  of CP_SEGMENT_SHAPE:
    WINDOW(winda).draw TOSPRITE(shape, csfml.VertexArray)
  of CP_POLY_SHAPE:
    TOSPRITE(shape, csfml.ConvexShape).position = shape.getBody().getPos.floor()
    TOSPRITE(shape, csfml.ConvexShape).rotation = math.radToDeg(shape.getBody().getAngle())
    WINDOW(winda).draw TOSPRITE(shape, csfml.ConvexShape)
  else:
    discard

proc initializeShape(shape: chipmunk.ShapePtr; userData: pointer = nil) {.cdecl.} =
  if not shape.data.isNil:
    return
  # Add the shape to the to the data[1] field
  case shape.klass.kind
  of CP_CIRCLE_SHAPE:
    shape.data = csfml.newCircleShape(shape.getCircleRadius(), 30)
    let radius = shape.getCircleRadius()
    cast[csfml.CircleShape](shape.data).origin = Vector2f(x:radius, y:radius)
    cast[csfml.CircleShape](shape.data).fillColor = colors[CP_CIRCLE_SHAPE]
  of CP_SEGMENT_SHAPE:
    ## VertexArray == array[x, ptr Vertex]
    shape.data = csfml.newVertexArray(PrimitiveType.Lines, 2)
    cast[VertexArray](shape.data)[0].position = shape.getSegmentA.cp2sfml()
    cast[VertexArray](shape.data)[1].position = shape.getSegmentB.cp2sfml()
    cast[VertexArray](shape.data)[0].color = colors[CP_SEGMENT_SHAPE]
    cast[VertexArray](shape.data)[1].color = colors[CP_SEGMENT_SHAPE]
  of CP_POLY_SHAPE:
    var poly = csfml.newConvexShape(shape.getNumVerts())
#    for i in 0.. <shape.getNumVerts():
#      poly.setPoint i, shape.getVert(i).cp2sfml()
#    poly.fillColor = colors[CP_POLY_SHAPE]
#    data[1] = poly
#    cast[ShapeDataPtr](shape.data)[1] = poly
  else: 
    echo "Unknown shape type! ", repr(shape.klass.kind)
    return

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

