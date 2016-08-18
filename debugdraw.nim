import 
  csfml, 
  chipmunk, 
  math, 
  random

let
  colors: array[ShapeType, Color] = [Green, Blue, Red, Yellow]
  
proc cp2sfml(vec: Vector): Vector2f =
  result.x = vec.x
  result.y = vec.y

proc floor(vec: Vector): Vector2f =
  result.x = vec.x.floor
  result.y = vec.y.floor

template WINDOW(shape: pointer): RenderWindow = 
  cast[RenderWindow](shape)

template TOSPRITE*(shape: chipmunk.ShapePtr, to: typedesc): expr =
  cast[to](shape.data)

proc drawShape(shape: chipmunk.ShapePtr, winda: pointer) {.cdecl.} =
  case shape.klass.kind
  of CP_CIRCLE_SHAPE:
    let circle = TOSPRITE(shape, csfml.CircleShape)
    circle.position = shape.getBody().getPos.floor()
    WINDOW(winda).draw(circle)
  of CP_SEGMENT_SHAPE:
    WINDOW(winda).draw TOSPRITE(shape, csfml.VertexArray)
  of CP_POLY_SHAPE:
    let polygon = TOSPRITE(shape, csfml.ConvexShape)
    polygon.position = shape.getBody().getPos.floor()
    polygon.rotation = math.radToDeg(shape.getBody().getAngle())
    WINDOW(winda).draw polygon
  else:
    discard

proc initializeShape(shape: chipmunk.ShapePtr; userData: pointer = nil) {.cdecl.} =
  if not shape.data.isNil:
    return
  case shape.klass.kind
  of CP_CIRCLE_SHAPE:
    shape.data = csfml.newCircleShape(shape.getCircleRadius(), 30)
    let 
      radius = shape.getCircleRadius()
      circleData = cast[csfml.CircleShape](shape.data)
    circleData.origin = Vector2f(x:radius, y:radius)
    circleData.fillColor = colors[CP_CIRCLE_SHAPE]
  of CP_SEGMENT_SHAPE:
    ## VertexArray == array[x, ptr Vertex]
    shape.data = csfml.newVertexArray(PrimitiveType.Lines, 2)
    let vertexData = cast[VertexArray](shape.data)
    vertexData.getVertex(0).position = shape.getSegmentA.cp2sfml()
    vertexData.getVertex(1).position = shape.getSegmentB.cp2sfml()
    vertexData.getVertex(0).color = colors[CP_SEGMENT_SHAPE]
    vertexData.getVertex(1).color = colors[CP_SEGMENT_SHAPE]
  of CP_POLY_SHAPE:
    shape.data = csfml.newConvexShape(shape.getNumVerts())
    let polygonData = cast[ConvexShape](shape.data)
    for i in 0.. <shape.getNumVerts():
      polygonData.setPoint(i, shape.getVert(i).cp2sfml())
    polygonData.fillColor = colors[CP_POLY_SHAPE]
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
  initializeShape(result, userData)

proc removeShape*(space: SpacePtr; shape: chipmunk.ShapePtr, deallocData = true) =
  chipmunk.removeShape space, shape

