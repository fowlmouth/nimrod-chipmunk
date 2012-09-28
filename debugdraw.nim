import sfml, sfml_colors, chipmunk, math

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
proc drawShape(shape: chipmunk.PShape, winda: pointer) {.cdecl.} =
  let 
    body = shape.getBody()
  case shape.klass.kind
  of CP_CIRCLE_SHAPE:
    let circ = cast[sfml.PCircleShape](shape.data)
    circ.setPosition body.getPos.floor()
    WINDOW(winda).draw circ
  of CP_SEGMENT_SHAPE:
    WINDOW(winda).draw cast[sfml.PVertexArray](shape.data)
  of CP_POLY_SHAPE:
    let poly = cast[sfml.PConvexShape](shape.data)
    poly.setPosition body.getPos.floor
    poly.setRotation body.getAngle.degrees
    WINDOW(winda).draw poly
  else:
    discard


proc initializeShape(shape: chipmunk.PShape; unused: pointer) {.cdecl.} =
  case shape.klass.kind
  of CP_CIRCLE_SHAPE:
    var circ = sfml.newCircleShape(shape.getCircleRadius(), 30)
    circ.setFillColor colors[CP_CIRCLE_SHAPE]
    shape.setUserData(circ)
  of CP_SEGMENT_SHAPE:
    var seg = sfml.newVertexArray(sfml.Lines, 2)
    seg[0].position = shape.getSegmentA.cp2sfml()
    seg[1].position = shape.getSegmentB.cp2sfml()
    seg[0].color = colors[CP_SEGMENT_SHAPE]
    seg[1].color = colors[CP_SEGMENT_SHAPE]
    shape.setUserData(seg)
  of CP_POLY_SHAPE:
    var poly = sfml.newConvexShape(shape.getNumVerts())
    for i in 0.. <shape.getNumVerts():
      poly.setPoint i.cuint, shape.getVert(i).cp2sfml()
    poly.setFillColor colors[CP_POLY_SHAPE]
    shape.setUserData poly
  else: 
    echo "Unknown shape type! ", repr(shape.klass.kind)

proc draw*(window: PRenderWindow; space: PSpace) {.inline.} =
  space.eachShape(drawShape, cast[pointer](window))

proc debugDrawInit*(space: PSpace) = 
  space.eachShape(initializeShape, nil)



  