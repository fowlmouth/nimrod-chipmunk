import 
  chipmunk, 
  csfml, 
  math

const
  Width = 800
  Height = 600

type 
  GameObjPtr = ref object
    circleSprite: csfml.CircleShape
    rectangleSprite: csfml.RectangleShape
    body: chipmunk.BodyPtr
    shape: chipmunk.ShapePtr

var 
  window = newRenderWindow(
    videoMode(Width, Height, 32), "Chipmunk Test", WindowStyle.Default
  )
  space = newSpace()
  gameobjects: seq[GameObjPtr] = @[]

window.framerateLimit = 60
space.setGravity(newVector(8.9, 82.3))
randomize()

proc cp2sfml(v: Vector): Vector2f {.inline.} =
  result.x = v.x
  result.y = v.y

let
  ClBorder = 1.Layers
  ClBall = (1 or 2).Layers
let
  CtBorder = 1.CollisionType
  CtBall = 2.CollisionType

proc borderballz(a: ArbiterPtr; space: SpacePtr; data: pointer): bool {.cdecl.} =
  echo("Borderballz()")
  result = true
space.addCollisionHandler(CtBorder, CtBall, borderballz, nil, nil, nil, nil)

var borders: seq[Vector]
borders = @[
  newVector(0.0, 0.0),
  newVector(500.0,0),
  newVector(500.0,500.0),
  newVector(0.0,  500.0)]
var sfBorders = newVertexArray(PrimitiveType.LinesStrip, 4)
for i in 0..3:
  var shape = space.addStaticShape(
    newSegmentShape(
        space.getStaticBody(), 
        borders[i], 
        borders[(i+1) mod 4], 
        1.0
      )
    )
  sfBorders.getVertex(i).position = borders[i].cp2sfml
  shape.setLayers(ClBorder)
  shape.setCollisionType(CtBorder)
  echo($ shape.getLayers())


proc vectorToVec2f(a: Vector): Vector2f =
  result.x = a.x
  result.y = a.y

proc newBall(mass = 10.0, radius = 10.0): GameObjPtr =
  #let pos = newVector(random(Width).float, random(Height).float)
  let pos = newVector(20.0, 30.0)
  new(result)
  result.rectangleSprite = nil
  result.circleSprite = newCircleShape()
  result.circleSprite.radius = radius
  result.circleSprite.origin = Vector2f(x: radius, y: radius)
  result.body = space.addBody(
    newBody(mass, MomentForCircle(mass, 0.0, radius, VectorZero))
  )
  result.body.p = pos
  result.shape = space.addShape(
    newCircleShape(result.body, radius, VectorZero)
  )
  result.shape.setLayers(ClBall)
  result.shape.setCollisionType(CtBall)
  echo($pos, $result.body.getPos(), $result.body.getMass()) #not being set for some reason .. >:\

proc newBox(mass = 10.0, width = 10.0, height = 10.0,
            position = newVector(30.0, 10.0)): GameObjPtr =
  new(result)
  result.circleSprite = nil
  result.rectangleSprite = newRectangleShape()
  result.rectangleSprite.size = Vector2f(x: width, y: height)
  result.rectangleSprite.origin = Vector2f(x: width/2, y: height/2)
  result.body = space.addBody(newBody(mass, MomentForBox(mass, width, height)))
  result.body.p = position
  result.shape = space.addShape(newBoxShape(result.body, width, height))
  result.shape.setLayers(ClBall)
  result.shape.setCollisionType(CtBall)

for i in 0..20:
    gameobjects.add(newBall(50.0, 30.0))
for i in 0..10:
    gameobjects.add(newBox(50.0, 30.0, 30.0, newVector(400.0, 50)))
var ball = newBall(10.0, 15.0)
ball.rectangleSprite = nil
ball.circleSprite.fillColor = Blue
gameobjects.add(ball)

var 
  font = newFont("sansation.ttf")
  text = newText()
  event: Event
  oldPos: Vector
text.characterSize = 18
text.font = font
text.strC = "Chipmunk2D TEST" 
var text2 = text.copy()
text2.position = text2.position + Vector2f(x:0.0, y:18.0)
while window.open():
  while window.pollEvent(event):
    if event.kind == EventType.Closed:
      window.close()
      break
    elif event.kind == EventType.KeyPressed:
      if event.key.code == KeyCode.R:
        #ball.body.setPos(newVector(100.0, 100.0))
        ball.body.resetForces()
        text.strC = $ball.body.getPos()
        oldPos = ball.body.getPos()
        echo("oldPos = ", repr(ball.body.getPos()))
      elif event.key.code == KeyCode.O:
        ball.body.setPos(oldPos)
      elif event.key.code == KeyCode.Escape:
        window.close()
        break
      
  space.step(1.0/60.0)
  window.clear(Black)
  for o in gameobjects: 
    if o.rectangleSprite == nil:
        o.circleSprite.position = o.body.getPos.vectorToVec2f
        o.circleSprite.rotation = o.body.getAngle().radToDeg()
        window.draw o.circleSprite
    else:
        o.rectangleSprite.position = o.body.getPos.vectorToVec2f
        o.rectangleSprite.rotation = o.body.getAngle().radToDeg()
        window.draw o.rectangleSprite
  window.draw text
  window.draw text2
  window.draw sfBorders
  window.display()

for o in gameobjects:
  o.body.destroy()
  if o.rectangleSprite == nil:
    o.circleSprite.destroy()
  else:
    o.rectangleSprite.destroy()

space.destroy()