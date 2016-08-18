import 
  csfml, 
  chipmunk, 
  debugdraw, 
  math,
  random,
  basic2d,
  strutils

const
  ScreenW = 800
  ScreenH = 600
  FontFile = "sansation.ttf"
  LGrabbable = (1 shl 0).Layers
  CTClutter = 2.CollisionType
  CTExpl = 3.CollisionType

type
  ExplosionPtr = ref Explosion
  Explosion = object
    shape: chipmunk.ShapePtr
    lifetime: float
  UserData = ptr tuple[x: ExplosionPtr; sprite: pointer]
  
var 
  space = newSpace()
  window = newRenderWindow(
    videoMode(800, 600, 32), 
    "Chipmunk DebugDraw", WindowStyle.Default
  )
  event: csfml.Event
  fps = newClock()
  guiFont = newFont(FontFile)
  view = window.defaultView.copy()
  forceMult = -2000.0
  explosions: seq[ExplosionPtr] = @[]
  
proc randomPoint(rect: var IntRect): Vector =
  result.x = (random(rect.width) + rect.left).CpFloat
  result.y = (random(rect.height) + rect.top).CpFloat

proc vector[A:Vector2f|Vector2i](vec: A): Vector =
  result.x = vec.x.CpFloat
  result.y = vec.y.CpFloat

proc vec2i(vec: Vector): Vector2i =
  result.x = vec.x.cint
  result.y = vec.y.cint

template CastUserCircle(a: chipmunk.ShapePtr): csfml.CircleShape =
  cast[csfml.CircleShape](cast[UserData](a.data).sprite)

##presolve
proc repel(arb: ArbiterPtr; space: SpacePtr; data: pointer): bool {.cdecl.} =
  var 
    dist: Vector = arb.bodyA.getPos() - arb.bodyB.getPos()
    distNorm = dist
  distNorm.normalize()
  arb.bodyB.applyImpulse(distNorm * (1.0 / basic2d.len(dist) * forceMult), VectorZero)

proc free(expl: ExplosionPtr) =
  expl.shape.body.free()
  expl.shape.free()

proc createExplosion(point: Vector, lifetime: float) =
  var x: ExplosionPtr
  new(x, free)
  x.lifetime = lifetime
  var b = newBody(CpInfinity, CpInfinity)
  b.setPos point
  x.shape = newCircleShape(b, 150, VectorZero)
  x.shape.setSensor true
  x.shape.setCollisionType CtExpl
  explosions.add x  
  debugdraw.addShape(space, x.shape, cast[pointer](x)) 
  var cs = cast[csfml.CircleShape](x.shape.data)
  cs.outlineColor = Red
  cs.outlineThickness = 3.4
  cs.fillColor = Transparent
  
proc random*(min, max: int): int {.inline.} = 
  random(max - min) + min


# Startup initialization
randomize()
window.framerateLimit = 60
space.addCollisionHandler(CtExpl, CtClutter, preSolve = repel)

# Create a bunch of objects
block:
  let borders = [vector(0, 0), vector(0, ScreenH),
    vector(ScreenW, ScreenH), vector(ScreenW, 0)]
  for i in 0..3:
    var shape = space.addStaticShape(space.getStaticBody.newSegmentShape(
      borders[i], borders[(i + 1) mod 4], 16.0))
  var area = IntRect(
    left: 20, 
    top: 20, 
    width: ScreenW - 20, 
    height: ScreenH - 20
  )
  for i in 0..30:
    var 
      body = space.addBody(newBody(random(5, 45).float / 5.0, 120.0))
      shape = debugdraw.addShape(space, body.newCircleShape(random(10000) / 700, VectorZero))
    body.setPos randomPoint(area)
    shape.setCollisionType CtClutter
  for i in 0..20:
    var 
      body = space.addBody(newBody(random(2, 40).float / 5.0, 120.0))
      shape = debugdraw.addShape(space, 
        body.newBoxShape(random(5, 30).float, random(5, 12).float))
    body.setPos randomPoint(area)
    shape.setCollisionType CtClutter

# Initialize the debugdraw module
debugdrawInit(space)

var 
  fpsText = newText("", guiFont, 16)
  debugText = fpsText.copy()
  mousePos = vector(0,0)
  activeShape: chipmunk.ShapePtr
  mouseShape = debugdraw.addShape(
    space, 
    newCircleShape(
      space.getStaticBody(), 20.0, VectorZero
    )
  )

debugtext.position = fpstext.position + Vector2f(x:0, y:16)

block:
  let circ = cast[csfml.CircleShape](mouseShape.data)
  circ.outlineColor = Blue
  circ.fillColor = Transparent
  circ.outlineThickness = 1.2

while window.open():
  while window.pollEvent(event):
    case event.kind
    of EventType.Closed:
      window.close()
    of EventType.MouseWheelMoved:
      if event.mouseWheel.delta == 1:
        forceMult += 200
      else:
        forceMult -= 200
      debugText.strC = $forcemult.int
    of EventType.MouseMoved:
      mousePos.x = event.mouseMove.x.CpFloat
      mousePos.y = event.mouseMove.y.CpFloat
      mouseShape.body.setPos mousePos
    of EventType.MouseButtonPressed:
      case event.mouseButton.button
      of MouseButton.Left:
        let pos = window.mapPixelToCoords(vec2i(mousePos), view)
        echo LGrabbable
        var shape = space.pointQueryFirst(vector(pos), LGrabbable, 0)
        if not shape.isNil:
          activeShape = shape
        else:
          echo "I got nothin'"
      of MouseButton.Right:
        createExplosion mousePos, 1.0
      else: discard
    of EventType.MouseButtonReleased:
      if event.mouseButton.button == MouseButton.Left:
        activeShape = nil
    of EventType.KeyPressed:
      if event.key.code == KeyCode.Escape:
        window.close()
        break
      
    else: discard
  
  let dt = fps.restart.asMilliseconds() / 1000
  
  fpsText.strC = formatFloat((1.0 / dt), ffDecimal, 2)
  
  var i = 0
  while i < explosions.len:
    explosions[i].lifetime -= dt
    if explosions[i].lifetime < 0:
      debugdraw.removeShape(space, explosions[i].shape)
      explosions.del i
    else:
      inc i
  if not activeShape.isNil:
    activeShape.body.setPos mousePos
  
  space.step(dt)
  
  window.clear(Black)
  window.view = view
  
  ## this is all thats required to draw all the shapes
  window.draw space
  
  window.draw fpsText
  window.draw debugText
  
  window.display