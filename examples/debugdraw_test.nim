import sfml, chipmunk, debugDraw, sfml_colors, math,
  strutils
const
  ScreenW = 800
  ScreenH = 600
  FontFile = "sansation.ttf"
var 
  space = newSpace()
  window = newRenderWindow(
    videoMode(800, 600, 32), 
    "chipmunk debugdraw", sfClose or sfTitlebar)
  event: sfml.TEvent
  fps = newClock()
  guiFont = newFont(FontFile)
  view = window.getDefaultView.copy()
const
  LGrabbable = (1 shl 0).TLayers
  CTClutter = 2.TCollisionType
  CTExpl = 3.TCollisionType

proc randomPoint(rect: var TIntRect): TVector =
  result.x = (random(rect.width) + rect.left).CpFloat
  result.y = (random(rect.height) + rect.top).CpFloat

proc vector[A:TVector2f|TVector2i](vec: A): TVector =
  result.x = vec.x.CpFloat
  result.y = vec.y.CpFloat

type
  PExplosion = ref TExplosion
  TExplosion = object
    shape: chipmunk.PShape
    lifetime: float

##presolve
var forceMult = -2000.0
proc repel(arb: PArbiter; space: PSpace; data: pointer): bool {.cdecl.} =
  let 
    dist = arb.bodyA.getPos() - arb.bodyB.getPos()
    lenSq= dist.lenSq()
    norml= dist.normalize()
  arb.bodyB.applyImpulse(norml * (1.0 / sqrt(lenSq) * forceMult), vectorZero)
  

randomize()
window.setFramerateLimit 60
space.addCollisionHandler(CtExpl, CtClutter, preSolve = repel)


var explosions: seq[PExplosion] = @[]

type PUserData = ptr tuple[x: PExplosion; z: pointer]

template USERDATA(a: pointer): PUserData = cast[PUserData](a)
template UserCircle(a: pointer): sfml.PCircleShape =
  cast[sfml.PCircleShape](USERDATA(a).z)

proc free(expl: PExplosion) =
  expl.shape.body.free()
  #cast[sfml.PCircleShape](expl.shape.data).destroy()
  expl.shape.free()
  echo "Explosion free'd"

proc createExplosion(point: TVector, lifetime: float) =
  var x: PExplosion
  new(x, free)
  x.lifetime = lifetime
  var b = newBody(CpInfinity, CpInfinity)
  b.setPos point
  x.shape = newCircleShape(b, 150, vectorZero)
  x.shape.setSensor true.bool32
  x.shape.setCollisionType CtExpl
  explosions.add x  
  debugDraw.addShape(space, x.shape, cast[pointer](x)) 
  var cs = UserCircle(x.shape.data)
  cs.setOutlineColor Red
  cs.setOutlineThickness 3.4
  cs.setFillColor Transparent
  
proc random*(min, max: int): int {.inline.} = random(max - min) + min


##Create a bunch of objects
block:
  let borders = [vector(0, 0), vector(0, ScreenH),
    vector(ScreenW, ScreenH), vector(ScreenW, 0)]
  for i in 0..3:
    var shape = space.addStaticShape(space.getStaticBody.newSegmentShape(
      borders[i], borders[(i + 1) mod 4], 16.0))
  var area = intRect(20, 20, ScreenW - 20, ScreenH - 20)
  for i in 0..30:
    var 
      body = space.addBody(newBody(random(5, 45).float / 5.0, 120.0))
      shape = debugDraw.addShape(space, body.newCircleShape(random(10000) / 700, vectorZero))
    body.setPos randomPoint(area)
    #shape.setLayers LGrabbable
    shape.setCollisionType CtClutter
  for i in 0..20:
    var 
      body = space.addBody(newBody(random(2, 40).float / 5.0, 120.0))
      shape = debugDraw.addShape(space, 
        body.newBoxShape(random(5, 30).float, random(5, 12).float))
    body.setPos randomPoint(area)
    #shape.setLayers LGrabbable
    shape.setCollisionType CtClutter

debugDrawInit space

proc vec2i(vec: TVector): TVector2i =
  result.x = vec.x.cint
  result.y = vec.y.cint

var 
  fpsText = newText("", guiFont, 16)
  debugText = fpsText.copy()
  mousePos = vector(0,0)
  activeShape: chipmunk.PShape
debugtext.setPosition(fpstext.getPosition() + vec2f(0, 16))

while window.isOpen():
  while window.pollEvent(event):
    case event.kind
    of EvtClosed:
      window.close()
    of EvtMouseWheelMoved:
      if event.mouseWheel.delta == 1:  ## upd
        forceMult += 200
      else:
        forceMult -= 200
      debugText.setString($forcemult.int)
    of EvtMouseMoved:
      mousePos.x = event.mouseMove.x.CpFloat
      mousePos.y = event.mouseMove.y.CpFloat
    of EvtMouseButtonPressed:
      case event.mouseButton.button
      of MouseLeft:
        let pos = window.convertCoords(vec2i(mousePos), view)
        var shape = space.pointQueryFirst(vector(pos), LGrabbable, 0)
        if not shape.isNil:
          activeShape = shape
        else:
          echo "I got nothin'"
      of MouseRight:
        createExplosion mousePos, 1.0
      else: discard
    of EvtMouseButtonReleased:
      if event.mouseButton.button == MouseLeft:
        activeShape = nil
      
    else: discard
  
  let dt = fps.restart.asMilliseconds() / 1000
  
  var i = 0
  while i < explosions.len:
    explosions[i].lifetime -= dt
    if explosions[i].lifetime < 0:
      debugdraw.removeShape(space, explosions[i].shape)
      explosions.del i
      echo "Explosion expired"
    else:
      inc i
  if not activeShape.isNil:
    activeShape.body.setPos mousePos
  
  space.step(dt)
  
  fpsText.setString(formatFloat((1.0 / dt), ffDecimal, 2))
  
  window.clear black
  
  window.setView view
  
  window.draw space
  window.draw fpsText
  window.draw debugText
  
  window.display