
import chipmunk, sfml, sfml_colors, math, debugDraw

const
  gravityStrength = 50.CpFloat
  CTplanet = 1.TCollisionType
  CTgravity= 2.TCollisionType
  ScreenW = 640
  ScreenH = 480
var 
  space = newSpace()
  window = newRenderWindow(videoMode(ScreenW, ScreenH, 32), "Planets demo", sfDefaultStyle)
  

proc gravityApplicator(arb: PArbiter; space: PSpace; data: pointer): bool {.cdecl.} =
  let 
    dist = arb.bodyA.getPos() - arb.bodyB.getPos()
  arb.bodyB.applyImpulse(dist.normalize() * (1.0 / dist.len() * gravityStrength), vectorZero)

randomize()

proc randomPoint(rect: var TIntRect): TVector =
  result.x = (random(rect.width) + rect.left).CpFloat
  result.y = (random(rect.height) + rect.top).CpFloat

var screenArea = intRect(20, 20, ScreenW-20, ScreenH-20)

proc addPlanet() =
  let
    mass = random(10_000)/10_000*10.0
    radius = mass * 2.0
    gravityRadius = radius * 8.8
    body = space.addBody(newBody(mass, momentForCircle(mass, 0.0, radius, vectorZero)))
    shape = debugDraw.addShape(space, body.newCircleShape(radius, vectorZero))
    gravity = debugDraw.addShape(space, body.newCircleShape(gravityRadius, vectorZero))
    gravityCirc = TOSPRITE(gravity, sfml.PCircleShape)
  body.setPos randomPoint(screenArea)
  shape.setCollisionType CTplanet
  gravity.setSensor true
  gravity.setCollisionType CTgravity
  gravityCirc.setFillColor Transparent
  gravityCirc.setOutlineColor Blue
  gravityCirc.setOutlineThickness 2.0


window.setFrameRateLimit 60
space.setIterations 20
space.addCollisionHandler(CTgravity, CTplanet, preSolve = gravityApplicator)

block:
  let borders = [vector(0, 0), vector(0, ScreenH),
    vector(ScreenW, ScreenH), vector(ScreenW, 0)]
  for i in 0..3:
    var shape = space.addStaticShape(space.getStaticBody.newSegmentShape(
      borders[i], borders[(i + 1) mod 4], 16.0))
  for i in 1..30: addPlanet()

debugDrawInit(space)

var 
  running = true
  event: TEvent
  c = newClock()
while running:
  while window.pollEvent(event):
    if event.kind == EvtClosed:
      running = false
      break
  let dt = c.restart.asMilliseconds / 1000
  
  space.step dt
  window.clear Black
  window.draw space
  window.display()



