import sfml, chipmunk, debugDraw, sfml_colors, math


import
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

window.setFramerateLimit 60

randomize()

proc randomPoint(rect: var TIntRect): TVector =
  result.x = (random(rect.width) + rect.left).CpFloat
  result.y = (random(rect.height) + rect.top).CpFloat

##Create a bunch of objects
block:
  let borders = [vector(0, 0), vector(0, ScreenH),
    vector(ScreenW, ScreenH), vector(ScreenW, 0)]
  for i in 0..3:
    var shape = space.addStaticShape(space.getStaticBody.newSegmentShape(
      borders[i], borders[(i + 1) mod 4], 16.0))
  var area = intRect(20, 20, ScreenW - 20, ScreenH - 20)
  for i in 0..30:
    var body = space.addBody(newBody((rand(1000) / 1000 * 10.0), 500.0))
    var shape = space.addShape(body.newCircleShape(random(10000) / 700, vectorZero))
    body.setPos randomPoint(area)
    shape.setLayers LGrabbable
  for i in 0..20:
    var body = space.addBody(newBody((rand(1000) / 1000 * 10.0), 500.0))
    var shape = space.addShape(body.newBoxShape(random(30).float, random(12).float))
    body.setPos randomPoint(area)
    shape.setLayers LGrabbable

debugDrawInit space

proc vector(vec: TVector2i): TVector =
  result.x = vec.x.float
  result.y = vec.y.float

var 
  fpsText = newText("", guiFont, 16)
  mousePos = vector(0,0)
  activeShape: chipmunk.PShape
while window.isOpen():
  while window.pollEvent(event):
    case event.kind
    of EvtClosed:
      window.close()
    of EvtMouseWheelMoved:
      if event.mouseWheel.delta == 1:  ## upd
        view.zoom(0.9)
      elif event.mouseWheel.delta == -1: ##down
        view.zoom(1.1)
    of EvtMouseMoved:
      mousePos.x = event.mouseMove.x.float
      mousePos.y = event.mouseMove.y.float
    of EvtMouseButtonPressed:
      if event.mouseButton.button == MouseLeft:
        var shape = space.pointQueryFirst(mousePos, LGrabbable, 0)
        if not shape.isNil:
          activeShape = shape
        else:
          echo "I got nothin'"
    of EvtMouseButtonReleased:
      if event.mouseButton.button == MouseLeft:
        activeShape = nil
    else: discard
  
  if not activeShape.isNil:
    activeShape.body.setPos mousePos
  
  let dt = fps.restart.asMilliseconds() / 1000
  space.step(dt)
  
  fpsText.setString(formatFloat((1.0 / dt), ffDecimal, 2))
  
  window.clear black
  
  window.setView view
  
  window.draw space
  window.draw fpsText
  
  window.display