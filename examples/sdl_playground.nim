import 
  sdl2,
  sdl2/gfx,
  chipmunk, 
  math, 
  basic2d, 
  json, 
  strutils

type
  MouseData = tuple
    position: Vector2d
    button_mask: uint8
    button_left: bool
    button_middle: bool
    button_right: bool

const
  winSize = (w: 640, h: 480)

var
  mainWindow: sdl2.WindowPtr
  mainRenderer: sdl2.RendererPtr
  mousePos: Vector2d #tuple[x,y: cint]
  mouseJoint: ConstraintPtr
  space = newSpace()
  mouseBody = space.addBody(newBody(1.0, 1.0))
  running = true

let
  data = json.parseFile("sdl_playground.json")

#[
proc addCircle (
    pos = newVector(winSize.x.random.cpfloat, winSize.y.random.cpfloat);
    mass = 0.9;
    radius = 10.0 ) =
  var 
    body = space.addBody(newBody(mass, momentForCircle(mass, 0.0, radius, VectorZero)))
    shape = space.addShape(
      newCircleShape(body, radius, VectorZero))
  body.setPos pos
proc addBox (
    pos = newVector(winSize.x.random.cpfloat, winSize.y.random.cpfloat);
    mass = 0.9;
    width = 10.0;
    height = 5.0 ) =
  var
    body = space.addBody(newBody(mass, MomentForBox(mass, width, height)))
    shape = space.addShape(
      newBoxShape(body, width, height))
  body.p = pos
]#

proc getFloat(some: JsonNode): float =
  case some.kind
  of JFloat: some.fnum.float
  of JInt: some.num.float
  of JString: some.str.parseFloat
  else: 0.0

proc getVector(some: JsonNode): Vector2d =
  Vector2d(x: some[0].getFLoat, y: some[1].getFloat)

proc loadBody (item: JsonNode): BodyPtr =
  let mass = item["mass"].getFloat
  case item["shape"].str.normalize
  of "circle":
    result = newBody(mass, momentForCircle(mass, 0.0, item["radius"].getFloat, VectorZero))
  of "box":
    result = newBody(mass, MomentForBox(mass, item["width"].getFloat, item["height"].getFloat))
  else:
    discard
  
  if not result.isNil:
    result.p = item["position"].getVector

proc loadShape(item: JsonNode, body: BodyPtr): ShapePtr =
  case item["shape"].str.normalize
  of "circle":
    result = newCircleShape(body, item["radius"].getFloat, VectorZero)
  of "box": 
    result = newBoxShape(body, item["width"].getFloat, item["height"].getFloat)
  of "segment":
    let
      a = item["a"].getVector
      b = item["b"].getVector
      w = if item.existsKey("width"): item["width"].getFLoat else: 2.0
    result = newSegmentShape(body, a, b, w)
  else: 
    result = nil

proc loadScene(scene: string) =
  let scene = data[scene]
  
  space.setGravity(scene["gravity"].getVector())
  
  if scene.existsKey("static"):
    for it in scene["static"]:
      discard space.addStaticShape(it.loadShape(space.getStaticBody))
  
  if scene.existsKey("dynamic"):
    for it in scene["dynamic"]:
      discard space.addShape(it.loadShape(space.addBody(it.loadBody)))

#[
block:
  let borders = [
    newVector(0, 0), newVector(winSize.x.cpfloat, 0),
    newVector(winSize.x.cpfloat, winSize.y.cpfloat), newVector(0, winSize.y.cpfloat)
  ]
  for i in 0 .. 3:
    var shape = space.addStaticShape(
      newSegmentShape(space.getStaticBody, borders[i], borders[(i + 1) mod borders.len], 2.0)
    )
  for i in random(100).times:
    addCircle(radius = random(3 .. 10).float)
  for i in random(100).times:
    addBox(width = random(5 .. 15).float, height = random(6 .. 12).float)
]#

proc byteRepr [T](anything: var T): string =
  result = ""
  let rawBytes = cast[ptr array[10_000, byte]](anything.addr)
  var index = 0
  for name, field in anything.toAny.fields:
    result.addf("$1: ", name, field.size)
    for i in 0 .. <field.size:
      result.add($ rawBytes[index + i])
      result.add ' '
    inc index, field.size
    result.add '\L'
    

#NG.addHandler do(E: PSdlEngine) -> bool:
#  if E.evt.kind in { MouseButtonDown, MouseButtonUp }:
#    let m = E.evt.evMOuseButton
#    if m.button == BUTTON_LEFT:
#      result = true
#      if m.state == 1:
#          let mousePos = vector(m.x.cpfloat, m.y.cpfloat)
#          var closestS = space.pointQueryFirst(mousePos, 0, 0)
#          if not closestS.isNil:
#            mouseJoint = newPivotJoint(mouseBody, closestS.body, VectorZero,
#              closestS.body.world2local(mousePos))
#            mouseJoint.maxForce = 50_000
#            #mouseJoint.biasCoef = 0.15
#            mouseJoint.errorBias = 0.15
#            discard space.addConstraint(mouseJoint)
#          else:
#            echo "No object found at $1,$2".format(mousePos.x.ff, mousepos.y.ff)
#      elif not mouseJoint.isNil:
#        # release
#        space.removeConstraint mouseJoint
#        mouseJoint.free
#        mouseJoint.reset
#    else:
#      echo "wrong mouse button! ", m.button
#      echo byterepr(m[])
#      #echo repr(m[])
#      #echo repr(cast[ptr array[100, byte]](m))

proc GetEvents*(): seq[sdl2.Event] =
    var
      event: sdl2.Event
      eventList: seq[sdl2.Event] = newSeq[sdl2.Event](0)
    while sdl2.pollEvent(event):
      eventList.add(event)
    return eventList

proc GetInputStates(inKeyState: var array[0 .. SDL_NUM_SCANCODES.int, uint8],
                    inMouseState: var MouseData) =
    ## Returns a list with the current SDL keyboard state,
    ## which is updated on SDL_PumpEvents.
    var
      numKeys: ptr int = nil
      mouseX, mouseY: cint
    inKeyState = sdl2.getKeyboardState(numkeys)[]
    inMouseState.button_mask = sdl2.getMouseState(mouseX, mouseY)
    inMouseState.position.x = float(mouseX)
    inMouseState.position.y = float(mouseY)
    # This has to be here to update the event queue
    pumpEvents()
    
proc ProcessInput() =
  # Process every input device
  var
    keysState: array[0 .. SDL_NUM_SCANCODES.int, uint8]
    mouseState: MouseData
    events: seq[sdl2.Event]
  # Get keyboard and mouse states, used when you need the input 
  # state at every frame, not just when an event fires
  GetInputStates(keysState, mouseState)
  # Get the input events from the event queue
  events = GetEvents()
  for i in 0..events.high:
    case events[i].kind:
      of QuitEvent:
        running = false
        break
      of KeyDown:
        var 
          keyEvent = sdl2.key(events[i])[]
          key = keyEvent.keysym.sym
        if key == K_ESCAPE:
          # Escape was pressed to quit application
          running = false
          break
      of MouseButtonDown:
        var 
          mouseEvent = sdl2.button(events[i])[]
          mouseButton = mouseEvent.button
        if mouseButton == BUTTON_LEFT:
          let curMousePos = vector(mouseState.position.x.CpFloat, mouseState.position.y.CpFloat)
          var closestS = space.pointQueryFirst(curMousePos, 0, 0)
          if not closestS.isNil:
            mouseJoint = newPivotJoint(
              mouseBody, 
              closestS.body, 
              VectorZero,
              closestS.body.world2local(curMousePos)
            )
            mouseJoint.maxForce = 50_000
            #mouseJoint.biasCoef = 0.15
            mouseJoint.errorBias = 0.15
            discard space.addConstraint(mouseJoint)
          else:
            echo "No object found at $1,$2".format(curMousePos.x, curMousePos.y)
        elif not mouseJoint.isNil:
          # release
          space.RemoveConstraint(mouseJoint)
          mouseJoint.free()
          mouseJoint.reset()
      else:
        discard

proc drawShape(shape: ShapePtr; data: pointer) {.cdecl.} =
  case shape.klass.kind
  of CP_CIRCLE_SHAPE:
    let 
      r = shape.getCircleRadius
      b = shape.getBody
      a = b.getAngle
      p2 = Vector2d(x:cos(a), y:sin(a)) * r.float
    # Draw the circle
    mainRenderer.circleRGBA(
      b.p.x.int16,
      b.p.y.int16,
      r.int16,
      0,255,0,255
    )
    # Draw the line
    mainRenderer.lineRGBA(
      b.p.x.int16,
      b.p.y.int16,
      (b.p.x + p2.x).int16,
      (b.p.y + p2.y).int16,
      0,255,0,255
    )

  of CP_SEGMENT_SHAPE:
    let
      a = shape.getSegmentA
      b = shape.getSegmentB
    # Draw the line
    mainRenderer.lineRGBA(
      a.x.int16,
      a.y.int16,
      b.x.int16,
      b.y.int16,
      0,0,255,255
    )

  of CP_POLY_SHAPE:
    let 
      nverts = shape.getNumVerts
      b = shape.getBody
      angle = b.getAngle
    
    proc vectorRotate(vec: Vector2d; rad: float): Vector2d {.inline.} =
      var outVector = vec
      outVector.rotate(rad)
      result = outVector
    proc `+`(a: Vector2d; b: Vector2d): Vector2d {.inline.} =
      Vector2d(x: a.x + b.x, y: a.y + b.y)
    var 
      lastVert = shape.getVert(< nverts).vectorRotate(angle) + b.p
    
    for i in 0 .. <nverts:
      let 
        thisV = shape.getVert(i).vectorRotate(angle) + b.p
      # Draw a line
      mainRenderer.lineRGBA(
        lastVert.x.int16,
        lastVert.y.int16,
        thisV.x.int16,
        thisV.y.int16,
        0,0,255,255
      )
      lastVert = thisV
      
  else:
    echo "Undrawn shape ", shape.klass.kind

# Initialize SDL
sdl2.init(INIT_EVERYTHING)
# Math randomize
randomize()
# Create the main window
mainWindow = sdl2.createWindow(
  "Chipmunk playground",
  SDL_WINDOWPOS_CENTERED,
  SDL_WINDOWPOS_CENTERED,
  winSize.w.cint,
  winSize.h.cint,
  SDL_WINDOW_SHOWN
)
# Check window creation succeeded
if mainWindow == nil:
  quit "Error initializing main window!"
# Create the rendered for the main window and initialize it
mainRenderer =  sdl2.createRenderer(
  mainWindow, -1, 
  Renderer_Accelerated or Renderer_PresentVsync or Renderer_TargetTexture
)
# Set the renderers blending mode
mainRenderer.setDrawBlendMode(sdl2.BlendMode.BlendMode_Blend)
# Load the scene
loadScene("basic")
# Main loop
while running:
  ProcessInput()
  mouseBody.p = mousePos #vector(mousePos.x.cpfloat, mousePos.y.cpfloat)
  
  space.step(1/60)
  
  mainRenderer.setDrawColor(r=0, g=0, b=0, a=255)
  mainRenderer.clear()
  
  space.eachShape(drawShape, nil)
  
  mainRenderer.present()

#Perform cleanup
destroy(mainRenderer)
destroy(mainWindow)