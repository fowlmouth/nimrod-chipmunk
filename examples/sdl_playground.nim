import 
  fowltek/sdl2/engine,
  chipmunk, math, basic2d
import_all_sdl2_things
randomize()

var
  NG = newSdlEngine(caption = "Chipmunk playground")
  space = newSpace()
  running = true
let
  winSize = NG.window.getSize
  
space.setGravity newVector(0,9.8)

proc addCircle (
    pos = newVector(winSize.x.random.cpfloat, winSize.y.random.cpfloat);
    mass = 0.9;
    radius = 10.0 ) =
  var 
    body = space.addBody(newBody(mass, momentForCircle(mass, 0.0, radius, vectorZero)))
    shape = space.addShape(
      newCircleShape(body, radius, vectorZero))
  body.setPos pos
proc addBox (
    pos = newVector(winSize.x.random.cpfloat, winSize.y.random.cpfloat);
    mass = 0.9;
    width = 10.0;
    height = 5.0 ) =
  var
    body = space.addBody(newBody(mass, momentForBox(mass, width, height)))
    shape = space.addShape(
      newBoxShape(body, width, height))
  body.p = pos


iterator times (n: int): int =
  for i in 1 .. n: yield n

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

NG.addHandler do(E: PSdlEngine) -> bool:
  result = E.evt.kind == QuitEvent
  running = not result

var 
  mouseBody = space.addBody(newBody(1.0, 1.0))
  mouseJoint: PConstraint

import typeinfo, strutils
proc ff (f: float; prec = 2; fmt = ffDecimal):string{.inline.} =
  formatFloat(f, fmt, prec)


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
    

NG.addHandler do(E: PSdlEngine) -> bool:
  if E.evt.kind in { MouseButtonDown, MouseButtonUp }:
    let m = E.evt.evMOuseButton
    if m.button == BUTTON_LEFT:
      result = true
      if m.state == 1:
          let mousePos = vector(m.x.cpfloat, m.y.cpfloat)
          var closestS = space.pointQueryFirst(mousePos, 0, 0)
          if not closestS.isNil:
            mouseJoint = newPivotJoint(mouseBody, closestS.body, vectorZero,
              closestS.body.world2local(mousePos))
            mouseJoint.maxForce = 50_000
            #mouseJoint.biasCoef = 0.15
            mouseJoint.errorBias = 0.15
            discard space.addConstraint(mouseJoint)
          else:
            echo "No object found at $1,$2".format(mousePos.x.ff, mousepos.y.ff)
      elif not mouseJoint.isNil:
        # release
        space.removeConstraint mouseJoint
        mouseJoint.free
        mouseJoint.reset
    else:
      echo "wrong mouse button! ", m.button
      echo byterepr(m[])
      #echo repr(m[])
      #echo repr(cast[ptr array[100, byte]](m))

proc drawShape (shape: PShape; data: pointer){.cdecl.} =
  case shape.klass.kind
  of CP_CIRCLE_SHAPE:
    let 
      r = shape.getCircleRadius
      b = shape.getBody
      a = b.getAngle
      p2 = vector2d(cos(a), sin(a)) * r.float
    
    NG.circleRGBA b.p.x.int16, b.p.y.int16, r.int16,
      0,255,0,255 
    NG.lineRGBA b.p.x.int16, b.p.y.int16, (b.p.x + p2.x).int16, (b.p.y + p2.y).int16,
      0,255,0,255

  of CP_SEGMENT_SHAPE:
    let
      a = shape.getSegmentA
      b = shape.getSegmentB
    
    NG.lineRGBA a.x.int16, a.y.int16, b.x.int16, b.y.int16,
      0,0,255,255
  
  of CP_POLY_SHAPE:
    let 
      nverts = shape.getNumVerts
      b = shape.getBody
      angle = b.getAngle
    
    proc rotate (vec: TVector; rad: float): TVector2d {.inline.} =
      result = TVector2d(x: vec.x, y: vec.y)
      result.rotate rad
    proc `+` (a: TVector2d; b: TVector): TVector2d {.inline.} =
      TVector2d(x: a.x + b.x, y: a.y + b.y)
    
    var 
      lastVert = shape.getVert(< nverts).rotate(angle) + b.p
    
    for i in 0 .. <nverts:
      let thisV = shape.getVert(i).rotate(angle) + b.p
      NG.lineRGBA lastVert.x.int16, lastVert.y.int16, thisV.x.int16, thisV.y.int16,
        0,0,255,255
      lastVert = thisV
  else:
    echo "Undrawn shape ", shape.klass.kind

var mousePos: tuple[x,y: cint]

while running:
  NG.handleEvents
  discard getMouseState(mousePos.x, mousePos.y)
  mouseBody.p = vector(mousePos.x.cpfloat, mousePos.y.cpfloat)
  
  space.step NG.frameDeltaFLT
  
  NG.setDrawColor 0,0,0,255
  NG.clear
  
  space.eachShape drawShape, nil
  
  NG.present
