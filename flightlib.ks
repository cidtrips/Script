 set ship:control:pilotmainthrottle to 0.


function Launch {
  parameter countDownBool is true.
  parameter countDownLength is round(random()*10).
  parameter gravityTurnBool is true.
  parameter speedInc is 125.
  parameter circularizeBool is true. 
  parameter apTarget is 80000.
  parameter ETATarget is 30.

  if countDownBool { 
    CountDown(countDownLength).
  }

  if gravityTurnBool {
    GravityTurn(apTarget).
  }

  if circularizeBool { 
    CircularizeOrbit(1, apTarget).
  }

}

function CheckStage { 
  if stage:number > 0 {
    if maxthrust = 0 {
        stage.
    }
    SET numOut to 0.
    LIST ENGINES IN engines. 
    FOR eng IN engines {
      IF eng:FLAMEOUT {
        SET numOut TO numOut + 1.
      }
    }
    if numOut > 0 { wait 1. stage. wait .5. }.
  }
}

function CountDown { 
  parameter countdownLength.

  clearscreen.
  print "Counting down: ".
  lock throttle to 1.
  from { local countdowntime is 10. } until countdowntime = 10 - countdownLength step { set countdowntime to countdowntime - 1.} do {
    print "T-MINUS " + countdowntime:tostring:padleft(2) + " seconds".
    wait 1.
  }
  if (countdownLength = 10) {
    print "Blastoff!".
    stage.
    wait 1.
  } else {
    print "T-MINUS ... um".


    stage.
    wait 1.
    print "blastoff?".
    wait 1.
  }
}

function GravityTurn {
  parameter apTarget.
  lock steering to mysteer.
  until ship:apoapsis > apTarget {
    set inclination to max( 5, 90 * (1 - ALT:RADAR / 50000)).
    set mysteer to lookdirup(heading(90, inclination):vector, ship:facing:topvector).
    print "Apoapsis: " + round(ship:apoapsis, 0) at (0, 3).
    CheckStage.
  }
  lock throttle to 0.
}

function CircularizeOrbit { 
  parameter direction. // 1 ascend, 0 descend
  parameter target.

  wait until ship:altitude > ship:orbit:body:atm:height.

  addApNode(ship:apoapsis, 10).
  execNode.
}

function execNode {
  set nd to nextnode.
  set deviation to 0.5.

  print "Node in: " + round(nd:eta) + ", DeltaV: " + round(nd:deltav:mag).


  set max_acc to ship:availablethrust/ship:mass.

  set burn_duration to nd:deltav:mag/max_acc.

  until burn_duration > 5 {
    LIST ENGINES IN engines.
    FOR eng IN engines {
      set eng:thrustlimit to eng:thrustlimit *.9.
      set deviation to 0.5 * eng:thrustlimit.
    }
    set max_acc to ship:availablethrust/ship:mass.
    set burn_duration to nd:deltav:mag/max_acc.
  }
  

print "Crude Estimated burn duration: " + round(burn_duration) + "s".
  wait until nd:eta <= (burn_duration/2 + 60).
  set np to nd:deltav. //points to node, don't care about the roll direction.
  lock steering to np.


  wait until abs(np:direction:pitch - ship:facing:pitch) < 0.15 and abs(np:direction:yaw - ship:facing:yaw) < 0.15.


  wait until nd:eta <= (burn_duration/2).

  set tset to 0.
  lock throttle to tset.

  set done to False.

  set dv0 to nd:deltav.
  until done {

    set max_acc to ship:availablethrust/ship:mass.


    set tset to min(nd:deltav:mag/max_acc, 1).


    if vdot(dv0, nd:deltav) < 0 {
      print "End burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
      lock throttle to 0.
      break.
    }


    if nd:deltav:mag < 0.1 {
      print "Finalizing burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).

      wait until vdot(dv0, nd:deltav) < deviation.

      lock throttle to 0.
      print "End burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
      set done to True.
    }
  }
  unlock steering.
  unlock throttle.
  wait 1.

  remove nd.
 

  SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
}

function addPeNode {
  parameter apTarget.


  parameter maxError is 0.005.
  set myNode to NODE(TIME:SECONDS + ETA:PERIAPSIS, 0, 0, 0).
  

ADD myNode.

  

SET Kp to 0.0000001.
  
SET Ki to 0.0000000006.
  
SET Kd to 0.0000000006.
  

SET PID to PIDLOOP(Kp, Ki, Kd, -100, 100).
  
SET PID:SETPOINT to apTarget.


  until (abs(myNode:orbit:apoapsis - apTarget) < maxError) {

    SET myNode:prograde to myNode:prograde + PID:UPDATE(TIME:SECONDS, myNode:orbit:apoapsis).
    if PID:ERROR > PID:KP * 1000 {
      SET PID:KP to PID:KP * 10.
      SET PID:KI to PID:KI * 10.
      SET PID:KD to PID:KD * 10.
    }
  
  WAIT 0.001.

    clearscreen.

    print "Target: " + apTarget at(0, 0).
  
  print "Current: " + myNode:orbit:apoapsis at(0, 1).
  
  print "DeltaV: " + myNode:prograde at (0, 2).
  
  print "PID Setpoint: " + PID:SETPOINT at(0, 3).
  
  print "PID Error: " + PID:ERROR at(0, 4).

    print "PID Output: " + PID:OUTPUT at(0, 5).
  
  print "PID Pterm: " + PID:PTERM at(0, 6).
  
  print "PID Iterm: " + PID:ITERM at(0, 7).
  
  print "PID Dterm: " + PID:DTERM at(0, 8).

  }


}

function addApNode {

  parameter peTarget.
  parameter maxError is 0.005.
  

set myNode to NODE(TIME:SECONDS + ETA:APOAPSIS, 0, 0, 0).
  

ADD myNode.



  SET Kp to 0.0000001.
  
SET Ki to 0.0000000006.
  
SET Kd to 0.0000000006.
  

SET PID to PIDLOOP(Kp, Ki, Kd, -100, 100).
  
SET PID:SETPOINT to peTarget.
  

until (abs(myNode:orbit:periapsis - peTarget) < maxError) {
  
    SET myNode:prograde to myNode:prograde + PID:UPDATE(TIME:SECONDS, myNode:orbit:periapsis).
    if PID:ERROR > PID:KP * 1000 {
      SET PID:KP to PID:KP * 10.
      SET PID:KI to PID:KI * 10.
      SET PID:KD to PID:KD * 10.
    }
  
  WAIT 0.001.
  
  clearscreen.
  
  print "Target: " + peTarget at(0, 0).
  
  print "Current: " + myNode:orbit:periapsis at(0, 1).
  
  print "DeltaV: " + myNode:prograde at (0, 2).
  
  print "PID Setpoint: " + PID:SETPOINT at(0, 3).
  
  print "PID Error: " + PID:ERROR at(0, 4).
  
  print "PID Output: " + PID:OUTPUT at(0, 5).
  
  print "PID Pterm: " + PID:PTERM at(0, 6).

    print "PID Iterm: " + PID:ITERM at(0, 7).
  
  print "PID Dterm: " + PID:DTERM at(0, 8).

  }
}

function ChangeOrbit { 
  parameter newAltitude.
  
  if newAltitude > ship:altitude { 
    Ascend(newAltitude).
    CircularizeOrbit(1, newAltitude, 30, false).
  } else {
    Descend(newAltitude).
    CircularizeOrbit(0, newAltitude, 30, false).
  }
}

function Ascend { 
  parameter apTarget.
  parameter ETATarget is 30.

  clearscreen.
  lock steering to prograde.
  until eta:periapsis <= ETATarget { 
    set paETA to eta:periapsis - ETATarget.
    print "Ascending in " + round(paETA):tostring:padleft(6) + "s" at (0, 1).
  }
  
  set Kp to 0.1.
  set Ki to 0.06.
  set Kd to 0.06.

  set PID to PIDLOOP(Kp, Ki, Kd, 0, 1).
  set PID:SETPOINT to ETATarget.

  set thrott to 1.
  lock throttle to thrott.
  print "Setting Apoapsis".
  until apTarget - apoapsis < 1 { 
    set thrott to PID:UPDATE(TIME:SECONDS, eta:periapsis).
    wait 0.001.
  }
  lock throttle to 0.
}

function Descend { 
  parameter paTarget.
  parameter ETATarget is 30.

  clearscreen.
  lock steering to retrograde.
  until eta:apoapsis <= ETATarget { 
    set paETA to eta:apoapsis - ETATarget.
    print "Descending in " + round(paETA):tostring:padleft(6) + "s" at (0, 1).
  }
  
  set Kp to 0.1.
  set Ki to 0.06.
  set Kd to 0.06.

  set PID to PIDLOOP(Kp, Ki, Kd, 0, 1).
  set PID:SETPOINT to ETATarget.

  set thrott to 1.
  lock throttle to thrott.
  print "Descending".
  until abs(periapsis - paTarget)  < 1 { 
    set thrott to PID:UPDATE(TIME:SECONDS, eta:apoapsis).
    wait 0.001.
  }
  lock throttle to 0.
}

function Mu {
  parameter body.
  return constant:G * body:Mass.
}

print "FlightLib Loaded".