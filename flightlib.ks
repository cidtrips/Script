set ship:control:pilotmainthrottle to 0.
local e is constant():e.

function Launch {
  parameter countDownBool is true.
  parameter countDownLength is round(random()*10).
  parameter gravityTurnBool is true.
  parameter circularizeBool is true. 
  parameter apTarget is 80000.

  if countDownBool { 
    CountDown(countDownLength).
  }

  if gravityTurnBool {
    GravityTurn(direction, apTarget).
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
    print "T-MINUS ... um".
    stage.
    wait 1.
    print "blastoff?".
    wait 1.
  }
}

function GravityTurn {
  parameter apTarget.
  parameter direction is 90.
  lock steering to mysteer.
  until ship:apoapsis > apTarget {
    set inclination to max( 5, 90 * (1 - ALT:RADAR / 50000)).
    set mysteer to lookdirup(heading(direction, inclination):vector, ship:facing:topvector).
    print "Apoapsis: " + round(ship:apoapsis, 0) at (0, 3).
    CheckStage.
  }
  lock throttle to 0.
}

function CircularizeOrbit { 
  parameter direction. // 1 ascend, 0 descend
  parameter target.

  addApNode(ship:apoapsis, 10).
  
  wait until ship:altitude > ship:orbit:body:atm:height.

  execNode.
}

function execNode {
  parameter nn is nextnode.

  runoncepath("lib_rocket_utility.ks").

  sas off.
  set th to 0.
  lock throttle to th.
  lock steering to lookdirup(nn:deltav, ship:facing:topvector). //points to node, keeping roll the same.

  local burn_stats is half_dv_duration(nn:deltav:mag).
  local first_half_duration is burn_stats[0].
  local burn_duration is first_half_duration + burn_stats[1].

  set kuniverse:timewarp:warp to 0.

  set node_time to time:seconds + nn:eta.
  set warp_target to node_time - 15 - first_half_duration.

  wait until vang(facing:vector, steering:vector) < 1 or time:seconds >= warp_target.

  HUDTEXT("Estimated burn duration: " + round(burn_duration,1) + "s", 15, 2, 20, yellow, false).
		
  if warp_target > time:seconds {
	   set kuniverse:timewarp:mode to "rails".
	   wait 0.
	   kuniverse:timewarp:warpto(warp_target).
  }

  wait until nn:eta - first_half_duration <= 0. //wait until we are close to executing the node
  set kuniverse:timewarp:mode to "physics". //se we can manually physics warp during a burn

  HUDTEXT("Begin burn. Physics warp is possible.", 5, 2, 20, yellow, false).

  set dv0 to nn:deltav.

  local done is false.
  until done {
	   set max_acc to ship:availablethrust/ship:mass.
	   if nn:deltav:mag/(max_acc*10) < 1 set warp to 0. //warp
	
	   if vang(facing:vector, steering:vector) > 1 { set th to 0. }
	   else { set th to min(nn:deltav:mag/(max_acc*1.2), 1). }
	
	  LIST engines IN engs.
	  for eng in engs { if eng:ignition = true and eng:flameout = true and stage:ready { stage. } }
	
	  if nn:deltav:mag < 0.05 set done to true.
	  wait 0.
 }

 HUDTEXT("Manouver mode has been executed!", 4, 2, 30, yellow, false).

 set kuniverse:timewarp:mode to "rails".
 unlock steering.
 set th to 0.
 unlock throttle.
 set ship:control:pilotmainthrottle to 0.
 remove nn.
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
  parameter body is ship:orbit:body.
  return constant:G * body:Mass.
}

// Thanks to Dunbaratu for the two following functions!
function burn_duration {
	parameter delta_v_mag, m0 is ship:mass. 
	
	local g0 is Mu() / ((ship:orbit:body:radius + ship:altitude) ^ 2). 
  
	// The ISP of first engine found active:
	// (For more accuracy with multiple differing engines,
	// some kind of weighted average would be needed.)
	local ISP is simple_isp().
	
	// mass after burn is done
	local m1 is m0*e^(-delta_v_mag / (g0*ISP)).
	
	// From rocket equation, and definition of ISP:
	local burn_dur is (g0*ISP*m0/SHIP:AVAILABLETHRUST)*( 1 - e^(-delta_v_mag/(g0*ISP)) ).
	
	return list(burn_dur,m1).
}

function simple_isp {
	list engines in engs.
	local totalFlow is 0.
	local totalThrust is 0.
	for eng in engs {
		if eng:ignition and not eng:flameout {
			set totalflow to totalflow + (eng:availablethrust / eng:isp).
			set totalthrust to totalthrust + eng:availablethrust.
		}
	}
	return totalthrust / max(0.1, totalflow).
}

function half_dv_duration {
	parameter deltav_mag.
	
	local first_half is burn_duration(deltav_mag / 2).
	local first_half_duration is first_half[0].
	
	// the duration of the second half of the burn, with the adjusted starting mass.
	local second_half is burn_duration(deltav_mag / 2, first_half[1]).
	
	
	// return list with: first half of deltaV duration, last half of dV duration, mass after full burn.
	return list(first_half_duration,second_half[0],second_half[1]).
}

print "FlightLib Loaded".
