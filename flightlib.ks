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
    if numOut > 0 { wait .5. stage. wait .5. }.
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

function Intercept {
  lock d to target:velocity:orbit - velocity:orbit.
  lock dv to d:mag.
  lock n to -1 * target:direction:vector.
  lock a to vectorangle (d, n).
  lock r to 3 * (d:normalized - n:normalized) + d:normalized.
  
  lock steering to r.
  set ao to a.
  
  until a > ao {
    set tb to min(.001, d:mag * mass / (maxthrust * 2)).
	set ti to target:distance / d:mag.
    PrintIntercept(a, vectorangle(d, r), target:distance, ti, tb, (target:distance * sin(vectorangle(d, n)))). 
	CheckStage.
  }
  lock timp to target:distance / min(0.001, d:mag).
  lock t to (mass * dv) / (maxthrust * 1.9).
  
  until timp < t {
    set u to vectorangle (d:normalized, n:normalized).
	set m to u * 6.
	if u > 9.2 {
	  set m to (90 / u).
	}
	
    set th to (vectorangle(d:normalized, n:normalized) - 1) / 4.
    PrintIntercept(vectorangle(u, vectorangle(d:normalized, r), target:distance, timp, t, (target:distance * sin(u)))).	
  }
  
}

function PrintIntercept {
  parameter angle.
  parameter tAngle.
  parameter distance.
  parameter eta.
  parameter burnTime.
  parameter closestApproach.
  
  print "Angle:         " + round(angle, 3) at (0, 1).
  print "Thrust Angle:  " + round(tAngle, 3) at (0, 2).
  print "Distance:      " + round(distance) at (0, 3).
  print "ETA:           " + round(eta) at (0, 4).
  print "Burn Time:     " + round(burnTime) at (0, 5).
  print "Closest Appr:  " + round(closestApproach) at (0, 6).
}

function GravityTurn {
  parameter direction is 90.
  parameter apTarget is 80000.
    
  clearscreen.
  lock distance to body:radius + ship:altitude.
  lock weight to ship:mass * body:mu / (distance)^2.
  //lock throttle to (1.2 * weight) / ship:maxthrustat(ship:sensors:pres).
    
  lock steering to mysteer.

  until ship:apoapsis > apTarget {
    set inclination to max( 5, 90 * (1 - ALT:RADAR / 50000)).
    set mysteer to lookdirup(heading(direction, inclination):vector, ship:facing:topvector).
    print "Apoapsis: " + round(ship:apoapsis, 0) at (0, 3).
    print (1.2 * weight) / ship:availablethrust at (0, 5).

    CheckStage.
    lock throttle to min(1, ship:mass * 2.5 * body:mu / (distance)^2 / ship:availablethrust).
  }
  lock throttle to 0.
}

function NewFunction {
  parameter orbitAlt.
  
  local th is 0.
  local st is heading(0, 0).
  
  lock throttle to th.
  
  lock steering to prograde.
  
  until (ship:altitude > ship:orbit:body:atm:height) {
    // Keep our Apoapsis at setpoint if we are below atm...
	if ship:apoapsis < orbitAlt {
	  set th to 1.
	} else {
	  set th to 0.
	}
  }
  
  set dv to sqrt(Mu()*((2/ship:apoapsis) - (1 / ((ship:apoapsis + ship:orbit:body:radius) / 2)))).
  set burnStats to half_dv_duration(dv).
  set firstHalfDvDuration to burnStats[0].
  set burnDuration to firstHalfDvDuration + burnStats[1].
  
  lock steering to prograde.
  
  wait until eta:apoapsis - firstHalfDvDuration <= 0.
  local startTime is TIME:SECONDS.
  local done is false.
  until done {
	  CheckStage.
	  if (TIME:SECONDS - startTime > burnDuration) { set done to true.}
	  wait 0.
  }
}

function InsertOrbit {
  parameter insertAltitude.
  parameter orbitAltitude.
  parameter direction.

  set st to heading(direction, 0).
  set th to 0.
  
  lock steering to st.
  lock throttle to th.
  lock steering to lookdirup(st:vector, ship:facing:topvector).
  until (ship:altitude > ship:orbit:body:atm:height) {
    // Keep our Apoapsis at setpoint if we are below atm...
	if ship:apoapsis < insertAltitude {
	  set th to 1.
	} else {
	  set th to 0.
	}
  }
  
  set Kp to 0.1.
  set Ki to 0.06.
  set Kd to 0.06.
  
  set PID to PIDLOOP(Kp, Ki, Kd, 0, 1).
  set PID:SETPOINT to 30.
  
  lock steering to lookdirup(prograde:vector, ship:facing:topvector).
  
  until insertAltitude - periapsis < 1 { 
    set th to PID:UPDATE(TIME:SECONDS, eta:apoapsis).
    wait 0.1.
    CheckStage.
  }
  
  set th to 0.
  
  unlock steering.

  if orbitAltitude = insertAltitude { unlock steering. unlock throttle. return. }

  wait until eta:periapsis < 60.
  
  lock steering to lookdirup(prograde:vector, ship:facing:topvector).
  wait until vang(facing:vector, steering:vector) < 1.
  until orbitAltitude - apoapsis < 1 { 
    set th to PID:UPDATE(TIME:SECONDS, eta:periapsis).
    wait 0.1.
  } 
  unlock steering.
  unlock throttle.
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

  lock steering to lookdirup(nn:deltav, ship:facing:topvector). //points to node, keeping roll the same.

  local burn_stats is half_dv_duration(nn:deltav:mag).
  local first_half_duration is burn_stats[0].
  local burnDuration is first_half_duration + burn_stats[1].

  set kuniverse:timewarp:warp to 0.

  set node_time to time:seconds + nn:eta.
  set warp_target to node_time - 15 - first_half_duration.

  wait until vang(facing:vector, steering:vector) < 1 or time:seconds >= warp_target.

  HUDTEXT("Estimated burn duration: " + round(burnDuration,1) + "s", 15, 2, 20, yellow, false).
		
  if warp_target > time:seconds {
	   set kuniverse:timewarp:mode to "rails".
	   wait 0.
	   kuniverse:timewarp:warpto(warp_target).
  }
  lock steering to lookdirup(nn:deltav, ship:facing:topvector). //points to 
  wait until vang(facing:vector, steering:vector) < 1. 

  wait until nn:eta - first_half_duration <= 0. //wait until we are close to executing the node
  set kuniverse:timewarp:mode to "physics". //se we can manually physics warp during a burn

  HUDTEXT("Begin burn. Physics warp is possible.", 5, 2, 20, yellow, false).

  set dv0 to nn:deltav.

  local done is false.
  
  set th to 0.
  lock throttle to th.
  
  until done {
	   set max_acc to ship:availablethrust/ship:mass.
	   if nn:deltav:mag/(max_acc*10) < 1 set warp to 0. //warp
	
	   if vang(facing:vector, steering:vector) > 1 { set th to 0. }
	   else { set th to min(nn:deltav:mag/(max_acc*1.2), 1). }
	
	  CheckStage.
	
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

  local e is constant():e.
	
  local g0 is body("Kerbin"):mu / body("Kerbin"):radius ^ 2. 

  local ISP is simple_isp().
	
  // mass after burn is done
  local m1 is m0*e^(-delta_v_mag / (g0*ISP)).
	
  // From rocket equation, and definition of ISP:
  local burn_dur is (g0*ISP*m0/SHIP:AVAILABLETHRUST)*( 1 - e^(-delta_v_mag/(g0*ISP)) ).
	
  return list(burn_dur,m1).
}

function WaitWindow {
  parameter myTarget is target.
  
  local torb is 3*60.
  
  local pos is positionat( myTarget, time + torb).
  
  local upv is up:vector.
  
  clearscreen.
  
  until (vectorangle (pos, upv) < 15) and vectorangle (pos, upv) > 0 {
    set pos to positionat( myTarget, time + torb).
	set upv to up:vector.
	
	print vectorangle ( pos, upv) at (0, 3).
  }
}

function mytwr {
  declare local thrust is 0.
  delcare local mass is ship:mass.
  declare local g is ship:orbit:body:mu / (ship:orbit:body:radius + ship:altitude)^2.
  
  list engines in eng.
  
  for e in eng {
    set thrust to thrust + e:availablethrust.
  }
  
  return twr(thrust, mass, g).
}

function twr {
  parameter thrust.
  parameter mass.
  parameter g.
  
  return thrust / (mass * g).
}

function simple_isp {
  list engines in engs.
  local totalFlow is 0.
  local totalThrust is 0.
  list activeEngines.
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
