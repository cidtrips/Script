parameter peTarget.

set myNode to NODE(TIME:SECONDS + ETA:APOAPSIS, 0, 0, 0).

ADD myNode.

SET Kp to 0.0000001.
SET Ki to 0.0000000006.
SET Kd to 0.0000000006.

SET PID to PIDLOOP(Kp, Ki, Kd, -1, 1).
SET PID:SETPOINT to peTarget.

until (abs(myNode:orbit:periapsis - peTarget) < .005) {
  SET myNode:prograde to myNode:prograde + PID:UPDATE(TIME:SECONDS, myNode:orbit:periapsis).
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

