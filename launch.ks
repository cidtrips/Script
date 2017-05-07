// launch initiation script

runoncepath("0:/flightlib.ks").

set ship:control:pilotmainthrottle to 0.

parameter default is 3.
parameter direction is 90.
parameter insertTarget is 80000.
parameter apTarget is 80000.
parameter countDownBool is true.
parameter gravityTurnBool is true.
parameter insertOrbitBool is true.
parameter countDownLength is round(random()*10).
if default = 3 {
  print " default: if not specified, prints this message" .
  print " [direction]: compass heading of launch direction".
  print " [insertTarget]: Apoapsis target for orbit insertion".
  print " [apTarget]: Apoapsis for target orbit".
  print " [countDownBool]: If true, countdown... sortof".
  print " [gravityTurnBool]: If true, do gravity turn".
  print " [insertOrbitBool]: If true, insert into orbit".
  print " [countDownLength]: 0-10 second countdown lenght. Kerbals don't pay this much mind".
} else {
  if countDownBool { 
    CountDown(countDownLength).
  }
  if gravityTurnBool {
    GravityTurn(direction, insertTarget).
  }
  if insertOrbitBool { 
    InsertOrbit(insertTarget, apTarget, direction).
  }
}
