set nd to nextnode.

print "Node in: " + round(nd:eta) + ", DeltaV: " + round(nd:deltav:mag).

LIST ENGINES IN engines.
FOR eng IN engines {
  set eng:thrustlimit to 100.
}

set max_acc to ship:availablethrust/ship:mass.

set burn_duration to nd:deltav:mag/max_acc.
until burn_duration > 5 {
  LIST ENGINES IN engines.
  FOR eng IN engines {
    set eng:thrustlimit to eng:thrustlimit *.9.
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
until done
{

    set max_acc to ship:availablethrust/ship:mass.


    set tset to min(nd:deltav:mag/max_acc, 1).


    if vdot(dv0, nd:deltav) < 0
    {
        print "End burn, remain dv " + round(nd:deltav:mag,1) + "m/s, vdot: " + round(vdot(dv0, nd:deltav),1).
        lock throttle to 0.
        break.
    }


    if nd:deltav:mag < 0.1
    {
        print "Finalizing burn, remain dv " + nd:deltav:mag + "m/s, vdot: " + vdot(dv0, nd:deltav).

        wait until vdot(dv0, nd:deltav) < 0.0000000000005.

        lock throttle to 0.
        print "End burn, remain dv " + nd:deltav:mag + "m/s, vdot: " + vdot(dv0, nd:deltav).
        set done to True.
    }
}
unlock steering.
unlock throttle.
wait 1.


remove nd.


SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.