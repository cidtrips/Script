parameter orb.


clearscreen.

lock TTW to (maxthrust+0.1)/mass.

set mode to 2.

if ALT:RADAR < 50 { set mode to 1. }
when SHIP:periapsis > 70000 then { set mode to 4. }


when SHIP:ALTITUDE > 9000 then { set mode to 3. }
when (ETA:APOAPSIS > 90) and (apoapsis > orb) and (ETA:PERIAPSIS > ETA:APOAPSIS) then { set mode to 4. preserve.}
until mode = 0 { 
  
  if mode = 1 { 
    
    // launch
    

    print "T-MINUS 10 seconds".

    lock steerin to up.
    wait 1.



    print "T-MINUS  9 seconds".
    lock throttle to 1.
    wait 1.

    print "T-MINUS  8 seconds".
    wait 1.

    print "T-MINUS  7 seco...".
    stage.
    wait 1.

    print "......and here we GO, i guess".
    wait 2.

    clearscreen.
    set mode to 2.
  }

  else if mode = 2 { // fly up to 9km
    lock steering to heading(90,90).
  }

  else if mode = 3{ 
    // gravity turn
    set targetPitch to max( 8, 90 * (1 - ALT:RADAR / 70000)). 
    lock steering to heading (90, targetPitch).

    if SHIP:APOAPSIS > orb{
        set mode to 4.
        }
    if TTW > 20{
        lock throttle to 20*mass/(maxthrust+0.1).
    }

  }

  else if mode = 4{ 
    // coast to orbit
    if (ship:maxthrust = 0) {
      unlock steering.
    }
    lock throttle to 0.
    if (SHIP:ALTITUDE > 70000) and (ETA:APOAPSIS > 70) and (VERTICALSPEED > 0) {
        if WARP = 0 {        
            wait 1.        
            SET WARP TO 3. 
            }
        }
    else if ETA:APOAPSIS < 70 {
        SET WARP to 0.
        lock steering to heading(90,0).
        wait 2.
        set mode to 5.
        }

    if (periapsis > 70000) and mode = 4{
     if WARP = 0 {        
            wait 1.         
            SET WARP TO 3. 
      }
    }

}

else if mode = 5 {
    if ETA:APOAPSIS < 15 or VERTICALSPEED < 0 {
      lock throttle to 1.
    }
    if (eta:periapsis - eta:apoapsis < 0) {
      
      lock steering to heading(90, 30).

    } else {

      lock steering to heading(90, 0).

    }

    if ship:periapsis > orb {
      lock throttle to 0.
      set mode to 6.
    }
  }

  else if mode = 6 {
    lock throttle to 0.
    panels on.     //Deploy solar panels
    lights on.
    unlock steering.
    //set mode to 0.
    print "WELCOME TO A STABE SPACE ORBIT!".
    wait 2.
  }

  // this is the staging code to work with all rockets //

  if stage:number > 0 {
    if maxthrust = 0 {
        stage.
    }
    SET numOut to 0.
    LIST ENGINES IN engines. 
    FOR eng IN engines 
    {
        IF eng:FLAMEOUT 
        {
            SET numOut TO numOut + 1.
        }
    }
    if numOut > 0 { stage. }.
  }


  // HERE is the code for the control pannel //

  print "LAUNCH PLAN STAGE " + mode at (0, 0).
print " " at (0, 1).

  print "Periapsis height: " + round(periapsis, 2) + " m" at (0, 2).
  
print " Apoapsis height: " + round(apoapsis, 2) + " m" at (0, 3).

  print " ETA to Apoapsis: " + round(ETA:APOAPSIS) + " s" at (0, 4).
  print "   Orbital speed: " + round(velocity:orbit:MAG, 2)+ " m/s" at (0, 5).
  
print "        altitude: " + round(altitude, 2) + " m" at (0, 6).

  print "thrust to weight: " + round((throttle*maxthrust)/mass) at (0, 7).
  
print " " at (0, 8).
print "Currently on Stage: " + stage:number at (0, 9).
wait 0.2.

}