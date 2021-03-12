@LAZYGLOBAL OFF.


// Helper: calculates burn time for given dV, based on current enabled engines.
// Will break if enabled engines have different specific impulse (ISP)
// Original author: gisikw
// https://old.reddit.com/r/Kos/comments/3ftcwk/compute_burn_time_with_calculus/
local function maneuver_time {
  parameter dv.

  local en is list().
  list engines in en.

  local f is ship:maxthrust * 1000.   // Engine Thrust (kg * m/s^2)
  local m is ship:mass * 1000.        // Starting mass (kg)
  local e is constant:e.              // Base of natural log
  local p is en[0]:isp.               // Engine ISP (s)
  local g is constant:g0.             // Gravitational acceleration constant (m/s^2)

  return g * m * p * (1 - e^(-dV/(g*p))) / f.
}


clearscreen.
print("Starting node.ks").
print("Author: greg2010").

if (not hasnode) {
    print("ERROR: no next node set.").
    print("Exiting...").
} else {
    // PRINT CONSTANTS
    local preburn_print_offset is 9.
    local burn_print_offset is 13.
    local postburn_print_offset is 15.
    // PRINT CONSTANTS END

    // BURN CONSTANTS
    local cut_early_sec is 0.4.
    local div_factor is 4.
    // BURN CONSTANTS END

    // PREPARE CONTROLS
    local prev_sas is sas.
    sas off.
    local tset is 0.
    lock throttle to tset.
    // Ignore roll
    lock steering to lookdirup(nextnode:deltav, ship:facing:topvector).
    // PREPARE CONTROLS END

    // Precalculate burn time
    local burn_time is maneuver_time(nextnode:deltav:mag).

    // Print maneuver information
    print("Executing next node.").
    print("Node in: " + round(nextnode:eta) + "s, DeltaV: " + round(nextnode:deltav:mag) + "m/s").
    print("Estimated burn time: " + burn_time + "s").


    // Warp to T-60
    print("Warping to node.").
    kuniverse:timewarp:warpto(nextnode:time - 60).
    wait until nextnode:eta <= 60.


    // PREBURN
    print("Waiting to start the burn.").
    print("").

    until  nextnode:eta <= 5 {
        local momentum is ship:angularmomentum:vec:mag.
        local angle is vectorangle(ship:facing:vector, nextnode:deltav).
        print("Momentum is: " + momentum) at(0, preburn_print_offset).
        print("Angle is: " + angle) at(0, preburn_print_offset + 1).

        if momentum <= 0.05 and angle < 1 {
            kuniverse:timewarp:warpto(nextnode:time - 5).
        } else if momentum > 0.05 {
            print("Cannot warp.") at(0, preburn_print_offset + 2).
        }
    }
    wait until nextnode:eta <= 0.

    // PREBURN END

    // MAIN BURN
    local finish_burn_t is nextnode:time + burn_time - cut_early_sec.
    // Perform the bulk of the burn at max throttle
    set tset to 1.
    until time:seconds > finish_burn_t {
        print("Burning main: " + (finish_burn_t - time:seconds) + " left.") at(0, burn_print_offset).
    }

    // At this point our node is about to disappear.
    local cur_node is nextnode.
    lock steering to lookdirup(cur_node:deltav, ship:facing:topvector).

    // Perform the finisher of the burn at 1/div_factor throttle
    set finish_burn_t to finish_burn_t + cut_early_sec * div_factor.
    set tset to 1/div_factor.
    until time:seconds > finish_burn_t {
        print("Burning finisher: " + (finish_burn_t - time:seconds) + " left.") at(0, burn_print_offset + 1).
    }
    // MAIN BURN END

    // POSTBURN
    set tset to 0.
    print("Maneuver has been executed. Relinquishing control.") at (0, postburn_print_offset).
    unlock steering.
    unlock throttle.

    // Restore SAS
    if (prev_sas) sas on.
    else sas off.

    //set throttle to 0 just in case.
    set ship:control:pilotmainthrottle to 0.
    // POSTBURN END
}