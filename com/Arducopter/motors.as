﻿/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// 10 = 1 second
//#define ARM_DELAY 20
//#define DISARM_DELAY 20
//#define AUTO_TRIM_DELAY 100

public const ARM_DELAY			:int = 20;
public const DISARM_DELAY		:int = 20;
public const AUTO_TRIM_DELAY	:int = 100;

public var arming_counter:int = 0;

public function arm_motors():void
{
    //static int16_t arming_counter;

    // don't allow arming/disarming in anything but manual
    if (g.rc_3.control_in > 0) {
        arming_counter = 0;
        return;
    }

    if ((control_mode > ACRO) && ((control_mode != TOY_A) && (control_mode != TOY_M))) {
        arming_counter = 0;
        return;
    }

    var tmp:int = g.rc_4.control_in;

    // full right
    if (tmp > 4000) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if( arming_counter <= AUTO_TRIM_DELAY ) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !motors.armed()) {
            init_arm_motors();
        }

        // arm the motors and configure for flight
        if (arming_counter == AUTO_TRIM_DELAY && motors.armed()) {
            auto_trim_counter = 250;
        }

    // full left
    }else if (tmp < -4000) {

        // increase the counter to a maximum of 1 beyond the disarm delay
        if( arming_counter <= DISARM_DELAY ) {
            arming_counter++;
        }

        // disarm the motors
        if (arming_counter == DISARM_DELAY && motors.armed()) {
            init_disarm_motors();
        }

    // Yaw is centered so reset arming counter
    }else{
        arming_counter = 0;
    }
}

public var did_ground_start:Boolean = false;
public function init_arm_motors():void
{
	// arming marker
    // Flag used to track if we have armed the motors the first time.
    // This is used to decide if we should run the ground_start routine
    // which calibrates the IMU
    //static bool did_ground_start = false;

	// Remember Orientation
	// --------------------
	init_simple_bearing();

    // Reset home position
    // -------------------
    if(ap.home_is_set)
        init_home();

	//calibrate_accels();

    // all I terms are invalid
    // -----------------------
    reset_I_all();

    if(did_ground_start == false) {
        did_ground_start = true;
        startup_ground();
    }

    // read Baro pressure at ground -
    // this resets Baro for more accuracy
    //-----------------------------------
    init_barometer();

    // finally actually arm the motors
    motors.arm(true);
    set_armed(true);

	update_arm_label();
}

public function init_disarm_motors():void
{
	trace("Disarm Motors");
    motors.arm(false);
    set_armed(false);

    motors.auto_arm(false);
    set_auto_armed(false);

    // we are not in the air
    set_takeoff_complete(false);

	//stopSIM();
	motors.arm(false);
	update_arm_label();
	//g.throttle_cruise.save();
	set_mode(STABILIZE);
}


public function set_servos_4()
{
	motors.output();
}

