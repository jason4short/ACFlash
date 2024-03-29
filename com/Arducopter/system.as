﻿private function init_ardupilot():void
{
	init_wp();

	// radio
	init_rc_in();               // sets up rc channels from radio
	init_rc_out();              // sets up the timer libs

	default_dead_zones();
	// sets throttle to be at hovering setpoint
	//left_sticks.knob_y.y = 0;

	// force sensors to new user defined location of the SIM copter
	barometer.init();
	g_gps.init();

	init_sonar();

    // initialise inertial nav
    trace("inertial_nav.init");
    inertial_nav.init();

	// initialize commands
	// -------------------
	init_commands();

	// set the correct flight mode
	// ---------------------------
	reset_control_switch();

	startup_ground();

	trace("Ready to Fly")
}

private function startup_ground():void
{
	// when we re-calibrate the gyros,
	// all previous I values are invalid
	reset_I_all();
}


private function air_start():void
{
	// do this one time before we begin
	update_sim_radio();
	read_radio();

	init_arm_motors();
	
	// auto Arm
    motors.auto_arm(true);

	// fix filters
	//baro_alt			= copter.position.z;
	update_altitude();

	// for a new home loc (0,0,0)
	// update GPS is missing this code
	init_home();

	// setup our next WP
	var nwp 			= new Location();
	nwp.lng 			= g.target_distance_BI.getNumber();
	nwp.lat 			= 0;
	nwp.alt 			= g.target_altitude_BI.getNumber();


	oldSwitchPosition = -1
	read_control_switch();

	// hack to make the SIM go right into Land
	if(control_mode == LAND)
		do_land();
	
	

	if(copter.position.z > 0){
		// force out decent values to the motors
		APM_RC.OutputCh(MOT_1, g.throttle_cruise + 1000);
		APM_RC.OutputCh(MOT_2, g.throttle_cruise + 1000);
		APM_RC.OutputCh(MOT_3, g.throttle_cruise + 1000);
		APM_RC.OutputCh(MOT_4, g.throttle_cruise + 1000);
		g.rc_3.servo_out = g.throttle_cruise;

	}else{

		APM_RC.OutputCh(MOT_1, 1000);
		APM_RC.OutputCh(MOT_2, 1000);
		APM_RC.OutputCh(MOT_3, 1000);
		APM_RC.OutputCh(MOT_4, 1000);
		g.rc_3.servo_out = 0;
	}
}


// ----------------------------------------
// System.pde
// ----------------------------------------
public function set_mode(mode:int):void
{
	// Switch to stabilize mode if requested mode requires a GPS lock
	if(!ap.home_is_set) {
		if (mode > ALT_HOLD && mode != TOY_A && mode != TOY_M && mode != OF_LOITER && mode != LAND) {
			mode = STABILIZE;
		}
	}

	// Switch to stabilize if OF_LOITER requested but no optical flow sensor
	if (mode == OF_LOITER && !g.optflow_enabled ) {
		mode = STABILIZE;
	}

    control_mode 	= mode;
    control_mode    = constrain(control_mode, 0, NUM_MODES - 1);

	// update pulldown in GUI
	modeMenu.setSelectedIndex(mode);
	modeMenuHandler(null);

    // if we change modes, we must clear landed flag
    set_land_complete(false);

    // report the GPS and Motor arming status
    led_mode = NORMAL_LEDS;

    switch(control_mode)
    {
    case ACRO:
    	ap.manual_throttle = true;
    	ap.manual_attitude = true;
        set_yaw_mode(ACRO_YAW);
        set_roll_pitch_mode(ACRO_RP);
        set_throttle_mode(ACRO_THR);
        set_nav_mode(NAV_NONE);
        // reset acro axis targets to current attitude
		if(g.axis_enabled){
            roll_axis 	= ahrs.roll_sensor;
            pitch_axis 	= ahrs.pitch_sensor;
            nav_yaw 	= ahrs.yaw_sensor;
        }
        break;

    case STABILIZE:
    	ap.manual_throttle = true;
    	ap.manual_attitude = true;
        set_yaw_mode(YAW_HOLD);
        set_roll_pitch_mode(ROLL_PITCH_STABLE);
        set_throttle_mode(THROTTLE_MANUAL_TILT_COMPENSATED);
        set_nav_mode(NAV_NONE);
        break;

    case ALT_HOLD:
    	ap.manual_throttle = false;
    	ap.manual_attitude = true;
        set_yaw_mode(ALT_HOLD_YAW);
        set_roll_pitch_mode(ALT_HOLD_RP);
        set_throttle_mode(ALT_HOLD_THR);
        set_nav_mode(NAV_NONE);
        break;

    case AUTO:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        set_yaw_mode(AUTO_YAW);
        set_roll_pitch_mode(AUTO_RP);
        set_throttle_mode(AUTO_THR);
        // we do not set nav mode for auto because it will be overwritten when first command runs
        // loads the commands from where we left off
        init_commands();
        break;

    case CIRCLE:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        set_roll_pitch_mode(CIRCLE_RP);
        set_throttle_mode(CIRCLE_THR);
        set_nav_mode(CIRCLE_NAV);
        set_yaw_mode(CIRCLE_YAW);
        break;

    case LOITER:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        set_yaw_mode(LOITER_YAW);
        set_roll_pitch_mode(LOITER_RP);
        set_throttle_mode(LOITER_THR);
        set_nav_mode(LOITER_NAV);
        break;

    case POSITION:
    	ap.manual_throttle = true;
    	ap.manual_attitude = false;
        set_yaw_mode(POSITION_YAW);
        set_roll_pitch_mode(POSITION_RP);
        set_throttle_mode(POSITION_THR);
        set_nav_mode(POSITION_NAV);
        wp_nav.clear_angle_limit();     // ensure there are no left over angle limits from throttle controller.  To-Do: move this to the exit routine of throttle controller
        break;

    case GUIDED:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        set_yaw_mode(GUIDED_YAW);
        set_roll_pitch_mode(GUIDED_RP);
        set_throttle_mode(GUIDED_THR);
        set_nav_mode(GUIDED_NAV);
        break;

    case LAND:
        // To-Do: it is messy to set manual_attitude here because the do_land function is reponsible for setting the roll_pitch_mode
        if( ap.home_is_set ) {
            // switch to loiter if we have gps
            ap.manual_attitude = false;
        }else{
            // otherwise remain with stabilize roll and pitch
            ap.manual_attitude = true;
        }
    	ap.manual_throttle = false;
        do_land();
        break;

    case RTL:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        do_RTL();
        break;

    case OF_LOITER:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        set_yaw_mode(OF_LOITER_YAW);
        set_roll_pitch_mode(OF_LOITER_RP);
        set_throttle_mode(OF_LOITER_THR);
        set_nav_mode(OF_LOITER_NAV);
        break;

    // THOR
    // These are the flight modes for Toy mode
    // See the defines for the enumerated values
    case TOY_A:
    	ap.manual_throttle = false;
    	ap.manual_attitude = true;
        set_yaw_mode(YAW_TOY);
        set_roll_pitch_mode(ROLL_PITCH_TOY);
        set_throttle_mode(THROTTLE_AUTO);
        set_nav_mode(NAV_NONE);

        // save throttle for fast exit of Alt hold
        saved_toy_throttle = g.rc_3.control_in;
        break;

    case TOY_M:
    	ap.manual_throttle = false;
    	ap.manual_attitude = true;
        set_yaw_mode(YAW_TOY);
        set_roll_pitch_mode(ROLL_PITCH_TOY);
        set_nav_mode(NAV_NONE);
        set_throttle_mode(THROTTLE_HOLD);
        break;

    default:
        break;
    }


    // used to stop fly_aways
    // set to false if we have low throttle
    if(ap.manual_throttle && g.rc_3.control_in == 0){
	    motors.auto_arm(false);
    	set_auto_armed(false);
    }

    if(ap.manual_attitude) {
        // We are under manual attitude control
        // remove the navigation from roll and pitch command
        reset_nav_params();
    }

	trace("Set Mode", flight_mode_strings[mode]);
}

public function
init_simple_bearing():void
{
	initial_simple_bearing = ahrs.yaw_sensor;
}
