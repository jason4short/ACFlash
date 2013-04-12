public function loop():void
{
	iteration++;

	fast_loop();
	// 50 hz pieces
	if(fifty_toggle){

		if(iteration > g.sim_iterations){
			stopSIM();
		}

		// IMU DCM Algorithm
		// --------------------
		// done in the copter class


		// check for new GPS messages
		// --------------------------
		update_GPS();

		// perform 10hz tasks
		// ------------------
		medium_loop();

		counter_one_herz++;

		// trgger our 1 hz loop
		if(counter_one_herz >= 50){
			super_slow_loop();
			counter_one_herz = 0;

			//offset_x_gain *= 0.95;
			//offset_y_gain *= 0.95;
			//offset_z_gain *= 0.95;

			//offset_x_gain = Math.max(offset_x_gain, 0);
			//offset_y_gain = Math.max(offset_y_gain, 0);
			//offset_z_gain = Math.max(offset_z_gain, 0);

			//trace(offset_x_gain, offset_y_gain, offset_z_gain);
		}

		// Stuff to run at full 50hz, but after the med loops
		// --------------------------------------------------
		fifty_hz_loop();

		if(fastPlot){
			plot(plotType_A, plot_A , 1);
			plot(plotType_B, plot_B , 2);
			plot(plotType_C, plot_C , 3);
		}
	}
	fifty_toggle = !fifty_toggle;

	// reposition Copter onscreen
	copter.update(G_Dt);
}

// Main loop - 100hz
public function fast_loop():void
{
    // IMU DCM Algorithm
    // --------------------
	//read_AHRS();
    // reads all of the necessary trig functions for cameras, throttle, etc.
    // --------------------------------------------------------------------
    update_trig();

	// Acrobatic control
    if (ap.do_flip) {
        if(Math.abs(g.rc_1.control_in) < 4000) {
            // calling roll_flip will override the desired roll rate and throttle output
            roll_flip();
        }else{
            // force an exit from the loop if we are not hands off sticks.
            ap.do_flip = false;
            Log_Write_Event(DATA_EXIT_FLIP);
        }
    }

    // run low level rate controllers that only require IMU data
    run_rate_controllers();

    // write out the servo PWM values
    // ------------------------------
    set_servos_4();

    // Inertial Nav
    // --------------------
    read_inertia();

    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();

    // custom code/exceptions for flight modes
    // ---------------------------------------
    update_yaw_mode();
    update_roll_pitch_mode();


    // update targets to rate controllers
    update_rate_contoller_targets();
}

public function medium_loop():void
{
	// This is the start of the medium (10 Hz) loop pieces
	// -----------------------------------------
	switch(medium_loopCounter) {

	// This case deals with the GPS and Compass
	//-----------------------------------------
	case 0:
		//-----------------
		//alt_hold_score += Math.abs((next_WP.alt - current_loc.alt) * G_Dt); // for judging

		if(fastPlot == false){
			plot(plotType_A, plot_A , 1);
			plot(plotType_B, plot_B , 2);
			plot(plotType_C, plot_C , 3);
		}
		elapsed_time_tf.text = formatTime(clock.elapsed) + " " + iteration.toString();

		debug_TF.text =   	current_loc.alt.toFixed(0) +"\n" +
							current_loc.lng.toFixed(0) +"\n" +
							current_loc.lat.toFixed(0) +"\n" +
							copter.velocity.x.toFixed(0) +"\n" +
							copter.velocity.x.toFixed(0) +"\n" +
							crosstrack_score.toFixed(0) +"\n" +
							alt_hold_score.toFixed(0);
		//------------------------------------------------------

		medium_loopCounter++;

		// auto_trim - stores roll and pitch radio inputs to ahrs
		auto_trim();

		// record throttle output
		// ------------------------------
		throttle_integrator += g.rc_3.servo_out;
		break;

	// This case performs some navigation computations
	//------------------------------------------------
	case 1:
		medium_loopCounter++;
        //read_receiver_rssi();
        update_altitude();
		break;

    // command processing
    //-------------------
    case 2:
        medium_loopCounter++;

        if(control_mode == TOY_A) {
            update_toy_throttle();

            if(throttle_mode == THROTTLE_AUTO) {
                update_toy_altitude();
            }
        }

        ap_system.alt_sensor_flag = true;
        break;

	// This case deals with sending high rate telemetry
	//-------------------------------------------------
	case 3:
		medium_loopCounter++;
		run_nav_updates();
		// perform next command
		// --------------------
		if(control_mode == AUTO){
            if(ap.home_is_set && g.command_total > 1) {
				update_commands();
			}
		}


		break;

	// This case controls the slow loop
	//---------------------------------
	case 4:
		medium_loopCounter = 0;

		// Accel trims 		= hold > 2 seconds
		// Throttle cruise  = switch less than 1 second
		// --------------------------------------------
		read_trim_switch();

        update_navigation();

        // Check for engine arming
        // -----------------------
        arm_motors();

		slow_loop();
		break;

	default:
		// this is just a catch all
		// ------------------------
		medium_loopCounter = 0;
		break;
	}
}

// stuff that happens at 50 hz
// ---------------------------
public function fifty_hz_loop():void
{
    // get altitude and climb rate from inertial lib
    read_inertial_altitude();

    // Update the throttle ouput
    // -------------------------
    update_throttle_mode();

	if(g.toy_edf){
		edf_toy();
	}


	// Read Sonar
	// ----------
	if(g.sonar_enabled){
		sonar_alt = sonar.read();
	}


}

public function slow_loop():void
{
    // Run the AP_Limits main loop
    //limits_loop();

	// This is the slow (3 1/3 Hz) loop pieces
	//----------------------------------------
	switch (slow_loopCounter){
	case 0:
		slow_loopCounter++;
		superslow_loopCounter++;

		//trace(ahrs.roll_sensor, Math.floor(g.rc_1.servo_out), ahrs.pitch_sensor, Math.floor(g.rc_2.servo_out));

		if(superslow_loopCounter > 1200){
			// save compass offsets
			superslow_loopCounter = 0;
		}

        if(!motors.armed()) {
            // check the user hasn't updated the frame orientation
            motors.set_frame_orientation(g.frame_orientation);
        }

		break;

	case 1:
		slow_loopCounter++;
		break;

	case 2:
		slow_loopCounter = 0;
		update_events();

		// blink if we are armed
		//update_lights();

		break;

	default:
		slow_loopCounter = 0;
		break;
	}
}


public const AUTO_DISARMING_DELAY			:int = 20;
// 1Hz loop
public function super_slow_loop():void
{
    // this function disarms the copter if it has been sitting on the ground for any moment of time greater than 25 seconds
    // but only of the control mode is manual
    if((control_mode <= ACRO) && (g.rc_3.control_in == 0) && motors.armed()) {
        auto_disarming_counter++;
		trace("auto_disarming_counter", auto_disarming_counter);
        if(auto_disarming_counter == AUTO_DISARMING_DELAY) {
            init_disarm_motors();
        }else if (auto_disarming_counter > AUTO_DISARMING_DELAY) {
            auto_disarming_counter = AUTO_DISARMING_DELAY + 1;
        }
    }else{
        auto_disarming_counter = 0;
    }
}

// called at 50hz
public function
update_GPS()
{
    //set_gps_healthy(g_gps->status() == g_gps->GPS_OK);

	if(g_gps.new_data == true){
		//iteration++;
		g_gps.new_data = false;

		// we read  GPS every 250 ms
		dTnav = .25;

		if(g.GPS_checkbox.getSelected())
			Log_Write_GPS();
	}
}


// set_yaw_mode - update yaw mode and initialise any variables required
public function
set_yaw_mode(new_yaw_mode:int):Boolean
{
    // boolean to ensure proper initialisation of throttle modes
    var yaw_initialised:Boolean = false;

    // return immediately if no change
    if( new_yaw_mode == yaw_mode ) {
        return true;
    }

    switch( new_yaw_mode ) {
        case YAW_HOLD:
        case YAW_ACRO:
            yaw_initialised = true;
            break;
        case YAW_LOOK_AT_NEXT_WP:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AT_LOCATION:
            if( ap.home_is_set ) {
                // update bearing - assumes yaw_look_at_WP has been intialised before set_yaw_mode was called
                yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), yaw_look_at_WP);
                yaw_initialised = true;
            }
            break;
        case YAW_CIRCLE:
            if( ap.home_is_set ) {
                // set yaw to point to center of circle
                yaw_look_at_WP = circle_center;
                // initialise bearing to current heading
                yaw_look_at_WP_bearing = ahrs.yaw_sensor;
                yaw_initialised = true;
            }
            break;            
        case YAW_LOOK_AT_HEADING:
            yaw_initialised = true;
            break;
        case YAW_LOOK_AT_HOME:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_TOY:
            yaw_initialised = true;
            break;
        case YAW_LOOK_AHEAD:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( yaw_initialised ) {
        yaw_mode = new_yaw_mode;
    }

    // return success or failure
    return yaw_initialised;
}

// update_yaw_mode - run high level yaw controllers
// 100hz update rate
public function
update_yaw_mode():void
{
	
    switch(yaw_mode) {
    case YAW_HOLD:
        // heading hold at heading held in nav_yaw but allow input from pilot
        get_yaw_rate_stabilized_ef(g.rc_4.control_in);
        break;

    case YAW_ACRO:
        // pilot controlled yaw using rate controller
        if(g.axis_enabled) {
            get_yaw_rate_stabilized_ef(g.rc_4.control_in);
        }else{
            get_acro_yaw(g.rc_4.control_in);
        }
        break;

    case YAW_LOOK_AT_NEXT_WP:
        // point towards next waypoint (no pilot input accepted)
        // we don't use wp_bearing because we don't want the copter to turn too much during flight
        nav_yaw = get_yaw_slew(nav_yaw, original_wp_bearing, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(nav_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if( g.rc_4.control_in != 0 ) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_LOCATION:
        // point towards a location held in yaw_look_at_WP
        get_look_at_yaw();
 
        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if( g.rc_4.control_in != 0 ) {
            set_yaw_mode(YAW_HOLD);
        }
        break;
 
    case YAW_CIRCLE:

        // points toward the center of the circle or does a panorama
        get_circle_yaw();

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if( g.rc_4.control_in != 0 ) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_HOME:
        // keep heading always pointing at home with no pilot input allowed
        nav_yaw = get_yaw_slew(nav_yaw, home_bearing, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(nav_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if( g.rc_4.control_in != 0 ) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_HEADING:
        // keep heading pointing in the direction held in yaw_look_at_heading with no pilot input allowed
        nav_yaw = get_yaw_slew(nav_yaw, yaw_look_at_heading, yaw_look_at_heading_slew);
        get_stabilize_yaw(nav_yaw);
        break;

	case YAW_LOOK_AHEAD:
		// Commanded Yaw to automatically look ahead.
        get_look_ahead_yaw(g.rc_4.control_in);
        break;

    case YAW_TOY:
        // update to allow external roll/yaw mixing
        // keep heading always pointing at home with no pilot input allowed
        nav_yaw = get_yaw_slew(nav_yaw, home_bearing, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(nav_yaw);
        break;
    }
}


// set_roll_pitch_mode - update roll/pitch mode and initialise any variables as required
public function
set_roll_pitch_mode(new_roll_pitch_mode:int):Boolean
{
    // boolean to ensure proper initialisation of throttle modes
    var roll_pitch_initialised:Boolean = false;

    // return immediately if no change
    if( new_roll_pitch_mode == roll_pitch_mode ) {
        return true;
    }

    switch( new_roll_pitch_mode ) {
        case ROLL_PITCH_STABLE:
        case ROLL_PITCH_ACRO:
        case ROLL_PITCH_AUTO:
        case ROLL_PITCH_STABLE_OF:
        case ROLL_PITCH_TOY:
            roll_pitch_initialised = true;
            break;

        case ROLL_PITCH_LOITER:
            // require gps lock
            if( ap.home_is_set ) {
                roll_pitch_initialised = true;
            }
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( roll_pitch_initialised ) {
        roll_pitch_mode = new_roll_pitch_mode;
    }

    // return success or failure
    return roll_pitch_initialised;
}


// update_roll_pitch_mode - run high level roll and pitch controllers
// 100hz update rate
public function
update_roll_pitch_mode():void
{
    switch(roll_pitch_mode) {
    case ROLL_PITCH_ACRO:
        // copy user input for reporting purposes
        control_roll            = g.rc_1.control_in;
        control_pitch           = g.rc_2.control_in;

		if(g.axis_enabled) {
            get_roll_rate_stabilized_ef(g.rc_1.control_in);
            get_pitch_rate_stabilized_ef(g.rc_2.control_in);
        }else{
            // ACRO does not get SIMPLE mode ability
            get_acro_roll(g.rc_1.control_in);
            get_acro_pitch(g.rc_2.control_in);
		}
        break;

    case ROLL_PITCH_STABLE:
        // apply SIMPLE mode transform
        if(ap.simple_mode && ap_system.new_radio_frame) {
            update_simple_mode();
        }

        control_roll            = g.rc_1.control_in;
        control_pitch           = g.rc_2.control_in;

        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);

        break;

    case ROLL_PITCH_AUTO:
        // copy user input for reporting purposes
        control_roll  = g.rc_1.control_in;
        control_pitch = g.rc_2.control_in;
    
        // copy latest output from nav controller to stabilize controller
        nav_roll    += constrain(wrap_180(wp_nav.get_desired_roll()  - nav_roll),  -g.auto_slew_rate, g.auto_slew_rate);  // 40 deg a second
        nav_pitch   += constrain(wrap_180(wp_nav.get_desired_pitch() - nav_pitch), -g.auto_slew_rate, g.auto_slew_rate);  // 40 deg a second
        get_stabilize_roll(nav_roll);
        get_stabilize_pitch(nav_pitch);

        // copy control_roll and pitch for reporting purposes
        control_roll = nav_roll;
        control_pitch = nav_pitch;
        break;

    case ROLL_PITCH_STABLE_OF:
        // apply SIMPLE mode transform
        if(ap.simple_mode && ap_system.new_radio_frame) {
            update_simple_mode();
        }

        control_roll            = g.rc_1.control_in;
        control_pitch           = g.rc_2.control_in;

        // mix in user control with optical flow
        get_stabilize_roll(get_of_roll(control_roll));
        get_stabilize_pitch(get_of_pitch(control_pitch));
        break;

    // THOR
    // a call out to the main toy logic
    case ROLL_PITCH_TOY:
        roll_pitch_toy();
        break;

    case ROLL_PITCH_LOITER:
        // apply SIMPLE mode transform
        if(ap.simple_mode && ap_system.new_radio_frame) {
            update_simple_mode();
            
        }
        // copy user input for logging purposes
        control_roll            = g.rc_1.control_in;
        control_pitch           = g.rc_2.control_in;

        // update loiter target from user controls - max velocity is 5.0 m/s
        wp_nav.move_loiter_target(control_roll, control_pitch,0.01);

        // copy latest output from nav controller to stabilize controller
        nav_roll    += constrain(wrap_180(wp_nav.get_desired_roll()  - nav_roll),  -g.auto_slew_rate, g.auto_slew_rate);  // 40 deg a second
        nav_pitch   += constrain(wrap_180(wp_nav.get_desired_pitch() - nav_pitch), -g.auto_slew_rate, g.auto_slew_rate);  // 40 deg a second
        get_stabilize_roll(nav_roll);
        get_stabilize_pitch(nav_pitch);
        break;
    }

    if(g.rc_3.control_in == 0 && control_mode <= ACRO) {
        reset_rate_I();
        reset_stability_I();
    }

    if(ap_system.new_radio_frame) {
        // clear new radio frame info
        ap_system.new_radio_frame = false;
    }
}

public var simple_counter	:int = 0;             // State machine counter for Simple Mode
public var simple_sin_y		:Number;
public var simple_cos_x		:Number;

// new radio frame is used to make sure we only call this at 50hz
public function
update_simple_mode():void
{
	trace("update_simple_mode")
	// used to manage state machine
	// which improves speed of function
	simple_counter++;

	var delta:int = wrap_360(ahrs.yaw_sensor - initial_simple_bearing)/100;

	if (simple_counter == 1){
		// roll
		simple_cos_x = Math.sin(radians(90 - delta));

	}else if (simple_counter > 2){
		// pitch
		simple_sin_y = Math.cos(radians(90 - delta));
		simple_counter = 0;
	}

	// Rotate input by the initial bearing
	var _roll:int 	= g.rc_1.control_in   * simple_cos_x + g.rc_2.control_in * simple_sin_y;
	var _pitch:int 	= -(g.rc_1.control_in * simple_sin_y - g.rc_2.control_in * simple_cos_x);

	g.rc_1.control_in = _roll;
	g.rc_2.control_in = _pitch;
}

public const SUPER_SIMPLE_RADIUS :int = 1000;

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
public function
update_super_simple_bearing():void
{
    // are we in SIMPLE mode?
    if(ap.simple_mode && g.super_simple) {
        // get distance to home
        if(home_distance > SUPER_SIMPLE_RADIUS) {        // 10m from home
            // we reset the angular offset to be a vector from home to the quad
            initial_simple_bearing = wrap_360(home_bearing+18000);
        }
    }
}

// set_throttle_mode - sets the throttle mode and initialises any variables as required
public function
set_throttle_mode(new_throttle_mode:int):Boolean
{
	trace("set_throttle_mode", new_throttle_mode);
	// boolean to ensure proper initialisation of throttle modes
	var throttle_initialised:Boolean = false;

	// return immediately if no change
	if( new_throttle_mode == throttle_mode ) {
		return true;
	}

	// initialise any variables required for the new throttle mode
	switch(new_throttle_mode) {
		case THROTTLE_MANUAL:
		case THROTTLE_MANUAL_TILT_COMPENSATED:
			throttle_accel_deactivate();                // this controller does not use accel based throttle controller
			altitude_error = 0;                         // clear altitude error reported to GCS
			throttle_initialised = true;
			break;

		case THROTTLE_ACCELERATION:                     // pilot inputs the desired acceleration
			if(g.throttle_accel_enabled){            // this throttle mode requires use of the accel based throttle controller
				altitude_error = 0;                     // clear altitude error reported to GCS
				throttle_initialised = true;
			}
			break;

		case THROTTLE_RATE:
			altitude_error = 0;                         // clear altitude error reported to GCS
			throttle_initialised = true;
			break;

		case THROTTLE_STABILIZED_RATE:
		case THROTTLE_DIRECT_ALT:
			controller_desired_alt = get_initial_alt_hold(current_loc.alt, climb_rate);   // reset controller desired altitude to current altitude
			trace("THROTTLE_DIRECT_ALT", controller_desired_alt)
			throttle_initialised = true;
			break;

		case THROTTLE_HOLD:
		case THROTTLE_AUTO:
            controller_desired_alt = get_initial_alt_hold(current_loc.alt, climb_rate);     // reset controller desired altitude to current altitude
            wp_nav.set_desired_alt(controller_desired_alt);                                 // same as above but for loiter controller

			if ( throttle_mode <= THROTTLE_MANUAL_TILT_COMPENSATED ) {      // reset the alt hold I terms if previous throttle mode was manual
				reset_throttle_I();
				set_accel_throttle_I_from_pilot_throttle(get_pilot_desired_throttle(g.rc_3.control_in));
			}
			throttle_initialised = true;
			break;

		case THROTTLE_LAND:
			set_land_complete(false);   // mark landing as incomplete
			land_detector = 0;          // A counter that goes up if our climb rate stalls out.
			controller_desired_alt = get_initial_alt_hold(current_loc.alt, climb_rate);   // reset controller desired altitude to current altitude
			trace("THROTTLE_LAND", controller_desired_alt)

			throttle_initialised = true;
			break;

		default:
			// To-Do: log an error message to the dataflash or tlogs instead of printing to the serial port
			trace("Unsupported throttle mode: !!", new_throttle_mode);
			break;
	}

	// update the throttle mode
	if( throttle_initialised ) {
		throttle_mode = new_throttle_mode;

		// reset some variables used for logging
		desired_climb_rate = 0;
		nav_throttle = 0;
	}

	// return success or failure
	return throttle_initialised;
}

// update_throttle_mode - run high level throttle controllers
// 50 hz update rate
// controls all throttle behavior
public function update_throttle_mode():void
{
	var pilot_climb_rate:int = 0;
	var pilot_throttle_scaled:Number = 0;

    if(ap.do_flip && throttle_mode == THROTTLE_MANUAL)     // this is pretty bad but needed to flip in AP modes.
        return;

    // do not run throttle controllers if motors disarmed
    if(!motors.armed()){
        set_throttle_out(0, false);
        throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
		set_target_alt_for_reporting(0);
        return;
    }

	switch(throttle_mode){

    case THROTTLE_MANUAL:
        // completely manual throttle
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
        }else{
            // send pilot's output directly to motors
            pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);
            set_throttle_out(pilot_throttle_scaled, false);

            // update estimate of throttle cruise
			update_throttle_cruise(pilot_throttle_scaled);


            // check if we've taken off yet
            if (!ap.takeoff_complete && motors.armed()) {
                if (pilot_throttle_scaled > g.throttle_cruise) {
                    // we must be in the air by now
                    set_takeoff_complete(true);
                }
            }
        }        
        set_target_alt_for_reporting(0);
        break;

    case THROTTLE_MANUAL_TILT_COMPENSATED:
        // manual throttle but with angle boost
        if (g.rc_3.control_in <= 0) {
            set_throttle_out(0, false); // no need for angle boost with zero throttle
        }else{
            pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);
            set_throttle_out(pilot_throttle_scaled, true);

            // update estimate of throttle cruise
			update_throttle_cruise(pilot_throttle_scaled);

            if (!ap.takeoff_complete && motors.armed()) {
                if (pilot_throttle_scaled > g.throttle_cruise) {
                    // we must be in the air by now
                    set_takeoff_complete(true);
                }
            }
        }
        set_target_alt_for_reporting(0);
        break;

    case THROTTLE_ACCELERATION:
        // pilot inputs the desired acceleration
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
            throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
        }else{
            var desired_acceleration:int = get_pilot_desired_acceleration(g.rc_3.control_in);
            set_throttle_accel_target(desired_acceleration);
        }
        set_target_alt_for_reporting(0);
        break;

    case THROTTLE_RATE:
        // pilot inputs the desired climb rate.  Note this is the unstabilized rate controller
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
            throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
        }else{
            pilot_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);
            get_throttle_rate(pilot_climb_rate);
        }
        set_target_alt_for_reporting(0);
        break;

    case THROTTLE_STABILIZED_RATE:
        // pilot inputs the desired climb rate.  Note this is the stabilized rate controller
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
            throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
            altitude_error = 0;             // clear altitude error reported to GCS - normally underlying alt hold controller updates altitude error reported to GCS
            set_target_alt_for_reporting(0);
        }else{
            pilot_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);
            get_throttle_rate_stabilized(pilot_climb_rate);     // this function calls set_target_alt_for_reporting for us
        }
        break;

    case THROTTLE_DIRECT_ALT:
        // pilot inputs a desired altitude from 0 ~ 10 meters
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
            throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
            altitude_error = 0;             // clear altitude error reported to GCS - normally underlying alt hold controller updates altitude error reported to GCS
            set_target_alt_for_reporting(0);
        }else{
            var desired_alt:int = get_pilot_desired_direct_alt(g.rc_3.control_in);
            get_throttle_althold_with_slew(desired_alt, g.auto_velocity_z_min, g.auto_velocity_z_max);
            set_target_alt_for_reporting(desired_alt);
        }
        break;

    case THROTTLE_HOLD:
        // alt hold plus pilot input of climb rate
        pilot_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);
        if( sonar_alt_health >= SONAR_ALT_HEALTH_MAX ) {
            // if sonar is ok, use surface tracking
            get_throttle_surface_tracking(pilot_climb_rate);    // this function calls set_target_alt_for_reporting for us
        }else{
            // if no sonar fall back stabilize rate controller
            get_throttle_rate_stabilized(pilot_climb_rate);     // this function calls set_target_alt_for_reporting for us
        }
        break;

    case THROTTLE_AUTO:
        // auto pilot altitude controller with target altitude held in wp_nav.get_desired_alt()
        if(motors.auto_armed() == true) {
            get_throttle_althold_with_slew(wp_nav.get_desired_alt(), g.auto_velocity_z_min, g.auto_velocity_z_max);
            set_target_alt_for_reporting(wp_nav.get_desired_alt()); // To-Do: return get_destination_alt if we are flying to a waypoint
        }
        break;

    case THROTTLE_LAND:
        // landing throttle controller
        get_throttle_land();
        set_target_alt_for_reporting(0);
        break;
    }
}

// set_target_alt_for_reporting - set target altitude for reporting purposes (logs and gcs)
public function set_target_alt_for_reporting(alt:Number):void
{
    target_alt_for_reporting = alt;
}
 
// get_target_alt_for_reporting - returns target altitude for reporting purposes (logs and gcs)
public function get_target_alt_for_reporting():Number
{
    return target_alt_for_reporting;
}
 

//sin_yaw_y -0.9198
//cos_yaw_x -0.3924
//sin_yaw -0.3924
//cos_yaw -0.9198

//static void read_AHRS(void)
//XXX
public function update_trig():void
{
	omega = ahrs.omega.clone();

	cos_pitch_x 	= Math.cos(radiansx100(ahrs.pitch_sensor));
	cos_roll_x 		= Math.cos(radiansx100(ahrs.roll_sensor));
    cos_pitch_x     = constrain(cos_pitch_x, 0, 1.0);
    cos_roll_x      = constrain(cos_roll_x, -1.0, 1.0);

	// added to convert earth frame to body frame for rate controllers
	sin_pitch 		= Math.sin(ahrs.pitch);
	sin_roll 		= Math.sin(ahrs.roll);

	sin_yaw_y 		= Math.sin(radiansx100(9000 - ahrs.yaw_sensor));	// 1y = north
	cos_yaw_x 		= Math.cos(radiansx100(9000 - ahrs.yaw_sensor));	// 0x = north

	cos_yaw			= sin_yaw_y;
	sin_yaw			= cos_yaw_x;


    // update wp_nav controller with trig values
    wp_nav.set_cos_sin_yaw(cos_yaw, sin_yaw, cos_roll_x);

	//flat:
	// 0  	= cos_yaw:  0.00, sin_yaw:  1.00,
	// 90 	= cos_yaw:  1.00, sin_yaw:  0.00,
	// 180 	= cos_yaw:  0.00, sin_yaw: -1.00,
	// 270 	= cos_yaw: -1.00, sin_yaw:  0.00,
}

// read baro and sonar altitude at 10hz
public function update_altitude()
{
    // read in baro altitude
    baro_alt            = barometer.read();

    // read in sonar altitude
    sonar_alt           = read_sonar();
}

