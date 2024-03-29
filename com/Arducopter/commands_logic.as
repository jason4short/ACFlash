﻿/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
// process_nav_command - main switch statement to initiate the next nav command in the command_nav_queue
public function process_nav_command()
{
    switch(command_nav_queue.id) {

	case MAV_CMD_NAV_TAKEOFF:	// 22
		trace("do command takeoff")
		do_takeoff();
		break;

	case MAV_CMD_NAV_WAYPOINT:	// 16  Navigate to Waypoint
		//trace("do command nav WP")
		do_nav_wp();
		break;

	case MAV_CMD_NAV_LAND:	// 21 LAND to Waypoint
		trace("do command Land")
		do_land();
		break;

	case MAV_CMD_NAV_LOITER_UNLIM:	// 17 Loiter indefinitely
		trace("do command Loiter unlimited")
		do_loiter_unlimited();
		break;

	case MAV_CMD_NAV_LOITER_TURNS:	//18 Loiter N Times
		do_circle();
		break;

	case MAV_CMD_NAV_LOITER_TIME:  // 19
		trace("do command Loiter time")
		do_loiter_time();
		break;

	case MAV_CMD_NAV_RETURN_TO_LAUNCH: //20
		trace("do command RTL")
		do_RTL();
		break;

	// point the copter and camera at a region of interest (ROI)
	case MAV_CMD_NAV_ROI:			 // 80
		do_nav_roi();
		break;

	default:
		break;
	}

}

public function process_cond_command()
{
	switch(command_cond_queue.id){

	case MAV_CMD_CONDITION_DELAY: // 112
		trace("do command delay")
		do_wait_delay();
		break;

	case MAV_CMD_CONDITION_DISTANCE: // 114
		trace("do command distance")
		do_within_distance();
		break;

	case MAV_CMD_CONDITION_CHANGE_ALT: // 113
		trace("do command change alt")
		do_change_alt();
		break;

	case MAV_CMD_CONDITION_YAW: // 115
		trace("do command change Yaw")
		do_yaw();
		break;

	default:
		break;
	}
}

public function process_now_command()
{
	switch(command_cond_queue.id){

	case MAV_CMD_DO_JUMP:  // 177
		trace("do command Jump")
		do_jump();
		break;

	case MAV_CMD_DO_CHANGE_SPEED: // 178
		trace("do command change speed")
		do_change_speed();
		break;

	case MAV_CMD_DO_SET_HOME: // 179
		trace("do command set home")
		do_set_home();
		break;

	case MAV_CMD_DO_SET_SERVO: // 183
		trace("do command set servo")
		do_set_servo();
		break;

	case MAV_CMD_DO_SET_RELAY: // 181
		trace("do command set relay")
		do_set_relay();
		break;

	case MAV_CMD_DO_REPEAT_SERVO: // 184
		trace("do command repeat servo")
		do_repeat_servo();
		break;

	case MAV_CMD_DO_REPEAT_RELAY: // 182
		trace("do command repeat relay")
		do_repeat_relay();
		break;

	default:
		// do nothing with unrecognized MAVLink messages
		break;
	}
}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

// verify_must - switch statement to ensure the active navigation command is progressing
// returns true once the active navigation command completes successfully
public function verify_must():Boolean
{
	switch(command_nav_queue.id) {

	case MAV_CMD_NAV_TAKEOFF:
		return verify_takeoff();
		break;

	case MAV_CMD_NAV_WAYPOINT:
		return verify_nav_wp();
		break;

	case MAV_CMD_NAV_LAND:
		return verify_land();
		break;

	case MAV_CMD_NAV_LOITER_UNLIM:
		return verify_loiter_unlimited();
		break;

	case MAV_CMD_NAV_LOITER_TURNS:
		return verify_circle();
		break;

	case MAV_CMD_NAV_LOITER_TIME:
		return verify_loiter_time();
		break;

	case MAV_CMD_NAV_RETURN_TO_LAUNCH:
		return verify_RTL();
		break;

	case MAV_CMD_NAV_ROI:			 // 80
		return verify_nav_roi();
		break;

	default:
		//gcs_send_text_P(SEVERITY_HIGH,PSTR("<verify_must: default> No current Must commands"));
		return false;
		break;
	}
}

// verify_may - switch statement to ensure the active conditional command is progressing
// returns true once the active conditional command completes successfully
public function verify_may():Boolean
{
	switch(command_cond_queue.id) {

		case MAV_CMD_CONDITION_DELAY:
			return verify_wait_delay();
			break;

		case MAV_CMD_CONDITION_DISTANCE:
			return verify_within_distance();
			break;

		case MAV_CMD_CONDITION_CHANGE_ALT:
			return verify_change_alt();
			break;

		case MAV_CMD_CONDITION_YAW:
			return verify_yaw();
			break;

		default:
			//gcs_send_text_P(SEVERITY_HIGH,PSTR("<verify_must: default> No current May commands"));
			return false;
			break;
	}
}

/********************************************************************************/
//
/********************************************************************************/

// do_RTL - start Return-to-Launch
public function do_RTL():void
{
	trace("do RTL")
    // set rtl state
    rtl_state = RTL_STATE_START;

    // verify_RTL will do the initialisation for us
    verify_RTL();
}

/********************************************************************************/
//	Nav (Must) commands
/********************************************************************************/

// do_takeoff - initiate takeoff navigation command
public function do_takeoff():void
{
	// set roll-pitch mode
	set_roll_pitch_mode(AUTO_RP);

	// set yaw mode
	set_yaw_mode(YAW_HOLD);

	// set throttle mode to AUTO although we should already be in this mode
	set_throttle_mode(THROTTLE_AUTO);

	// set our nav mode to loiter
	set_nav_mode(NAV_WP);

    // Set wp navigation target to safe altitude above current position
    var pos:Vector3D = inertial_nav.get_position();
    pos.z = command_nav_queue.alt;
    wp_nav.set_destination(pos);
	trace("wp_nav.set_destination", pos);
	
	// prevent flips
	// To-Do: check if this is still necessary
	reset_I_all();
}

// do_nav_wp - initiate move to next waypoint
// note: caller should set yaw mode
public function do_nav_wp():void
{
	// set roll-pitch mode
	set_roll_pitch_mode(AUTO_RP);

	// set throttle mode
	set_throttle_mode(THROTTLE_AUTO);

	// set nav mode
	set_nav_mode(NAV_WP);

    // Set wp navigation target
    wp_nav.set_destination(pv_location_to_vector(command_nav_queue));

    // initialise original_wp_bearing which is used to check if we have missed the waypoint
    wp_bearing = wp_nav.get_bearing_to_destination();
    original_wp_bearing = wp_bearing;

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time     = 0;
    // this is the delay, stored in seconds and expanded to millis
    loiter_time_max = command_nav_queue.p1;
    trace("time to loiter at WP: ", loiter_time_max);

	// reset control of yaw to default
	if( g.yaw_override_behaviour == YAW_OVERRIDE_BEHAVIOUR_AT_NEXT_WAYPOINT ) {
		set_yaw_mode(AUTO_YAW);
	}
}

// do_land - initiate landing procedure
// caller should set roll_pitch_mode to ROLL_PITCH_AUTO (for no pilot input) or ROLL_PITCH_LOITER (for pilot input)
// caller should set yaw_mode
public function do_land():void
{
	if( ap.home_is_set ) {
		// switch to loiter if we have gps
		set_roll_pitch_mode(ROLL_PITCH_LOITER);
	}else{
		// otherwise remain with stabilize roll and pitch
		set_roll_pitch_mode(ROLL_PITCH_STABLE);
	}

	// hold yaw while landing
	set_yaw_mode(YAW_HOLD);

	// set throttle mode to land
	set_throttle_mode(THROTTLE_LAND);

	// switch into loiter nav mode
	set_nav_mode(NAV_LOITER);
}

// do_loiter_unlimited - start loitering with no end conditions
// note: caller should set yaw_mode
public function do_loiter_unlimited():void
{
	// set roll-pitch mode (no pilot input)
	set_roll_pitch_mode(AUTO_RP);

	// set throttle mode to AUTO which, if not already active, will default to hold at our current altitude
	set_throttle_mode(THROTTLE_AUTO);

    // get current position
    // To-Do: change this to projection based on current location and velocity
    var curr:Vector3D = inertial_nav.get_position();

    // default to use position provided
    var pos:Vector3D = pv_location_to_vector(command_nav_queue);

    // use current altitude if not provided
    if( command_nav_queue.alt == 0 ) {
        pos.z = curr.z;
	}

    // use current location if not provided
	if(command_nav_queue.lat == 0 && command_nav_queue.lng == 0) {
        pos.x = curr.x;
        pos.y = curr.y;
	}

    // start way point navigator and provide it the desired location
    set_nav_mode(NAV_WP);
    wp_nav.set_destination(pos);
}

// do_circle - initiate moving in a circle
public function do_circle():void
{
	// set roll-pitch mode (no pilot input)
	set_roll_pitch_mode(AUTO_RP);

	// set throttle mode to AUTO which, if not already active, will default to hold at our current altitude
	set_throttle_mode(THROTTLE_AUTO);

    // set nav mode to CIRCLE
    set_nav_mode(NAV_CIRCLE);

	// set target altitude if provided
	if( command_nav_queue.alt != 0 ) {
        wp_nav.set_desired_alt(command_nav_queue.alt);
	}

	// override default horizontal location target
	if( command_nav_queue.lat != 0 || command_nav_queue.lng != 0) {
        circle_set_center(pv_location_to_vector(command_nav_queue), ahrs.yaw);
	}

	// set yaw to point to center of circle
	set_yaw_mode(CIRCLE_YAW);

	// set angle travelled so far to zero
	circle_angle_total = 0;

	// record number of desired rotations from mission command
	circle_desired_rotations = command_nav_queue.p1;
}

// do_loiter_time - initiate loitering at a point for a given time period
// note: caller should set yaw_mode
public function do_loiter_time():void
{
	// set roll-pitch mode (no pilot input)
	set_roll_pitch_mode(AUTO_RP);

	// set throttle mode to AUTO which, if not already active, will default to hold at our current altitude
	set_throttle_mode(THROTTLE_AUTO);

    // get current position
    // To-Do: change this to projection based on current location and velocity
    var curr:Vector3D = inertial_nav.get_position();

    // default to use position provided
    var pos:Vector3D = pv_location_to_vector(command_nav_queue);

    // use current altitude if not provided
    if( command_nav_queue.alt == 0 ) {
        pos.z = curr.z;
	}

    // use current location if not provided
	if(command_nav_queue.lat == 0 && command_nav_queue.lng == 0) {
        pos.x = curr.x;
        pos.y = curr.y;
    }

    // start way point navigator and provide it the desired location
		set_nav_mode(NAV_WP);
    wp_nav.set_destination(pos);

    // setup loiter timer
    loiter_time     = 0;
    loiter_time_max = command_nav_queue.p1;     // units are (seconds)
}

/********************************************************************************/
//	Verify Nav (Must) commands
/********************************************************************************/

// verify_takeoff - check if we have completed the takeoff
public function verify_takeoff():Boolean
{
	// wait until we are ready!
	if(g.rc_3.control_in == 0) {
		// To-Do: reset loiter target if we have not yet taken-off
		// do not allow I term to build up if we have not yet taken-off
		return false;
	}
    // have we reached our target altitude?
    return wp_nav._reached_destination;
}

// verify_land - returns true if landing has been completed
public function verify_land():Boolean
{
	// rely on THROTTLE_LAND mode to correctly update landing status
	return ap.land_complete;
}

public function verify_nav_wp():Boolean
{
    // check if we have reached the waypoint
    if( !wp_nav._reached_destination ) {
        return false;
	}

		// start timer if necessary
	if(loiter_time == 0) {
        loiter_time = clock.millis();
	}
	
	trace("loiter_time_max" , loiter_time_max , "loiter timer", ((clock.millis() - loiter_time) / 1000));
	
		// check if timer has run out
    if (((clock.millis() - loiter_time) / 1000) >= loiter_time_max) {
        trace("Reached Command ", command_nav_index);
		copter_leds_nav_blink = 15;			 // Cause the CopterLEDs to blink three times to indicate waypoint reached
		return true;
	}else{
        trace("Waiting!!!");	
		return false;
	}
}

public function verify_loiter_unlimited()
{
	return false;
}

public function verify_loiter_time():Boolean
{
    // return immediately if we haven't reached our destination
    if (!wp_nav._reached_destination) {
        return false;
		}

    // start our loiter timer
    if( loiter_time == 0 ) {
        loiter_time = clock.millis();
	}

    // check if loiter timer has run out
    return (((clock.millis() - loiter_time) / 1000) >= loiter_time_max);
}

// verify_circle - check if we have circled the point enough
public function verify_circle():Boolean
{
	// have we rotated around the center enough times?
	return Math.abs(circle_desired_rotations / Math.PI) > circle_desired_rotations;
}

// verify_RTL - handles any state changes required to implement RTL
// do_RTL should have been called once first to initialise all variables
// returns true with RTL has completed successfully
public function verify_RTL():Boolean
{
	var retval:Boolean = false;

	switch( rtl_state ) {
        case RTL_STATE_START:
            // set roll, pitch and yaw modes
            set_roll_pitch_mode(RTL_RP);
            set_throttle_mode(RTL_THR);

            // set navigation mode
            set_nav_mode(NAV_WP);

            // if we are below rtl alt do initial climb
            if( current_loc.alt < get_RTL_alt() ) {
                // first stage of RTL is the initial climb so just hold current yaw
                set_yaw_mode(YAW_HOLD);

                // get current position
                // To-Do: use projection of safe stopping point based on current location and velocity
                var target_pos:Vector3D = inertial_nav.get_position();
                target_pos.z = get_RTL_alt();
                wp_nav.set_destination(target_pos);

                // advance to next rtl state
                rtl_state = RTL_STATE_INITIAL_CLIMB;

                //cliSerial->printf_P(PSTR("\nRTL: initial climb to %4.2f"),target_pos.z);
            }else{
                // point nose towards home
                // To-Do: make this user configurable whether RTL points towards home or not
                set_yaw_mode(RTL_YAW);

                // Set wp navigation target to above home
                wp_nav.set_destination(new Vector3D(0,0,get_RTL_alt()));
                
                // advance to next rtl state
                rtl_state = RTL_STATE_RETURNING_HOME;
            }
            break;
		case RTL_STATE_INITIAL_CLIMB:
            // check if we've reached the safe altitude
            if (wp_nav._reached_destination) {
                // set nav mode
                set_nav_mode(NAV_WP);

                // Set wp navigation target to above home
                wp_nav.set_destination(new Vector3D(0,0,get_RTL_alt()));

				// set yaw mode
                // To-Do: make this user configurable whether RTL points towards home or not
				set_yaw_mode(RTL_YAW);

				// advance to next rtl state
				rtl_state = RTL_STATE_RETURNING_HOME;
			}
			break;

		case RTL_STATE_RETURNING_HOME:
            // check if we've reached home
            if (wp_nav._reached_destination) {
                // Note: we remain in NAV_WP nav mode which should hold us above home

                // start timer
                rtl_loiter_start_time = clock.millis();

				// give pilot back control of yaw
				set_yaw_mode(YAW_HOLD);

                // advance to next rtl state
                rtl_state = RTL_STATE_LOITERING_AT_HOME;
			}
			break;

		case RTL_STATE_LOITERING_AT_HOME:
			// check if we've loitered long enough
            if( clock.millis() - rtl_loiter_start_time > g.rtl_loiter_time) {
				// initiate landing or descent
				if(g.rtl_alt_final == 0 || ap.failsafe_radio) {
                    // land - this will switch us into land throttle mode and loiter nav mode and give horizontal control back to pilot
					do_land();
					// override landing location (do_land defaults to current location)
                    // Note: loiter controller ignores target altitude
                    wp_nav.set_loiter_target(new Vector3D(0,0,0));
					// update RTL state
					rtl_state = RTL_STATE_LAND;
				}else{
                    // descend using waypoint controller
					if(current_loc.alt > g.rtl_alt_final) {
                        // set navigation mode
                        set_nav_mode(NAV_WP);
                        // Set wp navigation alt target to rtl_alt_final
                        wp_nav.set_destination(new Vector3D(0,0,g.rtl_alt_final));
					}
					// update RTL state
					rtl_state = RTL_STATE_FINAL_DESCENT;
				}
			}
			break;

		case RTL_STATE_FINAL_DESCENT:
            // check we have reached final altitude
            if(current_loc.alt <= g.rtl_alt_final || wp_nav._reached_destination) {
                // indicate that we've completed RTL
				retval = true;
			}
			break;

		case RTL_STATE_LAND:
			// rely on verify_land to return correct status
			retval = verify_land();
			break;

		default:
			// this should never happen
			// TO-DO: log an error
			retval = true;
			break;
	}

	// true is returned if we've successfully completed RTL
	return retval;
}

/********************************************************************************/
//	Condition (May) commands
/********************************************************************************/

public function do_wait_delay():void
{
	//Serial.print("dwd ");
	condition_start = clock.millis();
	condition_value	= command_cond_queue.lat * 1000; // convert to milliseconds
	//Serial.println(condition_value,DEC);
}

public function do_change_alt():void
{
    // adjust target appropriately for each nav mode
    switch (nav_mode) {
        case NAV_CIRCLE:
        case NAV_LOITER:
            // update loiter target altitude
            wp_nav.set_desired_alt(command_cond_queue.alt);
            break;

        case NAV_WP:
            // To-Do: update waypoint nav's destination altitude
            break;
    }

    // To-Do: store desired altitude in a variable so that it can be verified later
}

public function do_within_distance():void
{
	condition_value	 = command_cond_queue.lat * 100;
}

public function do_yaw():void
{
	// get final angle, 1 = Relative, 0 = Absolute
	if( command_cond_queue.lng == 0 ) {
		// absolute angle
		yaw_look_at_heading = wrap_360(command_cond_queue.alt * 100);
	}else{
		// relative angle
		yaw_look_at_heading = wrap_360(nav_yaw + command_cond_queue.alt * 100);
	}

	// get turn speed
	if( command_cond_queue.lat == 0 ) {
		// default to regular auto slew rate
		yaw_look_at_heading_slew = AUTO_YAW_SLEW_RATE;
	}else{
		var turn_rate:int = (wrap_180(yaw_look_at_heading - nav_yaw) / 100) / command_cond_queue.lat;
		yaw_look_at_heading_slew = constrain(turn_rate, 1, 360);	// deg / sec
	}

	// set yaw mode
	set_yaw_mode(YAW_LOOK_AT_HEADING);

	// TO-DO: restore support for clockwise / counter clockwise rotation held in command_cond_queue.p1
	// command_cond_queue.p1; // 0 = undefined, 1 = clockwise, -1 = counterclockwise
}


/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

public function verify_wait_delay():Boolean
{
	//cliSerial->print("vwd");
	if (clock.millis() - condition_start > Math.max(condition_value, 0)) {
		//cliSerial->println("y");
		condition_value = 0;
		return true;
	}
	//cliSerial->println("n");
	return false;
}

public function verify_change_alt():Boolean
{
    // To-Do: use recorded target altitude to verify we have reached the target
    return true;
}

public function verify_within_distance():Boolean
{
	//cliSerial->printf("cond dist :%d\n", (int)condition_value);
	if (wp_distance < Math.max(condition_value,0)) {
		condition_value = 0;
		return true;
	}
	return false;
}

public function verify_yaw():Boolean
{
	if(Math.abs(wrap_180(ahrs.yaw_sensor - yaw_look_at_heading)) <= 200){
		return true;
	}else{
		return false;
	}
}

// verify_nav_roi - verifies that actions required by MAV_CMD_NAV_ROI have completed
//			  we assume the camera command has been successfully implemented by the do_nav_roi command
//			  so all we need to check is whether we needed to yaw the copter (due to the mount type) and
//			  whether that yaw has completed
//	TO-DO: add support for other features of MAV_NAV_ROI including pointing at a given waypoint
public function verify_nav_roi():Boolean
{
	// if we have no camera mount simply check we've reached the desired yaw
	// ensure yaw has gotten to within 2 degrees of the target
	if(Math.abs(wrap_180(ahrs.yaw_sensor - yaw_look_at_WP_bearing)) <= 200){
		return true;
	}else{
		return false;
	}
}

/********************************************************************************/
//	Do (Now) commands
/********************************************************************************/

public function do_change_speed():void
{
	wp_nav.set_horizontal_velocity(command_cond_queue.p1 * 100);
}


public var jump :int = -10;// used to track loops in jump command
public function do_jump():void
{
	// Used to track the state of the jump command in Mission scripting
	// -10 is a value that means the register is unused
	// when in use, it contains the current remaining jumps

	//cliSerial->printf("do Jump: %d\n", jump);

	if(jump == -10) {
		//cliSerial->printf("Fresh Jump\n");
		// we use a locally stored index for jump
		jump = command_cond_queue.lat;
	}
	//cliSerial->printf("Jumps left: %d\n",jump);

	if(jump > 0) {
		//cliSerial->printf("Do Jump to %d\n",command_cond_queue.p1);
		jump--;
		change_command(command_cond_queue.p1);

	} else if (jump == 0){
		//Serial.printf("Did last jump\n");
		// we're done, move along
		jump = -11;

	} else if (jump == -1) {
		//cliSerial->printf("jumpForever\n");
		// repeat forever
		change_command(command_cond_queue.p1);
	}
}

public function do_set_home():void
{
	if(command_cond_queue.p1 == 1) {
		init_home();
	} else {
		home.id		= MAV_CMD_NAV_WAYPOINT;
		home.lng	= command_cond_queue.lng;				// Lon * 10**7
		home.lat	= command_cond_queue.lat;				// Lat * 10**7
		home.alt	= 0;
		//home_is_set 	= true;
		set_home_is_set(true);
	}
}

public function do_set_servo():void
{
	trace("set ch:", command_cond_queue.p1 - 1, "to pwm:", command_cond_queue.alt)
	//APM_RC.OutputCh(command_cond_queue.p1 - 1, command_cond_queue.alt);
}

public function do_set_relay():void
{
	if (command_cond_queue.p1 == 1) {
		relay.on();
	} else if (command_cond_queue.p1 == 0) {
		relay.off();
	}else{
		relay.toggle();
	}
}

public function do_repeat_servo():void
{
	event_id = command_cond_queue.p1 - 1;

	if(command_cond_queue.p1 >= CH_5 + 1 && command_cond_queue.p1 <= CH_8 + 1) {

		event_timer		= 0;
		event_value		= command_cond_queue.alt;
		event_repeat	= command_cond_queue.lat * 2;
		event_delay		= command_cond_queue.lng * 500.0; // /2 (half cycle time) * 1000 (convert to milliseconds)

		switch(command_cond_queue.p1) {
			case CH_5:
				event_undo_value = g.rc_5.radio_trim;
				break;
			case CH_6:
				event_undo_value = g.rc_6.radio_trim;
				break;
			case CH_7:
				event_undo_value = g.rc_7.radio_trim;
				break;
			case CH_8:
				event_undo_value = g.rc_8.radio_trim;
				break;
		}
		update_events();
	}
}

public function do_repeat_relay():void
{
	event_id		= RELAY_TOGGLE;
	event_timer		= 0;
	event_delay		= command_cond_queue.lat * 500.0; // /2 (half cycle time) * 1000 (convert to milliseconds)
	event_repeat	= command_cond_queue.alt * 2;
	update_events();
}

// do_nav_roi - starts actions required by MAV_CMD_NAV_ROI
//			  this involves either moving the camera to point at the ROI (region of interest)
//			  and possibly rotating the copter to point at the ROI if our mount type does not support a yaw feature
//				Note: the ROI should already be in the command_nav_queue global variable
//	TO-DO: add support for other features of MAV_NAV_ROI including pointing at a given waypoint
public function do_nav_roi():void
{
	// if we have no camera mount aim the quad at the location
    yaw_look_at_WP = pv_location_to_vector(command_nav_queue);
	set_yaw_mode(YAW_LOOK_AT_LOCATION);
}

// do_take_picture - take a picture with the camera library
public function do_take_picture()
{
}
