// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
*/
public function failsafe_radio_on_event():void
{
    // if motors are not armed there is nothing to do
    if( !motors.armed() ) {
        return;
    }

	// This is how to handle a failsafe.
    switch(control_mode) {
        case STABILIZE:
        case ACRO:
            // if throttle is zero disarm motors
            if (g.rc_3.control_in == 0) {
                init_disarm_motors();
            }else if(ap.home_is_set == true && home_distance > wp_nav.get_waypoint_radius()) {
                set_mode(RTL);
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode(LAND);
            }
            break;
		case AUTO:
            // failsafe_throttle is 1 do RTL, 2 means continue with the mission
            if (g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_RTL) {
                if(home_distance > wp_nav.get_waypoint_radius()) {
				set_mode(RTL);
                }else{
                    // We are very close to home so we will land
                    set_mode(LAND);
                }
			}
            // if failsafe_throttle is 2 (i.e. FS_THR_ENABLED_CONTINUE_MISSION) no need to do anything
			break;
		default:
            if(ap.home_is_set == true && home_distance > wp_nav.get_waypoint_radius()) {
				set_mode(RTL);
			}else{
                // We have no GPS or are very close to home so we will land
				set_mode(LAND);
			}
			break;
	}

    // log the error to the dataflash
    //Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_OCCURRED);

}

// failsafe_off_event - respond to radio contact being regained
// we must be in AUTO, LAND or RTL modes
// or Stabilize or ACRO mode but with motors disarmed
public function failsafe_radio_off_event():void
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_RESOLVED);
}

public function low_battery_event():void
{
    // failsafe check
    if (g.failsafe_battery_enabled && !ap.low_battery && motors.armed()) {
        switch(control_mode) {
            case STABILIZE:
            case ACRO:
                // if throttle is zero disarm motors
                if (g.rc_3.control_in == 0) {
                    init_disarm_motors();
                }else{
                    set_mode(LAND);
                }
                break;
            case AUTO:
                if(ap.home_is_set == true && home_distance > wp_nav.get_waypoint_radius()) {
					set_mode(RTL);
                }else{
                    // We have no GPS or are very close to home so we will land
                    set_mode(LAND);
                }
                break;
            default:
                set_mode(LAND);
                break;
		}
	}

    // set the low battery flag
    set_low_battery(true);

    // warn the ground station and log to dataflash
    //gcs_send_text_P(SEVERITY_LOW,PSTR("Low Battery!"));
    //Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_BATT, ERROR_CODE_FAILSAFE_OCCURRED);
}

// failsafe_gps_check - check for gps failsafe
public function failsafe_gps_check():void
{
    var last_gps_update_ms:int;

    // return immediately if gps failsafe is disabled
    if( !g.failsafe_gps_enabled ) {
        return;
}

    // calc time since last gps update
    last_gps_update_ms = clock.millis() - g_gps.last_fix_time;

    // check if all is well
    if( last_gps_update_ms < FAILSAFE_GPS_TIMEOUT_MS) {
        // check for recovery from gps failsafe
        if( ap.failsafe_gps ) {
            failsafe_gps_off_event();
            set_failsafe_gps(false);
        }
		return;
    }

    // do nothing if gps failsafe already triggered or motors disarmed
    if( ap.failsafe_gps || !motors.armed()) {
        return;
	}

    // GPS failsafe event has occured
    // update state, warn the ground station and log to dataflash
    set_failsafe_gps(true);
    //gcs_send_text_P(SEVERITY_LOW,PSTR("Lost GPS!"));
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GPS, ERROR_CODE_FAILSAFE_OCCURRED);

    // take action based on flight mode
    switch(control_mode) {
        // for modes that do not require gps, do nothing
        case STABILIZE:
        case ACRO:
        case ALT_HOLD:
        case OF_LOITER:
            // do nothing
            break;

        // modes requiring GPS force a land
        case AUTO:
        case GUIDED:
        case LOITER:
        case RTL:
        case CIRCLE:
        case POSITION:
            // We have no GPS or are very close to home so we will land
            set_mode(LAND);
            break;

        case LAND:
            // if we're already landing do nothing
            break;
    }
}

// failsafe_gps_off_event - actions to take when GPS contact is restored
public function failsafe_gps_off_event():void
{
    // log recovery of GPS in logs?
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GPS, ERROR_CODE_FAILSAFE_RESOLVED);
}

public function update_events()     // Used for MAV_CMD_DO_REPEAT_SERVO and MAV_CMD_DO_REPEAT_RELAY
{
    if(event_repeat == 0 || (clock.millis() - event_timer) < event_delay)
        return;

	if(event_repeat != 0) {		// event_repeat = -1 means repeat forever
        event_timer = clock.millis();

		if (event_id >= CH_5 && event_id <= CH_8) {
			if(event_repeat%2) {
				APM_RC.OutputCh(event_id, event_value); // send to Servos
			} else {
				APM_RC.OutputCh(event_id, event_undo_value);
			}
		}

		if  (event_id == RELAY_TOGGLE) {
			relay.toggle();
		}
        if (event_repeat > 0) {
            event_repeat--;
        }
	}
}

