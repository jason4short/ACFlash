/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

public const CONTROL_SWITCH_COUNTER :int = 10 // 10 iterations at 100hz (i.e. 1/10th of a second) at a new switch position will cause flight mode change
public var switch_counter :int = 0 // 10 iterations at 100hz (i.e. 1/10th of a second) at a new switch position will cause flight mode change

public function read_control_switch():void
{
    //static uint8_t switch_counter = 0;

	var switchPosition:int = readSwitch();

    if (oldSwitchPosition != switchPosition) {
        switch_counter++;
        if(switch_counter >= CONTROL_SWITCH_COUNTER) {
			oldSwitchPosition 	= switchPosition;
            switch_counter          = 0;

            // ignore flight mode changes if in failsafe
            if( !ap.failsafe_radio ) {
				set_mode(flight_modes[switchPosition]);

                if(g.ch7_option != CH7_SIMPLE_MODE) {
					// set Simple mode using stored paramters from Mission planner
					// rather than by the control switch
                    //set_simple_mode(Boolean(g.simple_modes & (1 << switchPosition)));
                    set_simple_mode(g.simple_checkbox.getSelected());
                }
            }
			}
		}else{
        // reset switch_counter if there's been no change
        // we don't want 10 intermittant blips causing a flight mode change
        switch_counter = 0;
	}
}

public function readSwitch():int
{
	return radio_switch_position;
}

public function reset_control_switch():void
{
	oldSwitchPosition = -1;
	read_control_switch();
}

// read at 10 hz
// set this to your trainer switch
public function read_trim_switch():void
{
    // return immediately if the CH7 switch has not changed position
    if (ap_system.CH7_flag == (g.rc_7.radio_in >= CH7_PWM_TRIGGER)) {
        return;
    }

    // set the ch7 flag
    ap_system.CH7_flag = (g.rc_7.radio_in >= CH7_PWM_TRIGGER);

    // multi-mode
    var option:int;

    if(g.ch7_option == CH7_MULTI_MODE) {
        if (g.rc_6.radio_in < CH6_PWM_TRIGGER_LOW) {
			option = CH7_FLIP;
        }else if (g.rc_6.radio_in > CH6_PWM_TRIGGER_HIGH) {
			option = CH7_SAVE_WP;
		}else{
			option = CH7_RTL;
		}
	}else{
		option = g.ch7_option;
	}

    switch(option) {
        case CH7_FLIP:
            // flip if switch is on, positive throttle and we're actually flying
            if(ap_system.CH7_flag && g.rc_3.control_in >= 0 && ap.takeoff_complete) {
				init_flip();
			}
            break;

        case CH7_SIMPLE_MODE:
            set_simple_mode(ap_system.CH7_flag);
            break;

        case CH7_RTL:
            if (ap_system.CH7_flag) {
                // engage RTL
                set_mode(RTL);
            }else{
                // disengage RTL to previous flight mode if we are currently in RTL or loiter
                if (control_mode == RTL || control_mode == LOITER) {
                    reset_control_switch();
				}
			}
            break;

        //case CH7_SAVE_TRIM:
         //   if(ap_system.CH7_flag && control_mode <= ACRO && g.rc_3.control_in == 0) {
                //save_trim();
			//}
           // break;

        case CH7_SAVE_WP:
            // save when CH7 switch is switched off
            if (ap_system.CH7_flag == false) {

                // if in auto mode, reset the mission
                if(control_mode == AUTO) {
					CH7_wp_index = 0;
					g.command_total = 1;
					set_mode(RTL);
					return;
				}

                if(CH7_wp_index == 0) {
					// this is our first WP, let's save WP 1 as a takeoff
					// increment index to WP index of 1 (home is stored at 0)
					CH7_wp_index = 1;

					var temp:Location = home.clone();
					// set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    temp.id = MAV_CMD_NAV_TAKEOFF;
                    temp.alt = current_loc.alt;

					// save command:
					// we use the current altitude to be the target for takeoff.
					// only altitude will matter to the AP mission script for takeoff.
					// If we are above the altitude, we will skip the command.
					trace("save loc");
					set_cmd_with_index(temp, CH7_wp_index);
				}

				// increment index
				CH7_wp_index++;

				// set the next_WP (home is stored at 0)
				// max out at 100 since I think we need to stay under the EEPROM limit
				CH7_wp_index = constrain(CH7_wp_index, 1, 100);

                if(g.rc_3.control_in > 0) {
					// set our location ID to 16, MAV_CMD_NAV_WAYPOINT
					current_loc.id = MAV_CMD_NAV_WAYPOINT;
				}else{
					// set our location ID to 21, MAV_CMD_NAV_LAND
					current_loc.id = MAV_CMD_NAV_LAND;
				}

				// save command
				set_cmd_with_index(current_loc, CH7_wp_index);

                // Cause the CopterLEDs to blink twice to indicate saved waypoint
                copter_leds_nav_blink = 10;
            }
            break;

        case CH7_SONAR:
            // enable or disable the sonar
            g.sonar_enabled = ap_system.CH7_flag;
            break;

        case CH7_ARM:
            break;            
	}
}

// save_trim - adds roll and pitch trims from the radio to ahrs
public function save_trim():void
{
    // save roll and pitch trim
    //var roll_trim:Number 	= radians(Number(g.rc_1.control_in) / 100.0);
    //var pitch_trim:Number = radians(Number(g.rc_2.control_in) / 100.0);
    //ahrs.add_trim(roll_trim, pitch_trim);
}

// auto_trim - slightly adjusts the ahrs.roll_trim and ahrs.pitch_trim towards the current stick positions
// meant to be called continuously while the pilot attempts to keep the copter level
public function auto_trim():void
{
    if(auto_trim_counter > 0) {
        auto_trim_counter--;

        // flash the leds
        led_mode = SAVE_TRIM_LEDS;

        // calculate roll trim adjustment
        var roll_trim_adjustment:Number = radians(Number(g.rc_1.control_in) / 4000.0);

        // calculate pitch trim adjustment
        var pitch_trim_adjustment:Number = radians(Number(g.rc_2.control_in) / 4000.0);

        // make sure accelerometer values impact attitude quickly
        //ahrs.set_fast_gains(true);

        // add trim to ahrs object
        // save to eeprom on last iteration
        //ahrs.add_trim(roll_trim_adjustment, pitch_trim_adjustment, (auto_trim_counter == 0));

        // on last iteration restore leds and accel gains to normal
        if(auto_trim_counter == 0) {
            //ahrs.set_fast_gains(false);
            led_mode = NORMAL_LEDS;
        }
    }
}

