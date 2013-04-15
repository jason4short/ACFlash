public const TOY_ALT_SMALL :int = 25
public const TOY_ALT_LARGE :int = 100

//called at 10hz
public function update_toy_throttle():void
{
	// look for a change in throttle position to exit throttle hold
	if(Math.abs(g.rc_3.control_in - saved_toy_throttle) > 40){
		throttle_mode 	= THROTTLE_MANUAL;
	}
}

//called at 10hz
public function update_toy_altitude():void
{
    var input:int = g.rc_3.radio_in;     // throttle
    var current_target_alt:Number = wp_nav.get_desired_alt();

	// Trigger upward alt change
    if(false == CH7_toy_flag && input > 1666) {
		CH7_toy_flag = true;
		// go up
        if(current_target_alt >= 400) {
            wp_nav.set_desired_alt(current_target_alt + TOY_ALT_LARGE);
		}else{
            wp_nav.set_desired_alt(current_target_alt + TOY_ALT_SMALL);
		}

	// Trigger downward alt change
    }else if(false == CH7_toy_flag && input < 1333) {
		CH7_toy_flag = true;
		// go down
        if(current_target_alt >= (400 + TOY_ALT_LARGE)) {
            wp_nav.set_desired_alt(current_target_alt - TOY_ALT_LARGE);
        }else if(current_target_alt >= TOY_ALT_SMALL) {
            wp_nav.set_desired_alt(current_target_alt - TOY_ALT_SMALL);
        }else if(current_target_alt < TOY_ALT_SMALL) {
            wp_nav.set_desired_alt(0); 
		}

	// clear flag
    }else if (CH7_toy_flag && ((input < 1666) && (input > 1333))) {
		CH7_toy_flag = false;
	}
}

// called at 50 hz from all flight modes
public function edf_toy():void
{
	// EDF control:
	g.rc_8.radio_out = 1000 + ((Math.abs(g.rc_2.control_in) << 1) / 9);
	if(g.rc_8.radio_out < 1050)
		g.rc_8.radio_out = 1000;

	// output throttle to EDF
    if(motors.armed()) {
		APM_RC.OutputCh(CH_8, g.rc_8.radio_out);
	}else{
		APM_RC.OutputCh(CH_8, 1000);
	}

	// output Servo direction
	if(g.rc_2.control_in > 0){
		copter.edf = 1000 - g.rc_8.radio_out;
		APM_RC.OutputCh(CH_6, 1000); // 1000 : 2000
	}else{
		copter.edf = g.rc_8.radio_out - 1000;
		APM_RC.OutputCh(CH_6, 2000); // less than 1450
	}
}

// The function call for managing the flight mode Toy
public function roll_pitch_toy():void
{
	var yaw_rate:int = g.rc_1.control_in / g.toy_yaw_rate;

    if(g.rc_1.control_in != 0) {    // roll
		get_acro_yaw(yaw_rate/2);
        ap_system.yaw_stopped = false;
		yaw_timer = 150;

    }else if (!ap_system.yaw_stopped) {
        get_acro_yaw(0);
		yaw_timer--;

        if((yaw_timer == 0) || (Math.abs(omega.z) < 0.17)) {
            ap_system.yaw_stopped = true;
			nav_yaw = ahrs.yaw_sensor;
		}
	}else{
        if(motors.armed() == false || g.rc_3.control_in == 0)
			nav_yaw = ahrs.yaw_sensor;

        get_stabilize_yaw(nav_yaw);
	}

    // roll_rate is the outcome of the linear equation or lookup table
    // based on speed and Yaw rate
    var roll_rate:int = 0;

    roll_rate = -(g.rc_2.control_in * (yaw_rate/100)) /30;
    //trace("roll_rate: ",roll_rate);
    roll_rate = constrain(roll_rate, -2000, 2000);

	if(g.toy_edf){
		// Output the attitude
	    get_stabilize_roll(roll_rate);
    	get_stabilize_pitch(g.rc_6.control_in);             // use CH6 to trim pitch
	}else{
		// Output the attitude
	    get_stabilize_roll(roll_rate);
	    get_stabilize_pitch(g.rc_2.control_in);
	}

}
