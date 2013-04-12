/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

public function
get_stabilize_roll(target_angle:int):void
{
	var angle_error		:Number = 0;
	var rate			:Number = 0;
	var target_rate		:int = 0;
	var iterm			:Number = 0;

	target_angle 		= wrap_180(target_angle - ahrs.roll_sensor);

    // limit the error we're feeding to the PID
    target_angle            = constrain(target_angle, -4500, 4500);

	// convert to desired Rate:
	p_stab 				= g.pi_stabilize_roll.get_p(target_angle);

	if(Math.abs(ahrs.roll_sensor) < 500){
		angle_error 		= constrain(angle_error, -500, 500);
		i_stab 				= g.pi_stabilize_roll.get_i(angle_error, G_Dt);
	}else{
		i_stab 				= g.pi_stabilize_roll.get_integrator();
	}

	// set targets for rate controller
	set_roll_rate_target(p_stab+i_stab, EARTH_FRAME);
}

public function
get_stabilize_pitch(target_angle:Number):void
{
	var target_rate		:Number = 0;
	var i_stab			:Number = 0;

	// angle error
	target_angle 		= wrap_180(target_angle - ahrs.pitch_sensor);
	//trace(target_angle, ahrs.pitch_sensor);

	// convert to desired Rate:
	target_rate 		= g.pi_stabilize_pitch.get_p(target_angle);

	if(Math.abs(ahrs.roll_sensor) < 500){
		target_angle 		= constrain(target_angle, -500, 500);
		i_stab 				= g.pi_stabilize_pitch.get_i(target_angle, G_Dt);
	}else{
		i_stab 				= g.pi_stabilize_pitch.get_integrator();
	}

	// set targets for rate controller
	set_pitch_rate_target(target_rate + i_stab, EARTH_FRAME);
}

public function
get_stabilize_yaw(target_angle:Number):void
{
	var target_rate		:Number = 0;
	var i_term			:Number = 0;
	var angle_error		:Number = 0;
	var output			:Number = 0;

	// angle error
	angle_error	 	= wrap_180(target_angle - ahrs.yaw_sensor);

	// limit the error we're feeding to the PID
	angle_error 	= constrain(angle_error, -4000, 4000);

	// convert angle error to desired Rate:
	target_rate = g.pi_stabilize_yaw.get_p(angle_error);
	i_term = g.pi_stabilize_yaw.get_i(angle_error, G_Dt);

	// do not use rate controllers for helicotpers with external gyros
	output = get_rate_yaw(target_rate) + i_term;

	// set targets for rate controller
	set_yaw_rate_target(target_rate+i_term, EARTH_FRAME);
}



public function
get_acro_roll(target_rate:int):void
{
	target_rate = target_rate * g.acro_p;

	// set targets for rate controller
	set_roll_rate_target(target_rate, BODY_FRAME);
}

public function
get_acro_pitch(target_rate:int):void
{
	target_rate = target_rate * g.acro_p;

	// set targets for rate controller
	set_pitch_rate_target(target_rate, BODY_FRAME);
}

public function
get_acro_yaw(target_rate:int):void
{
	target_rate = g.pi_stabilize_yaw.get_p(target_rate);

	// set targets for rate controller
	set_yaw_rate_target(target_rate, BODY_FRAME);
}



// Roll with rate input and stabilized in the earth frame
public function
get_roll_rate_stabilized_ef(stick_angle:int):void
{
	var angle_error:int = 0;

	// convert the input to the desired roll rate
	var target_rate:int = stick_angle * g.acro_p - (roll_axis * g.acro_balance_roll)/100;

	// convert the input to the desired roll rate
	roll_axis += target_rate * G_Dt;
	roll_axis = wrap_180(roll_axis);

	// ensure that we don't reach gimbal lock
	if (Math.abs(roll_axis) > 4500 && g.acro_trainer_enabled) {
		roll_axis	= constrain(roll_axis, -4500, 4500);
		angle_error = wrap_180(roll_axis - ahrs.roll_sensor);
	} else {
		// angle error with maximum of +- max_angle_overshoot
		angle_error = wrap_180(roll_axis - ahrs.roll_sensor);
		angle_error	= constrain(angle_error, -MAX_ROLL_OVERSHOOT, MAX_ROLL_OVERSHOOT);
	}

	if (motors.armed() == false || ((g.rc_3.control_in == 0) && !ap.failsafe_radio)) {
		angle_error = 0;
	}

	// update roll_axis to be within max_angle_overshoot of our current heading
	roll_axis = wrap_180(angle_error + ahrs.roll_sensor);

	// set earth frame targets for rate controller

    // set earth frame targets for rate controller
	set_roll_rate_target(g.pi_stabilize_roll.get_p(angle_error) + target_rate, EARTH_FRAME);
}

// Pitch with rate input and stabilized in the earth frame
public function
get_pitch_rate_stabilized_ef(stick_angle:int):void
{
	var angle_error:int = 0;

	// convert the input to the desired pitch rate
	var target_rate:int = stick_angle * g.acro_p - (pitch_axis * g.acro_balance_pitch)/100;

	// convert the input to the desired pitch rate
	pitch_axis += target_rate * G_Dt;
	pitch_axis = wrap_180(pitch_axis);

	// ensure that we don't reach gimbal lock
	if (Math.abs(pitch_axis) > 4500) {
		pitch_axis	= constrain(pitch_axis, -4500, 4500);
		angle_error = wrap_180(pitch_axis - ahrs.pitch_sensor);
	} else {
		// angle error with maximum of +- max_angle_overshoot
		angle_error = wrap_180(pitch_axis - ahrs.pitch_sensor);
		angle_error	= constrain(angle_error, -MAX_PITCH_OVERSHOOT, MAX_PITCH_OVERSHOOT);
	}

	if (motors.armed() == false || ((g.rc_3.control_in == 0) && !ap.failsafe_radio)) {
		angle_error = 0;
	}

	// update pitch_axis to be within max_angle_overshoot of our current heading
	pitch_axis = wrap_180(angle_error + ahrs.pitch_sensor);

	// set earth frame targets for rate controller
	set_pitch_rate_target(g.pi_stabilize_pitch.get_p(angle_error) + target_rate, EARTH_FRAME);
}

// Yaw with rate input and stabilized in the earth frame
public function
get_yaw_rate_stabilized_ef(stick_angle:int):void
{

	var angle_error:int = 0;

	// convert the input to the desired yaw rate
	var target_rate:int = stick_angle * g.acro_p;

	// convert the input to the desired yaw rate
	nav_yaw += target_rate * G_Dt;
	nav_yaw = wrap_360(nav_yaw);

	// calculate difference between desired heading and current heading
	angle_error = wrap_180(nav_yaw - ahrs.yaw_sensor);

	// limit the maximum overshoot
	angle_error	= constrain(angle_error, -MAX_YAW_OVERSHOOT, MAX_YAW_OVERSHOOT);

	if (motors.armed() == false || ((g.rc_3.control_in == 0) && !ap.failsafe_radio)) {
		angle_error = 0;
	}

	// update nav_yaw to be within max_angle_overshoot of our current heading
	nav_yaw = wrap_360(angle_error + ahrs.yaw_sensor);

	// set earth frame targets for rate controller
	set_yaw_rate_target(g.pi_stabilize_yaw.get_p(angle_error)+target_rate, EARTH_FRAME);
}

// set_roll_rate_target - to be called by upper controllers to set roll rate targets in the earth frame
public function
set_roll_rate_target(desired_rate:int, earth_or_body_frame:int):void
{

	rate_targets_frame = earth_or_body_frame;
	if(earth_or_body_frame == BODY_FRAME){
		roll_rate_target_bf = desired_rate;
	}else{
		roll_rate_target_ef = desired_rate;
	}
}

// set_pitch_rate_target - to be called by upper controllers to set pitch rate targets in the earth frame
public function
set_pitch_rate_target(desired_rate:int, earth_or_body_frame:int):void
{
	rate_targets_frame = earth_or_body_frame;
	if( earth_or_body_frame == BODY_FRAME ) {
		pitch_rate_target_bf = desired_rate;
	}else{
		pitch_rate_target_ef = desired_rate;
	}
}

// set_yaw_rate_target - to be called by upper controllers to set yaw rate targets in the earth frame
public function
set_yaw_rate_target(desired_rate:int, earth_or_body_frame:int):void
{
	rate_targets_frame = earth_or_body_frame;
	if( earth_or_body_frame == BODY_FRAME ) {
		yaw_rate_target_bf = desired_rate;
	}else{
		yaw_rate_target_ef = desired_rate;
	}
}

// update_rate_contoller_targets - converts earth frame rates to body frame rates for rate controllers
public function
update_rate_contoller_targets():void
{
	if( rate_targets_frame == EARTH_FRAME ) {
		// convert earth frame rates to body frame rates
		roll_rate_target_bf 	= roll_rate_target_ef - sin_pitch * yaw_rate_target_ef;
		pitch_rate_target_bf 	= cos_roll_x  * pitch_rate_target_ef + sin_roll * cos_pitch_x * yaw_rate_target_ef;
		yaw_rate_target_bf 		= cos_pitch_x * cos_roll_x * yaw_rate_target_ef - sin_roll * pitch_rate_target_ef;
	}
}

// run roll, pitch and yaw rate controllers and send output to motors
// targets for these controllers comes from stabilize controllers
public function
run_rate_controllers():void
{
	// call rate controllers
	g.rc_1.servo_out = get_rate_roll(roll_rate_target_bf);
	g.rc_2.servo_out = get_rate_pitch(pitch_rate_target_bf);
	g.rc_4.servo_out = get_rate_yaw(yaw_rate_target_bf);

	// run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
	if( g.throttle_accel_enabled && throttle_accel_controller_active ) {
		set_throttle_out(get_throttle_accel(throttle_accel_target_ef), true);
	}
}

public function
get_rate_roll(target_rate:Number):Number
{
	var current_rate	:Number = 0;
	var output			:Number = 0;

	// get current rate
	current_rate 	= (ahrs.omega.x * DEGX100);

	// call pid controller
	roll_rate_error	= target_rate - current_rate;

	p_stab_rate 	= g.pid_rate_roll.get_p(roll_rate_error);

	// freeze I term if we've breached roll-pitch limits
	if( motors.reached_limit(AP_MOTOR_ROLLPITCH_LIMIT) ) {
		i_stab_rate	= g.pid_rate_roll.get_integrator();
	}else{
		i_stab_rate = g.pid_rate_roll.get_i(roll_rate_error, G_Dt);
	}

	d_stab_rate		= g.pid_rate_roll.get_d(roll_rate_error, G_Dt);
	output			= p_stab_rate + i_stab_rate + d_stab_rate;

	// constrain output
	output = constrain(output, -5000, 5000);

	// debug
	roll_output = output

	// output control
	return output;
}

public function get_rate_pitch(target_rate:Number):Number
{
	var p,i,d:int;
	var current_rate;
	var rate_error:int;
	var output;

	// get current rate
	current_rate 	= (ahrs.omega.y * DEGX100);

	// call pid controller
	rate_error = target_rate - current_rate;
	p = g.pid_rate_pitch.get_p(rate_error);

	// freeze I term if we've breached roll-pitch limits
	if( motors.reached_limit(AP_MOTOR_ROLLPITCH_LIMIT) ) {
		i = g.pid_rate_pitch.get_integrator();
	}else{
		i = g.pid_rate_pitch.get_i(rate_error, G_Dt);
	}
	d = g.pid_rate_pitch.get_d(rate_error, G_Dt);
	output = p + i + d;

	// constrain output
	output = constrain(output, -5000, 5000);

	// output control
	return output;
}

public function get_rate_yaw(target_rate:Number):int
{
	var p,i,d			:int;
	var rate_error		:int;
	var output			:int;

	// rate control
	rate_error	 	= target_rate - (ahrs.omega.z * DEGX100);

	// separately calculate p, i, d values for logging
	p = g.pid_rate_yaw.get_p(rate_error);
	// freeze I term if we've breached yaw limits
	if( motors.reached_limit(AP_MOTOR_YAW_LIMIT) ) {
		i = g.pid_rate_yaw.get_integrator();
	}else{
		i = g.pid_rate_yaw.get_i(rate_error, G_Dt);
	}
	d = g.pid_rate_yaw.get_d(rate_error, G_Dt);

	output  = p+i+d;
	output = constrain(output, -4500, 4500);


	// output control:
	var yaw_limit:int = 2200 + Math.abs(g.rc_4.control_in);

	// smoother Yaw control:
	return constrain(output, -yaw_limit, yaw_limit);
}


// calculate modified roll/pitch depending upon optical flow calculated position
public function
get_of_roll(input_roll:int):int
{
	return input_roll;
}

public function
get_of_pitch(input_pitch:int):int
{
	return input_pitch;
}

/*************************************************************
 * yaw controllers
 *************************************************************/

public var look_at_yaw_counter_cy:int = 0;     // used to reduce update rate to 10hz

 // get_look_at_yaw - updates bearing to look at center of circle or do a panorama
// should be called at 100hz
public function get_circle_yaw():void
{
    // if circle radius is zero do panorama
    if( g.circle_radius == 0 ) {
        // slew yaw towards circle angle
        nav_yaw = get_yaw_slew(nav_yaw, degrees(circle_angle) * 100, AUTO_YAW_SLEW_RATE);
    }else{
        look_at_yaw_counter_cy++;
        if( look_at_yaw_counter_cy >= 10 ) {
            look_at_yaw_counter_cy = 0;
            yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), yaw_look_at_WP);
        }
        // slew yaw
        nav_yaw = get_yaw_slew(nav_yaw, yaw_look_at_WP_bearing, AUTO_YAW_SLEW_RATE);
    }

    // call stabilize yaw controller
    get_stabilize_yaw(nav_yaw);
}

public var look_at_yaw_counter_y:int = 0;     // used to reduce update rate to 10hz
// get_look_at_yaw - updates bearing to location held in look_at_yaw_WP and calls stabilize yaw controller
// should be called at 100hz
public function get_look_at_yaw():void
{
    //static uint8_t look_at_yaw_counter_y = 0;     // used to reduce update rate to 10hz

    look_at_yaw_counter_y++;
    if( look_at_yaw_counter_y >= 10 ) {
        look_at_yaw_counter_y = 0;
        yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), yaw_look_at_WP);
    }

    // slew yaw and call stabilize controller
    nav_yaw = get_yaw_slew(nav_yaw, yaw_look_at_WP_bearing, AUTO_YAW_SLEW_RATE);
    get_stabilize_yaw(nav_yaw);
}

public function get_look_ahead_yaw(pilot_yaw:int):void
{
    // Commanded Yaw to automatically look ahead.
    if (g_gps.fix && g_gps.ground_course > YAW_LOOK_AHEAD_MIN_SPEED) {
        nav_yaw = get_yaw_slew(nav_yaw, g_gps.ground_course, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(wrap_360(nav_yaw + pilot_yaw));   // Allow pilot to "skid" around corners up to 45 degrees
    }else{
        nav_yaw += pilot_yaw * g.acro_p * G_Dt;
        nav_yaw = wrap_360(nav_yaw);
        get_stabilize_yaw(nav_yaw);
    }
}
/*************************************************************
 *  throttle control
 ****************************************************************/

// update_throttle_cruise - update throttle cruise if necessary
public function
update_throttle_cruise(throttle:int):void
{
	// ensure throttle_avg has been initialised
	if( throttle_avg == 0 ) {
		throttle_avg = g.throttle_cruise;
	}
	// calc average throttle if we are in a level hover
	if (throttle > g.throttle_min && Math.abs(climb_rate) < 60 && Math.abs(ahrs.roll_sensor) < 500 && Math.abs(ahrs.pitch_sensor) < 500) {
		throttle_avg = throttle_avg * 0.99 + Number(throttle) * 0.01;
		g.throttle_cruise = throttle_avg;
	}
}

// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
public function
get_angle_boost(throttle:int):int
{
	var throttle_out:int;
	var temp:Number = cos_pitch_x * cos_roll_x;
	temp = constrain(temp, 0.5, 1.0);

	// reduce throttle if we go inverted
	temp = constrain(9000 - Math.max(Math.abs(ahrs.roll_sensor), Math.abs(ahrs.pitch_sensor)), 0, 3000) / (3000 * temp);

	// apply scale and constrain throttle
	throttle_out = constrain((throttle - g.throttle_min) * temp + g.throttle_min, g.throttle_min, 1000);

	// to allow logging of angle boost
	angle_boost = throttle_out - throttle;

	return throttle_out;
}

 // set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
 // provide 0 to cut motors
public function
set_throttle_out(throttle_out:int, apply_angle_boost:Boolean ):void
{
	if(apply_angle_boost){
		g.rc_3.servo_out = get_angle_boost(throttle_out);
	}else{
		g.rc_3.servo_out = throttle_out;
		// clear angle_boost for logging purposes
		angle_boost = 0;
	}
	//trace("throttle_out", throttle_out, "g.rc_3.servo_out", g.rc_3.servo_out);

	// update compass with throttle value
	//compass.set_throttle(g.rc_3.servo_out / 1000.0);
}

// set_throttle_accel_target - to be called by upper throttle controllers to set desired vertical acceleration in earth frame
public function
set_throttle_accel_target(desired_acceleration:int):void
{
	if(g.throttle_accel_enabled){
		throttle_accel_target_ef = desired_acceleration;
		throttle_accel_controller_active = true;
	}else{
		// To-Do log dataflash or tlog error
		//cliSerial.print_P(PSTR("Err: target sent to inactive acc thr controller!\n"));
	}
}

// disable_throttle_accel - disables the accel based throttle controller
// it will be re-enasbled on the next set_throttle_accel_target
// required when we wish to set motors to zero when pilot inputs zero throttle
public function
throttle_accel_deactivate():void
{
	throttle_accel_controller_active = false;
}

// get_throttle_accel - accelerometer based throttle controller
// returns an actual throttle output (0 ~ 1000) to be sent to the motors

public var z_accel_error:Number = 0;	 // The acceleration error in cm.
public var last_call_ms_gta:int = 0;
public function
get_throttle_accel(z_target_accel:int):int
{

	var output:int = 0;
	var z_accel_meas:Number;
	var now:int = clock.millis();

	// Calculate Earth Frame Z acceleration
	//z_accel_meas = -(ahrs.get_accel_ef().z + GRAVITY_MSS) * 100;
	z_accel_meas = -(ahrs.get_accel_ef().z + GRAVITY_MSS) * 100;
	//trace(z_target_accel.toFixed(2), ahrs.accel.z.toFixed(2), z_accel_meas.toFixed(2), (ahrs.accel.z + GRAVITY_MSS).toFixed(2));
	
	// reset target altitude if this controller has just been engaged
	if( now - last_call_ms_gta > 100 ) {
		// Reset Filter
		z_accel_error = 0;
	} else {
		// calculate accel error and Filter with fc = 2 Hz
		z_accel_error += 0.11164 * (constrain(z_target_accel - z_accel_meas, -32000, 32000) - z_accel_error);
	}

	//trace("z_accel_error", (z_accel_error/100).toFixed(2));
	//trace("g.throttle_cruise", g.throttle_cruise.toFixed(2));

	last_call_ms_gta = now;

	// separately calculate p, i, d values for logging
	p_accel_rate = g.pid_throttle_accel.get_p(z_accel_error);
	// freeze I term if we've breached throttle limits
	if( motors.reached_limit(AP_MOTOR_THROTTLE_LIMIT) ) {
		i_accel_rate = g.pid_throttle_accel.get_integrator();
	}else{
		i_accel_rate = g.pid_throttle_accel.get_i(z_accel_error, G_Dt);
	}
	d_accel_rate = g.pid_throttle_accel.get_d(z_accel_error, G_Dt);

	//
	// limit the rate
	output =  constrain(p_accel_rate + i_accel_rate + p_accel_rate + g.throttle_cruise, g.throttle_min, g.throttle_max);
	//trace("output", output);

	return output;
}


// get_pilot_desired_throttle - transform pilot's throttle input to make cruise throttle mid stick
// used only for manual throttle modes
// returns throttle output 0 to 1000
public const THROTTLE_IN_MIDDLE:int = 500;
public function
get_pilot_desired_throttle(throttle_control:int):int
{
	var throttle_out:int = 0;

	// exit immediately in the simple cases
	if( throttle_control == 0 || g.throttle_mid == 500) {
		return throttle_control;
	}

	// ensure reasonable throttle values
	throttle_control = constrain(throttle_control, 0, 1000);
	g.throttle_mid = constrain(g.throttle_mid, 300, 700);

	// check throttle is above, below or in the deadband
	if (throttle_control < THROTTLE_IN_MIDDLE) {
		// below the deadband
		throttle_out = g.throttle_min + ((throttle_control - g.throttle_min)) * ((g.throttle_mid - g.throttle_min)) / ((500 - g.throttle_min));
	}else if(throttle_control > THROTTLE_IN_MIDDLE) {
		// above the deadband
		throttle_out = g.throttle_mid + ((throttle_control - 500)) * (1000 - g.throttle_mid) / 500.0;
	}else{
		// must be in the deadband
		throttle_out = g.throttle_mid;
	}
	return throttle_out;
}


// get_pilot_desired_climb_rate - transform pilot's throttle input to
// climb rate in cm/s.  we use radio_in instead of control_in to get the full range
// without any deadzone at the bottom
public const THROTTLE_IN_DEADBAND 	 		:int = 100;  // the throttle input channel's deadband in PWM
public const THROTTLE_IN_DEADBAND_TOP 		:int = (THROTTLE_IN_MIDDLE + THROTTLE_IN_DEADBAND); // top of the deadband
public const THROTTLE_IN_DEADBAND_BOTTOM 	:int = (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND); // bottom of the deadband
public function
get_pilot_desired_climb_rate(throttle_control:int):int
{
	var desired_rate:int = 0;

	// throttle failsafe check
	if(ap.failsafe_radio) {
		return 0;
	}

	// ensure a reasonable throttle value
	throttle_control = constrain(throttle_control,0,1000);

	// check throttle is above, below or in the deadband
	if (throttle_control < THROTTLE_IN_DEADBAND_BOTTOM) {
		// below the deadband
		desired_rate = g.pilot_velocity_z_max * (throttle_control-THROTTLE_IN_DEADBAND_BOTTOM) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
	}else if (throttle_control > THROTTLE_IN_DEADBAND_TOP) {
		// above the deadband
		desired_rate = g.pilot_velocity_z_max * (throttle_control-THROTTLE_IN_DEADBAND_TOP) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
	}else{
		// must be in the deadband
		desired_rate = 0;
	}

	// desired climb rate for logging
	//desired_climb_rate = desired_rate;
	return desired_rate;
}


// get_pilot_desired_acceleration - transform pilot's throttle input to a desired acceleration
// default upper and lower bounds are 500 cm/s/s (roughly 1/2 a G)
// returns acceleration in cm/s/s
public function
get_pilot_desired_acceleration(throttle_control:int):int
{
	var desired_accel:int = 0;

	// throttle failsafe check
	if( ap.failsafe_radio ) {
		return 0;
	}

	// ensure a reasonable throttle value
	throttle_control = constrain(throttle_control, 0, 1000);

	// check throttle is above, below or in the deadband
	if (throttle_control < THROTTLE_IN_DEADBAND_BOTTOM) {
		// below the deadband
		desired_accel = ACCELERATION_MAX_Z * (throttle_control-THROTTLE_IN_DEADBAND_BOTTOM) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
	}else if(throttle_control > THROTTLE_IN_DEADBAND_TOP) {
		// above the deadband
		desired_accel = ACCELERATION_MAX_Z * (throttle_control-THROTTLE_IN_DEADBAND_TOP) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
	}else{
		// must be in the deadband
		desired_accel = 0;
	}

	return desired_accel;
}


// get_pilot_desired_direct_alt - transform pilot's throttle input to a desired altitude
// return altitude in cm between 0 to 10m
public function
get_pilot_desired_direct_alt(throttle_control:int):int
{
	var desired_alt:int = 0;

	// radio failsafe check
	if( ap.failsafe_radio ) {
		return 0;
	}

	// ensure a reasonable throttle value
	throttle_control = constrain(throttle_control,0,1000);

	desired_alt = throttle_control;

	return desired_alt;
}

// get_initial_alt_hold - get new target altitude based on current altitude and climb rate
public function
get_initial_alt_hold(alt_cm:int, climb_rate_cms:int):int
{
	var target_alt:int;
	var linear_distance:int;	  // half the distace we swap between linear and sqrt and the distace we offset sqrt.
	var linear_velocity:int;	  // the velocity we swap between linear and sqrt.

    linear_velocity = ALT_HOLD_ACCEL_MAX/g.pi_alt_hold.kP();

	if(Math.abs(climb_rate_cms) < linear_velocity){
		target_alt = alt_cm + climb_rate_cms/g.pi_alt_hold.kP();
	} else {

		linear_distance = ALT_HOLD_ACCEL_MAX / (2 * g.pi_alt_hold.kP() * g.pi_alt_hold.kP());

		if (climb_rate_cms > 0){
			target_alt = alt_cm + linear_distance + climb_rate_cms* climb_rate_cms / (2 * ALT_HOLD_ACCEL_MAX);
		}else{
			target_alt = alt_cm - (linear_distance + climb_rate_cms * climb_rate_cms / (2 * ALT_HOLD_ACCEL_MAX));
		}
	}
	return constrain(target_alt, alt_cm - ALT_HOLD_INIT_MAX_OVERSHOOT, alt_cm + ALT_HOLD_INIT_MAX_OVERSHOOT);
}



public var last_call_ms_gtr					:int = 0;	// the last time this controller was called
public var z_target_speed_last:Number = 0;   // The requested speed from the previous iteration

// call at 10hz
public function
get_throttle_rate(z_target_speed:Number):void
{
	var output:Number = 0;
	var now:int = clock.millis();

	// reset target altitude if this controller has just been engaged
	if( now - last_call_ms_gtr > 100 ) {
		// Reset Filter
		z_rate_error	= 0;
        output = 0;
	} else {
		// calculate rate error and filter with cut off frequency of 2 Hz
        z_rate_error = z_rate_error + 0.20085 * ((z_target_speed - climb_rate) - z_rate_error);
        // feed forward acceleration based on change in desired speed.
        output = (z_target_speed - z_target_speed_last) * 50.0;   // To-Do: replace 50 with dt
	}
	last_call_ms_gtr = now;

    // store target speed for next iteration
    z_target_speed_last = z_target_speed;

	// separately calculate p, i, d values for logging
	p_alt_rate			= g.pid_throttle.get_p(z_rate_error);

	// freeze I term if we've breached throttle limits
	if(motors.reached_limit(AP_MOTOR_THROTTLE_LIMIT)) {
		i_alt_rate = g.pid_throttle.get_integrator();
	}else{
		i_alt_rate	= g.pid_throttle.get_i(z_rate_error, m_dt);
	}
	d_alt_rate = g.pid_throttle.get_d(z_rate_error, m_dt);

	// consolidate target acceleration
	output =  p_alt_rate + i_alt_rate + d_alt_rate;
    output = constrain(output, -32000, 32000);

	// send output to accelerometer based throttle controller if enabled otherwise send directly to motors	
	if( g.throttle_accel_enabled ) {
		// set target for accel based throttle controller
		set_throttle_accel_target(output);
		//trace("set_throttle_accel_target", output);
	}else{
		set_throttle_out(g.throttle_cruise + output, true);
		//trace("set_throttle_out", (g.throttle_cruise + output));
	}

    // limit loiter & waypoint navigation from causing too much lean
    // To-Do: ensure that this limit is cleared when this throttle controller is not running so that loiter is not left constrained for Position mode
    wp_nav.set_angle_limit(4500 - constrain((z_rate_error - 100) * 10, 0, 3500));

	// update throttle cruise
	// TO-DO: this may not be correct because g.rc_3.servo_out has not been updated for this iteration
	if( z_target_speed == 0 ) {
		update_throttle_cruise(g.rc_3.servo_out);
	}
}

// get_throttle_althold - hold at the desired altitude in cm
// updates accel based throttle controller targets
// Note: max_climb_rate is an optional parameter to allow reuse of this function by landing controller
public function
get_throttle_althold(target_alt:Number, min_climb_rate:Number, max_climb_rate:Number)
{
	var alt_error:int;
	var desired_rate:Number;
	var linear_distance:int;	  // half the distace we swap between linear and sqrt and the distace we offset sqrt.

	// calculate altitude error
	alt_error	= target_alt - current_loc.alt;

	// check kP to avoid division by zero
	if( g.pi_alt_hold.kP() != 0 ) {
		linear_distance = ALT_HOLD_ACCEL_MAX / (2 * g.pi_alt_hold.kP() * g.pi_alt_hold.kP());

		if(alt_error > (2 * linear_distance)){
			desired_rate = safe_sqrt(2 * ALT_HOLD_ACCEL_MAX * (alt_error - linear_distance));

		}else if(alt_error < -(2 * linear_distance)){
			desired_rate = -safe_sqrt(2 * ALT_HOLD_ACCEL_MAX * (-alt_error - linear_distance));

		}else{
			desired_rate = g.pi_alt_hold.get_p(alt_error);
		}
	}else{
		desired_rate = 0;
	}

	desired_rate = constrain(desired_rate, min_climb_rate, max_climb_rate);
	//trace("desired_rate", desired_rate);

	// call rate based throttle controller which will update accel based throttle controller targets
	get_throttle_rate(desired_rate);

	// update altitude error reported to GCS
	altitude_error = alt_error;

	// TO-DO: enabled PID logging for this controller
}

// get_throttle_althold_with_slew - altitude controller with slew to avoid step changes in altitude target
// calls normal althold controller which updates accel based throttle controller targets
public function
get_throttle_althold_with_slew(target_alt:int, min_climb_rate:int, max_climb_rate:int):void
{
	// limit target altitude change
	controller_desired_alt += constrain(target_alt - controller_desired_alt, min_climb_rate * 0.02, max_climb_rate * 0.02);
	// do not let target altitude get too far from current altitude
	controller_desired_alt = constrain(controller_desired_alt, current_loc.alt - 750, current_loc.alt + 750);

	get_throttle_althold(controller_desired_alt, min_climb_rate - 250, max_climb_rate + 250);   // 250 is added to give head room to alt hold controller
}


public function
get_throttle_rate_stabilized(target_rate:int):void
{
	controller_desired_alt += target_rate * 0.02;
	// do not let target altitude get too far from current altitude
	controller_desired_alt = constrain(controller_desired_alt, current_loc.alt - 750, current_loc.alt + 750);

    // update target altitude for reporting purposes
    set_target_alt_for_reporting(controller_desired_alt);
    
	//set_new_altitude(controller_desired_alt);
	get_throttle_althold(controller_desired_alt, -g.pilot_velocity_z_max-250, g.pilot_velocity_z_max+250);   // 250 is added to give head room to alt hold controller
}


// get_throttle_land - high level landing logic
// sends the desired acceleration in the accel based throttle controller
// called at 50hz
public function
get_throttle_land():void
{
	// if we are above 10m and the sonar does not sense anything perform regular alt hold descent
	if (current_loc.alt >= LAND_START_ALT && !(g.sonar_enabled && sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
		get_throttle_althold_with_slew(LAND_START_ALT, g.auto_velocity_z_min, -Math.abs(g.land_speed));
	}else{
		get_throttle_rate_stabilized(-Math.abs(g.land_speed));

		// detect whether we have landed by watching for minimum throttle and now movement
		if (Math.abs(climb_rate) < 20 && (g.rc_3.servo_out <= get_angle_boost(g.throttle_min) || g.pid_throttle_accel.get_integrator() <= -150)) {
			if( land_detector < LAND_DETECTOR_TRIGGER ) {
				land_detector++;
			}else{
				trace("Land complete");
				set_land_complete(true);
				if( g.rc_3.control_in == 0 || ap.failsafe_radio ) {
					init_disarm_motors();
				}
			}
		}else{
			// we've sensed movement up or down so decrease land_detector
			if (land_detector > 0 ) {
				land_detector--;
			}
		}
	}
}

// get_throttle_surface_tracking - hold copter at the desired distance above the ground
// updates accel based throttle controller targets
public var target_sonar_alt:Number = 0;
public var last_call_ms_st = 0;
public function
get_throttle_surface_tracking(target_rate:int):void
{
	   // The desired altitude in cm above the ground
	var distance_error:Number;
	var sonar_induced_slew_rate:Number;

	var now:int = clock.millis();

	// reset target altitude if this controller has just been engaged
	if( now - last_call_ms_st > 200 ) {
		target_sonar_alt = sonar_alt + controller_desired_alt - current_loc.alt;
	}
	last_call_ms_st = now;

	target_sonar_alt += target_rate * m_dt;

	distance_error = (target_sonar_alt-sonar_alt);
	sonar_induced_slew_rate = constrain(Math.abs(THR_SURFACE_TRACKING_P * distance_error),0,THR_SURFACE_TRACKING_VELZ_MAX);

	// do not let target altitude get too far from current altitude above ground
	// Note: the 750cm limit is perhaps too wide but is consistent with the regular althold limits and helps ensure a smooth transition
	target_sonar_alt = constrain(target_sonar_alt, sonar_alt - 750, sonar_alt + 750);
	controller_desired_alt = current_loc.alt+(target_sonar_alt - sonar_alt);

    // update target altitude for reporting purposes
    set_target_alt_for_reporting(controller_desired_alt);

	get_throttle_althold_with_slew(controller_desired_alt, target_rate - sonar_induced_slew_rate, target_rate + sonar_induced_slew_rate);   // VELZ_MAX limits how quickly we react
}

/*
 *  reset all I integrators
 */
public function
reset_I_all():void
{
	reset_rate_I();
	reset_stability_I();
	reset_wind_I();
	reset_throttle_I();
	//reset_optflow_I()

	// This is the only place we reset Yaw
	g.pi_stabilize_yaw.reset_I();
}

public function
reset_rate_I():void
{
	g.pid_rate_roll.reset_I();
	g.pid_rate_pitch.reset_I();
	g.pid_rate_yaw.reset_I();
}

public function
reset_optflow_I():void
{
	/*g.pid_optflow_roll.reset_I();
	g.pid_optflow_pitch.reset_I();
	of_roll = 0;
	of_pitch = 0;*/
}


public function
reset_wind_I():void
{
	// Wind Compensation
	// this i is not currently being used, but we reset it anyway
	// because someone may modify it and not realize it, causing a bug
	g.pi_loiter_lat.reset_I();
	g.pi_loiter_lon.reset_I();

	g.pid_loiter_rate_lat.reset_I();
	g.pid_loiter_rate_lon.reset_I();

	g.pid_nav_lat.reset_I();
	g.pid_nav_lon.reset_I();
}

public function
reset_throttle_I():void
{
	// For Altitude Hold
	g.pi_alt_hold.reset_I();
	g.pid_throttle.reset_I();
	g.pid_throttle_accel.reset_I();
}

public function
set_accel_throttle_I_from_pilot_throttle(pilot_throttle:int):void
{
	// shift difference between pilot's throttle and hover throttle into accelerometer I
	g.pid_throttle_accel.set_integrator(pilot_throttle - g.throttle_cruise);
}


public function
reset_stability_I():void
{
	// Used to balance a quad
	// This only needs to be reset during Auto-leveling in flight
	g.pi_stabilize_roll.reset_I();
	g.pi_stabilize_pitch.reset_I();
}
