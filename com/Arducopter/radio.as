// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------


public function default_dead_zones():void
{
	g.rc_1.set_dead_zone(60);
	g.rc_2.set_dead_zone(60);
	g.rc_3.set_dead_zone(60);
	g.rc_4.set_dead_zone(80);
}

public function init_rc_in():void
{
	// set rc channel ranges
	g.rc_1.set_angle(MAX_INPUT_ROLL_ANGLE);
	g.rc_2.set_angle(MAX_INPUT_PITCH_ANGLE);
	g.rc_3.set_range(g.throttle_min, g.throttle_max);
	g.rc_4.set_angle(4500);

	g.rc_1.set_type(g.rc_1.RC_CHANNEL_ANGLE_RAW);
	g.rc_2.set_type(g.rc_1.RC_CHANNEL_ANGLE_RAW);
	g.rc_4.set_type(g.rc_1.RC_CHANNEL_ANGLE_RAW);

    rc_ch[CH_1] = g.rc_1;
    rc_ch[CH_2] = g.rc_2;
    rc_ch[CH_3] = g.rc_3;
    rc_ch[CH_4] = g.rc_4;
    rc_ch[CH_5] = g.rc_5;
    rc_ch[CH_6] = g.rc_6;
    rc_ch[CH_7] = g.rc_7;
    rc_ch[CH_8] = g.rc_8;

	//set auxiliary ranges
	g.rc_5.set_range(0,1000);
	g.rc_6.set_range(0,1000);
	g.rc_7.set_range(0,1000);
	g.rc_8.set_range(0,1000);
}

public function init_rc_out():void
{
	//motors.set_update_rate(g.rc_speed);
	motors.set_frame_orientation(g.frame_orientation);
	motors.Init();                                              // motor initialisation
	motors.set_min_throttle(g.throttle_min);
	motors.set_max_throttle(g.throttle_max);

	g.rc_3.set_range_out(0,1000);

}

public function
output_min():void
{
    // enable motors
    motors.enable();
    motors.output_min();
}

public const FAILSAFE_RADIO_TIMEOUT_MS:int = 2000;
public function read_radio():void
{
	if (APM_RC.getState() == 1){
        ap_system.new_radio_frame = true;

		g.rc_1.set_pwm(APM_RC.InputCh(CH_1));
		g.rc_2.set_pwm(APM_RC.InputCh(CH_2));

		//g.rc_3.set_pwm(APM_RC.InputCh(CH_3));
		set_throttle_and_failsafe(APM_RC.InputCh(CH_3));

		g.rc_4.set_pwm(APM_RC.InputCh(CH_4));
		g.rc_5.set_pwm(APM_RC.InputCh(CH_5));
		g.rc_6.set_pwm(APM_RC.InputCh(CH_6));
		g.rc_7.set_pwm(APM_RC.InputCh(CH_7));
		g.rc_8.set_pwm(APM_RC.InputCh(CH_8));
	}
}

public const FS_COUNTER:int = 3;
public var failsafe_counter:int = 0;

public function
set_throttle_and_failsafe(throttle_pwm:int):void
{
    // if failsafe not enabled pass through throttle and exit
    if(g.failsafe_throttle == FS_THR_DISABLED) {
        g.rc_3.set_pwm(throttle_pwm);
        return;
    }

    //check for low throttle value
    if (throttle_pwm < g.failsafe_throttle_value) {

        // if we are already in failsafe or motors not armed pass through throttle and exit
        if (ap.failsafe_radio || !motors.armed()) {
            g.rc_3.set_pwm(throttle_pwm);
            return;
        }

        // check for 3 low throttle values
        // Note: we do not pass through the low throttle until 3 low throttle values are recieved
        failsafe_counter++;
        if( failsafe_counter >= FS_COUNTER ) {
            failsafe_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
            set_failsafe_radio(true);
            g.rc_3.set_pwm(throttle_pwm);   // pass through failsafe throttle
        }
    }else{
        // we have a good throttle so reduce failsafe counter
        failsafe_counter--;
        if( failsafe_counter <= 0 ) {
            failsafe_counter = 0;   // check to ensure we don't underflow the counter

            // disengage failsafe after three (nearly) consecutive valid throttle values
            if (ap.failsafe_radio) {
                set_failsafe_radio(false);
            }
        }
        // pass through throttle
        g.rc_3.set_pwm(throttle_pwm);
    }
}

﻿