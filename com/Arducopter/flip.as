// 2010 Jose Julio
// 2011 Adapted and updated for AC2 by Jason Short
//
// Automatic Acrobatic Procedure (AAP) v1 : Roll flip
// State machine aproach:
//    Some states are fixed commands (for a fixed time)
//    Some states are fixed commands (until some IMU condition)
//    Some states include controls inside
public var flip_timer						:int = 0;
public var flip_state 						:int = 0;

public const AAP_THR_INC					:int = 170;
public const AAP_THR_DEC					:int = 120;
public const AAP_ROLL_OUT					:int = 2000;

public var flip_dir 						:int;

public function init_flip():void
{
    if(false == ap.do_flip) {
        ap.do_flip = true;
        flip_state = 0;
        flip_dir = (ahrs.roll_sensor >= 0) ? 1 : -1;
    }
}

public function roll_flip():void
{
	var roll:int = ahrs.roll_sensor * flip_dir;
    // Roll State machine
    switch (flip_state) {
    case 0:
        if (roll < 4500) {
            // Roll control
			roll_rate_target_bf     = 40000 * flip_dir;
		    if(ap.manual_throttle){
    		    // increase throttle right before flip
                set_throttle_out(g.rc_3.control_in + AAP_THR_INC, false);
            }
        }else{
            flip_state++;
        }
        break;

    case 1:
        if((roll >= 4500) || (roll < -9000)) {
			roll_rate_target_bf = 40000 * flip_dir;
		    // decrease throttle while inverted
		    if(ap.manual_throttle){
                set_throttle_out(g.rc_3.control_in - AAP_THR_DEC, false);
            }
        }else{
            flip_state++;
            flip_timer = 0;
        }
        break;

    case 2:
        // 100 = 1 second with 100hz
        if (flip_timer < 100) {
            // we no longer need to adjust the roll_rate.
            // It will be handled by normal flight control loops

            // increase throttle to gain any lost alitude
            if(ap.manual_throttle){
                set_throttle_out(g.rc_3.control_in + AAP_THR_INC, false);
            }
            flip_timer++;
        }else{
        	Log_Write_Event(DATA_END_FLIP);
            ap.do_flip = false;
            flip_state = 0;
        }
        break;
    }
}
