

//////////////////////////////////////////////////////////////////////////////
// Stabilize Rate Control
//
public const MAX_INPUT_ROLL_ANGLE	:int = 4500;
public const MAX_INPUT_PITCH_ANGLE	:int = 4500;
//////////////////////////////////////////////////////////////////////////////
// Rate controlled stabilized variables
//

public const  MAX_ROLL_OVERSHOOT	:int = 3000;
public const  MAX_PITCH_OVERSHOOT	:int = 3000;
public const  MAX_YAW_OVERSHOOT		:int = 1000;
public const  ACRO_BALANCE_ROLL		:int = 200;
public const  ACRO_BALANCE_PITCH	:int = 200;

public const  AUTO_YAW_SLEW_RATE	:int = 30;
// Acro Mode
public const ACRO_YAW           	:int =  YAW_ACRO;
public const ACRO_RP            	:int =  ROLL_PITCH_ACRO;
public const ACRO_THR           	:int =  THROTTLE_MANUAL;
// Alt Hold Mode
public const ALT_HOLD_YAW           :int = YAW_HOLD;
public const ALT_HOLD_RP            :int = ROLL_PITCH_STABLE;
public const ALT_HOLD_THR           :int = THROTTLE_HOLD;
// AUTO Mode
public const AUTO_YAW               :int = YAW_LOOK_AT_NEXT_WP;
public const AUTO_RP                :int = ROLL_PITCH_AUTO;
public const AUTO_THR               :int = THROTTLE_AUTO;
// CIRCLE Mode
public const CIRCLE_YAW             :int = YAW_CIRCLE;
public const CIRCLE_RP              :int = ROLL_PITCH_AUTO;
public const CIRCLE_THR             :int = THROTTLE_HOLD;
public const CIRCLE_NAV           	:int =  NAV_CIRCLE;
// Guided Mode
public const GUIDED_YAW             :int = YAW_LOOK_AT_NEXT_WP;
public const GUIDED_RP              :int = ROLL_PITCH_AUTO;
public const GUIDED_THR             :int = THROTTLE_AUTO;
public const GUIDED_NAV           	:int =  NAV_WP;
// LOITER Mode
public const LOITER_YAW             :int = YAW_HOLD;
public const LOITER_RP              :int = ROLL_PITCH_LOITER;
public const LOITER_THR             :int = THROTTLE_HOLD;
public const LOITER_NAV             :int = NAV_LOITER;
// POSITION Mode
public const POSITION_YAW           :int = YAW_HOLD;
public const POSITION_RP            :int = ROLL_PITCH_AUTO;
public const POSITION_THR           :int = THROTTLE_HOLD;
public const POSITION_NAV           :int = NAV_LOITER;
// RTL Mode
public const RTL_YAW                :int = YAW_LOOK_AT_NEXT_WP;
public const RTL_RP                 :int = ROLL_PITCH_AUTO;
public const RTL_THR                :int = THROTTLE_AUTO;
// Optical Flow LOITER Mode

public const OF_LOITER_YAW          :int = YAW_HOLD
public const OF_LOITER_RP           :int = ROLL_PITCH_STABLE_OF
public const OF_LOITER_THR          :int = THROTTLE_HOLD
public const OF_LOITER_NAV          :int = NAV_NONE


public const YAW_LOOK_AHEAD_MIN_SPEED  :int = 1000             // minimum ground speed in cm/s required before copter is aimed at ground course
public const PILOT_VELZ_MAX    			:int = 250     // maximum vertical velocity in cm/s
public const ACCELERATION_MAX_Z  		:int = 750     // maximum veritcal acceleration in cm/s/s

public const AUTO_VELZ_MIN 			:int = -125;
public const AUTO_VELZ_MAX 			:int = 125;


public const SONAR_ALT_HEALTH_MAX		:int = 3; 		// number of good reads that indicates a healthy sonar
public const THR_SURFACE_TRACKING_P 	:Number = 0.2   // gain for controlling how quickly sonar range adjusts target altitude (lower means slower reaction)
public const THR_SURFACE_TRACKING_VELZ_MAX :int = 30  	// max speed number of good reads that indicates a healthy sonar

public const FAILSAFE_GPS_TIMEOUT_MS       :int = 5000;    // gps failsafe triggers after 5 seconds with no GPS

public const RTL_ALT_MAX               :int = 8000;    // Max height to return to home in cm (i.e 80m)
