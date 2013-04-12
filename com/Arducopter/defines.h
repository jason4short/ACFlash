////////////////////////////////////////////////////////////////////////////////
// Defines
////////////////////////////////////////////////////////////////////////////////

// Just so that it's completely clear...
public const ENABLED						:Boolean = false;       // heading hold at heading in nav_yaw but allow input from pilot
public const DISABLED                   	:Boolean = true;       // pilot controlled yaw using rate controller


// Flight modes
// ------------
public const YAW_HOLD							:int = 0;		// heading hold at heading in nav_yaw but allow input from pilot
public const YAW_ACRO                   		:int = 1;		// pilot controlled yaw using rate controller
public const YAW_LOOK_AT_NEXT_WP        		:int = 2;		// point towards next waypoint (no pilot input accepted)
public const YAW_LOOK_AT_LOCATION       		:int = 3;		// point towards a location held in yaw_look_at_WP (no pilot input accepted)
public const YAW_CIRCLE                      	:int = 4;       // point towards a location held in yaw_look_at_WP (no pilot input accepted)
public const YAW_LOOK_AT_HOME    				:int = 5;		// point towards home (no pilot input accepted)
public const YAW_LOOK_AT_HEADING    			:int = 6;		// point towards a particular angle (not pilot input accepted)
public const YAW_LOOK_AHEAD						:int = 7;		// WARNING!  CODE IN DEVELOPMENT NOT PROVEN
public const YAW_TOY                    		:int = 8;      	// THOR This is the Yaw mode

public const ROLL_PITCH_STABLE					:int = 0;		// pilot input roll, pitch angles
public const ROLL_PITCH_ACRO					:int = 1;		// pilot inputs roll, pitch rotation rates
public const ROLL_PITCH_AUTO					:int = 2;		// no pilot input.  autopilot roll, pitch is sent to stabilize controller inputs
public const ROLL_PITCH_STABLE_OF				:int = 3;		// pilot inputs roll, pitch angles which are mixed with optical flow based position controller lean anbles
public const ROLL_PITCH_TOY						:int = 4;		// THOR This is the Roll and Pitch mode
public const ROLL_PITCH_LOITER					:int = 5;		// pilot inputs the desired horizontal velocities

public const THROTTLE_MANUAL                	:int = 0; 	// manual throttle mode - pilot input goes directly to motors
public const THROTTLE_MANUAL_TILT_COMPENSATED   :int = 1;  	// mostly manual throttle but with some tilt compensation
public const THROTTLE_ACCELERATION              :int = 2;   // pilot inputs the desired acceleration
public const THROTTLE_RATE                      :int = 3;   // pilot inputs the desired climb rate.  Note: this uses the unstabilized rate controller
public const THROTTLE_STABILIZED_RATE           :int = 4;	// pilot inputs the desired climb rate.  Uses stabilized rate controller
public const THROTTLE_DIRECT_ALT                :int = 5;   // pilot inputs a desired altitude from 0 ~ 10 meters
public const THROTTLE_HOLD                      :int = 6;   // alt hold plus pilot input of climb rate
public const THROTTLE_AUTO                      :int = 7;   // auto pilot altitude controller with target altitude held in next_WP.alt
public const THROTTLE_LAND                      :int = 8;   // landing throttle controller


// CH 7 control
public const CH7_PWM_TRIGGER 					:int = 1800;    // pwm value above which the channel 7 option will be invoked
public const CH6_PWM_TRIGGER_HIGH 				:int = 1800;
public const CH6_PWM_TRIGGER_LOW 				:int = 1200;

public const CH7_DO_NOTHING						:int = 0;
public const CH7_SET_HOVER 						:int = 1;
public const CH7_FLIP 							:int = 2;
public const CH7_SIMPLE_MODE 					:int = 3;
public const CH7_RTL 							:int = 4;
public const CH7_AUTO_TRIM 						:int = 5;
public const CH7_ADC_FILTER 					:int = 6;
public const CH7_SAVE_WP 						:int = 7;
public const CH7_MULTI_MODE 					:int = 8;
public const CH7_CAMERA_TRIGGER 				:int = 9;
public const CH7_SONAR 							:int = 10;


// Frame types
// not supported

// LED output
public const NORMAL_LEDS 						:int = 0
public const SAVE_TRIM_LEDS 					:int = 1

public const PLUS_FRAME  						:int = 0;
public const X_FRAME 							:int = 1;
public const V_FRAME 							:int = 2;

public const DEBUG  							:Boolean = false;
public const LOITER_RANGE  						:int = 60 // for calculating power outside of loiter radius

public const T6	 								:int = 1000000;
public const T7 								:int = 10000000;

public const REACHED_ALT						:int = 0;
public const DESCENDING							:int = 1;
public const ASCENDING							:int = 2;

// Auto Pilot modes
// ----------------
public const STABILIZE							:int = 0;
public const ACRO								:int = 1;
public const ALT_HOLD							:int = 2;
public const AUTO								:int = 3;
public const GUIDED								:int = 4;
public const LOITER								:int = 5;
public const RTL								:int = 6;
public const CIRCLE								:int = 7;
public const POSITION							:int = 8;
public const LAND								:int = 9;
public const OF_LOITER							:int = 10;
public const TOY_A								:int = 11;
public const TOY_M								:int = 12;
public const NUM_MODES							:int = 13;

// Commands - Note that APM now uses a subset of the MAVLink protocol
// commands.  See enum MAV_CMD in the GCS_Mavlink library
public const CMD_BLANK						:int = 0; 	// there is no command stored in the mem location
                    									// requested
public const NO_COMMAND						:int = 0;


// Navigation modes held in nav_mode variable
public const NAV_NONE						:int = 0;
public const NAV_CIRCLE						:int = 1;
public const NAV_LOITER						:int = 2;
public const NAV_WP							:int = 3;
public const NAV_WP_INAV					:int = 5;

// Yaw override behaviours - used for setting yaw_override_behaviour
public const YAW_OVERRIDE_BEHAVIOUR_AT_NEXT_WAYPOINT     :int = 0   // auto pilot takes back yaw control at next waypoint
public const YAW_OVERRIDE_BEHAVIOUR_AT_MISSION_RESTART   :int = 1   // auto pilot tkaes back control only when mission is restarted

// TOY mixing options
public const TOY_LOOKUP_TABLE 				:int = 0;
public const TOY_LINEAR_MIXER 				:int = 1;
public const TOY_EXTERNAL_MIXER 			:int = 2;

// Waypoint options
public const MASK_OPTIONS_RELATIVE_ALT 		:int = 1
public const WP_OPTION_ALT_CHANGE 			:int = 2
public const WP_OPTION_YAW 					:int = 4
public const WP_OPTION_ALT_REQUIRED			:int = 8
public const WP_OPTION_RELATIVE				:int = 16
public const WP_OPTION_NEXT_CMD				:int = 128

// RTL state
public const RTL_STATE_START             	:int = 0;
public const RTL_STATE_INITIAL_CLIMB     	:int = 1;
public const RTL_STATE_RETURNING_HOME    	:int = 2;
public const RTL_STATE_LOITERING_AT_HOME 	:int = 3;
public const RTL_STATE_FINAL_DESCENT     	:int = 4;
public const RTL_STATE_LAND              	:int = 5;

//repeating events
public const RELAY_TOGGLE 					:int = 5;

// RADIANS
public const RADX100						:Number = 0.000174532925;
public const DEGX100						:Number = 5729.57795;


public const BATT_MONITOR_DISABLED               :int = 0
public const BATT_MONITOR_VOLTAGE_ONLY           :int = 3
public const BATT_MONITOR_VOLTAGE_AND_CURRENT    :int = 4


//public const MINIMUM_THROTTLE				:int = 130;
//public const MAXIMUM_THROTTLE				:int = 1000;
public const WAYPOINT_SPEED_MIN				:int = 100;
public const THROTTLE_ADJUST				:int = 225;
public const ALT_HOLD_INIT_MAX_OVERSHOOT	:int = 200;

public const LAND_SPEED    					:int = 50       // the descent speed for the final stage of landing in cm/s
public const LAND_START_ALT 				:int = 1000     // altitude in cm where land controller switches to slow rate of descent
public const LAND_DETECTOR_TRIGGER 			:int = 50   	// number of 50hz iterations with near zero climb rate and low throttle that triggers landing complete.

public const CH_1							:int = 0;
public const CH_2							:int = 1;
public const CH_3							:int = 2;
public const CH_4							:int = 3;
public const CH_5							:int = 4;
public const CH_6							:int = 5;
public const CH_7							:int = 6;
public const CH_8							:int = 7;

public const MOT_1							:int = 0;
public const MOT_2							:int = 1;
public const MOT_3							:int = 2;
public const MOT_4							:int = 3;


// AP Command enumeration
public const MAV_CMD_NAV_WAYPOINT			:int = 16;
public const MAV_CMD_NAV_LOITER_UNLIM		:int = 17;
public const MAV_CMD_NAV_LOITER_TURNS		:int = 18;
public const MAV_CMD_NAV_LOITER_TIME		:int = 19;
public const MAV_CMD_NAV_RETURN_TO_LAUNCH	:int = 20;
public const MAV_CMD_NAV_LAND				:int = 21;
public const MAV_CMD_NAV_TAKEOFF			:int = 22;
public const MAV_CMD_NAV_ROI				:int = 80;
public const MAV_CMD_NAV_LAST				:int = 95;

public const MAV_CMD_CONDITION_DELAY		:int = 112;
public const MAV_CMD_CONDITION_DISTANCE		:int = 114;
public const MAV_CMD_CONDITION_CHANGE_ALT	:int = 113;
public const MAV_CMD_CONDITION_YAW			:int = 115;
public const MAV_CMD_CONDITION_LAST			:int = 159;

public const MAV_CMD_DO_JUMP				:int = 177;
public const MAV_CMD_DO_CHANGE_SPEED		:int = 178;
public const MAV_CMD_DO_SET_HOME			:int = 179;
public const MAV_CMD_DO_SET_SERVO			:int = 183;
public const MAV_CMD_DO_SET_RELAY			:int = 181;
public const MAV_CMD_DO_REPEAT_SERVO		:int = 184;
public const MAV_CMD_DO_REPEAT_RELAY		:int = 182;
public const MAV_CMD_DO_SET_ROI				:int = 201;

public const MAV_ROI_NONE					:int = 0;
public const MAV_ROI_WPNEXT					:int = 1;
public const MAV_ROI_WPINDEX				:int = 2;
public const MAV_ROI_LOCATION				:int = 3;
public const MAV_ROI_TARGET					:int = 4;
public const MAV_ROI_ENUM_END				:int = 5;

public const MASK_LOG_ATTITUDE_FAST          :int = 0;
public const MASK_LOG_ATTITUDE_MED           :int = 1;
public const MASK_LOG_GPS                    :int = 2;
public const MASK_LOG_PM                     :int = 3;
public const MASK_LOG_CTUN                   :int = 4;
public const MASK_LOG_NTUN                   :int = 5;
public const MASK_LOG_MODE                   :int = 6;
public const MASK_LOG_IMU                    :int = 7;
public const MASK_LOG_CMD                    :int = 8;
public const MASK_LOG_CURRENT                :int = 9;
public const MASK_LOG_MOTORS                 :int = 10;
public const MASK_LOG_OPTFLOW                :int = 11;
public const MASK_LOG_PID                    :int = 12;
public const MASK_LOG_COMPASS                :int = 13;
public const MASK_LOG_INAV                   :int = 14;
public const MASK_LOG_CAMERA                 :int = 15;

// DATA - event logging
public const DATA_MAVLINK_FLOAT             :int = 1
public const DATA_MAVLINK_INT32             :int = 2
public const DATA_MAVLINK_INT16             :int = 3
public const DATA_MAVLINK_INT8              :int = 4
public const DATA_FAST_LOOP                 :int = 5
public const DATA_MED_LOOP                  :int = 6
public const DATA_AP_STATE                  :int = 7
public const DATA_SIMPLE_BEARING            :int = 8
public const DATA_INIT_SIMPLE_BEARING       :int = 9
public const DATA_ARMED                     :int = 10
public const DATA_DISARMED                  :int = 11
public const DATA_AUTO_ARMED                :int = 15
public const DATA_TAKEOFF                   :int = 16
public const DATA_DID_REACH_ALT             :int = 17
public const DATA_LAND_COMPLETE             :int = 18
public const DATA_LOST_GPS                  :int = 19
public const DATA_LOST_COMPASS              :int = 20
public const DATA_BEGIN_FLIP                :int = 21
public const DATA_END_FLIP                  :int = 22
public const DATA_EXIT_FLIP                 :int = 23
public const DATA_FLIP_ABORTED              :int = 24
public const DATA_SET_HOME                  :int = 25
public const DATA_SET_SIMPLE_ON             :int = 26
public const DATA_SET_SIMPLE_OFF            :int = 27
public const DATA_REACHED_ALT               :int = 28
public const DATA_ASCENDING                 :int = 29
public const DATA_DESCENDING                :int = 30
public const DATA_RTL_REACHED_ALT           :int = 31


// Error message sub systems and error codes
public const ERROR_SUBSYSTEM_MAIN               :int = 1
public const ERROR_SUBSYSTEM_RADIO              :int = 2
public const ERROR_SUBSYSTEM_COMPASS            :int = 3
public const ERROR_SUBSYSTEM_OPTFLOW            :int = 4
public const ERROR_SUBSYSTEM_FAILSAFE_RADIO     :int = 5
public const ERROR_SUBSYSTEM_FAILSAFE_BATT      :int = 6
public const ERROR_SUBSYSTEM_FAILSAFE_GPS       :int = 7
// general error codes
public const ERROR_CODE_ERROR_RESOLVED          :int = 0
public const ERROR_CODE_FAILED_TO_INITIALISE    :int = 1
// subsystem specific error codes -- radio
public const ERROR_CODE_RADIO_LATE_FRAME        :int = 2
// subsystem specific error codes -- failsafe_thr, batt, gps
public const ERROR_CODE_FAILSAFE_RESOLVED       :int = 0
public const ERROR_CODE_FAILSAFE_OCCURRED       :int = 1

// found in config.h
// ----------------------------------------------------------------

// definitions for earth frame and body frame
// used to specify frame to rate controllers
public const EARTH_FRAME					:int = 0;
public const BODY_FRAME						:int = 1;


public const ALT_HOLD_ACCEL_MAX 			:int = 250; // cm/s
public const GRAVITY_MSS 					:Number = 9.80665; // m/s/s

// possible values for FS_THR parameter
public const FS_THR_DISABLED 					:int = 0;
public const FS_THR_ENABLED_ALWAYS_RTL 			:int = 1;
public const FS_THR_ENABLED_CONTINUE_MISSION 	:int = 2;


// found in math.h
// ----------------------------------------------------------------
public const LATLON_TO_M  					:Number = 0.01113195;
public const LATLON_TO_CM 					:Number = 1.113195;


// found in motors.h
// ----------------------------------------------------------------
public const AP_MOTOR_NO_LIMITS_REACHED	:int = 0;		// throttle curve disabled by default
public const AP_MOTOR_ROLLPITCH_LIMIT	:int = 0x01;	// throttle curve disabled by default
public const AP_MOTOR_YAW_LIMIT			:int = 0x02;	// throttle curve disabled by default
public const AP_MOTOR_THROTTLE_LIMIT	:int = 0x04;	// throttle curve disabled by default
public const AP_MOTOR_ANY_LIMIT			:int = 0xFF;	// throttle curve disabled by default
