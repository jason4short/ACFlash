package com {

	//import caurina.transitions.Tweener;

	// Flash
	// ---------------------------------------------
	import flash.display.MovieClip;
    import flash.display.Sprite;
    import flash.display.Stage;
    import flash.display.StageDisplayState;
	import flash.display.DisplayObject;
    import flash.events.*;
	import flash.geom.Rectangle;
	import flash.geom.Point;
	import flash.geom.Vector3D;
	import flash.geom.Matrix3D;
    import flash.display.StageAlign;
    import flash.display.StageScaleMode;
    import flash.utils.*;
    import flash.display.StageScaleMode;
	import flash.text.TextField;
	import flash.ui.Keyboard;


	// Sim
	// ---------------------------------------------
	import com.UI.Plot;
	import com.UI.PlotView;
	import com.UI.QuickMenu;
	import com.UI.QuickMenuItem;
	import com.UI.QuickMenuDivider;
	import com.UI.QuickPopupMenu;
	import com.UI.BasicInput;

	import com.Sim.Sky
	import com.Sim.Ground
	import com.Sim.Wind
	import com.Sim.Copter
	import com.Sim.WaypointManager;
	import com.Sim.Clock;

	////////////////////////////////////////////////////////////////////////////////
	// Header includes
	////////////////////////////////////////////////////////////////////////////////

	import com.Parameters;

	// Application dependencies
	
	import com.libraries.AC_WPNav;
	import com.libraries.AP_InertialNav;
	import com.libraries.AP_Buffer;
	import com.libraries.AP_MotorsMatrix;
	import com.libraries.AP_MotorsQuad;
	import com.libraries.GPS;
	import com.libraries.AP_Compass;
	import com.libraries.Location;
	import com.libraries.AverageFilter;
	import com.libraries.LeadFilter;
	import com.libraries.AP_AHRS;
	import com.libraries.AC_PID;
	import com.libraries.RC_Channel;
	import com.libraries.AP_Baro;
	import com.libraries.Sonar;
	import com.libraries.Motors;
	import com.libraries.Relay;
	import com.libraries.AP_RC;
	import com.libraries.AP_State;
	import com.libraries.AP_System;



	public class Main extends MovieClip
	{
		include "Arducopter/defines.h";

		////////////////////////////////////////////////////////////////////////////////
		// SIM
		////////////////////////////////////////////////////////////////////////////////
		//
		//public var user								:User;
		public var simIsRunnning					:Boolean = false;
		public var iteration						:int = 0;
		public var copter							:Copter;
		public var wp_manager						:WaypointManager;
		public var waypoints						:Array;

		public var flight_mode_strings				:Array;
		public var flight_modes						:Array;

		// --------------------------------------
		// Plotting
		// --------------------------------------
		public var colors							:Array;
		public var colorIndex						:int = -1;
		public var plotView							:PlotView;
		public var plot_A							:int = 0;
		public var plot_B							:int = 0;
		public var plot_C							:int = 0;
		public var plotType_A						:String = "";
		public var plotType_B						:String = "";
		public var plotType_C						:String = "";
		public var fastPlot							:Boolean = true;
		public var roll_output						:Number;

		// --------------------------------------
		// Sim Radio
		// --------------------------------------
		public var radio_failure					:Boolean = false;// simulate a radio failure
		public var radio_switch_position			:int;
		public var ch_1_pwm							:int = 1500;
		public var ch_2_pwm							:int = 1500;
		public var ch_3_pwm							:int = 1500;
		public var ch_4_pwm							:int = 1500;
		public var ch_5_pwm							:int = 1500;
		public var ch_6_pwm							:int = 1500;
		public var ch_7_pwm							:int = 1500;
		public var ch_8_pwm							:int = 1500;

		public var rc_ch							:Array;

		////////////////////////////////////////////////////////////////////////////////
		// Parameters
		////////////////////////////////////////////////////////////////////////////////
		//
		// Global parameters are all contained within the 'g' class.
		//
		public var g								:Parameters;
		// main loop scheduler
		//AP_Scheduler scheduler;



		////////////////////////////////////////////////////////////////////////////////
		// the rate we run the main loop at
		////////////////////////////////////////////////////////////////////////////////
		//static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_200HZ;



		////////////////////////////////////////////////////////////////////////////////
		// Sensors
		////////////////////////////////////////////////////////////////////////////////
		//
		public var g_gps							:GPS;
		public var barometer						:AP_Baro;
		public var sonar							:Sonar;
		public var ahrs								:AP_AHRS;
		public var wp_nav							:AC_WPNav;
		//public var wp_nav							:AC_WPNav;

		////////////////////////////////////////////////////////////////////////////////
		// Global variables
		////////////////////////////////////////////////////////////////////////////////
		public var ap								:AP_State;
		public var ap_system						:AP_System;
		public var APM_RC							:AP_RC;

		////////////////////////////////////////////////////////////////////////////////
		// Radio
		////////////////////////////////////////////////////////////////////////////////
		// This is the state of the flight control system
		// There are multiple states defined such as STABILIZE, ACRO,
		public var control_mode						:int = STABILIZE;
		// Used to maintain the state of the previous control switch position
		// This is set to -1 when we need to re-read the switch
		public var oldSwitchPosition				:int = 0;


		////////////////////////////////////////////////////////////////////////////////
		// Motor Output
		////////////////////////////////////////////////////////////////////////////////
		public var motors							:AP_MotorsQuad;


		////////////////////////////////////////////////////////////////////////////////
		// PIDs
		////////////////////////////////////////////////////////////////////////////////
		// This is a convienience accessor for the IMU roll rates. It's currently the raw IMU rates
		// and not the adjusted omega rates, but the name is stuck
		public var omega							:Vector3D;

		////////////////////////////////////////////////////////////////////////////////
		// GPS variables
		////////////////////////////////////////////////////////////////////////////////
		public var scaleLongUp						:Number = 1;
		public var scaleLongDown					:Number = 1;


		////////////////////////////////////////////////////////////////////////////////
		// Location & Navigation
		////////////////////////////////////////////////////////////////////////////////
		// This is the angle from the copter to the "next_WP" location in degrees * 100
		public var wp_bearing						:int;
		// navigation mode - options include NAV_NONE, NAV_LOITER, NAV_CIRCLE, NAV_WP
		public var nav_mode							:int = 0;
		public var command_cond_index				:int = 0;
		public var prev_nav_index					:int = 0;
		public var command_nav_index				:int = 0;

		public var lon_error, lat_error				:Number;      // Used to report how many cm we are from the next waypoint or loiter target position
		public var control_roll						:int = 0;
		public var control_pitch					:int = 0;
		public var rtl_state						:int = 0;



		////////////////////////////////////////////////////////////////////////////////
		// Orientation
		////////////////////////////////////////////////////////////////////////////////
		// Convienience accessors for commonly used trig functions. These values are generated
		// by the DCM through a few simple equations. They are used throughout the code where cos and sin
		// would normally be used.
		// The cos values are defaulted to 1 to get a decent initial value for a level state
		public var cos_roll_x						:Number = 1;
		public var cos_pitch_x						:Number = 1;
		public var cos_yaw_x						:Number = 1;
		public var sin_yaw_y						:Number = 1;
		public var cos_yaw							:Number = 1;
		public var sin_yaw							:Number = 1;
		public var sin_roll							:Number = 1;
		public var sin_pitch						:Number = 1;

		////////////////////////////////////////////////////////////////////////////////
		// SIMPLE Mode
		////////////////////////////////////////////////////////////////////////////////
		// Used to track the orientation of the copter for Simple mode. This value is reset at each arming
		// or in SuperSimple mode when the copter leaves a 20m radius from home.
		public var initial_simple_bearing			:int = 0;

		////////////////////////////////////////////////////////////////////////////////
		// Rate contoller targets
		////////////////////////////////////////////////////////////////////////////////
		public var rate_targets_frame 			:int = EARTH_FRAME;    // indicates whether rate targets provided in earth or body frame
		public var roll_rate_target_ef 			:int;
		public var pitch_rate_target_ef 		:int;
		public var yaw_rate_target_ef 			:int;
		public var roll_rate_target_bf 			:int;   // body frame roll rate target
		public var pitch_rate_target_bf 		:int;   // body frame pitch rate target
		public var yaw_rate_target_bf 			:int;   // body frame yaw rate target


		////////////////////////////////////////////////////////////////////////////////
		// Throttle variables
		////////////////////////////////////////////////////////////////////////////////
		public var throttle_accel_target_ef			:int = 0;    	// earth frame throttle acceleration target
		public var throttle_accel_controller_active	:Boolean;   	// true when accel based throttle controller is active, false when higher level throttle controllers are providing throttle output directly
		public var throttle_avg						:Number = 0;    // g.throttle_cruise as a float
		public var desired_climb_rate				:int = 0;      	// pilot desired climb rate - for logging purposes only
	 	public var target_alt_for_reporting			:Number = 0;      // target altitude for reporting (logs and ground station)

		////////////////////////////////////////////////////////////////////////////////
		// ACRO Mode
		////////////////////////////////////////////////////////////////////////////////
		// Used to control Axis lock
		public var roll_axis						:int = 0;
		public var pitch_axis						:int = 0;


		////////////////////////////////////////////////////////////////////////////////
		// Circle Mode / Loiter control
		////////////////////////////////////////////////////////////////////////////////
		// used to control the speed of Circle mode in radians/second, default is 5° per second
		public var circle_rate						:Number = 0.0872664625;
		public var circle_center					:Vector3D; // circle position expressed in cm from home location.  x = lat, y = lon
		// angle from the circle center to the copter's desired location.  Incremented at circle_rate / second
		public var circle_angle						:Number = 0;
		// the total angle (in radians) travelled
		public var circle_angle_total				:Number = 0;
		// deg : how many times to circle as specified by mission command
		public var circle_desired_rotations			:int = 0;
		// How long we should stay in Loiter Mode for mission scripting (time in seconds)
		public var loiter_time_max					:Number = 0;
		// How long have we been loitering - The start time in millis
		public var loiter_time						:Number = 0;

		////////////////////////////////////////////////////////////////////////////////
		// CH7 control
		////////////////////////////////////////////////////////////////////////////////
		// This register tracks the current Mission Command index when writing
		// a mission using CH7 in flight
		public var CH7_wp_index						:int = 0;


		////////////////////////////////////////////////////////////////////////////////
		// Altitude
		////////////////////////////////////////////////////////////////////////////////
		// The (throttle) controller desired altitude in cm
		public var controller_desired_alt			:Number = 0;
		// The cm we are off in altitude from next_WP.alt – Positive value means we are below the WP
		public var altitude_error					:int = 0;
		// The cm/s we are moving up or down based on filtered data - Positive = UP
		public var climb_rate						:int = 0;
		// The altitude as reported by Sonar in cm – Values are 20 to 700 generally.
		public var sonar_alt						:int	= 0;
		public var sonar_alt_health					:int	= 0; // true if we can trust the altitude from the sonar
		// The altitude as reported by Baro in cm – Values can be quite high
		public var baro_alt							:int	= 0;

		public var saved_toy_throttle				:int;

		////////////////////////////////////////////////////////////////////////////////
		// flight modes
		////////////////////////////////////////////////////////////////////////////////
		// Flight modes are combinations of Roll/Pitch, Yaw and Throttle control modes
		// Each Flight mode is a unique combination of these modes
		//
		// The current desired control scheme for Yaw
		public var yaw_mode							:int = 0;
		// The current desired control scheme for roll and pitch / navigation
		public var roll_pitch_mode					:int = 0;
		// The current desired control scheme for altitude hold
		public var throttle_mode					:int = 0;


		////////////////////////////////////////////////////////////////////////////////
		// flight specific
		////////////////////////////////////////////////////////////////////////////////
		// An additional throttle added to keep the copter at the same altitude when banking
		public var angle_boost						:int = 0;
		// counter to verify landings
		public var land_detector						:int = 0;



		////////////////////////////////////////////////////////////////////////////////
		// Navigation general
		////////////////////////////////////////////////////////////////////////////////
		// The location of home in relation to the copter, updated every GPS read
		public var home_bearing						:int = 0;
		// distance between plane and home in cm
		public var home_distance					:int = 0;
		// distance between plane and next_WP in cm
		// is not static because AP_Camera uses it
		public var wp_distance				:int = 0;
		// wpinav variables
		public var wpinav_origin			:Point;	// starting point of trip to next waypoint in cm from home (equivalent to next_WP)
		public var wpinav_destination		:Point;	// target destination in cm from home (equivalent to next_WP)
		public var wpinav_target			:Point;	// the intermediate target location in cm from home
		public var wpinav_pos_delta			:Point; // position difference between origin and destination
		public var wpinav_track_length		:Number = 0;      // distance in cm between origin and destination
		public var wpinav_track_desired		:Number = 0;     // the desired distance along the track in cm


		////////////////////////////////////////////////////////////////////////////////
		// 3D Location vectors
		////////////////////////////////////////////////////////////////////////////////
		// home location is stored when we have a good GPS lock and arm the copter
		// Can be reset each the copter is re-armed
		public var home					:Location;
		// Current location of the copter
		public var current_loc			:Location;
		// Next WP is the desired location of the copter - the next waypoint or loiter location
		public var next_WP				:Location;
		// Holds the current loaded command from the EEPROM for navigation
		public var command_nav_queue	:Location;
		// Holds the current loaded command from the EEPROM for conditional scripts
		public var command_cond_queue	:Location;
		// Holds the current loaded command from the EEPROM for guided mode

		////////////////////////////////////////////////////////////////////////////////
		// Crosstrack
		////////////////////////////////////////////////////////////////////////////////
		// deg * 100, The original angle to the next_WP when the next_WP was set
		// Also used to check when we pass a WP
		public var original_wp_bearing	:int;


		////////////////////////////////////////////////////////////////////////////////
		// Navigation Roll/Pitch functions
		////////////////////////////////////////////////////////////////////////////////
		// all angles are deg * 100 : target yaw angle
		// The Commanded ROll from the autopilot.
		public var nav_roll			:int = 0;
		// The Commanded pitch from the autopilot. negative Pitch means go forward.
		public var nav_pitch		:int = 0;
		// The desired bank towards North (Positive) or South (Negative)

		// Don't be fooled by the fact that Pitch is reversed from Roll in its sign!
		public var nav_lat			:int = 0;
		// The desired bank towards East (Positive) or West (Negative)
		public var nav_lon			:int = 0;
		// The Commanded ROll from the autopilot based on optical flow sensor.
		public var of_roll			:int = 0;
		// The Commanded pitch from the autopilot based on optical flow sensor. negative Pitch means go forward.
		public var of_pitch			:int = 0;

		////////////////////////////////////////////////////////////////////////////////
		// Navigation Throttle control
		////////////////////////////////////////////////////////////////////////////////
		// The Commanded Throttle from the autopilot.
		public var nav_throttle						:int = 0; // 0-1000 for throttle control
		// This is a simple counter to track the amount of throttle used during flight
		// This could be useful later in determining and debuging current usage and predicting battery life
		public var throttle_integrator				:int = 0;


		////////////////////////////////////////////////////////////////////////////////
		// Navigation Yaw control
		////////////////////////////////////////////////////////////////////////////////
		// The Commanded Yaw from the autopilot.
		public var nav_yaw							:int = 0;
		public var yaw_timer						:int = 0;
		// Yaw will point at this location if yaw_mode is set to YAW_LOOK_AT_LOCATION
		public var yaw_look_at_WP					:Vector3D;
		// bearing from current location to the yaw_look_at_WP
		public var yaw_look_at_WP_bearing			:int = 0;
		// yaw used for YAW_LOOK_AT_HEADING yaw_mode
		public var yaw_look_at_heading				:int = 0;
		// Deg/s we should turn
		public var yaw_look_at_heading_slew			:int = 0;

		////////////////////////////////////////////////////////////////////////////////
		// Repeat Mission Scripting Command
		////////////////////////////////////////////////////////////////////////////////
		// The type of repeating event - Toggle a servo channel, Toggle the APM1 relay, etc
		public var event_id							:int = 0
		// Used to manage the timimng of repeating events
		public var event_timer						:int = 0;
		// How long to delay the next firing of event in millis
		public var event_delay						:int = 0;
		// how many times to fire : 0 = forever, 1 = do once, 2 = do twice
		public var event_repeat						:int = 0;
		// per command value, such as PWM for servos
		public var event_value						:int = 0;
		// the stored value used to undo commands - such as original PWM command
		public var event_undo_value					:int = 0

		////////////////////////////////////////////////////////////////////////////////
		// Delay Mission Scripting Command
		////////////////////////////////////////////////////////////////////////////////
		public var condition_value					:int = 0;// used in condition commands (eg delay, change alt, etc.)
		public var condition_start					:int = 0;

		////////////////////////////////////////////////////////////////////////////////
		// IMU variables
		////////////////////////////////////////////////////////////////////////////////
		// Integration time for the gyros (DCM algorithm)
		// Updated with the fast loop
		public const G_Dt							:Number = 0.01;


		////////////////////////////////////////////////////////////////////////////////
		// Inertial Navigation
		////////////////////////////////////////////////////////////////////////////////
		public var inertial_nav						:AP_InertialNav;

		////////////////////////////////////////////////////////////////////////////////
		// Performance monitoring
		////////////////////////////////////////////////////////////////////////////////
		private var gps_fix_count					:int = 0;


		// System Timers
		// --------------
		public var clock							:Clock;
		public var dTnav							:Number = 0.25;
		public var m_dt								:Number = 0.02;
		//public var elapsed						:int = 0;
		private var fifty_toggle					:Boolean = false;
		// Counters for branching from 10 hz control loop
		private var medium_loopCounter				:int = 0;
		// Counters for branching from 3 1/3hz control loop
		private var slow_loopCounter				:int = 0;
		// Counters for branching from 4 minute control loop used to save Compass offsets
		private var superslow_loopCounter			:int = 0;
		// Loiter timer - Records how long we have been in loiter
		public var rtl_loiter_start_time			:int = 0;
		// disarms the copter while in Acro or Stabilize mode after 30 seconds of no flight
		private var auto_disarming_counter			:int = 0;
		// prevents duplicate GPS messages from entering system
		private var last_gps_time					:int = 0;
		// Used to exit the roll and pitch auto trim function
		private var auto_trim_counter				:int = 0;
		private var counter_one_herz				:int = 0;

		// Reference to the relay object (APM1 -> PORTL 2) (APM2 -> PORTB 7)
		public var relay							:Relay;

		////////////////////////////////////////////////////////////////////////////////
		// Experimental AP_Limits library - set constraints, limits, fences, minima, maxima on various parameters
		////////////////////////////////////////////////////////////////////////////////
		// todo

		////////////////////////////////////////////////////////////////////////////////
		// LED output
		////////////////////////////////////////////////////////////////////////////////
		// This is current status for the LED lights state machine
		// setting this value changes the output of the LEDs
		public var led_mode							:int = NORMAL_LEDS;
		// Blinking indicates GPS status
		public var copter_leds_GPS_blink			:int = 0;
		// Blinking indicates battery status
		public var copter_leds_motor_blink	 		:int = 0;
		// Navigation confirmation blinks
		public var copter_leds_nav_blink 			:int = 0;



		// -----------------------------------------
		// Flight Modes
		// -----------------------------------------

		////////////////////////////////////////////////////////////////////////////////
		// Toy Mode
		////////////////////////////////////////////////////////////////////////////////
		public var toy_lookup						:Array;
		public var toy_alt							:int = -1;
		public var CH7_toy_flag						:Boolean;
		//public var CH6_toy_flag						:Boolean;


		// -----------------------------------------
		// Logging
		// -----------------------------------------
		public var p_stab							:int = 0;
		public var i_stab							:int = 0;

		public var p_stab_rate						:int = 0;
		public var i_stab_rate						:int = 0;
		public var d_stab_rate						:int = 0;
		public var roll_rate_error					:int = 0;
		public var pitch_rate_error					:int = 0;

		public var p_loiter_rate					:Number = 0;
		public var i_loiter_rate					:Number = 0;
		public var d_loiter_rate					:Number = 0;


		public var crosstrack_score					:Number = 0;
		public var alt_hold_score					:Number = 0;

		public var z_rate_error						:Number = 0;

		public var p_alt_rate						:Number = 0;
		public var i_alt_rate						:Number = 0;
		public var d_alt_rate						:Number = 0;

		public var p_accel_rate						:Number = 0;
		public var i_accel_rate						:Number = 0;
		public var d_accel_rate						:Number = 0;

		public function Main():void
		{
			// AP_InertialNav inertial_nav(&ahrs, &ins, &barometer, &g_gps);

			clock 					= Clock.getInstance();

			// on the stage
			sky.controller 			= this;
			ground.controller 		= this;

			copter 					= new Copter();
			ahrs					= AP_AHRS.getInstance();
			APM_RC					= AP_RC.getInstance();
			g_gps					= new GPS(copter.true_loc);
			barometer				= AP_Baro.getInstance();
			barometer.set_location(copter.true_loc);
			sonar					= Sonar.getInstance();
			sonar.set_location(copter.true_loc);
			ap						= new AP_State();
			ap_system				= new AP_System()
			waypoints				= new Array();
			sky.copter 				= copter;
			sky.gps					= g_gps;
			ground.copter 			= copter;
			toy_lookup 				= new Array();

			g = Parameters.getInstance();
			g.controller = this;
			g.visible = false;

			// for radio.as
			rc_ch					= new Array(g.rc_1, g.rc_2, g.rc_3, g.rc_4, g.rc_5, g.rc_6, g.rc_7, g.rc_8);

			//THOR Added for additional Fligt mode
			flight_mode_strings = new Array("STABILIZE","ACRO","ALT_HOLD","AUTO","GUIDED","LOITER","RTL","CIRCLE","POSITION","LAND","OF_LOITER","TOY_A","TOY_M");
			flight_modes 		= new Array(0,1,2,3,4,5,6,7,8,9,10,11,12,13);

			colors = new Array(0xD6C274, 0xDB9E46, 0x95706B, 0x9D2423, 0x7B962E, 0xB5BC87, 0x7EBC5F, 0x74287D, 0x765A70, 0xA82DBC, 0xD9B64E, 0xF28B50, 0xF25E3D, 0x79735E, 0x6D78F4);

			populateMenus(plotMenu);
			populateMenus(plotMenu2);
			populateMenus(plotMenu3);

			modeMenu.addItem(new QuickMenuItem("0  STABILIZE",	"0"));
			modeMenu.addItem(new QuickMenuItem("1  ACRO",		"1"));
			modeMenu.addItem(new QuickMenuItem("2  ALT_HOLD",	"2"));
			modeMenu.addItem(new QuickMenuItem("3  AUTO",		"3"));
			modeMenu.addItem(new QuickMenuItem("4  GUIDED",		"4"));
			modeMenu.addItem(new QuickMenuItem("5  LOITER",		"5"));
			modeMenu.addItem(new QuickMenuItem("6  RTL",		"6"));
			modeMenu.addItem(new QuickMenuItem("7  CIRCLE",		"7"));
			modeMenu.addItem(new QuickMenuItem("8  POSITION",	"8"));
			modeMenu.addItem(new QuickMenuItem("9  LAND",		"9"));
			modeMenu.addItem(new QuickMenuItem("10 OF_LOITER",	"10"));
			modeMenu.addItem(new QuickMenuItem("11 TOY_AltHold","11"));
			modeMenu.addItem(new QuickMenuItem("12 TOY_Manual",	"12"));

			addEventListener(Event.ADDED_TO_STAGE, addedToStage);

			init_sim();
	    }


	    public function populateMenus(m:QuickPopupMenu):void
		{
			// Stability
			m.addItem(new QuickMenuItem("Roll Sensor",			"roll_sensor"));
			m.addItem(new QuickMenuItem("Pitch Sensor",			"pitch_sensor"));
			m.addItem(new QuickMenuItem("Desired Roll",			"control_roll"));

			/*
			m.addItem(new QuickMenuItem("Roll Error", 			"roll_error"));
			m.addItem(new QuickMenuItem("Stabilize P", 			"stab_p"));
			m.addItem(new QuickMenuItem("Stabilize I", 			"stab_i"));

			m.addItem(new QuickMenuItem("Rate Roll Error", 		"roll_rate_error"));
			m.addItem(new QuickMenuItem("Stabilize Rate P", 	"rate_p"));
			m.addItem(new QuickMenuItem("Stabilize Rate I", 	"rate_i"));
			m.addItem(new QuickMenuItem("Stabilize Rate D", 	"rate_d"));
			m.addItem(new QuickMenuItem("Stabilize Dampener", 	"rate_damp"));
			m.addItem(new QuickMenuItem("Roll Output", 			"roll_output"));

			m.addDivider(new QuickMenuDivider());
			*/

			m.addItem(new QuickMenuItem("Stab Yaw I", 			"yaw_i"));
			m.addItem(new QuickMenuItem("Rate Yaw I", 			"rate_yaw_i"));
			m.addItem(new QuickMenuItem("Yaw Out", 				"yaw_out"));

			m.addDivider(new QuickMenuDivider());

			// Alt Hold
			m.addItem(new QuickMenuItem("throttle Cruise",		"throttle_cruise"));
			m.addItem(new QuickMenuItem("Throttle Output",		"throttle_out"));
			
			m.addItem(new QuickMenuItem("Altitude",				"altitude"));
			m.addItem(new QuickMenuItem("Next WP Alt",			"next_wp_alt"));
			m.addItem(new QuickMenuItem("Altitude Err",			"altitude_error"));
			m.addItem(new QuickMenuItem("Actual Altitude Err",	"act_altitude_error"));

			m.addItem(new QuickMenuItem("Climb Rate Error",		"z_rate_error"));
			m.addItem(new QuickMenuItem("Alt Rate P",		"alt_rate_p"));
			m.addItem(new QuickMenuItem("Alt Rate I",		"alt_rate_i"));
			m.addItem(new QuickMenuItem("Alt Rate D",		"alt_rate_d"));

			m.addItem(new QuickMenuItem("Z accel_error",		"z_accel_error"));

			m.addItem(new QuickMenuItem("Z accel P",		"alt_accel_p"));
			m.addItem(new QuickMenuItem("Z accel I",		"alt_accel_i"));
			m.addItem(new QuickMenuItem("Z accel D",		"alt_accel_d"));
			
			m.addItem(new QuickMenuItem("INAV Pos Err Z",			"z_error"));
			m.addItem(new QuickMenuItem("INAV Z Pos Correction",	"z_correction"));
			m.addItem(new QuickMenuItem("INAV Z Accel Correction",	"z_accel_correction"));
			m.addItem(new QuickMenuItem("hist_position_base_z",		"hist_position_base_z"));



			m.addDivider(new QuickMenuDivider());
			
			m.addItem(new QuickMenuItem("Accel Lon", 			"accel_lon"));
			// AP
			m.addItem(new QuickMenuItem("WP Distance", 			"wp_distance"));
			m.addItem(new QuickMenuItem("X Actual Speed", 		"x_speed"));
			m.addItem(new QuickMenuItem("X Target Speed",		"x_target_speed"));
			m.addItem(new QuickMenuItem("X Rate Error", 		"x_rate_error"));

			m.addItem(new QuickMenuItem("Y Actual Speed", 		"y_speed"));
			m.addItem(new QuickMenuItem("Y Target Speed",		"y_target_speed"));
			m.addItem(new QuickMenuItem("Y Rate Error", 		"y_rate_error"));
			m.addDivider(new QuickMenuDivider());

			// Loiter
			m.addItem(new QuickMenuItem("Lon Rate P", 	"loiter_lon_rate_p"));
			m.addItem(new QuickMenuItem("Lon Rate I", 	"loiter_lon_rate_i"));
			m.addItem(new QuickMenuItem("Lon Rate D", 	"loiter_lon_rate_d"));

			m.addItem(new QuickMenuItem("Lat Rate P", 	"loiter_lat_rate_p"));
			m.addItem(new QuickMenuItem("Lat Rate I", 	"loiter_lat_rate_i"));
			m.addItem(new QuickMenuItem("Lat Rate D", 	"loiter_lat_rate_d"));
			m.addDivider(new QuickMenuDivider());


			m.addItem(new QuickMenuItem("cross_track_dist", 		"cross_track_dist"));
			// Nav
			//m.addItem(new QuickMenuItem("Nav Lon Rate P", 		"nav_lon_rate_p"));
			//m.addItem(new QuickMenuItem("Nav Lon Rate I", 		"nav_lon_rate_i"));
			//m.addItem(new QuickMenuItem("Nav Lon Rate D", 		"nav_lon_rate_d"));

			m.addDivider(new QuickMenuDivider());

			m.addItem(new QuickMenuItem("Nav LonX",		 		"accel_lon"));
			m.addItem(new QuickMenuItem("Nav LatY",		 	"accel_lat"));

			m.addItem(new QuickMenuItem("Auto Roll",		 	"auto_roll"));
			m.addItem(new QuickMenuItem("Auto Pitch",		 	"auto_pitch"));
			
			m.addItem(new QuickMenuItem("Groudn Speed",		 	"ground_speed"));


			m.addItem(new QuickMenuItem("Nav Lat Rate I", 		"nav_lat_rate_i"));
			m.addItem(new QuickMenuItem("Y Lat Error", 			"lat_error"));
			m.addDivider(new QuickMenuDivider());

			//m.addItem(new QuickMenuItem("Accel Velocity X",		"vel_x"));


			m.addDivider(new QuickMenuDivider());


			m.addItem(new QuickMenuItem("Angle Boost",			"angle_boost"));
			//m.addItem(new QuickMenuItem("Motor 1",				"motor_1"));
			//m.addItem(new QuickMenuItem("Motor 2",				"motor_2"));

			m.addItem(new QuickMenuItem("Wind Speed",			"wind_speed"));
			m.addItem(new QuickMenuItem("Yaw Sensor",			"yaw_sensor"));

			m.addDivider(new QuickMenuDivider());

			m.addItem(new QuickMenuItem("WP_Bearing",			"t_angle"));
			m.addItem(new QuickMenuItem("WP Angle Err",			"t_angle_err"));



		}

	    public function addedToStage(even:Event):void
		{
			stop();
            stage.scaleMode		= StageScaleMode.NO_SCALE;
          	stage.align			= StageAlign.TOP_LEFT;

			motors = new AP_MotorsQuad(g.rc_1, g.rc_2, g.rc_3, g.rc_4);

			wp_manager.set_waypoint_array(waypoints);
			wp_manager.controller = this;

			stage.addEventListener(KeyboardEvent.KEY_UP,keyUpHandler);
			stage.addEventListener(KeyboardEvent.KEY_DOWN,keyDownHandler);

			sim_controller.setLabel("START SIM");
			sim_controller.setEventName("RUN_SIM");
			stage.addEventListener("RUN_SIM",simHandler);

			graph_button.setLabel("Clear Graph");
			graph_button.setEventName("CLEAR_GRAPH");
			stage.addEventListener("CLEAR_GRAPH",graphHandler);

			joystick_button.setLabel("Show/Hide Joysticks");
			joystick_button.setEventName("JOYSTICKS");
			stage.addEventListener("JOYSTICKS",joyHandler);

			waypoint_button.setLabel("Show Waypoints");
			waypoint_button.setEventName("WAYPOINTS");
			stage.addEventListener("WAYPOINTS",wpHandler);

			arm_button.setLabel("ARM");
			arm_button.setEventName("ARM");
			stage.addEventListener("ARM",armHandler);

			plotMenu.setEventName("PLOT_MENU");
			plotMenu2.setEventName("PLOT_MENU");
			plotMenu3.setEventName("PLOT_MENU");
			stage.addEventListener("PLOT_MENU",plotMenuHandler);


			modeMenu.setEventName("MODE_MENU");
			stage.addEventListener("MODE_MENU",modeMenuHandler);

			gains_button.setEventName("TOGGLE_GAINS");
			gains_button.setLabel("Show Gains");
			stage.addEventListener("TOGGLE_GAINS", gainsHandler);


			// setup radio
			//rc_throttle.sticky = true;
			//right_sticks.sticky_x = true;
			//right_sticks.sticky_y = true;
			left_sticks.sticky_y = true;
			left_sticks.sticky_x = false;
			left_sticks.knob_y.y = 11;

			init_sim();

			// PLOTTING
			//addChildAt(plotView, 0);
			plotView.dataScaleY		= 0.25
			plotView.dataScaleX		= 3.0

			left_sticks.visible = true;
			right_sticks.visible = true;

			plotMenu.setSelectedItemByName("WP Distance");
			plotMenu.setSelectedItemByName("Roll Sensor");
			plotMenu.setSelectedItemByName("Altitude");

			this.addEventListener(Event.ENTER_FRAME, idle);
		}

		public function idle(e:Event):void
		{
			if(medium_loopCounter++ >= 50){
				medium_loopCounter	= 0;
				user_settings_sim();
				set_home_is_set(true);
				//air_start();
				//update_altitude();
				//next_WP.lng				= g.target_distance_BI.getNumber();
				//next_WP.alt				= g.target_altitude_BI.getNumber();
				//copter.position.z		= copter.loc.alt = g.start_height_BI.getNumber(); // add in some delay

				//if(g.start_position_BI.getNumber() != 0)
				//	copter.position.x = g.start_position_BI.getNumber();
				sky.draw();
				ground.draw(false);
				sky.failsafe_MC.visible = false;
				sky.copter_mc.stop()				
			}

			if (g_gps.new_data == true){
				g_gps.new_data = false;
				update_GPS();
				iteration = 0;
			}
		}

		public function runSim(e:Event):void
		{
			// 50hz update
			update_sim_radio();

			// run 2x to get 100hz updates
			for(var i:int = 0; i < (g.sim_speed * 2); i++){
				g_gps.read();// fake a GPS read

				// run the main loop
				loop();
				// add 10 millis to the loop
				clock.add_millis(10);
				//elapsed += 10; // 50 * 20  = 1000 ms
			}

			sky.draw();
				
			if(APM_RC.output_pwm[0] > 1001)
				sky.copter_mc.play()
			else
				sky.copter_mc.stop()
			
			armed_flag.visible =  ap.armed;

			if(control_mode == AUTO)
				ground.draw(true);
			else
				ground.draw(false);
				
			sky.failsafe_MC.visible = radio_failure;
		}

		include "Arducopter/config.h";
		include "Arducopter/location.as";
		include "Arducopter/AP_State.as";
		include "Arducopter/Arducopter.as";
		include "Arducopter/attitude.as";
		include "Arducopter/toy.as";
		include "Arducopter/commands.as";
		include "Arducopter/commands_logic.as";
		include "Arducopter/commands_process.as";
		include "Arducopter/control_modes.as";
		include "Arducopter/events.as";
		include "Arducopter/flip.as";
		include "Arducopter/inertia.as";
		include "Arducopter/motors.as";
		include "Arducopter/navigation.as";
		include "Arducopter/radio.as";
		include "Arducopter/log.as";
		include "Arducopter/sensors.as";
		include "Arducopter/system.as";
		include "Arducopter/compat.as";
		include "Arducopter/position_vector.as";


		// ----------------------------------------
		// SIM State
		// ----------------------------------------
		private function gainsHandler(e:Event):void
		{
			if(g.visible){
				g.visible = false;
				gains_button.setLabel("Show Gains");
			}else{
				g.visible = true;
				g.y = 0;
				gains_button.setLabel("Hide Gains");
			}
		}

		private function graphHandler(e:Event):void
		{
			plotView.clearPlots();
			colorIndex = 0;
			plotMenuHandler(null);
			plot_A = plotView.addPlot("plot_A", 	getNewColor());
			plot_B = plotView.addPlot("plot_B", 	getNewColor());
			plot_C = plotView.addPlot("plot_C", 	getNewColor());

			//stopSIM();
		}

		private function wpHandler(e:Event):void
		{
			wp_manager.visible = !wp_manager.visible;
			if(wp_manager.visible){
				waypoint_button.setLabel("Hide Waypoints");
			}else{
				waypoint_button.setLabel("Show Waypoints");
			}
		}

		private function joyHandler(e:Event):void
		{
			left_sticks.visible = !left_sticks.visible;
			right_sticks.visible = !right_sticks.visible;
			/*
			plotView.visible = !plotView.visible;
			if(plotView.visible){
				// hide joysticks
				left_sticks.visible = false;
				right_sticks.visible = false;
				joystick_button.setLabel("Show Joysticks");
			}else{
				// show joysticks
				left_sticks.visible = true;
				right_sticks.visible = true;
				joystick_button.setLabel("Show Graph");
			}
			*/
		}



		private function simHandler(e:Event):void
		{
			if(simIsRunnning == true){
				stopSIM();
			}else{
				startSIM();
			}
		}

		private function stopSIM():void
		{
			trace("---------------Stop sim---------------")
			trace("Alt Hold score:\t", alt_hold_score);
			trace("WP Nav score:\t", crosstrack_score);
			simIsRunnning = false;
			this.removeEventListener(Event.ENTER_FRAME, runSim);
			this.addEventListener(Event.ENTER_FRAME, idle);
			sim_controller.setLabel("START SIM");
			update_arm_label();
			init_disarm_motors();
			init_sim();
		}

		private function startSIM():void
		{
			trace("---------------Start sim---------------")
			sim_controller.setLabel("STOP SIM");
			simIsRunnning = true;
			g.updateGains();
			init_sim();
			user_settings_sim();
			init_ardupilot();
			air_start();
			ground.clearTracks();
			
			
			// force a new state into the system for "Air Start"
			plotMenuHandler(null);
			plot_A = plotView.addPlot("plot_A", 	getNewColor());
			plot_B = plotView.addPlot("plot_B", 	getNewColor());
			plot_C = plotView.addPlot("plot_C", 	getNewColor());
			this.removeEventListener(Event.ENTER_FRAME, idle);
			this.addEventListener(Event.ENTER_FRAME, runSim);


			// hide gains
			g.visible = false;
			gains_button.setLabel("Show Gains");
			sky.draw();
			ground.draw(false);
			//motors.arm(true)
		}


		private function user_settings_sim():void
		{
			// Attitude
			ahrs.dcm.appendRotation(g.start_angle_BI.getNumber(), 	Vector3D.X_AXIS);	// ROLL
			ahrs.dcm.appendRotation(0, 	Vector3D.Y_AXIS);	// PITCH
			ahrs.dcm.appendRotation(0, 	Vector3D.Z_AXIS);	// Yaw

			// rotation speed
			ahrs.gyro.x				= g.start_rotation_BI.getNumber();
			ahrs.gyro.y				= 0;
			ahrs.gyro.z				= 0;

			// Velocity
			copter.velocity.x		= g.start_speed_BI.getNumber();
			copter.velocity.y		= 0;
			copter.velocity.z		= g.start_climb_rate_BI.getNumber();

			// Position
			copter.position.x 		= g.start_position_BI.getNumber();
			copter.position.y 		= 0;
			copter.position.z 		= g.start_height_BI.getNumber(); // add in some delay

			copter.true_loc.lng 	= copter.position.x;
			copter.true_loc.lat 	= copter.position.y;
			copter.true_loc.alt 	= copter.position.z;

			current_loc.lng			= copter.position.x;
			current_loc.lat			= copter.position.y;
			current_loc.alt			= copter.position.z;
			
			
			next_WP.lat				= current_loc.lat;
			next_WP.lng				= current_loc.lng;
			next_WP.alt				= current_loc.alt;

			barometer.init();
			g_gps.init();
					
			barometer.enable_noise	= g.baro_noise_checkbox.getSelected();
			fastPlot				= g.fastPlot_checkbox.getSelected();


			// Setup Wind
			if(g.wind_checkbox.getSelected()){
				copter.windGenerator.setDirection(g.windDir_BI.getNumber());
				copter.windGenerator.setSpeed(g.wind_low_BI.getNumber(), g.wind_high_BI.getNumber());
				copter.windGenerator.setPeriod(g.wind_period_BI.getNumber()*1000);
			}else{
				copter.windGenerator.setSpeed(0, 0);
			}

			g.rc_3.servo_out 	= g.rc_3.control_in;
		}

		private function init_wp():void
		{
			include "Sim/waypoints.txt"
		}

		private function init_sim():void
		{
			// initialize all internal values to default state
			copter.init();
			ahrs.init();
			ap.init()
			ap_system.init()
			
			//user.init();
			copter.windGenerator.resetWind();

			//inertial_nav			= new AP_InertialNav(ahrs, barometer, g_gps);
			inertial_nav	= new AP_InertialNav(ahrs, barometer, g_gps);
			wp_nav			= new AC_WPNav(inertial_nav, g.pi_loiter_lat, g.pi_loiter_lon, g.pid_loiter_rate_lat, g.pid_loiter_rate_lon);
			sky.wp_nav = wp_nav;
			ground.wp_nav = wp_nav;
			ground.inertial_nav = inertial_nav;
			
			
			//copter.setThrottleCruise(g.throttle_cruise);
			//g.throttle_cruise 	= g.THROTTLE_CRUISE + g.throttle_cruise_e;

			home					= new Location();
			current_loc				= new Location();
			next_WP					= new Location();
			command_nav_queue 		= new Location();
			command_cond_queue 		= new Location();
			yaw_look_at_WP			= new Vector3D();
			circle_center			= new Vector3D();

			sky.current_loc 		= this.current_loc; // Pass a reference to Sky
			ground.current_loc 		= this.current_loc;

			wpinav_origin			= new Point();
			wpinav_destination		= new Point();
			wpinav_target			= new Point();
			wpinav_pos_delta		= new Point();
			ground.wpinav_target = wpinav_target;
			// --------------------------------------
			// Timers
			// --------------------------------------
			iteration 						= 0;
			//elapsed							= 0;
			fifty_toggle					= false;
			medium_loopCounter				= 0;
			slow_loopCounter				= 0;
			superslow_loopCounter			= 0;
			auto_disarming_counter			= 0;
			last_gps_time					= 0;
			counter_one_herz				= 0;
			crosstrack_score				= 0;
			alt_hold_score					= 0;
			event_undo_value				= 0;
			event_id						= 0;

			// -----------------------------------------
			// Flight Modes
			// -----------------------------------------
			roll_pitch_mode					= 0;
			yaw_mode						= 0;
			throttle_mode					= 0;
			throttle_avg					= 0;
			circle_angle					= 0;
			loiter_time						= 0;
			loiter_time_max					= 0;

			// -----------------------------------------
			// Simple Mode
			// -----------------------------------------
			oldSwitchPosition				= 0;
			initial_simple_bearing			= 0;
			simple_counter					= 0;
			CH7_wp_index					= 0;

			// -----------------------------------------
			// Loiter and NAV
			// -----------------------------------------
			p_loiter_rate					= 0;
			i_loiter_rate					= 0;
			d_loiter_rate					= 0;
			nav_lon							= 0;
			nav_lat							= 0;
			nav_roll						= 0;
			nav_pitch						= 0;
			sonar_alt						= 0;
			wp_distance						= 0;
			home_distance					= 0;
			jump							= -10;
			command_cond_index				= 0;
			prev_nav_index					= 0;
			command_nav_index				= 0;
			condition_value					= 0;
			condition_start					= 0;
			nav_yaw							= 0;
			cos_roll_x						= 1;
			cos_pitch_x						= 1;
			cos_yaw_x						= 1;
			sin_yaw_y						= 0;

			// -----------------------------------------
			// Acro
			// -----------------------------------------
			roll_axis						= 0;
			pitch_axis						= 0;
			flip_timer						= 0;
			flip_state 						= 0;

			// -----------------------------------------
			// Stabilize
			// -----------------------------------------
			p_stab							= 0;
			i_stab							= 0;
			p_stab_rate						= 0;
			i_stab_rate						= 0;
			d_stab_rate						= 0;
			roll_rate_error					= 0;
			pitch_rate_error				= 0;

			control_roll					= 0;
			control_pitch					= 0;

			// -----------------------------------------
			// Altitude hold
			// -----------------------------------------
			angle_boost						= 0;
			nav_throttle					= 0;
			p_alt_rate						= 0;
			i_alt_rate						= 0;
			d_alt_rate						= 0;
			z_rate_error					= 0;
			z_accel_error					= 0;

			altitude_error					= 0;

			climb_rate						= 0;


			reset_I_all();
			reset_nav_params();

			// Copter state
			radio_failure 		= false;

			ch_1_pwm			= 1500;
			ch_2_pwm			= 1500;
			ch_3_pwm			= 1000;
			ch_4_pwm			= 1500;
			ch_5_pwm			= 1500;
			ch_6_pwm			= 1100;
			ch_7_pwm			= 1500;

		}


		public function update_sim_radio():void
		{
			//
			//read out rc_throttle and rc_roll

			if(radio_failure){
				ch_3_pwm = 900;
			}else{
				ch_1_pwm = right_sticks.pwm_x;
				ch_2_pwm = right_sticks.pwm_y;
				ch_3_pwm = 3000 - left_sticks.pwm_y; // reversed
				ch_4_pwm = left_sticks.pwm_x;
			}
			//trace("ch_3_pwm",ch_3_pwm);

			APM_RC.set_PWM_channel(ch_1_pwm,	 	CH_1);
			APM_RC.set_PWM_channel(ch_2_pwm, 		CH_2);
			APM_RC.set_PWM_channel(ch_3_pwm,	 	CH_3);
			APM_RC.set_PWM_channel(ch_4_pwm, 		CH_4);
			APM_RC.set_PWM_channel(ch_5_pwm, 		CH_5);
			APM_RC.set_PWM_channel(ch_6_pwm, 		CH_6);
			APM_RC.set_PWM_channel(ch_7_pwm, 		CH_7);
			APM_RC.set_PWM_channel(ch_8_pwm, 		CH_8); // not used
		}

		// -----------------------------------------------------------------------------------
		// GUI Handlers
		//------------------------------------------------------------------------------------
		private function keyDownHandler(k:KeyboardEvent):void
		{
			switch(k.keyCode){
				case 81:  // Q up
					ch_7_pwm = 1800;
					break;
				case 90:  // Z down
					ch_7_pwm = 1200;
					break;


				case 118:  // F7 or channel 7
					if(k.shiftKey){
						ch_7_pwm = 1200;
					}else{
						ch_7_pwm = 1800;
					}
					break;

				case 65: 	// a
					//if()
					break;
					
				case 117:  	// F6 or channel 6
					if(ch_6_pwm >= 1500){
						trace("ch_6_pwm LOW")
						ch_6_pwm = 1200;
					}else{
						trace("ch_6_pwm HIGH")
						ch_6_pwm = 1900;
					}
					break;
			}

		}

		private function keyUpHandler(k:KeyboardEvent):void
		{
			trace(k.keyCode);

			if(k.shiftKey){
				switch(k.keyCode){
					// --------------------------------------------------------------------------
					// Flight modes
					// --------------------------------------------------------------------------
					case 48: // 0
						modeMenu.setSelectedIndex(STABILIZE);
						radio_switch_position = STABILIZE;
						break;
					case 49: // 1
						modeMenu.setSelectedIndex(ACRO);
						radio_switch_position = ACRO;
						break;
					case 50: // 2
						modeMenu.setSelectedIndex(ALT_HOLD);
						radio_switch_position = ALT_HOLD;
						break;
					case 51: // 3
						//set_mode(AUTO);
						modeMenu.setSelectedIndex(AUTO);
						radio_switch_position = AUTO;
						break;
					case 52: // 4
						modeMenu.setSelectedIndex(GUIDED);
						radio_switch_position = GUIDED;
						break;
					case 53: // 5
						modeMenu.setSelectedIndex(LOITER);
						radio_switch_position = LOITER;
						break;
					case 54: // 6
						modeMenu.setSelectedIndex(RTL);
						radio_switch_position = RTL;
						break;
					case 55: // 7
						modeMenu.setSelectedIndex(CIRCLE);
						radio_switch_position = CIRCLE;
						break;
					case 56: // 8
						modeMenu.setSelectedIndex(POSITION);
						radio_switch_position = POSITION;
						break;
					case 57: // 9
						modeMenu.setSelectedIndex(LAND);
						radio_switch_position = LAND;
						break;

					// --------------------------------------------------------------------------
					// Plot control
					// --------------------------------------------------------------------------
					case 38: // Up
						plotView.setScale(plotView.dataScaleX, (plotView.dataScaleY + .01))
						break;
					case 40: // Down
						plotView.setScale(plotView.dataScaleX, (plotView.dataScaleY -.01))
						break;
					case 37: // Left
						plotView.setScale((plotView.dataScaleX/2), plotView.dataScaleY)
						break;
					case 39: // Right
						plotView.setScale((plotView.dataScaleX*2), plotView.dataScaleY)
						break;
				}
			}
			trace("k.keyCode", k.keyCode)
			switch(k.keyCode){
				case 81:  // Q up
				case 90:  // Z down
					ch_7_pwm = 1500;
					break;

				case 220:  //  \ for entering full screen
				
					ground.toggle()
					break;
					
					
				case 66:  // b
					copter.jump();
					break;

				case 84:  // 1 for
					test_radio_rage_output();
					break;

				case Keyboard.SPACE:
					gainsHandler(null);
					break;

				case 87:  // w for waypoint dump
					report_wp();
					break;

				case 118:  // F7 or channel 7
					ch_7_pwm = 1500;
					break;

				case 70: // f for fail
					radio_failure = !radio_failure
					trace("RC Failure");
					break;

				case 83: // s for start/stop
					simHandler(null);
					break;

				case 72: // h for hover
					left_sticks.knob_y.y = 11;

					break;

				case 76: // l for loop
					init_flip();
					break;

				case 65: // q for up NextWP by 100
					//toy_alt_hold = !toy_alt_hold;
					//force_new_altitude(current_loc.alt);
					break;

				//case 81: // q for up NextWP by 100
					break;

				//case 90: // z for downp NextWP by 100

					//toy_alt = Math.max(toy_alt, 200);
					//force_new_altitude(toy_alt);
					//next_WP.alt -= 100;
					//next_WP.alt = Math.max(next_WP.alt, 100);
					break;

			}
		}

		private function plotMenuHandler(e:Event):void
		{
			var item:QuickMenuItem;
			
			item = plotMenu.getSelectedItem();
			this.plotType_A = item.getCode();
			plotMenu.setLabel(item.getLabel());

			item = plotMenu2.getSelectedItem();
			this.plotType_B = item.getCode();
			plotMenu2.setLabel(item.getLabel());

			item = plotMenu3.getSelectedItem();
			this.plotType_C = item.getCode();
			plotMenu3.setLabel(item.getLabel());
		}

		private function modeMenuHandler(e:Event):void
		{
			var item:QuickMenuItem = modeMenu.getSelectedItem();
			modeMenu.setLabel(item.getLabel());
			if(e != null)
				radio_switch_position = modeMenu.getSelectedIndex();
		}

		private function armHandler(e:Event):void
		{
			motors.arm(!motors.armed());
			update_arm_label();
		}

		private function update_arm_label():void
		{
			//trace("motors.armed", motors.armed);

			if(motors.armed())
				arm_button.setLabel("Motors Armed");
			else
				arm_button.setLabel("Motors Disarmed");
		}
		// -----------------------------------------------------------------------------------
		// Plotting
		//------------------------------------------------------------------------------------

		private function plot(ptype:String, plot_num:int, plot_label:int):void
		{
			var val		:Number = 0;
			var _scale	:Number = 1;

			switch (ptype)
			{
				// stabilze
				case "roll_sensor":
					val = ahrs.roll_sensor;
					_scale = .1;
				break;

				case "pitch_sensor":
					val = ahrs.pitch_sensor;
					_scale = .1;
				break;
				case "control_roll":
					val = control_roll;
				break;



				case "roll_error":
					val = (control_roll - ahrs.roll_sensor);
				break;

				case "stab_p":
					val = p_stab;
				break;
				case "stab_i":
					val = i_stab;
				break;


				case "roll_rate_error": // roll rate error
					val = roll_rate_error;
				break;
				case "rate_p":
					val = p_stab_rate;
				break;
				case "rate_i":
					val = i_stab_rate;
				break;
				case "rate_d":
					val = d_stab_rate;
				break;
				case "roll_output":
					val = roll_output;
				break;

				case "yaw_i":
					_scale = .1;
					val = g.pi_stabilize_yaw.get_integrator();
				break;
				case "rate_yaw_i":
					_scale = .1;
					val = g.pid_rate_yaw.get_integrator();
				break;


				case "wp_distance":
					_scale = .1;
					val = wp_distance;
				break;

				case "x_speed":
					val = copter.velocity.x;
				break;

				case "x_rate_error":
					//val = x_rate_error;
				break;

				case "y_speed":
					val = copter.velocity.y;
				break;

				case "y_target_speed":
					//val = y_target_speed;
				break;

				case "y_rate_error":
					//val = y_rate_error;
				break;


				//case "accel_lon":
					//_scale = .1;
					//val = accel_lon;
				//break;

				case "accel_lat":
					//_scale = .1;
					//val = accel_lat;
				break;

				case "auto_roll":
					//_scale = .1;
					val = wp_nav.get_desired_roll();
				break;

				case "auto_pitch":
					//_scale = .1;
					val = wp_nav.get_desired_pitch();
				break;


				case "ground_speed":
					val = g_gps.ground_speed;
				break;



				case "loiter_lat_rate_p":
					val = wp_nav.d_loiter_lat_rate;
				break;

				case "loiter_lat_rate_i":
					val = wp_nav.d_loiter_lat_rate
				break;

				case "loiter_lat_rate_d":
					val = wp_nav.d_loiter_lat_rate;
				break;

				case "loiter_lon_rate_p":
					val = wp_nav.d_loiter_lon_rate;
				break;

				case "loiter_lon_rate_i":
					val = wp_nav.d_loiter_lon_rate
				break;

				case "loiter_lon_rate_d":
					val = wp_nav.d_loiter_lon_rate;
				break;



				case "cross_track_dist":
					//val = cross_track_dist;
				break;

				case "nav_lon_rate_p":
					//val = p_nav_rate;
				break;

				case "nav_lon_rate_i":
					//val = i_nav_rate;
				break;

				case "nav_lon_rate_d":
					//val = d_nav_rate;
				break;

				case "nav_lat_rate_i":
					val = g.pid_nav_lat.get_integrator();
				break;

				case "accel_lon":
					val = ahrs.accel_ef.y;
					_scale = 100;
				break;


				// -----------------------------------------------------------------------------------
				// Alt hold
				case "throttle_cruise":
					val = g.throttle_cruise;
				break;

				case "throttle_out":
					val = g.rc_3.servo_out;
				break;

				case "altitude":
					val = current_loc.alt;
				break;
				case "next_wp_alt":

					val = next_WP.alt;
				break;

				case "act_altitude_error":
					val = (next_WP.alt - copter.true_loc.alt);
					_scale = .01;
				break;

				case "altitude_error":
					val = (next_WP.alt - current_loc.alt);
					_scale = .01;
				break;


				case "z_rate_error":
					val = z_rate_error;
				break;

				case "alt_rate_p":
					val = p_alt_rate;
				break;

				case "alt_rate_i":
					val = i_alt_rate;
					_scale = 10;
				break;

				case "alt_rate_d":
					val = d_alt_rate;
				break;


				case "z_accel_error":
					val = z_accel_error;
				break;

				case "alt_accel_p":
					val = p_accel_rate;
				break;

				case "alt_accel_i":
					val = i_accel_rate;
					//_scale = 10;
				break;

				case "alt_accel_d":
					val = d_accel_rate;
				break;



				case "z_correction":
					val = inertial_nav._position_correction.z;
				break;

				case "z_accel_correction":
					val = inertial_nav.accel_correction_bf.z;
				break;

				case "z_error":
					val = inertial_nav._position_error.z;
				break;
				case "hist_position_base_z":
					val = inertial_nav.hist_position_base_z;
				break;


				// -----------------------------------------------------------------------------------
				// 


				//case "z_boost":
			//		val = z_boost;
					//_scale = 100;
			//	break;


				case "angle_boost":
					val = angle_boost;
				break;



				/*case "motor_1":
					val = motor_out[MOT_1] - 1000;
				break;

				case "motor_2":
					val = motor_out[MOT_2] - 1000;
				break;
				*/

				case "wind_speed":
					val = copter.wind.x;
				break;

				case "yaw_sensor":
					_scale = .01;
					val = ahrs.yaw_sensor;
				break;

				case "t_angle_err":
					_scale = .01;
					val = wp_bearing - original_wp_bearing;
				break;

				case "t_angle":
					_scale = .01;
					val = wp_bearing;
				break;



				default:
					val = 0;
				//no case tested true;
			}

			plotView.setValue(val,	plot_num, _scale);

			if(plot_label == 1)
				plot_TF.text = val.toFixed(2);
			else if (plot_label == 2)
				plot2_TF.text = val.toFixed(2);
			else
				plot3_TF.text = val.toFixed(2);
		}

		// -----------------------------------------------------------------------------------
		// Utility functions
		//------------------------------------------------------------------------------------

		public function test_radio_rage_input(index:int = 255)
		{
			ch_6_pwm = 990;
			g.rc_6.set_dead_zone(60);
			g.rc_6.set_range(1000,2000);
			trace("----------------------------")
			trace(g.rc_6._high, g.rc_6._high);
			trace("----------------------------")

			for (var i:int = 0; i < 1020; i++){
				APM_RC.set_PWM_channel(ch_6_pwm, CH_6);
				g.rc_6.set_pwm(APM_RC.InputCh(CH_6));
				ch_6_pwm++;
				trace(g.rc_6.radio_in, g.rc_6.control_in);
			}
		}

		public function test_radio_rage_output(index:int = 255)
		{
			g.rc_3.set_range(g.throttle_min, g.throttle_max);
			g.rc_3.set_range_out(0, 1000);
			g.rc_3.set_dead_zone(60);
			var s_out:int = 0;

			trace("----------------------------")
			trace(g.rc_3._high_out, g.rc_3._low_out);
			trace("----------------------------")

			for (s_out = 0; s_out < 1000; s_out++){
				//APM_RC.set_PWM_channel(ch_3_pwm, CH_3);
				//g.rc_3.set_pwm(APM_RC.InputCh(CH_3));

				g.rc_3.servo_out = s_out;
				g.rc_3.calc_pwm();
				trace("servo_out:", s_out, Math.floor(g.rc_3.pwm_out), Math.floor(g.rc_3.radio_out));
			}
		}

		public function report_wp(index:int = 255)
		{
			current_loc.lat = 389539260;
			current_loc.lng = -1199540200;

			next_WP.lat = 389538528;
			next_WP.lng = -1199541248;

			//filtered_loc = next_WP.clone();


			trace("dist:", wp_distance, "bear:", wp_bearing, "scale lon:", scaleLongDown);
		}

		public function report_wp2(index:int = 255)
		{
			var temp:Location;
			if(index == 255){
				for(var i:int = 0; i < g.command_total; i++){
					temp = get_cmd_with_index(i);
					print_wp(temp, i);
				}
			}else{
				temp = get_cmd_with_index(index);
				print_wp(temp, index);
			}
		}

		public function print_wp(cmd:Location, index:int)
		{
			trace("WP, " + index + " id:" + cmd.id + " op:" + cmd.options + " p1:" + cmd.p1 + " p2:" + cmd.alt + " p3:" + cmd.lat + " p4:" + cmd.lng);
			//trace("CMD, " + index + ", " + cmd.id + ", " + cmd.options + ", " + cmd.p1 + ", " + cmd.alt + ", " + cmd.lat + ", " + cmd.lng);
		}

		public function getNewColor():Number
		{
			colorIndex++;
			if (colorIndex >= colors.length)
				colorIndex = 0;
			return colors[colorIndex];

			//Math.floor(Math.random() * 0xFFFFFF);
		}

	}
}

