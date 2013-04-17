package com {
	import flash.display.MovieClip;
    import flash.display.Sprite;
    import flash.display.Stage;
    import flash.display.StageDisplayState;
    import flash.display.Bitmap;
    import flash.display.BitmapData;

	import flash.display.DisplayObject;
	import flash.events.*;
	import flash.geom.Rectangle;
	import flash.geom.Point;
	import flash.display.StageAlign;
	import flash.display.StageScaleMode;

	import com.UI.Plot;
	import com.UI.PlotView;
	import com.libraries.AverageFilter;

	import com.UI.QuickMenu;
	import com.UI.QuickMenuItem;
	import com.UI.QuickMenuDivider;
	import com.UI.QuickPopupMenu;
	import com.UI.BasicInput;

	import com.Sim.Wind;
	import com.libraries.AP_AHRS;
	import com.libraries.AC_PID;
	import com.libraries.RC_Channel;
	import com.Main;

	public class Parameters extends MovieClip
	{
		private static var instance			:Parameters = null;
		public var controller				:Main;
		public var frame					:Rectangle;
		public var originalX				:Number = 0;

		public var simple_checkbox			:QuickCheckBox;

		// ---------------------------------------------
		// Sim Details controls
		// ---------------------------------------------
		public var sim_iterations			:int 		= 99999;
		public var sim_speed				:int 		= 1;

		public var windSpeedMin				:Number 	= 150;
		public var windSpeedMax				:Number 	= 200;
		public var windPeriod				:Number 	= 30000;
		public var windDir					:Number 	= 45;

		public var airDensity				:Number 	= 1.184;
		//public var crossSection				:Number 	= 0.015;
		public var crossSection				:Number 	= 0.012;
		public var dragCE					:Number 	= 0.20;
		public var speed_filter_size		:Number 	= 2;
		public var motor_kv					:Number 	= 1000;
		public var moment					:Number 	= 3;
		public var mass						:Number 	= 500;
		public var esc_delay				:int 		= 12;
		public var test						:Boolean = false;
		public var toy_edf					:Boolean = true;

	// WPNAV
	public var _speed_cms				:Number = 500;
	public var _wp_radius_cm			:Number = 200;

	public var rtl_altitude				:int 		= 2500; // ALT_HOLD_HOME  height to return to Home, 0 = Maintain current altitude
	public var sonar_enabled			:Boolean 	= false;
	public var battery_monitoring		:Boolean 	= false;
	// ...
	public var failsafe_battery_enabled	:Boolean 	= false;
	public var failsafe_gps_enabled		:Boolean 	= false;

	public var compass_enabled			:Boolean 	= false;
	public var optflow_enabled			:Boolean 	= false;
	// ..
	public var super_simple				:Boolean 	= false;
	public var rtl_alt_final			:int 		= 200;
	public var axis_enabled				:Boolean 	= false;
	public var copter_leds_mode			:int 		= 9;
	// ..
	public var throttle_accel_enabled 	:Boolean 	= true;
	public var yaw_override_behaviour	:int 		= 0; //YAW_OVERRIDE_BEHAVIOUR_AT_NEXT_WAYPOINT;  // maximum vertical velocity the pilot may request

    // Waypoints
    //
	public var command_total			:int 		= 0;
	public var command_index			:int 		= 0;
	public var circle_radius			:int 		= 10;
	public var rtl_loiter_time			:int 		= 5000;
	public var land_speed				:int 		= 50;
	public var auto_velocity_z_min		:int	 	= -125;	// minimum vertical velocity (i.e. maximum descent) the autopilot may request
	public var auto_velocity_z_max		:int 		= 125;	// maximum vertical velocity the autopilot may request
	public var pilot_velocity_z_max		:int 		= 250;  // maximum vertical velocity the pilot may request


    // Throttle
    //
	public var throttle_min				:int 		= 130;
	public var throttle_max				:int 		= 1000;
	public var failsafe_throttle		:int 		= 0; //FS_THR_DISABLED;
	public var failsafe_throttle_value	:int 		= 975;
	public var throttle_cruise			:int 		= 500;
	public var throttle_mid				:int 		= 500;

    // Flight modes
    //
	public var flight_mode1				:int 		= 0; // STABILIZE
	public var flight_mode2				:int 		= 0;
	public var flight_mode3				:int 		= 0;
	public var flight_mode4				:int 		= 0;
	public var flight_mode5				:int 		= 0;
	public var flight_mode6				:int 		= 0;
	public var simple_modes				:int 		= 0;
	public var toy_alt_large			:int 		= 100;
	public var toy_alt_small			:int 		= 25;



    // Misc
    //
    public var log_bitmask				:int 		= 0;
	public var toy_yaw_rate				:int 		= 1; // 1 = fast, 2 = med, 3 = slow

	//public var esc_calibrate			:int 		= 0;
	//public var radio_tuning			:int 		= 0;
	//public var radio_tuning_high		:int 		= 0;
	//public var radio_tuning_low		:int 		= 0;
	public var frame_orientation		:int 		= 1;
	public var ch7_option				:int 		= 8;
	public var auto_slew_rate			:int 		= 100;

	// RC channels
	public var rc_1						:RC_Channel;
	public var rc_2						:RC_Channel;
	public var rc_3						:RC_Channel;
	public var rc_4						:RC_Channel;
	public var rc_5						:RC_Channel;
	public var rc_6						:RC_Channel;
	public var rc_7						:RC_Channel;
	public var rc_8						:RC_Channel;

    // Acro parameters
	public var acro_p					:Number = 4.5; // 4.5 default
	public var acro_balance_roll		:int = 200;
	public var acro_balance_pitch		:int = 200;
	public var acro_trainer_enabled		:Boolean = true;

    // PI/D controllers
	public var pid_rate_roll			:AC_PID;
	public var pid_rate_pitch			:AC_PID;
	public var pid_rate_yaw				:AC_PID;
	public var pid_loiter_rate_lon		:AC_PID
	public var pid_loiter_rate_lat		:AC_PID
	public var pid_nav_lon				:AC_PID;
	public var pid_nav_lat				:AC_PID;


	public var pid_throttle				:AC_PID;
	public var pid_throttle_accel		:AC_PID;
	//public var pid_optflow_roll			:AC_PID;
	//public var pid_optflow_pitch		:AC_PID;

	public var pi_loiter_lat			:AC_PID // APM_PI controllers
	public var pi_loiter_lon			:AC_PID
	public var pi_stabilize_roll		:AC_PID;
	public var pi_stabilize_pitch		:AC_PID;
	public var pi_stabilize_yaw			:AC_PID;
	public var pi_alt_hold				:AC_PID;


	// PID gains
	//
	// stabilize
	private var stabilize_p				:Number = 4.5;
	private var stabilize_i				:Number = 0.0;
	private var stabilize_imax			:Number = 800;

	private var rate_p					:Number = 0.15;
	private var rate_i					:Number = 0.1;
	private var rate_d					:Number = 0.004;
	private var rate_imax				:Number = 500;

	// Yaw
	private var stabilize_yaw_p			:Number = 7.0;
	private var stabilize_yaw_i			:Number = .02;
	private var stabilize_yaw_imax		:Number = 800;

	private var rate_yaw_p				:Number = 0.25;
	private var rate_yaw_i				:Number = 0.015;
	private var rate_yaw_d				:Number = 0.000;  // .002
	private var rate_yaw_imax			:Number = 800;


	//Alt
	//*
	private var alt_hold_p				:Number = 2.0;
	private var alt_hold_i				:Number = 0.0;
	private var alt_hold_imax			:Number = 300;

	private var throttle_rate_p			:Number = 6.0;
	private var throttle_rate_i			:Number = 0.0;
	private var throttle_rate_d			:Number = 0.2;
	private var throttle_rate_imax		:Number = 300;

	private var throttle_accel_p		:Number = 1;
	private var throttle_accel_i		:Number = 0; //1.5;
	private var throttle_accel_d		:Number = 0.2;
	private var throttle_accel_imax		:Number = 300;

	// Loiter
	//
	private var loiter_p				:Number = 1.0;
	private var loiter_i				:Number = 0.0;
	private var loiter_imax				:Number = 3000;
	private var loiter_rate_p			:Number = 1.0;
	private var loiter_rate_i			:Number = 0.5;
	private var loiter_rate_d			:Number = 0.0; // was .25
	private var loiter_rate_imax		:Number = 400;

	// Nav
	//
	private var nav_p					:Number = 2.4;
	private var nav_i					:Number = 0.17;
	private var nav_d					:Number = 0.00;
	private var nav_imax				:Number = 1800;

	public var time_constant_xy			:Number = 3.0;// default time constant for complementary filter's X & Y axis
	public var time_constant_z			:Number = 7;// default time constant for complementary filter's Z axis


	public var crosstrack_gain			:Number = .25;
	public var tilt_comp				:int 	= 44;
	

	public function Parameters():void
	{
		if(instance == null)
		  instance = this;
		this.visible = false;
		frame 		= new Rectangle(0, 0, 250, 450);

		// radio
		rc_1					= new RC_Channel(); // instantiated onscreen
		rc_2					= new RC_Channel();
		rc_3					= new RC_Channel();	// instantiated onscreen
		rc_4					= new RC_Channel();
		rc_5					= new RC_Channel();
		rc_6					= new RC_Channel();
		rc_7					= new RC_Channel();
		rc_8					= new RC_Channel();

		// Stabilie
		pi_stabilize_roll 		= new AC_PID(stabilize_p, 		stabilize_i, 		0, 					stabilize_imax);
		pi_stabilize_pitch 		= new AC_PID(stabilize_p, 		stabilize_i, 		0, 					stabilize_imax);
		pid_rate_roll 			= new AC_PID(rate_p, 			rate_i, 			rate_d, 			rate_imax);
		pid_rate_pitch 			= new AC_PID(rate_p, 			rate_i, 			rate_d, 			rate_imax);

		pi_stabilize_yaw 		= new AC_PID(stabilize_yaw_p, 	stabilize_yaw_i,	0, 					stabilize_yaw_imax);
		pid_rate_yaw 			= new AC_PID(rate_yaw_p, 		rate_yaw_i, 		rate_yaw_d, 		rate_yaw_imax);

		// Alt Hold
		pi_alt_hold 			= new AC_PID(alt_hold_p, 		alt_hold_i, 		0, 					alt_hold_imax);
		pid_throttle 			= new AC_PID(throttle_rate_p, 	throttle_rate_i, 	throttle_rate_d, 	throttle_rate_imax);
		pid_throttle_accel 		= new AC_PID(throttle_accel_p, 	throttle_accel_i, 	throttle_accel_d, 	throttle_accel_imax);

		// Loiter
		pi_loiter_lon 			= new AC_PID(loiter_p, 			loiter_i, 			0, 					loiter_imax);				// Raise P to decrease frequency
		pi_loiter_lat 			= new AC_PID(loiter_p, 			loiter_i, 			0, 					loiter_imax);				// Raise P to decrease frequency
		pid_loiter_rate_lon 	= new AC_PID(loiter_rate_p, 	loiter_rate_i, 		loiter_rate_d, 		loiter_rate_imax);
		pid_loiter_rate_lat 	= new AC_PID(loiter_rate_p, 	loiter_rate_i, 		loiter_rate_d, 		loiter_rate_imax);

		// nav
		pid_nav_lon 			= new AC_PID(nav_p, 			nav_i, 				nav_d, 				nav_imax);
		pid_nav_lat 			= new AC_PID(nav_p, 			nav_i, 				nav_d, 				nav_imax);

		addEventListener(Event.ADDED_TO_STAGE, addedToStage);
	}

	//Parameters.getInstance();
	static public function getInstance():Parameters
	{
		//if (instance == null)
		  //instance = new DragManager();
		return instance;
	}

	public function addedToStage(event:Event):void {
		originalX = this.x;
		wind_checkbox.setLabel("Enable Wind");
		fastPlot_checkbox.setLabel("Fast plot");
		baro_noise_checkbox.setLabel("Baro Noise");
		axis_enabled_checkbox.setLabel("Axis Lock");
		sonar_checkbox.setLabel("Sonar Enabled");

		NTUN_checkbox.setLabel("Log NTUN");
		CTUN_checkbox.setLabel("Log CTUN");
		GPS_checkbox.setLabel("Log GPS");
		ATT_checkbox.setLabel("Log ATT");
		test_checkbox.setLabel("A/B Test Option");
		simple_checkbox.setLabel("Simple Mode");
		super_simple_checkbox.setLabel("Super Simple Mode");
		acro_trainer_checkbox.setLabel("Acro Trainer Mode");
		throttle_accel_checkbox.setLabel("Throttle Accel");
		initGains();
	}

	// called at startup to fill in values for defaults
	private function initGains():void
	{
		// SIM
		sim_iterations_BI.setNumber(sim_iterations);
		sim_speed_BI.setNumber(sim_speed);
		drag_BI.setNumber(dragCE);
		airDensity_BI.setNumber(airDensity);
		crossSection_BI.setNumber(crossSection);
		speed_filter_BI.setNumber(speed_filter_size);

		test_checkbox.setSelected(test); // for testing alternatives
		start_height_BI.setNumber(300);
		start_position_BI.setNumber(0);
		target_distance_BI.setNumber(0);
		target_altitude_BI.setNumber(300);
		wind_low_BI.setNumber(windSpeedMin);
		wind_high_BI.setNumber(windSpeedMax);
		wind_period_BI.setNumber(windPeriod / 1000);
		windDir_BI.setNumber(windDir);
		start_speed_BI.setNumber(0);
		start_rotation_BI.setNumber(0);
		start_climb_rate_BI.setNumber(0);
		start_angle_BI.setNumber(0);

		motor_kv_BI.setNumber(motor_kv);
		moment_BI.setNumber(moment);
		mass_BI.setNumber(mass);
		esc_delay_BI.setNumber(esc_delay);

		// -------------------------------------
		// Gains
		// -------------------------------------

		// stabilize
		stab_roll_P_BI.setNumber(stabilize_p);
		stab_roll_I_BI.setNumber(stabilize_i);
		stab_roll_Imax_BI.setNumber(stabilize_imax);

		stab_rate_P_BI.setNumber(rate_p);
		stab_rate_I_BI.setNumber(rate_i);
		stab_rate_D_BI.setNumber(rate_d);
		stab_rate_Imax_BI.setNumber(rate_imax);

		// Yaw
		stabilize_yaw_p_BI.setNumber(stabilize_yaw_p);
		stabilize_yaw_i_BI.setNumber(stabilize_yaw_i);
		stabilize_yaw_imax_BI.setNumber(stabilize_yaw_imax);

		rate_yaw_p_BI.setNumber(rate_yaw_p);
		rate_yaw_i_BI.setNumber(rate_yaw_i);
		rate_yaw_d_BI.setNumber(rate_yaw_d);
		rate_yaw_imax_BI.setNumber(rate_yaw_imax);

		// alt hold
		alt_hold_P_BI.setNumber(alt_hold_p);
		alt_hold_I_BI.setNumber(alt_hold_i);
		alt_hold_Imax_BI.setNumber(alt_hold_imax);

		alt_rate_P_BI.setNumber(throttle_rate_p);
		alt_rate_I_BI.setNumber(throttle_rate_i);
		alt_rate_D_BI.setNumber(throttle_rate_d);
		alt_rate_Imax_BI.setNumber(throttle_rate_imax);

		alt_accel_P_BI.setNumber(throttle_accel_p);
		alt_accel_I_BI.setNumber(throttle_accel_i);
		alt_accel_D_BI.setNumber(throttle_accel_d);
		alt_accel_Imax_BI.setNumber(throttle_accel_imax);


		// loiter
		loiter_hold_P_BI.setNumber(loiter_p);
		loiter_hold_I_BI.setNumber(loiter_i);
		loiter_hold_Imax_BI.setNumber(loiter_imax);

		loiter_rate_P_BI.setNumber(loiter_rate_p);
		loiter_rate_I_BI.setNumber(loiter_rate_i);
		loiter_rate_D_BI.setNumber(loiter_rate_d);
		loiter_rate_Imax_BI.setNumber(loiter_rate_imax);


		// nav
		nav_P_BI.setNumber(nav_p);
		nav_I_BI.setNumber(nav_i);
		nav_D_BI.setNumber(nav_d);
		nav_Imax_BI.setNumber(nav_imax);

		// -------------------------------------
		// Parameters
		// -------------------------------------

		rtl_altitude_BI.setNumber(rtl_altitude);
		sonar_checkbox.setSelected(sonar_enabled);
		// battery monitoring
		//failsafe_battery_checkbox.setSelected(failsafe_battery_enabled);
		//failsafe_gps_checkbox.setSelected(failsafe_gps_enabled);

		//compass_checkbox.setSelected(compass_enabled);
		//optflow_checkbox.setSelected(optflow_enabled);
		super_simple_checkbox.setSelected(super_simple);
		rtl_alt_final_BI.setNumber(rtl_alt_final);
		axis_enabled_checkbox.setSelected(axis_enabled);
		//copter_leds_mode
		throttle_accel_checkbox.setSelected(throttle_accel_enabled);
		//yaw_override_BI.setNumber(yaw_override_behaviour);

		// Waypoints
		//
		// waypoint_radius
		circle_radius_BI.setNumber(circle_radius);
		waypoint_speed_max_BI.setNumber(_speed_cms);
		rtl_loiter_time_BI.setNumber(rtl_loiter_time);
		land_speed_BI.setNumber(land_speed);
		auto_velocity_z_min_BI.setNumber(auto_velocity_z_min);
		auto_velocity_z_max_BI.setNumber(auto_velocity_z_max);
		pilot_velocity_z_max_BI.setNumber(pilot_velocity_z_max);


		throttle_cruise_BI.setNumber(throttle_cruise);
		throttle_mid_BI.setNumber(throttle_mid);

		time_constant_xy_BI.setNumber(time_constant_xy);
		time_constant_z_BI.setNumber(time_constant_z);


		auto_slew_rate_BI.setNumber(auto_slew_rate);
		crosstrack_BI.setNumber(crosstrack_gain);

		//toy_yaw_rate_BI.setNumber(toy_yaw_rate);
		//toy_alt_large_BI.setNumber(toy_alt_large);
		//toy_alt_small_BI.setNumber(toy_alt_small);

		// Acro parameters
		acro_P_BI.setNumber(acro_p);
		acro_trainer_enabled
		acro_trainer_checkbox.setSelected(acro_trainer_enabled);
	}

	public function updateGains():void
	{
		// SIM
		sim_iterations				= sim_iterations_BI.getNumber();
		sim_speed					= sim_speed_BI.getNumber();

		dragCE						= drag_BI.getNumber();
		airDensity					= airDensity_BI.getNumber();
		crossSection				= crossSection_BI.getNumber();
		speed_filter_size			= speed_filter_BI.getNumber();
		//start_height				= start_height_BI.getNumber();

		test						= test_checkbox.getSelected();

		motor_kv					= motor_kv_BI.getNumber();
		moment						= moment_BI.getNumber();
		mass						= mass_BI.getNumber();
		esc_delay					= esc_delay_BI.getNumber();


		// stabilize
		pi_stabilize_roll._kp		= stab_roll_P_BI.getNumber();
		pi_stabilize_roll._ki		= stab_roll_I_BI.getNumber();
		pi_stabilize_roll._imax		= stab_roll_Imax_BI.getNumber();

		pi_stabilize_pitch._kp		= stab_roll_P_BI.getNumber();
		pi_stabilize_pitch._ki		= stab_roll_I_BI.getNumber();
		pi_stabilize_pitch._imax	= stab_roll_Imax_BI.getNumber();

		// Roll Pitch Rate
		pid_rate_roll._kp			= stab_rate_P_BI.getNumber();
		pid_rate_roll._ki			= stab_rate_I_BI.getNumber();
		pid_rate_roll._kd			= stab_rate_D_BI.getNumber();
		pid_rate_roll._imax			= stab_rate_Imax_BI.getNumber();

		pid_rate_pitch._kp			= stab_rate_P_BI.getNumber();
		pid_rate_pitch._ki			= stab_rate_I_BI.getNumber();
		pid_rate_pitch._kd			= stab_rate_D_BI.getNumber();
		pid_rate_pitch._imax		= stab_rate_Imax_BI.getNumber();

		// Yaw
		pi_stabilize_yaw._kp		= stabilize_yaw_p_BI.getNumber();
		pi_stabilize_yaw._ki		= stabilize_yaw_i_BI.getNumber();
		pi_stabilize_yaw._imax		= stabilize_yaw_imax_BI.getNumber();

		pid_rate_yaw._kp			= rate_yaw_p_BI.getNumber();
		pid_rate_yaw._ki			= rate_yaw_i_BI.getNumber();
		pid_rate_yaw._kd			= rate_yaw_d_BI.getNumber();
		pid_rate_yaw._imax			= rate_yaw_imax_BI.getNumber();

		// alt hold
		pi_alt_hold._kp 			= alt_hold_P_BI.getNumber();
		pi_alt_hold._ki 			= alt_hold_I_BI.getNumber();
		pi_alt_hold._imax 			= alt_hold_Imax_BI.getNumber();

		pid_throttle._kp 			= alt_rate_P_BI.getNumber();
		pid_throttle._ki 			= alt_rate_I_BI.getNumber();
		pid_throttle._imax 			= alt_rate_Imax_BI.getNumber();
		pid_throttle._kd 			= alt_rate_D_BI.getNumber();

		pid_throttle._kp 			= alt_accel_P_BI.getNumber();
		pid_throttle._ki 			= alt_accel_I_BI.getNumber();
		pid_throttle._imax 			= alt_accel_Imax_BI.getNumber();
		pid_throttle._kd 			= alt_accel_D_BI.getNumber();

		// loiter
		pi_loiter_lon._kp			= loiter_hold_P_BI.getNumber();
		pi_loiter_lon._ki			= loiter_hold_I_BI.getNumber();
		pi_loiter_lon._imax			= loiter_hold_Imax_BI.getNumber();

		pid_loiter_rate_lon._kp 	= loiter_rate_P_BI.getNumber();
		pid_loiter_rate_lon._ki 	= loiter_rate_I_BI.getNumber();
		pid_loiter_rate_lon._imax 	= loiter_rate_Imax_BI.getNumber();
		pid_loiter_rate_lon._kd 	= loiter_rate_D_BI.getNumber();

		pi_loiter_lat._kp			= loiter_hold_P_BI.getNumber();
		pi_loiter_lat._ki			= loiter_hold_I_BI.getNumber();
		pi_loiter_lat._imax			= loiter_hold_Imax_BI.getNumber();

		pid_loiter_rate_lat._kp 	= loiter_rate_P_BI.getNumber();
		pid_loiter_rate_lat._ki 	= loiter_rate_I_BI.getNumber();
		pid_loiter_rate_lat._imax 	= loiter_rate_Imax_BI.getNumber();
		pid_loiter_rate_lat._kd 	= loiter_rate_D_BI.getNumber();

		// nav
		_speed_cms					= waypoint_speed_max_BI.getNumber();
		pid_nav_lon._kp 			= nav_P_BI.getNumber();
		pid_nav_lon._ki 			= nav_I_BI.getNumber();
		pid_nav_lon._imax 			= nav_Imax_BI.getNumber();
		pid_nav_lon._kd 			= nav_D_BI.getNumber();

		pid_nav_lat._kp 			= nav_P_BI.getNumber();
		pid_nav_lat._ki 			= nav_I_BI.getNumber();
		pid_nav_lat._imax 			= nav_Imax_BI.getNumber();
		pid_nav_lat._kd 			= nav_D_BI.getNumber();

		super_simple				= super_simple_checkbox.getSelected();

		// acro
		acro_p						= acro_P_BI.getNumber();
		axis_enabled				= axis_enabled_checkbox.getSelected();
		sonar_enabled				= sonar_checkbox.getSelected();
		//rtl_land_enabled			= rtl_land_checkbox.getSelected();

		rtl_altitude				= rtl_altitude_BI.getNumber();
		rtl_alt_final				= rtl_alt_final_BI.getNumber();
		rtl_loiter_time				= rtl_loiter_time_BI.getNumber();
		//toy_alt_large				= toy_alt_large_BI.getNumber();
		//toy_alt_small				= toy_alt_small_BI.getNumber();
		//toy_yaw_rate				= toy_yaw_rate_BI.getNumber();
		circle_radius				= circle_radius_BI.getNumber();

		time_constant_xy			= time_constant_xy_BI.getNumber();
		time_constant_z				= time_constant_z_BI.getNumber();


		throttle_mid				= throttle_mid_BI.getNumber();
		throttle_cruise				= throttle_cruise_BI.getNumber();
		throttle_accel_enabled		= throttle_accel_checkbox.getSelected();

		auto_slew_rate				= auto_slew_rate_BI.getNumber();
		crosstrack_gain				= crosstrack_BI.getNumber();
		
		// data fixes
		if(moment == 0) moment = 1;
		if(mass == 0) mass = 1;
	}

}
}
