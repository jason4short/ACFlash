/*
 *       AP_MotorsMatrix.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 */

package com.libraries
{
	import com.libraries.AP_RC;

	public class AP_MotorsMatrix
	{
		private var APM_RC:AP_RC;

		public const AP_MOTORS_MOT_1:int = 0;
		public const AP_MOTORS_MOT_2:int = 1;
		public const AP_MOTORS_MOT_3:int = 2;
		public const AP_MOTORS_MOT_4:int = 3;
		public const AP_MOTORS_MOT_5:int = 4;
		public const AP_MOTORS_MOT_6:int = 5;
		public const AP_MOTORS_MOT_7:int = 6;
		public const AP_MOTORS_MOT_8:int = 7;


		private const AP_MOTORS_MAX_NUM_MOTORS:int = 8;

		private const AP_MOTORS_DEFAULT_MIN_THROTTLE:int = 130;
		private const AP_MOTORS_DEFAULT_MAX_THROTTLE:int = 850;

		// frame definitions
		public const AP_MOTORS_PLUS_FRAME:int = 0;
		public const AP_MOTORS_X_FRAME:int = 1;
		public const AP_MOTORS_V_FRAME:int = 2;

		// top-bottom ratio (for Y6)
		private const AP_MOTORS_TOP_BOTTOM_RATIO:Number = 1;


		public const THROTTLE_CURVE_ENABLED		:Number = 1;	// throttle curve disabled by default
		public const THROTTLE_CURVE_MID_THRUST	:Number = 52;	// throttle which produces 1/2 the maximum thrust.  expressed as a percentage of the full throttle range (i.e 0 ~ 100)
		public const THROTTLE_CURVE_MAX_THRUST	:Number = 93; 	// throttle which produces the maximum thrust.  expressed as a percentage of the full throttle range (i.e 0 ~ 100)

		public const AP_MOTOR_NO_LIMITS_REACHED	:int = 0;		// throttle curve disabled by default
		public const AP_MOTOR_ROLLPITCH_LIMIT	:int = 0x01;	// throttle curve disabled by default
		public const AP_MOTOR_YAW_LIMIT			:int = 0x02;	// throttle curve disabled by default
		public const AP_MOTOR_THROTTLE_LIMIT	:int = 0x04;	// throttle curve disabled by default
		public const AP_MOTOR_ANY_LIMIT			:int = 0xFF;	// throttle curve disabled by default


		private var  _rc_roll, _rc_pitch, _rc_throttle, _rc_yaw:RC_Channel;
		private var _motor_to_channel_map		:Array;     // mapping of motor number (as received from upper APM code) to RC channel output - used to account for differences between APM1 and APM2
		//uint16_t            _speed_hz;                    // speed in hz to send updates to motors
		private var _armed						:Boolean;   // true if motors are armed
		private var _auto_armed					:Boolean;   // true is throttle is above zero, allows auto pilot to take control of throttle
		protected var _frame_orientation			:int;     	// PLUS_FRAME 0, X_FRAME 1, V_FRAME 2
		private var _min_throttle				:int;       // the minimum throttle to be sent to the engines when they're on (prevents issues with some motors on while other off at very low throttle)
		private var _max_throttle				:int;       // the minimum throttle to be sent to the engines when they're on (prevents issues with some motors on while other off at very low throttle)

		private var  _throttle_curve			:AP_Curve;  // curve used to linearize the pwm.thrust //AP_CurveInt16_Size4
		private var _throttle_curve_enabled		:Boolean = false;       // enable throttle curve
		private var _throttle_curve_mid			:int;  		// throttle which produces 1/2 the maximum thrust.  expressed as a percentage (i.e. 0 ~ 100 ) of the full throttle range
		private var _throttle_curve_max			:int;	  	// throttle which produces the maximum thrust.  expressed as a percentage (i.e. 0 ~ 100 ) of the full throttle range
		private var _reached_limit				:int;		// bit mask to record which motor limits we hit (if any) during most recent output.  Used to provide feedback to attitude controllers

		private var _num_motors					:int; 		// not a very useful variable as you really need to check the motor_enabled array to see which motors are enabled
    	private var _roll_factor				:Array; 	// each motors contribution to roll
    	private var _pitch_factor				:Array; 	// each motors contribution to pitch
    	private var _yaw_factor					:Array; 	// each motors contribution to yaw (normally 1 or -1)
    	private var test_order					:Array; 	// order of the motors in the test sequence


		// vars
		public const AP_MOTORS_MATRIX_MOTOR_UNDEFINED:int = -1;
		public const AP_MOTORS_MATRIX_ORDER_UNDEFINED:int = -1;

		public const AP_MOTORS_MATRIX_MOTOR_CW:int = -1;
		public const AP_MOTORS_MATRIX_MOTOR_CCW:int = 1;

		public const AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM:int = 100;

		private var motor_enabled	:Array;
		private var motor_out		:Array;


		/// Constructor
		public function AP_MotorsMatrix(rc_roll:RC_Channel, rc_pitch:RC_Channel, rc_throttle:RC_Channel, rc_yaw:RC_Channel) {
			APM_RC			= AP_RC.getInstance();
			_throttle_curve = new AP_Curve(4);
			_rc_roll 		= rc_roll;
			_rc_pitch 		= rc_pitch;
			_rc_throttle 	= rc_throttle;
			_rc_yaw 		= rc_yaw;

			motor_enabled 	= new Array(AP_MOTORS_MAX_NUM_MOTORS);
			motor_out 		= new Array(AP_MOTORS_MAX_NUM_MOTORS);

			_roll_factor 	= new Array(AP_MOTORS_MAX_NUM_MOTORS);
			_pitch_factor 	= new Array(AP_MOTORS_MAX_NUM_MOTORS);
			_yaw_factor 	= new Array(AP_MOTORS_MAX_NUM_MOTORS);
			test_order 		= new Array(AP_MOTORS_MAX_NUM_MOTORS);

			_motor_to_channel_map = new Array(0,1,2,3,4,5,6,7);
		}

		// Init
		public function Init():void
		{
		    // set-up throttle curve - motors classes will decide whether to use it based on _throttle_curve_enabled parameter
			setup_throttle_curve();

			// call parent Init function to set-up throttle curve
			//AP_Motors::Init();

			// setup the motors
			setup_motors();

			// enable fast channels or instant pwm
			//set_update_rate(_speed_hz);
		}

		public function armed():Boolean
		{
			return _armed;
		}

		public function arm(arm:Boolean):void
		{
			_armed = arm;
		}

		public function auto_armed():Boolean
		{
			return _auto_armed;
		}

		public function auto_arm(arm:Boolean):void
		{
			_auto_armed = arm;
		}

		public function set_min_throttle(min_throttle:int)
		{
			_min_throttle = min_throttle;
		}

		public function set_max_throttle(max_throttle:int)
		{
			_max_throttle = max_throttle;
		}

		public function output() {
			if( _armed && _auto_armed ) {
					output_armed();
				}else{
					output_disarmed();
			}
		}

		// reached_limits - return whether we hit the limits of the motors
		public function reached_limit(which_limit:int = AP_MOTOR_ANY_LIMIT):int
		{
			return _reached_limit & which_limit;
		}

		// get basic information about the platform
		public function get_num_motors():int
		{
			return _num_motors;
		};


		// setup_throttle_curve - used to linearlise thrust output by motors
		// returns true if set up successfully
		private function setup_throttle_curve():Boolean
		{
			var min_pwm				:int = _rc_throttle.radio_min;
			var max_pwm				:int = _rc_throttle.radio_max;
			var mid_throttle_pwm	:int = (max_pwm + min_pwm) / 2;
			var mid_thrust_pwm		:int = min_pwm + (max_pwm - min_pwm) * (Number(_throttle_curve_mid)/100.0);
			var max_thrust_pwm		:int = min_pwm + (max_pwm - min_pwm) * (Number(_throttle_curve_max)/100.0);
			var retval_int			:int = 0;
			var retval				:Boolean = true;

			// some basic checks that the curve is valid
			if(mid_thrust_pwm >= (min_pwm+_min_throttle) && mid_thrust_pwm <= max_pwm && max_thrust_pwm >= mid_thrust_pwm && max_thrust_pwm <= max_pwm){
				// clear curve
				_throttle_curve.clear();

				// curve initialisation
				if(_throttle_curve.add_point(min_pwm, min_pwm) == false)
					retval_int++;

				if(_throttle_curve.add_point(min_pwm+_min_throttle, min_pwm+_min_throttle) == false)
					retval_int++;

				if(_throttle_curve.add_point(mid_throttle_pwm, mid_thrust_pwm) == false)
					retval_int++;

				if(_throttle_curve.add_point(max_pwm, max_thrust_pwm) == false)
					retval_int++;

				// return success
				return (retval_int == 0);
			}else{
				retval = false;
			}

			// disable throttle curve if not set-up corrrectly
			if( !retval ) {
				_throttle_curve_enabled = false;
				trace("AP_Motors: failed to create throttle curve");
			}

			return retval;
		}

		// set frame orientation (normally + or X)
		public function set_frame_orientation(new_orientation:int):void
		{
			// return if nothing has changed
			if(new_orientation == _frame_orientation){
				return;
			}

			// call parent
			//super.set_frame_orientation(new_orientation);

			// setup the motors
			setup_motors();

			// enable fast channels or instant pwm
			//set_update_rate(_speed_hz);
		}

		public function setup_motors():void
		{
		}

		// enable - starts allowing signals to be sent to motors
		public function enable():void
		{
			var i:int;

			// enable output channels
			for(i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++){
				if(motor_enabled[i]){
					//hal.rcout.enable_ch(_motor_to_channel_map[i]);
					APM_RC.enable_ch(_motor_to_channel_map[i]);
				}
			}
		}

		// output_min - sends minimum values out to the motors
		public function output_min():void
		{
			var i:int;

			// fill the motor_out[] array for HIL use and send minimum value to each motor
			for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
				if( motor_enabled[i] ) {
					motor_out[i] = _rc_throttle.radio_min;
					//hal.rcout.write(_motor_to_channel_map[i], motor_out[i]);
					APM_RC.OutputCh(_motor_to_channel_map[i], motor_out[i]);
				}
			}
		}

		// output_armed - sends commands to the motors
		public function output_armed():void
		{
			var i:int;

			var out_min						:int = _rc_throttle.radio_min;
			var out_max						:int = _rc_throttle.radio_max;
			var rc_yaw_constrained_pwm 		:int;
			var rc_yaw_excess				:int;
			var upper_margin				:int;
			var lower_margin				:int;
			var motor_adjustment			:int = 0;
			var yaw_to_execute				:int = 0;

			// initialize reached_limit flag
			_reached_limit = AP_MOTOR_NO_LIMITS_REACHED;

			// Throttle is 0 to 1000 only
			_rc_throttle.servo_out = constrain(_rc_throttle.servo_out, 0, _max_throttle);

			// capture desired roll, pitch, yaw and throttle from receiver
			_rc_roll.calc_pwm();
			_rc_pitch.calc_pwm();
			_rc_throttle.calc_pwm();
			_rc_yaw.calc_pwm();

			// if we are not sending a throttle output, we cut the motors
			if(_rc_throttle.servo_out == 0) {
				for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
					if( motor_enabled[i] ) {
						motor_out[i]    = _rc_throttle.radio_min;
					}
				}
				// if we have any roll, pitch or yaw input then it's breaching the limit
				if( _rc_roll.pwm_out != 0 || _rc_pitch.pwm_out != 0 ) {
					_reached_limit |= AP_MOTOR_ROLLPITCH_LIMIT;
				}
				if( _rc_yaw.pwm_out != 0 ) {
					_reached_limit |= AP_MOTOR_YAW_LIMIT;
				}
			} else {    // non-zero throttle

				out_min = _rc_throttle.radio_min + _min_throttle;

				// initialise rc_yaw_contrained_pwm that we will certainly output and rc_yaw_excess that we will do on best-efforts basis.
				// Note: these calculations and many others below depend upon _yaw_factors always being 0, -1 or 1.
				if( _rc_yaw.pwm_out < -AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM ) {
					rc_yaw_constrained_pwm = -AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM;
					rc_yaw_excess = _rc_yaw.pwm_out+AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM;
				}else if( _rc_yaw.pwm_out > AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM ) {
					rc_yaw_constrained_pwm = AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM;
					rc_yaw_excess = _rc_yaw.pwm_out-AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM;
				}else{
					rc_yaw_constrained_pwm = _rc_yaw.pwm_out;
					rc_yaw_excess = 0;
				}

				// initialise upper and lower margins
				upper_margin = lower_margin = out_max - out_min;

				// add roll, pitch, throttle and constrained yaw for each motor
				for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
					if( motor_enabled[i] ) {
						motor_out[i] = _rc_throttle.radio_out +
									   _rc_roll.pwm_out * _roll_factor[i] +
									   _rc_pitch.pwm_out * _pitch_factor[i] +
									   rc_yaw_constrained_pwm * _yaw_factor[i];

						// calculate remaining room between fastest running motor and top of pwm range
						if( out_max - motor_out[i] < upper_margin) {
							upper_margin = out_max - motor_out[i];
						}
						// calculate remaining room between slowest running motor and bottom of pwm range
						if( motor_out[i] - out_min < lower_margin ) {
							lower_margin = motor_out[i] - out_min;
						}
					}
				}

				// if motors are running too fast and we have enough room below, lower overall throttle
				if( upper_margin < 0 || lower_margin < 0 ) {

					// calculate throttle adjustment that equalizes upper and lower margins.  We will never push the throttle beyond this point
					motor_adjustment = (upper_margin - lower_margin) / 2;      // i.e. if overflowed by 20 on top, 30 on bottom, upper_margin = -20, lower_margin = -30.  will adjust motors -5.

					// if we have overflowed on the top, reduce but no more than to the mid point
					if( upper_margin < 0 ) {
						motor_adjustment = Math.max(upper_margin, motor_adjustment);
					}

					// if we have underflowed on the bottom, increase throttle but no more than to the mid point
					if( lower_margin < 0 ) {
						motor_adjustment = Math.min(-lower_margin, motor_adjustment);
					}
				}

				// move throttle up or down to to pull within tolerance
				if( motor_adjustment != 0 ) {
					for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
						if( motor_enabled[i] ) {
							motor_out[i] += motor_adjustment;
						}
					}

					// we haven't even been able to apply roll, pitch and minimal yaw without adjusting throttle so mark all limits as breached
					_reached_limit |= AP_MOTOR_ROLLPITCH_LIMIT | AP_MOTOR_YAW_LIMIT | AP_MOTOR_THROTTLE_LIMIT;
				}

				// if we didn't give all the yaw requested, calculate how much additional yaw we can add
				if( rc_yaw_excess != 0 ) {

					// try for everything
					yaw_to_execute = rc_yaw_excess;

					// loop through motors and reduce as necessary
					for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
						if( motor_enabled[i] && _yaw_factor[i] != 0 ) {

							// calculate upper and lower margins for this motor
							upper_margin = Math.max(0,out_max - motor_out[i]);
							lower_margin = Math.max(0,motor_out[i] - out_min);

							// motor is increasing, check upper limit
							if( rc_yaw_excess > 0 && _yaw_factor[i] > 0 ) {
								yaw_to_execute = Math.min(yaw_to_execute, upper_margin);
							}

							// motor is decreasing, check lower limit
							if( rc_yaw_excess > 0 && _yaw_factor[i] < 0 ) {
								yaw_to_execute = Math.min(yaw_to_execute, lower_margin);
							}

							// motor is decreasing, check lower limit
							if( rc_yaw_excess < 0 && _yaw_factor[i] > 0 ) {
								yaw_to_execute = Math.max(yaw_to_execute, -lower_margin);
							}

							// motor is increasing, check upper limit
							if( rc_yaw_excess < 0 && _yaw_factor[i] < 0 ) {
								yaw_to_execute = Math.max(yaw_to_execute, -upper_margin);
							}
						}
					}
					// check yaw_to_execute is reasonable
					if( yaw_to_execute != 0 && ((yaw_to_execute>0 && rc_yaw_excess>0) || (yaw_to_execute<0 && rc_yaw_excess<0)) ) {
						// add the additional yaw
						for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
							if( motor_enabled[i] ) {
								motor_out[i] += _yaw_factor[i] * yaw_to_execute;
							}
						}
					}
					// mark yaw limit reached if we didn't get everything we asked for
					if( yaw_to_execute != rc_yaw_excess ) {
						_reached_limit |= AP_MOTOR_YAW_LIMIT;
					}
				}

				// adjust for throttle curve
				if( _throttle_curve_enabled ) {
					for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
						if( motor_enabled[i] ) {
							motor_out[i] = _throttle_curve.get_y(motor_out[i]);
						}
					}
				}

				// clip motor output if required (shouldn't be)
				for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
					if( motor_enabled[i] ) {
						motor_out[i] = constrain(motor_out[i], out_min, out_max);
					}
				}
			}

			// send output to each motor
			for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
				if( motor_enabled[i] ) {
					//hal.rcout.write(_motor_to_channel_map[i], motor_out[i]);
					APM_RC.OutputCh(_motor_to_channel_map[i], motor_out[i]);
				}
			}
		}

		// output_disarmed - sends commands to the motors
		public function output_disarmed():void
		{
			if(_rc_throttle.control_in > 0) {
				// we have pushed up the throttle
				// remove safety for auto pilot
				_auto_armed = true;
			}

			// Send minimum values to all motors
			output_min();
		}

		// output_disarmed - sends commands to the motors
		public function output_test():void
		{
			/*
			var min_order, max_order:int;
			var i,j:int;

			// find min and max orders
			min_order = test_order[0];
			max_order = test_order[0];
			for(i=1; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
				if( test_order[i] < min_order )
					min_order = test_order[i];
				if( test_order[i] > max_order )
					max_order = test_order[i];
			}

			// shut down all motors
			output_min();

			// first delay is longer
			//hal.scheduler.delay(4000);

			// loop through all the possible orders spinning any motors that match that description
			for( i=min_order; i<=max_order; i++ ) {
				for( j=0; j<AP_MOTORS_MAX_NUM_MOTORS; j++ ) {
					if( motor_enabled[j] && test_order[j] == i ) {
						// turn on this motor and wait 1/3rd of a second
						hal.rcout.write(_motor_to_channel_map[j], _rc_throttle.radio_min + 100);
						hal.scheduler.delay(300);
						hal.rcout.write(_motor_to_channel_map[j], _rc_throttle.radio_min);
						hal.scheduler.delay(2000);
					}
				}
			}

			// shut down all motors
			output_min();*/
		}

		// add_motor
		public function add_motor_raw(motor_num:int, roll_fac:Number, pitch_fac:Number, yaw_fac:Number, testing_order:int):void
		{
			// ensure valid motor number is provided
			if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {

				// increment number of motors if this motor is being newly motor_enabled
				if( !motor_enabled[motor_num] ) {
					motor_enabled[motor_num] = true;
					_num_motors++;
				}

				// set roll, pitch, thottle factors and opposite motor (for stability patch)
				_roll_factor[motor_num] 	= roll_fac;
				_pitch_factor[motor_num] 	= pitch_fac;
				_yaw_factor[motor_num] 		= yaw_fac;

				//trace("add_motor", motor_num, roll_fac.toFixed(6), pitch_fac.toFixed(6), yaw_fac.toFixed(6));

				// set order that motor appears in test
				//if( testing_order == AP_MOTORS_MATRIX_ORDER_UNDEFINED ) {
				//	test_order[motor_num] = motor_num;
				//}else{
				//	test_order[motor_num] = testing_order;
				//}
			}
		}

		// add_motor using just position and prop direction
		public function add_motor(motor_num:int, angle_degrees:Number, direction:int, testing_order:int):void
		{
			// call raw motor set-up method
			add_motor_raw(
				motor_num,
				Math.cos(radians(angle_degrees + 90)),               // roll factor
				Math.cos(radians(angle_degrees)),                    // pitch factor
				Number(direction),                                       // yaw factor
				testing_order);

		}

		// remove_motor - disabled motor and clears all roll, pitch, throttle factors for this motor
		public function remove_motor(motor_num:int):void
		{
			// ensure valid motor number is provided
			if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {

				// if the motor was enabled decrement the number of motors
				if( motor_enabled[motor_num] )
					_num_motors--;

				// disable the motor, set all factors to zero
				motor_enabled[motor_num] = false;
				_roll_factor[motor_num] = 0;
				_pitch_factor[motor_num] = 0;
				_yaw_factor[motor_num] = 0;
			}
		}

		// remove_all_motors - removes all motor definitions
		public function remove_all_motors():void
		{
			for( var i:int = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
				remove_motor(i);
			}
			_num_motors = 0;
		}

		public function constrain(val:Number, min:Number, max:Number):Number
		{
			val = Math.max(val, min);
			val = Math.min(val, max);
			return val;
		}
		public function radians(n:Number):Number
		{
			return 0.0174532925 * n;
		}

	}
}