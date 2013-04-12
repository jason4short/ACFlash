package com.libraries {
	import flash.display.MovieClip;
	import flash.geom.Vector3D;
	import flash.geom.Matrix3D;


	import com.libraries.AP_AHRS;
	import com.libraries.AP_Baro;
	//import AP_InertialSensor;
	import com.libraries.AP_Buffer;
	import com.libraries.GPS;
	import com.Sim.Clock;
	import com.Parameters;

	public class AP_InertialNav
	{
		public var g								:Parameters;
		private const AP_INTERTIALNAV_GRAVITY:Number		= 9.80665;
		private const AP_INTERTIALNAV_TC_XY:Number 			= 3.0;
		private const AP_INTERTIALNAV_TC_Z:Number 			= 7.0;
		private const AP_INTERTIALNAV_ACCEL_CORR_MAX:int 	= 100; // max allowed accelerometer offset correction

		// #defines to control how often historical accel based positions are saved
		// so they can later be compared to laggy gps readings
		private const AP_INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS:int 	= 10;
		private const AP_INTERTIALNAV_GPS_LAG_IN_10HZ_INCREMENTS:int 	= 4; 	// must not be larger than size of _hist_position_estimate_x and _hist_position_estimate_y
		private const AP_INTERTIALNAV_GPS_TIMEOUT_MS:int 				= 300; 	// timeout after which position error from GPS will fall to zero
		private const AP_INERTIALNAV_LATLON_TO_CM:Number 				= 1.1113175;

		public var _ahrs					:AP_AHRS;                   // pointer to ahrs object
		//public var _ins						:AP_InertialSensor;         // pointer to inertial sensor
		public var _baro					:AP_Baro;                   // pointer to barometer
		public var _gps_ptr                	:GPS;                  		// pointer to pointer to gps

		// XY Axis specific variables
		public var _xy_enabled				:Boolean = false;           // xy position estimates enabled
		public var _time_constant_xy		:Number;          			// time constant for horizontal corrections
		public var _k1_xy					:Number;                    // gain for horizontal position correction
		public var _k2_xy					:Number;                    // gain for horizontal velocity correction
		public var _k3_xy					:Number;                    // gain for horizontal accelerometer offset correction
		public var _gps_last_update			:int = 0;           		// system time of last gps update
		public var _gps_last_time			:int = 0;             		// time of last gps update according to the gps itself
		public var _historic_xy_counter		:int = 0;       			// counter used to slow saving of position estimates for later comparison to gps
		public var _hist_position_estimate_x	:AP_Buffer;  			// buffer of historic accel based position to account for lag
		public var _hist_position_estimate_y	:AP_Buffer;  			// buffer of historic accel based position to account for lag
		public var _base_lat               	:int = 0;                  	// base latitude
		public var _base_lon               	:int = 0;               	// base longitude
		public var _lon_to_m_scaling       	:Number;          			// conversion of longitude to meters

		// Z Axis specific variables
		public var _time_constant_z			:Number;           			// time constant for vertical corrections
		public var _k1_z					:Number;                    // gain for vertical position correction
		public var _k2_z					:Number;               		// gain for vertical velocity correction
		public var _k3_z					:Number;               		// gain for vertical accelerometer offset correction
		public var _baro_last_update		:int = 0;           		// time of last barometer update
		public var _hist_position_estimate_z	:AP_Buffer;  			// buffer of historic accel based altitudes to account for lag

		// general variables
		public var _position_base			:Vector3D;             		// position estimate
		public var _position_correction		:Vector3D; 			      	// sum of correction to _comp_h from delayed 1st order samples
		public var _velocity				:Vector3D;                  // latest velocity estimate (integrated from accelerometer values)
		public var _position_error			:Vector3D;

		public var accel_correction_bf		:Vector3D;
		public var accel_correction_ef		:Vector3D;
		public var clock					:Clock;

		// for reporting
		public var hist_position_base_z:Number;
		public var _baro_alt:int;
		

		public function AP_InertialNav(ahrs:AP_AHRS, /*ins:AP_InertialSensor,*/ baro:AP_Baro, gps_ptr:GPS):void
		{
			clock		= Clock.getInstance();
			_ahrs 		= ahrs;
			//_ins 		= ins
			_baro 		= baro;
			_gps_ptr	= gps_ptr;
			_hist_position_estimate_x = new AP_Buffer(15);
			_hist_position_estimate_y = new AP_Buffer(15);
			_hist_position_estimate_z = new AP_Buffer(15);

			_position_base			= new Vector3D();
			_position_correction	= new Vector3D();
			_velocity				= new Vector3D();
			_position_error			= new Vector3D();
			accel_correction_ef		= new Vector3D();
			accel_correction_bf		= new Vector3D();
			g = Parameters.getInstance();
		}

		// init - initialise library
		public function init():void
		{
			// recalculate the gains
			update_gains();
		}

		public function get_position():Vector3D
		{
			return _position_base.add(_position_correction).clone(); 
		}

		public function get_velocity():Vector3D
		{
			//_velocity2.z = 23
			//_velocity2.x = 22
			return _velocity.clone();
		}
		
		// update - updates velocities and positions using latest info from ahrs, ins and barometer if new data is available;
		public function update(dt:Number)
		{
			var _temp				:Vector3D;
			var accel_ef			:Vector3D;
			var velocity_increase	:Vector3D;

			// discard samples where dt is too large
			if( dt > 0.1 ) {
				return;
			}

			// check barometer
			check_baro();

			// check gps
			check_gps();

			accel_ef = _ahrs.get_accel_ef();

			// remove influence of gravity
			accel_ef.z += AP_INTERTIALNAV_GRAVITY;
			// switch to cm/s/s
			accel_ef.scaleBy(100);

			// remove xy if not enabled
			if( !_xy_enabled ) {
				accel_ef.x = 0;
				accel_ef.y = 0;
			}

			//Convert North-East-Down to North-East-Up
			accel_ef.z = -accel_ef.z;

			accel_correction_ef.x 	+= _position_error.x * _k3_xy * dt;
			accel_correction_ef.y 	+= _position_error.y * _k3_xy * dt;
			accel_correction_ef.z 	+= _position_error.z * _k3_z  * dt;

			_velocity.x 			+= _position_error.x * _k2_xy * dt;
			_velocity.y 			+= _position_error.y * _k2_xy * dt;
			_velocity.z 			+= _position_error.z * _k2_z  * dt;

			_position_correction.x 	+= _position_error.x * _k1_xy * dt;
			_position_correction.y 	+= _position_error.y * _k1_xy * dt;
			_position_correction.z 	+= _position_error.z * _k1_z  * dt;

			// calculate velocity increase adding new acceleration from accelerometers
			//velocity_increase = (accel_ef + accel_correction_ef) * dt;
			velocity_increase = accel_ef.clone();
			velocity_increase = velocity_increase.add(accel_correction_ef);
			velocity_increase.scaleBy(dt);


			// calculate new estimate of position
			//_position_base += (_velocity + velocity_increase * 0.5) * dt;
			_temp = velocity_increase.clone();
			_temp.scaleBy(0.5);
			_temp = _temp.add(_velocity);
			_temp.scaleBy(dt);
			_position_base = _position_base.add(_temp);

			// calculate new velocity
			//_velocity += velocity_increase;
			_velocity = _velocity.add(velocity_increase);

			// store 3rd order estimate (i.e. estimated vertical position) for future use
			_hist_position_estimate_z.add(_position_base.z);

			// store 3rd order estimate (i.e. horizontal position) for future use at 10hz
			_historic_xy_counter++;
			if( _historic_xy_counter >= AP_INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS ) {
				_historic_xy_counter = 0;
				_hist_position_estimate_x.add(_position_base.x);
				_hist_position_estimate_y.add(_position_base.y);
			}

			/*
			trace(_baro_alt, accel_ef.z.toFixed(2), _position_base.z.toFixed(2), 
				  _position_error.x.toFixed(2), _position_correction.z.toFixed(2), 
				  velocity_increase.z.toFixed(2), accel_correction_ef.z.toFixed(2), 
				  hist_position_base_z.toFixed(2));
		*/
					}

		//
		// XY Axis specific methods
		//

		// set time constant - set timeconstant used by complementary filter
		public function set_time_constant_xy( time_constant_in_seconds:Number )
		{
			// ensure it's a reasonable value
			if( time_constant_in_seconds > 0 && time_constant_in_seconds < 30 ) {
				_time_constant_xy = time_constant_in_seconds;
				update_gains();
			}
		}

		// position_ok - return true if position has been initialised and have received gps data within 3 seconds
		public function position_ok():Boolean
		{
			return _xy_enabled;
		}

		// check_gps - check if new gps readings have arrived and use them to correct position estimates
		public function check_gps()
		{
			var gps_time:Number;
			var now:Number = clock.millis();

			//if( _gps_ptr == NULL || *_gps_ptr == NULL )
			//	return;

			// get time according to the gps
			gps_time = _gps_ptr.time;

			// compare gps time to previous reading
			if( gps_time != _gps_last_time ) {

				// calculate time since last gps reading
				var dt:Number = (now - _gps_last_update) / 1000.0;

				// call position correction method
				correct_with_gps(_gps_ptr.longitude, _gps_ptr.latitude, dt);

				// record gps time and system time of this update
				_gps_last_time = gps_time;
				_gps_last_update = now;
			}

			// clear position error if GPS updates stop arriving
			if( now - _gps_last_update > AP_INTERTIALNAV_GPS_TIMEOUT_MS ) {
				_position_error.x = 0;
				_position_error.y = 0;
			}
		}

		// correct_with_gps - modifies accelerometer offsets using gps.  dt is time since last gps update
		public function correct_with_gps(lon:int,  lat:int, dt:Number):void
		{
			var x:Number;
			var y:Number;
			var hist_position_base_x, hist_position_base_y:Number;

			// discard samples where dt is too large
			if( dt > 1.0 || dt == 0 || !_xy_enabled) {
				return;
			}

			// calculate distance from base location
			x = Number(lat - _base_lat) * AP_INERTIALNAV_LATLON_TO_CM;
			y = Number(lon - _base_lon) * _lon_to_m_scaling * AP_INERTIALNAV_LATLON_TO_CM;

			// ublox gps positions are delayed by 400ms
			// we store historical position at 10hz so 4 iterations ago
			if( _hist_position_estimate_x.num_items() >= AP_INTERTIALNAV_GPS_LAG_IN_10HZ_INCREMENTS ) {
				hist_position_base_x = _hist_position_estimate_x.peek(AP_INTERTIALNAV_GPS_LAG_IN_10HZ_INCREMENTS-1);
				hist_position_base_y = _hist_position_estimate_y.peek(AP_INTERTIALNAV_GPS_LAG_IN_10HZ_INCREMENTS-1);
			}else{
				hist_position_base_x = _position_base.x;
				hist_position_base_y = _position_base.y;
			}

			// calculate error in position from gps with our historical estimate
			_position_error.x = x - (hist_position_base_x + _position_correction.x);
			_position_error.y = y - (hist_position_base_y + _position_correction.y);
		}

		// get accel based latitude
		public function get_latitude():int
		{
			// make sure we've been initialised
			if( !_xy_enabled ) {
				return 0;
			}

			return _base_lat + ((_position_base.x + _position_correction.x) / AP_INERTIALNAV_LATLON_TO_CM);
		}

		// get accel based longitude
		public function get_longitude():int
		{
			// make sure we've been initialised
			if( !_xy_enabled ) {
				return 0;
			}

			return _base_lon + ((_position_base.y + _position_correction.y) / (_lon_to_m_scaling * AP_INERTIALNAV_LATLON_TO_CM));
		}

		// set_current_position - all internal calculations are recorded as the distances from this point
		public function set_current_position(lon:int, lat:int):void
		{
			// set base location
			_base_lon = lon;
			_base_lat = lat;

			// set longitude->meters scaling
			// this is used to offset the shrinking longitude as we go towards the poles
			_lon_to_m_scaling = Math.cos((Math.abs(Number(lat)) / 10000000.0) * 0.017453292);

			// reset corrections to base position to zero
			_position_base.x = 0;
			_position_base.y = 0;
			_position_correction.x = 0;
			_position_correction.y = 0;

			// clear historic estimates
			_hist_position_estimate_x.clear();
			_hist_position_estimate_y.clear();

			// set xy as enabled
			_xy_enabled = true;
		}

		// get accel based latitude
		public function get_latitude_diff():Number
		{
			// make sure we've been initialised
			if(!_xy_enabled){
				return 0;
			}

			return ((_position_base.x + _position_correction.x) / AP_INERTIALNAV_LATLON_TO_CM);
		}

		// get accel based longitude
		public function get_longitude_diff():Number
		{
			// make sure we've been initialised
			if(!_xy_enabled){
				return 0;
			}

			return (_position_base.y+_position_correction.y) / (_lon_to_m_scaling * AP_INERTIALNAV_LATLON_TO_CM);
		}

		// get velocity in latitude & longitude directions
		public function get_latitude_velocity():Number
		{
			// make sure we've been initialised
			if(!_xy_enabled){
				return 0;
			}
			//trace("_velocity.x", _velocity.x)
			return _velocity.x;
			// Note: is +_velocity.x the output velocity in logs is in reverse direction from accel lat
		}

		public function get_longitude_velocity():Number
		{
			// make sure we've been initialised
			if( !_xy_enabled ) {
				return 0;
			}
			//trace("_velocity.y", _velocity.y)

			return _velocity.y;
		}

		// set_velocity_xy - set velocity in latitude & longitude directions (in cm/s)
		public function set_velocity_xy(x:Number, y:Number):void
		{
			//trace("set_velocity_xy", x, y)
			_velocity.x = x;
			_velocity.y = y;
		}

		//
		// Z Axis methods
		//

		// set time constant - set timeconstant used by complementary filter
		public function set_time_constant_z(time_constant_in_seconds:Number):void
		{
			// ensure it's a reasonable value
			if( time_constant_in_seconds > 0 && time_constant_in_seconds < 30 ) {
				_time_constant_z = time_constant_in_seconds;
				update_gains();
			}
		}

		// check_baro - check if new baro readings have arrived and use them to correct vertical accelerometer offsets
		public function check_baro():void
		{
			var baro_update_time:int;

			if( _baro == null ){
				return;
			}
			// calculate time since last baro reading
			baro_update_time = _baro.get_last_update();

			if( baro_update_time != _baro_last_update ) {
				var dt:Number = Number(baro_update_time - _baro_last_update) / 1000.0;
				// call correction method
				correct_with_baro(_baro.get_altitude() * 100, dt);
				_baro_last_update = baro_update_time;
			}
		}


		// correct_with_baro - modifies accelerometer offsets using barometer.  dt is time since last baro reading
		private var first_reads:int = 0;

		public function correct_with_baro(baro_alt:Number, dt:Number):void
		{
			_baro_alt = baro_alt;
			//static uint8_t first_reads = 0;
			//var hist_position_base_z:Number;

			// discard samples where dt is too large
			if( dt > 0.5 ) {
				return;
			}

			// discard first 10 reads but perform some initialisation
			if( first_reads <= 10 ) {
				set_altitude(baro_alt);
				first_reads++;
			}

			// 3rd order samples (i.e. position from baro) are delayed by 150ms (15 iterations at 100hz)
			// so we should calculate error using historical estimates
			if( _hist_position_estimate_z.num_items() >= 15 ) {
				hist_position_base_z = _hist_position_estimate_z.peek(14);
			}else{
				hist_position_base_z = _position_base.z;
			}

			// calculate error in position from baro with our estimate
			_position_error.z = baro_alt - (hist_position_base_z + _position_correction.z);
			
			//trace("z", _position_base.z.toFixed(0), "_position_error.z",_position_error.z.toFixed(0), "pos corr", 
			//	_position_correction.z.toFixed(0), "accel_cor", accel_correction_ef.z.toFixed(2));

			//z -2370 _position_error.z 1203 pos corr 1167 accel_cor 7.95
	
		}

		// set_altitude - set base altitude estimate in cm
		public function set_altitude(new_altitude:Number):void
		{
			_position_base.z = new_altitude;
			_position_correction.z = 0;
		}

		public function get_altitude():Number
		{
			return _position_base.z + _position_correction.z;
		}

		//
		// Private methods
		//

		// update_gains - update gains from time constant (given in seconds)
		public function update_gains():void
		{
			_time_constant_xy			= g.time_constant_xy;
			_time_constant_z			= g.time_constant_z;

			// X & Y axis time constant
			if(_time_constant_xy == 0){
				_k1_xy = _k2_xy = _k3_xy = 0;
			}else{
				_k1_xy = 3 / _time_constant_xy;
				_k2_xy = 3 / (_time_constant_xy * _time_constant_xy);
				_k3_xy = 1 / (_time_constant_xy * _time_constant_xy * _time_constant_xy);
			}

			// Z axis time constant
			if(_time_constant_z == 0){
				_k1_z = _k2_z = _k3_z = 0;
			}else{
				_k1_z = 3 / _time_constant_z;
				_k2_z = 3 / (_time_constant_z * _time_constant_z);
				_k3_z = 1 / (_time_constant_z * _time_constant_z * _time_constant_z);
			}

			//trace("_k1_z", _k1_z);
			//trace("_k2_z", _k2_z);
			//trace("_k3_z", _k3_z);
		}

		public function get_velocity_z():Number
		{
			return _velocity.z;
		}

		// set_velocity_z - get latest climb rate (in cm/s)
		public function set_velocity_z(z:Number):void
		{
			//trace("set_velocuty",z);
			_velocity.z = z;
		}
	}
}