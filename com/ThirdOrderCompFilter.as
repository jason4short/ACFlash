package com {
	import flash.geom.Vector3D;
	import flash.geom.Matrix3D;

	public class ThirdOrderCompFilter
	{
		public const THIRD_ORDER_SAVE_POS_10HZ :int = 10;
		public const THIRD_ORDER_SAVE_POS_5HZ :int = 20;
		public const THIRD_ORDER_COMP_FILTER_HISTORIC_XY_SAVE_COUNTER_DEFAULT :int = 10;

		public var _k1_xy	:Number;				    // 1st order error correction gain for horizontal position
		public var _k2_xy	:Number;				    // 2nd order error correction gain for horizontal position
		public var _k3_xy	:Number;				    // 3rd order error correction gain for horizontal position
		public var _k1_z	:Number;				    // 1st order error correction gain for altitude
		public var _k2_z	:Number;					// 2nd order error correction gain for altitude
		public var _k3_z	:Number;				   	// 3rd order error correction gain for altitude

		public var _comp_k1o			:Vector3D;  	// acceleration estimate
		public var _comp_v				:Vector3D;  	// acceleration estimate
		public var _comp_h				:Vector3D;  	// acceleration estimate
		public var _first_order_sample	:Vector3D;  	// acceleration estimate

		public var	_historic_xy_counter :int;			// historic positions saved when this counter reaches 10

		public var _hist_3rd_order_estimates_x	:AP_Buffer;    // buffer of historic accel based position to account for lag
		public var _hist_3rd_order_estimates_y	:AP_Buffer;    // buffer of historic accel based position to account for lag
		public var _hist_3rd_order_estimates_z	:AP_Buffer;    // buffer of historic accel based altitudes to account for lag

		public var _comp_h_correction	:Vector3D;  	// sum of correction to _comp_h from delayed 1st order samples
		public var _comp_h_total		:Vector3D;  	// sum of _comp_h + _comp_h_correction
		public var _comp_k1o_ef			:Vector3D;  	// accelerometer correction in earth frame (only z element is used).  here for debug purposes


		private var last_time_constant_xy:Number = 0;
		private var last_time_constant_z:Number = 0;


		public function ThirdOrderCompFilter(time_constant_seconds_xy:Number, time_constant_seconds_z:Number){
            update_gains(time_constant_seconds_xy, time_constant_seconds_z);
            _hist_3rd_order_estimates_x = new AP_Buffer(10);
            _hist_3rd_order_estimates_y = new AP_Buffer(10);
            _hist_3rd_order_estimates_z = new AP_Buffer(15);
		}

		// update_gains - update gains from time constant (given in seconds)
		public function update_gains(time_constant_seconds_xy:Number, time_constant_seconds_z:Number):void
		{

			// X & Y axis time constant
			if( time_constant_seconds_xy == 0 ) {
				_k1_xy = _k2_xy = _k3_xy = 0;
			}else{
				if( time_constant_seconds_xy != last_time_constant_xy ) {
					_k1_xy = 3 / time_constant_seconds_xy;
					_k2_xy = 3 / (time_constant_seconds_xy*time_constant_seconds_xy);
					_k3_xy = 1 / (time_constant_seconds_xy*time_constant_seconds_xy*time_constant_seconds_xy);
					last_time_constant_xy = time_constant_seconds_xy;
				}
			}

			// Z axis time constant
			if( time_constant_seconds_z == 0 ) {
				_k1_z = _k2_z = _k3_z = 0;
			}else{
				if( time_constant_seconds_z != last_time_constant_z ) {
					_k1_z = 3 / time_constant_seconds_z;
					_k2_z = 3 / (time_constant_seconds_z*time_constant_seconds_z);
					_k3_z = 1 / (time_constant_seconds_z*time_constant_seconds_z*time_constant_seconds_z);
					last_time_constant_z = time_constant_seconds_z;
				}
			}
		}

		// set_3rd_order - resets the first order value (i.e. position)
		public function set_3rd_order_xy(x:Number, y:Number):void
		{
			_comp_h.x = x;
			_comp_h.y = y;
			_comp_h_correction.x = 0;
			_comp_h_correction.y = 0;

			// clear historic estimates
			_hist_3rd_order_estimates_x.clear();
			_hist_3rd_order_estimates_y.clear();
		}

		// set_3rd_order - resets the first order value (i.e. position)
		public function set_3rd_order_z(z:Number):void
		{
			_comp_h.z = z;
			_comp_h_correction.z = 0;
		}

		// set_2nd_order - resets the second order value (i.e. velocity)
		public function set_2nd_order_xy(x:Number, y:Number):void
		{
			_comp_v.x = x;
			_comp_v.y = y;
		}

		// set_2nd_order - resets the second order value (i.e. velocity)
		public function set_2nd_order_z(z:Number):void
		{
			_comp_v.z = z;
		}

		// correct_3rd_order_z - correct accelerometer offsets using barometer or gps
		public function correct_3rd_order_xy(x:Number, y:Number, dcm_matrix:Matrix3D, deltat:Number):void
		{
			var hist_comp_h_x:Number;
			var hist_comp_h_y:Number;

			// 3rd order samples (i.e. position from gps) are delayed by 500ms
			// we store historical position at 10hz so 5 iterations ago
			if( _hist_3rd_order_estimates_x.num_items() >= 4 ) {
				hist_comp_h_x = _hist_3rd_order_estimates_x.peek(3);
				hist_comp_h_y = _hist_3rd_order_estimates_y.peek(3);
			}else{
				hist_comp_h_x = _comp_h.x;
				hist_comp_h_y = _comp_h.y;
			}

			// calculate error in position from gps with our historical estimate
			var err_x:Number = x - (hist_comp_h_x + _comp_h_correction.x);
			var err_y:Number = y - (hist_comp_h_y + _comp_h_correction.y);

			// calculate correction to accelerometers and apply in the body frame
			_comp_k1o += dcm_matrix.mul_transpose(new Vector3D( (err_x * _k3_xy) * deltat, (err_y * _k3_xy) * deltat,0));

			// correct velocity
			_comp_v.x += (err_x*_k2_xy) * deltat;
			_comp_v.y += (err_y*_k2_xy) * deltat;

			// correct position
			_comp_h_correction.x += err_x*_k1_xy * deltat;
			_comp_h_correction.y += err_y*_k1_xy * deltat;
		}

		// correct_3rd_order_z - correct accelerometer offsets using barometer or gps
		public function correct_3rd_order_z(third_order_sample:Number, dcm_matrix:Matrix3D, deltat:Number):void
		{
			var hist_comp_h_z:Number;

			// 3rd order samples (i.e. position from baro) are delayed by 150ms (15 iterations at 100hz)
			// so we should calculate error using historical estimates
			if( _hist_3rd_order_estimates_z.num_items() >= 15 ) {
				//hist_comp_h_z = _hist_3rd_order_estimates_z.get();
				hist_comp_h_z = _hist_3rd_order_estimates_z.peek(14);
			}else{
				hist_comp_h_z = _comp_h.z;
			}

			// calculate error in position from baro with our estimate
			var err:Number = third_order_sample - (hist_comp_h_z + _comp_h_correction.z);

			// calculate correction to accelerometers and apply in the body frame
			_comp_k1o += dcm_matrix.mul_transpose(new Vector3D(0, 0, (err*_k3_z) * deltat));

			// correct velocity
			_comp_v.z += (err*_k2_z) * deltat;

			// correct position
			_comp_h_correction.z += err*_k1_z * deltat;
		}

		// recalculate the 2nd and 3rd order estimates
		public function calculate(deltat:Number, dcm_matrix:Matrix3D):void
		{
			// get earth frame accelerometer correction
			_comp_k1o_ef = dcm_matrix * _comp_k1o;

			// calculate velocity by adding new acceleration from accelerometers
			_comp_v += (-_first_order_sample + _comp_k1o_ef) * deltat;

			// calculate new estimate of position
			_comp_h += _comp_v * deltat;

			// store 3rd order estimate (i.e. estimated vertical position) for future use
			_hist_3rd_order_estimates_z.add(_comp_h.z);

			// store 3rd order estimate (i.e. horizontal position) for future use at 10hz
			_historic_xy_counter++;
			if( _historic_xy_counter >= THIRD_ORDER_SAVE_POS_10HZ ) {
				_historic_xy_counter = 0;
				_hist_3rd_order_estimates_x.add(_comp_h.x);
				_hist_3rd_order_estimates_y.add(_comp_h.y);
			}
		}

		// add_1st_order_sample - Add a new 1st order sample (i.e. acceleration) to the filter, but don't recalculate
		public function add_1st_order_sample(sample:Vector3D):void
		{
			_first_order_sample = sample;
		}

		// return the new estimate for the 3rd order (i.e. position)
		public function get_3rd_order_estimate()
		{
			_comp_h_total = _comp_h + _comp_h_correction;
			return _comp_h_total;
		}

		// return the new estimate for the 2nd order (i.e. velocity)
		public function get_2nd_order_estimate()
		{
			return _comp_v;
		}


		// set the 1st order correction vector (i.e. correction to be applied to the accelerometer)
		public function set_1st_order_correction(correction_vector:Vector3D)
		{
			_comp_k1o = correction_vector;
		}


		// get the 1st order correction vector (i.e. correction to be applied to the accelerometer)
		public function get_1st_order_correction()
		{
			return _comp_k1o;
		}

	}
}



