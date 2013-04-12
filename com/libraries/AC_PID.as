package com.libraries {

	public class AC_PID {

		public var _kp				:Number = 0;
		public var _ki				:Number = 0;
		public var _kd				:Number = 0;
		public var _imax			:Number = 0;

		public var _integrator		:Number = 0;
		public var _last_input		:Number = 0;
		public var _last_derivative	:Number = 0;
		public var _output			:Number = 0;
		public var _derivative		:Number = 0;

		// Examples for _filter:
		// f_cut = 10 Hz -> _filter = 15.9155e-3
		// f_cut = 15 Hz -> _filter = 10.6103e-3
		// f_cut = 20 Hz -> _filter =  7.9577e-3
		// f_cut = 25 Hz -> _filter =  6.3662e-3
		// f_cut = 30 Hz -> _filter =  5.3052e-3
		public var _filter			:Number = 7.9577e-3; 		// Set to  "1 / ( 2 * PI * f_cut )";

		public function AC_PID(kp:Number, ki:Number, kd:Number, imax:Number) {
			_kp = kp;
			_ki = ki;
			_kd = kd;
			_imax = imax;
		}

		public function get_integrator():Number
		{
			return _integrator;
		}
		public function set_integrator(i:Number):void
		{
			_integrator = i;
		}

		public function kP():Number
		{
			return _kp;
		}

		public function get_p(error:Number):Number
		{
			return error * _kp;
		}

		public function get_i(error:Number, dt:Number):Number
		{
			if((_ki != 0) && (dt != 0)){
				_integrator += (error * _ki) * dt;
				if (_integrator < -_imax) {
					_integrator = -_imax;
				} else if (_integrator > _imax) {
					_integrator = _imax;
				}
				return _integrator;
			}
			return 0;
		}


		// This is an integrator which tends to decay to zero naturally
		// if the error is zero.

		public function get_leaky_i(error:Number, dt:Number, leak_rate:Number):Number
		{
			if((_ki != 0) && (dt != 0)){
				_integrator -= _integrator * leak_rate;
				_integrator += (error * _ki) * dt;

				if (_integrator < -_imax) {
					_integrator = -_imax;
				} else if (_integrator > _imax) {
					_integrator = _imax;
				}

				return _integrator;
			}
			return 0;
		}



		public function get_d(input:Number, dt:Number):Number
		{
			if ((_kd != 0) && (dt != 0)) {

				if(isNaN(_last_derivative)){
					// we've just done a reset, suppress the first derivative
					// term as we don't want a sudden change in input to cause
					// a large D output change
					_derivative = 0;
					_last_derivative = 0;
				} else {
					// calculate instantaneous derivative
					_derivative = (input - _last_input) / dt;
				}

				// discrete low pass filter, cuts out the
				// high frequency noise that can drive the controller crazy
				_derivative = _last_derivative +
							  (dt / ( _filter + dt)) * (_derivative - _last_derivative);

				// update state
				_last_input 		= input;
				_last_derivative    = _derivative;

				// add in derivative component
				return _kd * _derivative;
			}
			return 0;
		}

		public function get_pid(error:Number, dt:Number):Number
		{
			return get_p(error) + get_i(error, dt) + get_d(error, dt);
		}

		public function get_pi(error:Number, dt:Number):Number
		{
			return get_p(error) + get_i(error, dt);
		}


		public function reset_I():void
		{
			_integrator = 0;
			// mark derivative as invalid
			_last_derivative = NaN;
		}


		public function constrain(val:Number, min:Number, max:Number){
			val = Math.max(val, min);
			val = Math.min(val, max);
			return val;
		}

	}
}

