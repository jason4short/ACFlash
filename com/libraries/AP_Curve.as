package com.libraries
{
	public class AP_Curve
	{

	    private var _num_points		:int = 0;       // number of points in the cruve
		private var _x				:Array; 	// x values of each point on the curve
		private var _y				:Array; 	// x values of each point on the curve
		private var _slope			:Array; 	// x values of each point on the curve
		private var _constrained	:Boolean; 	// if true, first and last points added will constrain the y values returned by get_y function

		private var SIZE		:int = 0;

		// Constructor
		public function AP_Curve(_size:int) {
			SIZE 		= _size;
			_x 			= new Array(SIZE);
			_y 			= new Array(SIZE);
			_slope 		= new Array(SIZE);
			clear();
		}

		// clear the curve
		public function clear():void
		{
			for(var i:int = 0; i < SIZE; i++ ) {
				_x[i] = 0;
				_y[i] = 0;
				_slope[i] = 0;
			}
			_num_points = 0;
		}

		// add_point - adds a point to the curve
		public function add_point(x:Number, y:Number):Boolean
		{
			if(_num_points < SIZE){
				_x[_num_points] = x;
				_y[_num_points] = y;

				// increment the number of points
				_num_points++;

				// if we have at least two points calculate the slope
				if(_num_points > 1){
					_slope[_num_points - 2] = (_y[_num_points - 1] - _y[_num_points - 2]) / (_x[_num_points - 1] - _x[_num_points - 2]);
					_slope[_num_points - 1] = _slope[_num_points - 2];	// the final slope is for interpolation beyond the end of the curve
				}
				return true;
			}else{
				// we do not have room for the new point
				return false;
			}
		}



		// get_y - returns the y value on the curve for a given x value
		public function get_y(x:Number):Number
		{
			var i:int;
			var result:Number;

			// deal with case where ther is no curve
			if( _num_points == 0 ) {
				return x;
			}

			// when x value is lower than the first point's x value, return minimum y value
			if(x <= _x[0]){
				return _y[0];
			}

			// when x value is higher than the last point's x value, return maximum y value
			if(x >= _x[_num_points - 1]){
				return _y[_num_points - 1];
			}

			// deal with the normal case
			for(i = 0; i < _num_points - 1; i++){
				if(x >= _x[i] && x <= _x[i + 1]){
					result = _y[i] + (x - _x[i]) * _slope[i];
					return result;
				}
			}

			// we should never get here
			return x;
		}

		// get_y - returns the y value on the curve for a given x value
		public function dump_curve():void
		{
			trace("Curve:")
			for(var i:int = 0; i < _num_points; i++){
				trace("x:", _x[i].toFixed(2),"\ty:", _y[i].toFixed(2),"\tslope:", _slope[i].toFixed(2));
			}
		}
	}
}