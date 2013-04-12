package com.libraries {
	public class LeadFilter
	{
		// takes velocity and position and outputs pos estimate
		//

		private var _last_velocity:Number = 0;

		public function LeadFilter(){
		}

		public function init():void
		{
			_last_velocity = 0;
		}

		public function get_position(pos:Number, vel:Number, lag_in_seconds:Number = 1.0):Number
		{
		    // assumes a 1 second delay in the GPS
		    var accel_contribution:Number 	= (vel - _last_velocity) * lag_in_seconds * lag_in_seconds;
   			var vel_contribution:Number 	= vel * lag_in_seconds;

		    // store velocity for next iteration
			_last_velocity = vel;

			return pos + vel_contribution + accel_contribution;
		}
		public function clear():void
		{
			_last_velocity = 0;
		}
	}
}
