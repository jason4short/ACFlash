package com.libraries {
	import flash.geom.Vector3D;
	import flash.geom.Matrix3D;
	import com.libraries.Location;
	import com.libraries.AP_Baro;

	public class AP_AHRS extends Location{

		private static var instance:AP_AHRS = null;
		public var roll_sensor			:Number = 0;
		public var pitch_sensor			:Number = 0;
		public var yaw_sensor			:Number = 0;

		public var roll					:Number = 0;
		public var pitch				:Number = 0;
		public var yaw					:Number = 0;

		public var gyro					:Vector3D;
		public var omega				:Vector3D;
		public var dcm					:Matrix3D;

		public var accel_bf				:Vector3D;
		public var accel_ef				:Vector3D;
		public var barometer			:AP_Baro;

		public function AP_AHRS() {
			gyro 			= new Vector3D(0,0,0);
			omega 			= new Vector3D(0,0,0);
			dcm				= new Matrix3D();
		}

		//AP_RC.getInstance();
		static public function getInstance():AP_AHRS
		{
			if (instance == null)
			  instance = new AP_AHRS();
			return instance;
		}

		public function init() {
			roll_sensor 	= 0;
			pitch_sensor 	= 0;
			yaw_sensor 		= 0;
			accel_bf 		= new Vector3D(0,0,-9.805);
			accel_ef 		= new Vector3D(0,0,-9.805);

			gyro 			= new Vector3D(0,0,0);
			omega 			= new Vector3D(0,0,0);
			dcm				= new Matrix3D();
		}


		public function get_accel_ef()
		{
			return accel_ef.clone();
		}

		public function get_accel_bf()
		{
			return accel_bf.clone();
		}

		public function set_barometer(baro:AP_Baro)
		{
			barometer = baro;
		}

		public function addToRoll(r:Number)
		{
			roll_sensor 	+= r;
			roll_sensor = wrap_180(roll_sensor);
		}

		public function wrap_180(error:Number):Number
		{
			if (error > 18000)	error -= 36000;
			if (error < -18000)	error += 36000;
			return error;
		}

	}
}




