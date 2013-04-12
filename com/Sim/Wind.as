package com.Sim {
	import com.Parameters;
	import flash.geom.*;

	public class Wind extends Object
	{
			public var g				:Parameters;
			private var speedDelta		:Number = 100;
			public var windSpeed		:Number = 0;
			private var actual_speed	:Number = 0;
			private var period_radians	:Number = 0;
			private var angle			:Number = 0;
			private var myRandom		:Array;
			private var pointer			:int = 0;
			private var m3d				:Matrix3D;
			private var wind3D			:Vector3D;


		public function Wind()
		{
			g = Parameters.getInstance();
			wind3D	= new Vector3D();
			// 60
			m3d = new Matrix3D();
			myRandom = [0.998, 0.643, 0.023, -0.952, -0.176, 0.306, 0.926, -0.139, 0.112, -0.785, 0.326, 0.686, -0.384, 0.818, 0.914, -0.153, -0.001, -0.905, -0.151, 0.259, 0.723, 0.277, -0.199, -0.968, -0.083, -0.307, -0.417, 0.692, 0.821, -0.167, 0.01, -0.804, -0.411, -0.566, -0.914, -0.221, 0.713, -0.107, -0.443, 0.973, 0.961, -0.134, 0.477, -0.281, -0.953, 0.212, -0.184, 0.804, 0.364, 0.12, -0.612, 0.672, -0.677, -0.213, -0.1, 0.505, -0.495, -0.642, 0.079, -0.478, 0.917, -0.738];
		}

		public function init():void
		{
			setDirection(g.windDir);
			setSpeed(g.windSpeedMin, g.windSpeedMax);
			setPeriod(g.windPeriod);
			resetWind();
		}

		public function setSpeed(_min:Number, _max:Number):void
		{
			g.windSpeedMin = _min;
			g.windSpeedMax = _max;
			speedDelta = g.windSpeedMax - g.windSpeedMin;
			resetWind();
		}

		public function resetWind(){
			pointer = 0;
			angle = 0;
			//trace("reset pointer ");
			getNewSpeed();
		}

		public function setPeriod(n:Number):void
		{
			g.windPeriod = n;
			period_radians  = getNewSpeed();
			resetWind();
		}

		// call at 50hz
		public function read():Point
		{
			angle += period_radians;
			windSpeed = g.windSpeedMin + (1 + Math.sin(angle)) * speedDelta;
			if(angle > Math.PI * 2){
				angle = 0;
				period_radians  = getNewSpeed();
			}

			var speed:Point = new Point(windSpeed,windSpeed);
			speed.x *= wind3D.x;
			speed.y *= wind3D.y;
			return  speed;
		}

		public function setDirection(d:Number):void
		{
			m3d = new Matrix3D();
			wind3D = new Vector3D(0, -1, 0);
			m3d.appendRotation(-d, Vector3D.Z_AXIS);	// Yaw
			wind3D = m3d.transformVector(wind3D);
		}

		private function getNewSpeed():Number
		{
			var r:Number = myRandom[pointer];
			//trace("pointer " + pointer);
			pointer++;

			if(pointer>50)
				pointer = 0;

			var v:Number = (g.windPeriod/2) + r * (g.windPeriod / 2);
			var s:Number = (Math.PI * 2) / 50;
			return  s / (v / 1000);
		}

	}
}




