package com.libraries {
	import com.libraries.Location;
	import flash.geom.Point;

	public class Sonar extends Location{
		private static var instance			:Sonar = null;

		public var altitude:Number = 0;
		public const max_distance:int = 765
		public const min_distance:int = 20

		public var _copter_loc:Location;

		public function Sonar() {
		}

		//Sonar.getInstance();
		static public function getInstance():Sonar
		{
			if (instance == null)
				instance = new Sonar();
			return instance;
		}
		public function set_location(c:Location):void
		{
			_copter_loc = c;
		}

        public function read():Number
		{
			return constrain(_copter_loc.alt, 30, 700);
        }

		public function constrain(val:Number, min:Number, max:Number) :Number
		{
			val = Math.max(val, min);
			val = Math.min(val, max);
			return val;
		}
	}
}




