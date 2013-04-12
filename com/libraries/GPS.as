package com.libraries {
	import com.libraries.Location;
	import flash.geom.Point;
	import com.Sim.Clock;

	public class GPS extends Location{

		public var longitude		:Number = 0;
		public var long_est			:Number = 0;
		public var latitude			:Number = 0;
		public var altitude			:Number = 0;
		private var gps_counter		:int = 0;


		public var new_data		:Boolean = false;
		public var fix			:Boolean = true;

		public var _true_loc:Location;

		private var delay_array:Array;
		private var obj0:Location;
		private var obj1:Location;
		private var obj2:Location;
		private var obj3:Location;
		private var obj4:Location;
		private var obj5:Location;
		private var obj6:Location;
		private var obj7:Location;
		private var obj8:Location;
		private var obj9:Location;
		private var obj10:Location;

		private var pointer:int;
		private var max_delay:int = 5;
		private var clock					:Clock;
		public  var time					:int;
		public  var last_fix_time			:int;


		public function GPS(true_loc:Location)
		{
			clock		= Clock.getInstance();
			_true_loc = true_loc;
			obj0 = new Location();
			obj1 = new Location();
			obj2 = new Location();
			obj3 = new Location();
			obj4 = new Location();
			obj5 = new Location();
			obj6 = new Location();
			obj7 = new Location();
			obj8 = new Location();
			obj9 = new Location();
			obj10 = new Location();
			delay_array = [obj0, obj1, obj2, obj3, obj4, obj5, obj6, obj7, obj8, obj9, obj10];
			pointer = 0;
		}

		private function get_bearing(p1:Point, p2:Point):Number
		{
			var bearing:Number = 90 + degrees(Math.atan2(p1.y- p2.y, p1.x- p2.x));
			if (bearing < 0) bearing += 360;
			return bearing * 100;
		}

        public function init():void {
			for(var i:int = 0; i < delay_array.length; i++){
				delay_array[i].alt = _true_loc.alt;
				delay_array[i].lat = _true_loc.lat;
				delay_array[i].lng = _true_loc.lng;
			}
			new_data 	= false;
        	pointer 	= 0;
        	gps_counter = 0;
        	longitude 	= _true_loc.lng;
			latitude  	= _true_loc.lat;
			altitude  	= _true_loc.alt;
        }

        public function read():Boolean
        {
			gps_counter++;
			if(gps_counter >= 25){
				time = clock.millis();
				last_fix_time = time
				gps_counter = 0;
				read_internal()
				return true;
			}
			return false;
        }

        public function read_internal():void {
        	pointer++;
        	if (pointer >= max_delay)
        		pointer = 0;

        	// store current copter loc into the Delay Array
			delay_array[pointer].alt = _true_loc.alt;
			delay_array[pointer].lat = fixrez(_true_loc.lat);
			delay_array[pointer].lng = fixrez(_true_loc.lng);
			/*
			0  previous
			1  oldest - current
			2  older
			3  old
			4  new   - pointer


			0  older
			1  old
			2  new   - pointer
			3  previous
			4  oldest - current

			*/
			var old_pointer = (pointer + 1) % max_delay;
			var prev_pointer = (pointer + 2) % max_delay;
			//trace("pointer ",pointer, old_pointer, prev_pointer );

            var p1 = new Point(delay_array[prev_pointer].lng, delay_array[prev_pointer].lat);
            var p2 = new Point(delay_array[old_pointer].lng, delay_array[old_pointer].lat);

			ground_course = get_bearing(p1, p2);
			ground_speed  = Math.abs(Point.distance(p1, p2)) * 4;

            new_data = true;
			
			// grab the loc out of the delay array
			longitude = delay_array[old_pointer].lng;
			latitude  = delay_array[old_pointer].lat;
			altitude  = delay_array[old_pointer].alt;
        }

		public override function toString():String
		{
			return("alt:"+ altitude.toString()+ "  latitude:"+ latitude.toString()+ "  longitude:"+ longitude.toString())
		}

		public function fixrez(n:Number):Number
		{
			var i:int = 0;
			i = Math.floor(n /10);
			return i * 10;
		}

		public function radians(n:Number):Number
		{
			return 0.0174532925 * n;
		}

		public function degrees(radians:Number):Number
		{
			return radians * 180/Math.PI
		}

		public function constrain(val:Number, min:Number, max:Number) :Number
		{
			val = Math.max(val, min);
			val = Math.min(val, max);
			return val;
		}
	}
}




