package com.libraries {
	public class Location
	{
		public var id				:int = 0;
		public var options			:int = 0;
		public var p1				:int = 0;
		public var alt				:int = 0;
		public var lat				:int = 0;
		public var lng				:int = 0;
		public var ground_course	:int = 0;
		public var ground_speed		:int = 0;

		public function Location(){
		}

		public function clone(){
			var tmp = new Location();
			tmp.id 		= this.id;
			tmp.options = this.options;
			tmp.p1 		= this.p1;
			tmp.alt 	= this.alt;
			tmp.lat 	= this.lat;
			tmp.lng 	= this.lng;
			return tmp;
		}

		public function toString():String
		{
			return("id:"+ id.toString() + " p1:"+ p1.toString()+ "  alt:"+ alt.toString()+ "  lat:"+ lat.toString()+ "  lng:"+ lng.toString())
		}
	}
}



