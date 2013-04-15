package com.Sim {
	import flash.display.MovieClip;
    import flash.display.Sprite;
    import flash.display.Stage;
    import flash.display.StageDisplayState;
    import flash.display.Bitmap;
    import flash.display.BitmapData;

	import flash.display.DisplayObject;
	import flash.events.*;
	import flash.geom.Rectangle;
	import flash.geom.Point;
	import flash.display.StageAlign;
	import flash.display.StageScaleMode;
	import flash.utils.*;
	import flash.ui.Keyboard;
    import flash.geom.Rectangle;

	import com.Sim.Copter;
	import com.libraries.Location;
	import com.libraries.AP_Baro;
	import com.libraries.GPS;
	import com.Main;
	import com.libraries.AC_WPNav2;

	public class Sky extends MovieClip
	{
		//public var lines			:int = 4;
		public var copter_XY		:Point;
		public var ghost_XY			:Point;
		public var frame			:Rectangle;
		public var copter			:Copter;
		public var copter_mc		:MovieClip;
		public var copter_lag		:MovieClip;
		public var ghost			:MovieClip;
		public var controller		:Main;
		public var current_loc		:Location;
		public var baro				:AP_Baro;
		public var gps				:GPS;
		public var x_page			:int;
		public var y_page			:int;
		public var x_page_g			:int;
		public var y_page_g			:int;
		public var wp_nav			:AC_WPNav2;


		public function Sky(){
			super();
			addEventListener(Event.ADDED_TO_STAGE, addedToStage);
			frame 			= new Rectangle(0,0,100,100);
			copter_XY 		= new Point(0,0);
			ghost_XY 		= new Point(0,0);
			baro			= AP_Baro.getInstance();
		}

		public function draw():void
		{
			var tll:int  = copter.true_loc.lng;
			copter_XY.x 	= frame.width/2 + tll;
			copter_mc.x 	= copter_XY.x % frame.width;
			if (copter_mc.x < 0)
				copter_mc.x	+= frame.width;

			copter_XY.y 	= frame.height - copter.true_loc.alt;
			copter_mc.y 	= copter_XY.y % frame.height;
			if (copter_mc.y < 0)
				copter_mc.y	+= frame.height;

			calc_copter_page();

			grass.visible = (y_page == 0);
			grass.scaleX  = ((x_page % 2) == 0) ? 1 : -1;

			//-----------------------------------------------------------------
			baro_mc.x = copter_mc.x;

			baro_mc.y 	= frame.height - baro.altitude;
			baro_mc.y 	= baro_mc.y % frame.height;
			if (baro_mc.y < 0)
				baro_mc.y	+= frame.height;

			//-----------------------------------------------------------------

			copter_lag.x 	= frame.width/2 + current_loc.lng; //* 1.123;
			copter_lag.x 	= copter_lag.x % frame.width;
			if (copter_lag.x < 0)
				copter_lag.x	+= frame.width;

			copter_lag.y 	= frame.height - current_loc.alt;

			copter_lag.y 	= copter_lag.y % frame.height;
			if (copter_lag.y < 0)
				copter_lag.y	+= frame.height;



			//-----------------------------------------------------------------
			gps_mc.x 	= frame.width/2 + gps.longitude;// * 1.123;
			gps_mc.x 	= gps_mc.x % frame.width;
			if (gps_mc.x < 0)
				gps_mc.x	+= frame.width;

			gps_mc.y 	= frame.height - gps.altitude;

			gps_mc.y 	= gps_mc.y % frame.height;
			if (gps_mc.y < 0)
				gps_mc.y	+= frame.height;



			//-----------------------------------------------------------------

			var lon_:int = controller.pv_get_lon(wp_nav._target )- tll;

			ghost_XY.x 		= frame.width/2 + controller.pv_get_lon(wp_nav._target);
			ghost.x 		= ghost_XY.x % frame.width;
			if (ghost.x < 0)
				ghost.x	+= frame.width;

			ghost_XY.y 		= frame.height - Math.max(controller.target_alt_for_reporting, 1);
			ghost.y 		= ghost_XY.y % frame.height;
			if (ghost.y < 0)
				ghost.y	+= frame.height;

			calc_ghost_page();

			ghost.visible = ((x_page == x_page_g) && (y_page == y_page_g));

			//-----------------------------------------------------------------

			//if(copter.angle3D.z >0)
			//	copter_mc.rotation = -degrees(Math.asin(copter.angle3D.y));
			//else
			//	copter_mc.rotation = degrees(Math.asin(copter.angle3D.y));

			//trace(copter.ahrs.roll_sensor/100);
			//copter_mc.rotation = copter.ahrs.roll_sensor/100;

			copter_mc.rotation = degrees(Math.atan2(copter.angle3D.z, copter.angle3D.y)) -90;
			//trace("copter.angle3D.y", copter.angle3D.y, copter_mc.rotation);
		}

		public function calc_copter_page():void
		{
			x_page		= copter_XY.x / frame.width;
			y_page		= (frame.height - copter_XY.y) / frame.height;

			if (copter_XY.x  < 0)
				x_page--;
		}

		public function calc_ghost_page():void
		{
			x_page_g		= ghost_XY.x / frame.width;
			y_page_g		= (frame.height - ghost_XY.y) / frame.height;

			if (ghost_XY.x  < 0)
				x_page_g--;
		}

		public function addedToStage(event:Event):void
		{
			_preview.visible 	= false;
			this.frame.width 	= Math.round(_preview.width);
			this.frame.height 	= Math.round(_preview.height);
			scaleX = 1;
			scaleY = 1;
		}

		public function degrees(r:Number):Number
		{
			return r * 57.2957795;
		}

		public function radians(n:Number):Number
		{
			return 0.0174532925 * n;
		}

	}
}
