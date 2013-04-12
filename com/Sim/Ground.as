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
	import com.Main;
	import com.libraries.AC_WPNav;

	public class Ground extends MovieClip
	{
		//public var lines			:int = 4;
		public var copter_XY		:Point;
		public var nextwp_XY		:Point;
		public var prevwp_XY		:Point;
		public var wpinav_target	:Point;

		//public var frame			:Rectangle;
		public var copter			:Copter;
		public var ghost			:MovieClip;
		public var controller		:Main;
		public var current_loc		:Location;
		public var scale		:Number = .5;

		public var line				:MovieClip;
		
		private const SMALL_X 		:int = 1000;
		private const SMALL_Y 		:int = 200;

		private const LARGE_X 		:int = 650;
		private const LARGE_Y 		:int = 400;
		private const LONSCALE 		:Number = 1//.123;
		public var wp_nav			:AC_WPNav;


public const LATLON_TO_CM 					:Number = 1.113195;

		public function Ground(){
			super();
			addEventListener(Event.ADDED_TO_STAGE, addedToStage);
			//frame 			= new Rectangle(0,0,100,100);
			copter_XY 		= new Point(0,0);
			nextwp_XY 		= new Point(0,0);
			prevwp_XY 		= new Point(0,0);
			trace("create ground")
			maskMe.visible = false;
		}
		
		public function toggle():void
		{
			if(this.x == LARGE_X){
				shrink()
			}else{
				grow();
			}
		}		
		public function shrink():void
		{
			this.mask = maskMe;
			this.x = SMALL_X;
			this.y = SMALL_Y;
		}
		public function grow():void
		{
			this.mask = null;
			this.x = LARGE_X;
			this.y = LARGE_Y;
		}

		public function draw():void
		{
			//             c           	|     	n
			//            -40		   	0		20
			//							+40     + 60
			//             0  - -40
			//             20 - -40


			nav_yaw_MC.rotation = controller.nav_yaw/100;

			copter_XY.x 	= copter.true_loc.lng * LONSCALE;
			copter_XY.y 	= -copter.true_loc.lat;

			wp_target_mc.x = controller.pv_get_lon(wp_nav._target )- copter_XY.x;
			wp_target_mc.y = -controller.pv_get_lat(wp_nav._target )- copter_XY.y;

			loc.x = (controller.current_loc.lng - copter.true_loc.lng) * LONSCALE;
			loc.y = (-controller.current_loc.lat) - copter_XY.y;

			gps.x = (controller.g_gps.longitude - copter.true_loc.lng) * LONSCALE;
			gps.y = (-controller.g_gps.latitude) - copter_XY.y;

			//prevwp_XY.x 	= controller.prev_WP.lng * LONSCALE - copter_XY.x;
			//prevwp_XY.y 	= (-controller.prev_WP.lat) - copter_XY.y;

			nextwp_XY.x 	= controller.pv_get_lon(wp_nav._destination) - copter_XY.x;
			nextwp_XY.y 	= (-controller.pv_get_lat(wp_nav._destination)) - copter_XY.y;


			grass.x = -copter_XY.x % 200 + 50;
			grass.y = -copter_XY.y % 200 - 50;

			target_.x = nextwp_XY.x;
			target_.y = nextwp_XY.y;




			copter_mc.rotation 	= copter.ahrs.yaw_sensor/100;
			copter_mc.copter_xy.scaleX 	= controller.cos_roll_x;
			copter_mc.copter_xy.scaleY 	= controller.cos_pitch_x;


			copter_shadow.x = copter_mc.x + copter.true_loc.alt / 10;
			copter_shadow.y = copter_mc.y + copter.true_loc.alt / 10;

			copter_shadow.rotation 	= copter_mc.rotation;
			copter_shadow.copter_xy.scaleX 	= copter_mc.copter_xy.scaleX;
			copter_shadow.copter_xy.scaleY 	= copter_mc.copter_xy.scaleY;
			copter_shadow.scaleX = copter_shadow.scaleY = 1 - (copter.true_loc.alt / 4000);
			
			/*
			copter_shadow.x *= scale;
			copter_shadow.y *= scale;

			copter_mc.x *= scale;
			copter_mc.y *= scale;
			
			prevwp_XY.x *= scale;
			prevwp_XY.y *= scale;
			target_.x *= scale;
			target_.y *= scale;
			*/


			line.graphics.clear();
			line.graphics.lineStyle(2, 0x000000);
			line.graphics.moveTo(prevwp_XY.x, prevwp_XY.y);
			line.graphics.lineTo(target_.x, target_.y);

			line.graphics.lineStyle(2, 0xFF7C00);
			line.graphics.moveTo(copter_mc.x, copter_mc.y);
			line.graphics.lineTo(wp_target_mc.x/ LATLON_TO_CM, wp_target_mc.y/ LATLON_TO_CM);
		}


		public function addedToStage(event:Event):void
		{
					trace("added ground to stage")

			//this.frame.width 	= Math.round(_preview.width);
			//this.frame.height 	= Math.round(_preview.height);
			_preview.visible = false;
			scaleX = .35;
			scaleY = .35;
			//draw();
			line = new MovieClip()
			addChildAt(line, 3);
		}

		public function degrees(r:Number):Number
		{
			return r * 57.2957795;
		}

		public function radians(n:Number):Number
		{
			return 0.0174532925 * n;
		}

		public function radiansx100(n:Number):Number
		{
			return 0.000174532925 * n;
		}

	}
}