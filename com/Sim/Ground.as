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
	import flash.geom.Vector3D;
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
	import com.libraries.AP_InertialNav;

	public class Ground extends MovieClip
	{
		//public var lines			:int = 4;
		public var copter_XY		:Point;
		public var nextwp_XY		:Point;
		public var prevwp_XY		:Point;
		public var wpinav_target	:Point;

		//public var frame			:Rectangle;
		public var copter			:Copter;
		public var inertial_nav		:AP_InertialNav;
				
		public var ghost			:MovieClip;
		public var controller		:Main;
		public var current_loc		:Location;
		public var scale			:Number = .5;

		public var line				:MovieClip;
		public var tracks			:MovieClip;
		
		private const SMALL_X 		:int = 1000;
		private const SMALL_Y 		:int = 200;

		private const LARGE_X 		:int = 650;
		private const LARGE_Y 		:int = 400;
		private const LONSCALE 		:Number = 1//.123;
		public var wp_nav			:AC_WPNav;
		private var counter			:int = 0;


		public const LATLON_TO_CM 					:Number = 1.113195;

		public function Ground(){
			super();
			addEventListener(Event.ADDED_TO_STAGE, addedToStage);
			//frame 			= new Rectangle(0,0,100,100);
			copter_XY 		= new Point(0,0);
			nextwp_XY 		= new Point(0,0);
			prevwp_XY 		= new Point(0,0);
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

		public function draw(recordTrack:Boolean):void
		{
			//             c           	|     	n
			//            -40		   	0		20
			//							+40     + 60
			//             0  - -40
			//             20 - -40

		    var curr:Vector3D = inertial_nav.get_position();

			nav_yaw_MC.rotation = controller.nav_yaw/100;

			copter_XY.x 	= copter.position.x;
			copter_XY.y 	= -copter.position.y;

			wp_target_mc.x = wp_nav._target.y - copter_XY.x;
			wp_target_mc.y = -wp_nav._target.x - copter_XY.y;

			//loc.x = (controller.current_loc.lng - copter.true_loc.lng) * LATLON_TO_CM;
			//loc.y = (-controller.current_loc.lat) - copter_XY.y;

			loc.x = curr.y - copter.position.x; // East
			loc.y = curr.x - copter.position.y; // North

			gps.x = (controller.g_gps.longitude * LATLON_TO_CM) - copter.position.x;
			gps.y = -((controller.g_gps.latitude * LATLON_TO_CM) - copter.position.y);

			//wp_nav._prev Vector3D(0, 0, 500) (x=19.47424604666384, y=-442.53393665794965)
			
			prevwp_XY.x 	= wp_nav._prev.y - copter.position.x;
			prevwp_XY.y 	= -(wp_nav._prev.x - copter.position.y);

			//Vector3D(5565.974999999999, 0, 500) (x=14.519686996651958, y=-448.674346006199)
			nextwp_XY.x 	= wp_nav._destination.y - copter.position.x;
			nextwp_XY.y 	= -(wp_nav._destination.x - copter.position.y);


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

			if(recordTrack && (counter >= 25)){
				counter = 0;
				tracks.graphics.lineTo(copter_XY.x, copter_XY.y);
			}else{
				counter++;
			}

			line.graphics.clear();
			line.graphics.lineStyle(2, 0x000000);
			line.graphics.moveTo(prevwp_XY.x, prevwp_XY.y);
			line.graphics.lineTo(target_.x, target_.y);

			line.graphics.lineStyle(2, 0xFF7C00);
			line.graphics.moveTo(copter_mc.x, copter_mc.y);
			line.graphics.lineTo(wp_target_mc.x/ LATLON_TO_CM, wp_target_mc.y/ LATLON_TO_CM);
			tracks.x = -copter_XY.x;
			tracks.y = -copter_XY.y;
			
		}


		public function addedToStage(event:Event):void
		{
			//this.frame.width 	= Math.round(_preview.width);
			//this.frame.height 	= Math.round(_preview.height);
			_preview.visible = false;
			scaleX = .35;
			scaleY = .35;
			//draw();
			line = new MovieClip()
			addChildAt(line, 3);
			tracks = new MovieClip()
			addChild(tracks);
		}

		public function clearTracks():void
		{
			//tracks.graphics.clear();
			tracks.graphics.lineStyle(1, 0x006666);
			tracks.graphics.moveTo(copter_mc.x, copter_mc.y);
			counter = 0;
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
