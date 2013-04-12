package com.UI {
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


	public class PlotView extends MovieClip
	{
		public var plots			:Array
		public var plot_colors		:Array
		public var maxWidth			:Number = 0;
		public var frame			:Rectangle;
		public var dataScaleX		:Number = 1;
		public var dataScaleY		:Number = 1;
		public var lines			:int = 4;

		public function PlotView(){
			super();
			addEventListener(Event.ADDED_TO_STAGE, addedToStage);
			frame = new Rectangle(0,0,100,100);
			plots = new Array();
		}

		public function draw():void {
			// background
			graphics.clear();
			graphics.beginFill(0);
			graphics.drawRect(0, 0, frame.width, frame.height);
			graphics.endFill();

			// unit lines
            graphics.lineStyle(1, 0x222222);
            var n:int = (lines * 2)+2;
			var h:int = frame.height/n;

			for (var i:int = 0; i < n; i++){
				//if(i == 0)
				//	next;
				//if(i == lines-1)
				//	break;
	            graphics.moveTo(0, 				h * i);
    	        graphics.lineTo(frame.width, 	h * i);
    	    }

			// CenterLine
            graphics.lineStyle(1, 0x990000);
            graphics.moveTo(0, frame.height/2);
            graphics.lineTo(frame.width, frame.height/2);
		}

		public function addedToStage(event:Event):void {
			_preview.visible 	= false;
			this.frame.width 	= Math.round(this.width);
			this.frame.height 	= Math.round(this.height);
			scaleX = 1;
			scaleY = 1;
			draw();

		}

		public function addPlot(_name:String, _color:Number):int
		{
			var _index:Number = plots.length - 1;

			if(_index < 0)
				_index = 0;
			var p:Plot  = new Plot(_name, _color, maxWidth);
			p.y = Math.round(frame.height/2);
			plots.push(p);
			addChild(p);
			p.scaleX = dataScaleX;
			p.scaleY = dataScaleY;
			return plots.length - 1;
		}

		public function setScale(x:Number, y:Number):void
		{
			x = Math.max(x,  .001);
			y = Math.max(y, .001);

			dataScaleX = x;
			dataScaleY = y

			for(var i:int = 0; i < plots.length; i++){
				plots[i].scaleX = x
				plots[i].scaleY = y
			}
		}

		public function clearPlots():void
		{
			for(var i:int = 0; i < plots.length; i++){
				removeChild(plots[i]);
			}
			plots = new Array();
		}

		public function setValue(val:Number, plot_index:Number, _scale:Number = 1){
			//val = 0;
			plots[plot_index].set_value(-val, _scale);
		}
	}
}



