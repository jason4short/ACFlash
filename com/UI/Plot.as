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
	import flash.text.TextField;
	import flash.text.TextFieldAutoSize;
    import flash.text.TextFormat;

	public class Plot extends MovieClip
	{
		var bmp:BitmapData;
		var bm:Bitmap;
		var last_value:Number = 0;
		var _color:Number;
		var _index:Number = 0;
		var _name:String;
		var index:Number;
		var label_TF:TextField
		var maxWidth:Number;

		public function Plot(name_:String, color_:Number, _width){
			super();
			//addEventListener(Event.ADDED_TO_STAGE, addedToStage);
			_color = color_;
			_name = name_;
			maxWidth = _width;
		}

		public function addedToStage(event:Event):void {
		}

		public function set_value(val:Number, _scale:Number = 1){
			val *= _scale;
			if(_index == 0)
				last_value = val;

            graphics.lineStyle(.25, _color);
            graphics.moveTo(_index-1, last_value);
            graphics.lineTo(_index, val);
			last_value = val;
			_index++;
		}
	}
}
