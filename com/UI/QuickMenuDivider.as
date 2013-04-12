package com.UI {
	import flash.display.MovieClip;
	import flash.text.TextField;
    import flash.display.Stage;
	import flash.events.*;
	import com.UI.QuickMenu;
    import flash.geom.Rectangle;

	public class QuickMenuDivider extends MovieClip
	{
		private var isEnabled:Boolean = false;

		public var index:int;
		public var frame:Rectangle;
		public var menu:QuickMenu;
		public var item_height:int = 9;

		public function QuickMenuDivider(){
            stop();
			addEventListener(Event.ADDED_TO_STAGE, addedToStage);
			frame = new Rectangle(0,0,300,300);
		}

		public function addedToStage(event:Event):void {
			draw();
		}

		public function getLabel():String
		{
			return "div";
		}

		public function draw(){
			graphics.clear();
			graphics.lineStyle(1, 0xCCCCCC);
            graphics.moveTo(0,10);
            graphics.lineTo(frame.width, 10);
		}
	}
}




/*		public function setXY(xVal:Number, yVal:Number){
			//if(maxWidth == _index){
			//	graphics.clear();
			//	_index = 0;
			//}
            graphics.lineStyle(1, _color);
            graphics.moveTo(_index-1, last_value);
            graphics.lineTo(_index, yVal);
			last_value = yVal;
			_index++;
			label_TF.x = _index;
			label_TF.y = val;
		}
*/