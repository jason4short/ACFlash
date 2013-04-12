package com.UI {
	import flash.display.MovieClip;
	import flash.text.TextField;
    import flash.display.Stage;
	import flash.events.*;
	import com.UI.QuickMenu;
    import flash.geom.Rectangle;

	public class QuickMenuItem extends MovieClip
	{
		public var label_TF:TextField;
		public var pressed:Boolean;
		public var over:Boolean;
		//public var EVENT_NAME:String = "CLICK";
		private var isEnabled:Boolean = true;

		public var _label		:String;
		private var _code		:String;
		public var index		:int;
		public var frame:Rectangle;
		public var menu:QuickMenu;
		public var item_height:int = 14;

		public function QuickMenuItem(label:String="menuItem", code:String="1"){
            stop();
			addEventListener(Event.ADDED_TO_STAGE, addedToStage);
			_label = label;
			_code = code;
			frame = new Rectangle(0,0,300,item_height);
		}

		public function addedToStage(event:Event):void {
			setEnabled();
			setLabel(_label);
		}

		public function setLabel(label:String):void
		{
			_label = label;
			label_TF.text = _label;
		}
		public function getLabel():String
		{
			return _label;
		}

		public function getCode():String
		{
			return _code;
		}

		public function updateLayout():void
		{
			label_TF.height = frame.height;
			label_TF.width = frame.width;
		}

		public function draw(){
			//trace(over);
			graphics.clear();
			if(over){
				graphics.beginFill(0x83D5F5);
				graphics.drawRect(0, 0, frame.width, frame.height);
				graphics.endFill();
			}
		}

		public function setEnabled():void
		{
			addEventListener(MouseEvent.MOUSE_DOWN, mouseDownHandler);
			addEventListener(MouseEvent.MOUSE_OUT, mouseOutHandler);
			addEventListener(MouseEvent.MOUSE_OVER, mouseOverHandler);
			isEnabled = true;
			alpha = 1.0;
		}

		public function setDisabled():void
		{
			removeEventListener(MouseEvent.MOUSE_DOWN, mouseDownHandler);
			removeEventListener(MouseEvent.MOUSE_OUT, mouseOutHandler);
			removeEventListener(MouseEvent.MOUSE_OVER, mouseOverHandler);
			isEnabled = false;
			alpha = .5;
			draw();
		}


		public function mouseDownHandler(event:MouseEvent):void {
			pressed = true;
			//trace("pressed")
			//visible = false
			draw();
		}


		public function mouseUpHandler(event:MouseEvent):void {
			stage.removeEventListener(MouseEvent.MOUSE_UP, mouseUpHandler);
			//trace("up" + this._label)
			pressed = false;
			if(over){
				draw();
				//trace("pressed!")
				//stage.dispatchEvent(new Event(EVENT_NAME));
				menu.didPress(this);
			}else{
				draw();
			}
		}

		public function mouseOutHandler(event:MouseEvent):void {
			over = false;
			draw();
		}

		public function mouseOverHandler(event:MouseEvent):void {
			stage.addEventListener(MouseEvent.MOUSE_UP, mouseUpHandler);
			over = true;
			draw();
			//trace(this._label)

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