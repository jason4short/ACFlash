package com.UI {
	import flash.display.MovieClip;
	import flash.text.TextField;
    import flash.display.Stage;
	import flash.events.*;

	public class QuickCheckBox extends MovieClip
	{
		public var label_TF:TextField;
		public var pressed:Boolean;
		public var over:Boolean;
		public var EVENT_NAME:String = "CheckBox";
		private var isEnabled:Boolean = true;
		private var _selected:Boolean = false;

		public function QuickCheckBox(label:String = "Checkbox!"){
            setLabel(label);
            stop();
			addEventListener(Event.ADDED_TO_STAGE, addedToStage);
			//hitArea
		}

		public function addedToStage(event:Event):void {
			setEnabled();
			draw();
		}

		public function setLabel(label:String = "button"){
            label_TF.text = label;
		}

		public function draw(){
			if (_selected)
				gotoAndStop(2);
			else
				gotoAndStop(1);
		}

		public function getSelected():Boolean
		{
			return _selected;
		}

		public function setSelected(s:Boolean):void
		{
			_selected = s;
			draw();
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
			gotoAndStop(1);
		}


		public function mouseDownHandler(event:MouseEvent):void {
			stage.addEventListener(MouseEvent.MOUSE_UP, mouseUpHandler);
			pressed = true;
		}


		public function mouseUpHandler(event:MouseEvent):void {
			pressed = false;
			if(over){
				_selected = !_selected;
				//trace("pressed!")
				stage.dispatchEvent(new Event(EVENT_NAME));
				draw();
			}
		}

		public function mouseOutHandler(event:MouseEvent):void {
			over = false;
		}

		public function mouseOverHandler(event:MouseEvent):void {
			over = true;
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