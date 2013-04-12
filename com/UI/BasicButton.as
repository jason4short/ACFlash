package com.UI {
	import flash.display.MovieClip;
	import flash.text.TextField;
    import flash.display.Stage;
	import flash.events.*;

	public class BasicButton extends MovieClip
	{
		public var label_TF			:TextField;
		public var pressed			:Boolean;
		public var over				:Boolean;
		private var EVENT_NAME		:String = "CLICK";
		private var isEnabled		:Boolean = true;
		public var _buttonArt		:MovieClip;

		public function BasicButton(label:String = "button!"){
            setLabel(label);
            stop();
			addEventListener(Event.ADDED_TO_STAGE, addedToStage);
		}

		public function addedToStage(event:Event):void {
			setEnabled();
			_buttonArt.stop();
		}

		public function setEventName(e:String):void
		{
			EVENT_NAME = e;
		}

		public function setLabel(label:String = "button"){
			if(label_TF != null)
            label_TF.text = label;
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
			_buttonArt.gotoAndStop(1);
		}


		public function mouseDownHandler(event:MouseEvent):void {
			stage.addEventListener(MouseEvent.MOUSE_UP, mouseUpHandler);
			pressed = true;
			_buttonArt.gotoAndStop(3);
		}


		public function mouseUpHandler(event:MouseEvent):void {
			pressed = false;
			if(over){
				_buttonArt.gotoAndStop(2);
				//trace("pressed!")
				stage.dispatchEvent(new Event(EVENT_NAME));
			}else{
				_buttonArt.gotoAndStop(1);
			}
		}

		public function mouseOutHandler(event:MouseEvent):void {
			_buttonArt.gotoAndStop(1);
			over = false;
		}

		public function mouseOverHandler(event:MouseEvent):void {
			_buttonArt.gotoAndStop(2);
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