package com.UI {
	import flash.display.MovieClip;
	import flash.text.TextField;
    import flash.display.Stage;
	import flash.events.*;
	import com.UI.QuickMenu;
	import com.UI.QuickMenuItem;
	import com.UI.QuickMenuDivider;

    import flash.geom.Rectangle;

	public class QuickPopupMenu extends MovieClip
	{
		public var label_TF:TextField;
		public var pressed:Boolean;
		public var over:Boolean;
		//public var EVENT_NAME:String = "CLICK";
		private var isEnabled:Boolean = true;
		public var menu:QuickMenu;
		public var hasInited:Boolean = false;
		public var _buttonArt		:MovieClip;


		public function QuickPopupMenu(){
            stop();
			addEventListener(Event.ADDED_TO_STAGE, addedToStage);
			menu = new QuickMenu();
			addChild(menu);
			menu.y = -390;
			menu.visible = false;
		}

		public function addedToStage(event:Event):void {
			setEnabled();
			getSelectedItem();
			_buttonArt.stop();
		}

		public function setEventName(e:String):void
		{
			menu.setEventName(e);
		}

		public function addDivider(item:QuickMenuDivider){
			menu.addDivider(item);
			menu.y = -menu.frame.height;
		}

		public function addItem(item:QuickMenuItem){
			menu.addItem(item);
			if (hasInited == false){
				getSelectedItem();
				hasInited = true;
			}
			menu.y = -menu.frame.height;
		}

		public function setSelectedIndex(n:int)
		{
			menu.setSelectedIndex(n);
			setLabel(menu.getSelectedItem()._label);
		}

		public function getSelectedItem():QuickMenuItem
		{
			if(menu.hasItems()){
				//trace("jason ", menu.getSelectedItem()._label)
				setLabel(menu.getSelectedItem()._label);
				//trace(menu.current_index.getCode());
				return menu.getSelectedItem();
			}else
				return null;
		}

		public function setSelectedItemByName(s:String)
		{
			menu.setSelectedItemByName(s);
			setLabel(menu.getSelectedItem()._label);
		}

		public function setSelectedItemByCode(s:String)
		{
			menu.setSelectedItemByCode(s);
			setLabel(menu.getSelectedItem()._label);
		}

		public function getSelectedIndex():Number
		{
			return menu.current_index;
		}

		public function setLabel(label:String = "button"){
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
			menu.visible = true;
			stage.addEventListener(MouseEvent.MOUSE_UP, mouseUpHandler);
			pressed = true;
			_buttonArt.gotoAndStop(3);
		}


		public function mouseUpHandler(event:MouseEvent):void {
			//trace("pop")
			pressed = false;
			//if(this.contains(menu))
			//	removeChild(menu);
			menu.visible = false;

			if(over){
				_buttonArt.gotoAndStop(2);
				//trace("pressed!")
				//stage.dispatchEvent(new Event(EVENT_NAME));
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