package com.UI {
	import flash.display.MovieClip;
	import flash.text.TextField;
    import flash.display.Stage;
	import flash.events.*;
	import com.UI.QuickMenu;
	import com.UI.QuickMenuItem;
	import com.UI.QuickMenuDivider;

    import flash.geom.Rectangle;

	public class QuickMenu extends MovieClip
	{
		private var EVENT_NAME		:String = "QuickMenu";
		public var menu				:QuickMenu;
		public var frame			:Rectangle;

		//private var item_height:int = 18;
		private var items:Array;

		//public var pressed_item:QuickMenuItem;
		public var current_index:int = -1;

		public function QuickMenu(){
            stop();
			addEventListener(Event.ADDED_TO_STAGE, addedToStage);
			items = new Array();
			frame = new Rectangle(0,0,150,300);
			mouseEnabled = false;
		}

		public function addedToStage(event:Event):void {
			//setEnabled();
		}

		public function setEventName(e:String):void
		{
			EVENT_NAME = e;
		}

		public function addDivider(item:QuickMenuDivider){
        	items.push(item);
        	item.menu = this;
        	item.index = items.length -1;
			//trace("addItem", items.length)
			item.frame.width = frame.width;

        	this.addChild(item);
			if(current_index < 0){
				current_index = 0;
			}
        	updateLayout();
		}

		public function addItem(item:QuickMenuItem){
			//trace("addItem", items)
        	items.push(item);
        	item.menu = this;
        	item.index = items.length -1;
			item.frame.width = frame.width;

			//trace("addItem", items.length)
        	this.addChild(item);

			if(current_index < 0){
				current_index = 0;
			}
        	updateLayout();
		}

		public function setWidth(n:int){
			this.frame.width = n;
			updateLayout();
		}

		public function didPress(m:QuickMenuItem)
		{
			//trace("didPress ",m.getCode());
			current_index = m.index;
			//pressed_item = m;
			stage.dispatchEvent(new Event(EVENT_NAME));
		}

		public function getSelectedItem():QuickMenuItem
		{
			if(current_index >=0)
				return items[current_index];
			else
				return null;
		}

		public function setSelectedIndex(n:int)
		{
			if (n > (items.length -1))
				n = items.length-1;
			if (n < 0)
				n = 0;

			current_index = n;
		}

		public function setSelectedItemByName(s:String)
		{
			for(var i:int=1; i <items.length; i++){
				if (items[i].getLabel() == s){
					setSelectedIndex(items[i].index);
				}
			}
		}

		public function setSelectedItemByCode(s:String)
		{
			for(var i:int=1; i <items.length; i++){
				if (items[i].getCode() == s){
					setSelectedIndex(items[i].index);
				}
			}
		}



		public function hasItems():Boolean
		{
			return (items.length > 0);
		}

		private function updateLayout():void
		{
			// setup first menu item
			items[0].x = 0;
			items[0].y = 0;
			items[0].frame.width = this.frame.width;
			this.frame.height = items[0].item_height;

			for(var i:int=1; i <items.length; i++){
				items[i].x = 0;
				items[i].y = items[i-1].y + items[i].item_height;
				items[i].frame.width = this.frame.width;
				this.frame.height = items[i].y + items[i].item_height;
			}
			/*
			//trace("updateLayout")
			for(var i:int=0; i <items.length; i++){
				items[i].x = 0;
				items[i].y = items[i-1].item_height * i;
				//items[i].y = items[i-1].item_height * i;
				//trace(item_height * i)
				items[i].frame.width = this.frame.width;
				items[i].frame.height = items[i].item_height;
				items[i].updateLayout();
				this.frame.height = items[i].item_height * i + items[i].item_height;
			}
			*/

			graphics.clear();
			graphics.beginFill(0xffffff);
			graphics.drawRect(0, 0, frame.width, frame.height);
			graphics.endFill();
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