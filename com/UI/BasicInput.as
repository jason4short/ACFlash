package com.UI {
	import flash.display.MovieClip;
	import flash.text.TextField;
    import flash.display.Stage;
	import flash.events.*;

	public class BasicInput extends MovieClip
	{
		public var value_TF:TextField;
		public var value:Number;
		private var dirty:Boolean;

		public function BasicInput(){
            stop();
			addEventListener(Event.ADDED_TO_STAGE, addedToStage);
		}

		public function addedToStage(event:Event):void {
			addEventListener(Event.CHANGE, changeHandler);
		}

		public function setNumber(val:Number = 0){
			value = val;
			dirty = false;
			value_TF.text = val.toString();
		}

		public function isUpdated(val:Number = 0):Boolean
		{
			return dirty;
		}

		public function getNumber():Number
		{
			dirty = false;
			return Number(value_TF.text);
		}


        private function changeHandler(e:Event):void {
			dirty = true;
			stage.dispatchEvent(new Event("EditedGains"));
        }


	}
}
