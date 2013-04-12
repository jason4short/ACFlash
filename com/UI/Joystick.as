/*
	RC_Channel.cpp - Radio library for Arduino
	Code by Jason Short. DIYDrones.com

	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

*/

package com.UI {
	import flash.display.MovieClip;
    import flash.events.*;

	public class Joystick extends MovieClip
	{
		// GUI
		public var pressed			:Boolean = false;
		public var sticky			:Boolean = false;
		public var pwm				:int = 0;

		public function Joystick()
		{
			addEventListener(Event.ADDED_TO_STAGE, addedToStage);
		}
		// ========================================================
		//  Flash glue
		// ========================================================
	    public function addedToStage(even:Event):void
		{
			knob.addEventListener(MouseEvent.MOUSE_UP, mouseUp);
			knob.addEventListener(MouseEvent.MOUSE_DOWN, mouseDown);
			addEventListener(Event.ENTER_FRAME, update);
		}

		private function update(e:Event)
		{
			if(pressed){
				knob.x = mouseX;
				knob.x = Math.min(knob.x, 100);
				knob.x = Math.max(knob.x, -100);
			}else{
				if(sticky == false)
					knob.x = knob.x /2;
			}
			pwm = 1500 + knob.x * 5;
			//trace(pwm, )
		}

		private function mouseDown(e:MouseEvent)
		{
			pressed = true;
			stage.addEventListener(MouseEvent.MOUSE_UP, mouseUp);
		}

		private function mouseUp(e:MouseEvent)
		{
			pressed = false;
			stage.removeEventListener(MouseEvent.MOUSE_UP, mouseUp);
		}

	}
}
