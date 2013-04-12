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

	public class Joystick_XY extends MovieClip
	{
		// GUI
		public var pressed_x		:Boolean = false;
		public var pressed_y		:Boolean = false;
		public var pressed_pad		:Boolean = false;
		public var sticky_x			:Boolean = false;
		public var sticky_y			:Boolean = false;
		public var pwm_x			:int = 0;
		public var pwm_y			:int = 0;

		public function Joystick_XY()
		{
			addEventListener(Event.ADDED_TO_STAGE, addedToStage);
		}
		// ========================================================
		//  Flash glue
		// ========================================================
	    public function addedToStage(even:Event):void
		{
			knob_x.addEventListener(MouseEvent.MOUSE_DOWN, mouseDown_x);
			knob_y.addEventListener(MouseEvent.MOUSE_DOWN, mouseDown_y);
			trackpad.addEventListener(MouseEvent.MOUSE_DOWN, mouseDown_pad);
			addEventListener(Event.ENTER_FRAME, update);
		}

		private function update(e:Event)
		{
			if(pressed_x){
				knob_x.x = mouseX;
				knob_x.x = Math.min(knob_x.x, 100);
				knob_x.x = Math.max(knob_x.x, -100);
			}else if(pressed_y){
				knob_y.y = mouseY;
				knob_y.y = Math.min(knob_y.y, 100);
				knob_y.y = Math.max(knob_y.y, -100);
			}

			if (pressed_pad){
				knob_x.x = mouseX;
				knob_y.y = mouseY;
				knob_x.x = Math.min(knob_x.x, 100);
				knob_x.x = Math.max(knob_x.x, -100);
				knob_y.y = Math.min(knob_y.y, 100);
				knob_y.y = Math.max(knob_y.y, -100);
			}else{

				if(!pressed_x && !sticky_x){
					knob_x.x = knob_x.x / 2;
				}
				if(!pressed_y && !sticky_y){
					knob_y.y = knob_y.y / 2;
				}
			}

			pwm_x = 1500 + knob_x.x * 5;
			pwm_y = 1500 + knob_y.y * 5;
		}

		private function mouseDown_x(e:MouseEvent)
		{
			pressed_x = true;
			stage.addEventListener(MouseEvent.MOUSE_UP, mouseUp);
		}

		private function mouseDown_y(e:MouseEvent)
		{
			pressed_y = true;
			stage.addEventListener(MouseEvent.MOUSE_UP, mouseUp);
		}

		private function mouseDown_pad(e:MouseEvent)
		{
			pressed_pad = true;
			stage.addEventListener(MouseEvent.MOUSE_UP, mouseUp);
		}

		private function mouseUp(e:MouseEvent)
		{
			pressed_x = false;
			pressed_y = false;
			pressed_pad = false;
			stage.removeEventListener(MouseEvent.MOUSE_UP, mouseUp);
		}

	}
}
