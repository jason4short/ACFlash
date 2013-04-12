/*
	RC_Channel.cpp - Radio library for Arduino
	Code by Jason Short. DIYDrones.com

	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

*/

package com.libraries {

	public class AP_RC extends Object
	{
		// input channels from PPM encoder
		private static var instance:AP_RC = null;

		public var pwm_channels:Array;
		public var output_pwm:Array;

		var state:int = 0;

		public function AP_RC()
		{
			pwm_channels = new Array(8);
			output_pwm = new Array(8);
		}

		//AP_RC.getInstance();
		static public function getInstance():AP_RC
		{
			if (instance == null)
			  instance = new AP_RC();
			return instance;
		}

		// set by SIM
	    public function set_PWM_channel(pwm:int, ch:int):void
		{
			pwm_channels[ch] = pwm;
			state = 1;
		}

		public function InputCh(ch:int):int
		{
			return  pwm_channels[ch];
		}

		public function enable_ch(ch:int):void
		{
			//return  pwm_channels[ch];
		}

		public function OutputCh(ch:int, pwm:int)
		{
			output_pwm[ch] = pwm;
		}

		public function get_motor_output(ch:int):int
		{
			return output_pwm[ch] - 1000;
		}

		public function getState():int
		{
			return state;
		}
	}
}
