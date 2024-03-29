﻿package com.libraries {

	public class AP_System {

		public var  GPS_light           :Boolean = false; // 1   // Solid indicates we have full 3D lock and can navigate, flash = read
		public var  motor_light         :Boolean = false; // 2   // Solid indicates Armed state
		public var  new_radio_frame     :Boolean = false; // 3   // Set true if we have new PWM data to act on from the Radio
		public var  nav_ok              :Boolean = false; // 4   // deprecated
		public var  CH7_flag            :Boolean = false; // 5   // manages state of the ch7 toggle switch
		public var  usb_connected       :Boolean = false; // 6   // true if APM is powered from USB connection
		public var  alt_sensor_flag     :Boolean = false; // 7   // used to track when to read sensors vs estimate alt
		public var  yaw_stopped         :Boolean = false; // 8   // Used to manage the Yaw hold capabilities

		public function AP_System()
		{
		}

		public function init():void
		{
			GPS_light            = false;
			motor_light          = false;
			new_radio_frame      = false;
			nav_ok               = false;
			CH7_flag             = false;
			usb_connected        = false;
			alt_sensor_flag      = false;
			yaw_stopped          = false;
		}

	}
}


