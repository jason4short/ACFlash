﻿package com.libraries {

	public class AP_State {

		public var isArmed			  :Boolean = false;
        public var home_is_set        :Boolean = false; // 1
        public var simple_mode        :Boolean = false; // 2    // This is the state of simple mode
        public var manual_attitude    :Boolean = false; // 3
        public var manual_throttle    :Boolean = false; // 4

        public var low_battery        :Boolean = false; // 5    // Used to track if the battery is low - LED output flashes when the batt is low
        public var armed              :Boolean = false; // 6
        public var auto_armed         :Boolean = false; // 7

        public var failsafe_radio     :Boolean = false; // 8    // A status flag for the radio failsafe
        public var failsafe_batt      :Boolean = false; // 9    // A status flag for the battery failsafe
        public var failsafe_gps       :Boolean = false; // 10   // A status flag for the gps failsafe
        public var do_flip            :Boolean = false; // 11   // Used to enable flip code
        public var takeoff_complete   :Boolean = false; // 12
        public var land_complete      :Boolean = false; // 13
        public var compass_status     :Boolean = false; // 14
        public var gps_status         :Boolean = false; // 15

		public function AP_State()
		{
		}



		public function init():void
		{
			isArmed			   = false;
			home_is_set        = false;
			simple_mode        = false;
			manual_attitude    = false;
			manual_throttle    = false;

			low_battery        = false;
			armed              = false;
			auto_armed         = false;

			failsafe_radio     = false;
			failsafe_batt      = false;
			failsafe_gps       = false;
			do_flip            = false;
			takeoff_complete   = false;
			land_complete      = false;
			compass_status     = false;
			gps_status         = false;
		}


	}
}

