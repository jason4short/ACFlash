package com.libraries {

	public class Motors {

		public var isAutoArmed		:Boolean = false;
		public var isArmed			:Boolean = false;

		public function Motors() {
		}

		public function set auto_armed(a:Boolean):void
		{
			isAutoArmed = a;
		}

		public function get auto_armed():Boolean
		{
			return isAutoArmed;
		}


		public function set armed(a:Boolean):void
		{
			isArmed = a;
		}

		public function get armed():Boolean
		{
			return isArmed;
		}

	}
}

