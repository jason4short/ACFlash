package com.Sim {

	public class Clock
	{
		private static var instance			:Clock = null;
		public var elapsed					:int = 0;

		public function Clock():void
		{

		}

		//Clock.getInstance();
		static public function getInstance():Clock
		{
			if (instance == null)
				instance = new Clock();
			return instance;
		}



		public function add_millis(n:int):void
		{
			elapsed += n;
		}

		public function millis():int
		{
			return elapsed;
		}

		// called at startup to fill in values for defaults
		public function micros():int
		{
			return elapsed * 1000;
		}

		// called at startup to fill in values for defaults
		public function delay(n:int):void
		{
			elapsed += n;
		}

	}
}
