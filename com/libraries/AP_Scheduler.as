package com {

	public class AP_Scheduler{

		private var	_debug					:Boolean = false;
		private var _tasks					:Object;

		private var	_num_tasks				:int;
		private var	_tick_counter			:int;
		private var	_last_run				:Array;
		private var	_task_time_allowed		:int;
		private var	_task_time_started		:int;


		public function AP_Scheduler() {

		}

	    // clear - removes all points from the curve
		public function init(tasks:Object, num_tasks:int):void
		{
			_tasks 		= tasks;
			_num_tasks 	= num_tasks;
			_last_run 	= new Array(_num_tasks);
			for(var i:int = 0; i<_num_tasks; i++){
				_last_run[i] = 0;
			}
		}

		public function tick():void
		{
		    _tick_counter++;
		}


		public function run(time_available:int)
		{
			for (var i:int = 0; i < _num_tasks; i++){

				var dt:int = _tick_counter - _last_run[i];
				var interval_ticks:int = _tasks[i].interval_ticks;

				if(dt >= interval_ticks){
					// this task is due to run. Do we have enough time to run it?
					_task_time_allowed = _tasks[i].max_time_micros;

					if(dt >= interval_ticks * 2){
						// we've slipped a whole run of this task!
						if (_debug != 0) {
							trace ("Scheduler slip task", i, dt, interval_ticks, _task_time_allowed)
						}
					}
					if (_task_time_allowed <= time_available) {
						// run it
						_task_time_started = clock.micros();

						// do callback;
						// myfunction()

						// record the tick counter when we ran. This drives
						// when we next run the event
						_last_run[i] = _tick_counter;

						// work out how long the event actually took
						var time_taken:int = clock.micros() - _task_time_started;

						if (time_taken > _task_time_allowed) {
							// the event overran!
							if (_debug != 0) {
								trace ("Scheduler overrun task", i, time_taken, _task_time_allowed)
							}
							return;
						}
						time_available -= time_taken;
					}
				}
			}
		}

		public function time_available_usec():void
		{
			var dt:int = clock.micros() - _task_time_started;
			if (dt > _task_time_allowed) {
				return 0;
			}
			return _task_time_allowed - dt;
		}


	}
}




