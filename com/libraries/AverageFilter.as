package com.libraries {
	public class AverageFilter
	{
		public var samples:Array;
		public var filter_size:int = 0;
		public var sample_index:int = 0;
		public var first_use:Boolean = true;

		public function AverageFilter(_size:int = 4){
			samples = new Array(_size);
			filter_size = _size;

			// init
			for (var i:int = 0; i < _size; i++){
				samples[i] = 0;
			}
		}

		public function init():void
		{
			force_sample(0);
		}

		public function force_sample(_sample:Number):void
		{
			for(var i:int = 0; i < filter_size; i++){
				samples[i] = _sample;
			}
		}

		public function apply(_sample:Number):Number
		{
			if(filter_size <= 1)
				return _sample;

			if(first_use){
				force_sample(_sample);
				first_use = false;
			}

			samples[sample_index] = _sample;
			sample_index++;

			if (sample_index >= filter_size)
				sample_index = 0;

			return average_filter();
		}


		private function average_filter():Number
		{
			var sum:Number = 0;

			for(var i:int = 0; i < filter_size; i++){
				sum += samples[i];
			}
			return sum/filter_size;
		}

	}
}



