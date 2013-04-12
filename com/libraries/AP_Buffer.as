package com.libraries
{
	public class AP_Buffer
	{

	    private var _num_items	:int;       // number of items in the buffer
	    private var _head		:int;       // first item in the buffer (will be returned with the next get call)
		private var _buff		:Array; 	// x values of each point on the curve

		private var SIZE		:int = 0;

		public function AP_Buffer(_bufferSize) {
			_num_items = 0;
			_head = 0;
			SIZE = _bufferSize;
			_buff 	= new Array(SIZE);
		}

	    // clear - removes all points from the curve
		public function clear():void
		{
			// clear the curve
			_num_items = 0;
			_head = 0;
		}

		public function num_items():int
		{
			return _num_items;
		}

		public function add(item:Number):Boolean
		{
			// determine position of new item
			var tail:int = _head + _num_items;
			if( tail >= SIZE ) {
				tail -= SIZE;
			}

			// add item to buffer
			_buff[tail] = item;

			// increment number of items
			if( _num_items < SIZE ) {
				_num_items++;
			}else{
				// no room for new items so drop oldest item
				_head++;
				if( _head >= SIZE ) {
					_head = 0;
				}
			}

			// indicate success
			return true;
		}


	    // add - adds an item to the buffer.  returns TRUE if successfully added
		public function get():Number
		{
			var result:Number;

			// return zero if buffer is empty
			if( _num_items == 0 ) {
				return 0;
			}

			// get next value in buffer
			result = _buff[_head];

			// increment to next point
			_head++;
			if( _head >= SIZE )
				_head = 0;

			// reduce number of items
			_num_items--;

			// return item
			return result;
	    }

		// peek - check what the next value in the buffer is but don't pull it off
		public function peek(position:int = 0):Number
		{
			var j:int = _head + position;

			// return zero if position is out of range
			if(position >= _num_items){
				return 0;
			}

			// wrap around if necessary
			if(j >= SIZE)
				j -= SIZE;

			// return desired value
			return _buff[j];
	    }
	}
}




