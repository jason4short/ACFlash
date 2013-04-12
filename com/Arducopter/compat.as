public static function formatTime(time:Number):String
{
	time /=1000;
	var remainder:Number;

	var hours:Number = time / ( 60 * 60 );

	remainder = hours - (Math.floor ( hours ));

	hours = Math.floor ( hours );

	var minutes = remainder * 60;

	remainder = minutes - (Math.floor ( minutes ));

	minutes = Math.floor ( minutes );

	var seconds = remainder * 60;

	remainder = seconds - (Math.floor ( seconds ));

	seconds = Math.floor ( seconds );

	var hString:String = hours < 10 ? "0" + hours : "" + hours;
	var mString:String = minutes < 10 ? "0" + minutes : "" + minutes;
	var sString:String = seconds < 10 ? "0" + seconds : "" + seconds;

	if ( time < 0 || isNaN(time)) return "00:00";

	if ( hours > 0 )
	{
		return hString + ":" + mString + ":" + sString;
	}else{
		return mString + ":" + sString;
	}
}

public function safe_sqrt(v:Number):Number
{
	var ret:Number = Math.sqrt(v);
	if (isNaN(ret)) {
		return 0;
	}
	return ret;
}

public function constrain(val:Number, min:Number, max:Number):Number
{
	val = Math.max(val, min);
	val = Math.min(val, max);
	return val;
}

public function radians(n:Number):Number
{
	return 0.0174532925 * n;
}

public function radiansx100(n:Number):Number
{
	return 0.000174532925 * n;
}

public function degrees(radians:Number):Number
{
	return radians * 180/Math.PI
}




