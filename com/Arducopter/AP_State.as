// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

public function set_home_is_set(b:Boolean):void
{
	// if no change, exit immediately
	if( ap.home_is_set == b )
		return;

	ap.home_is_set 	= b;
}

// ---------------------------------------------
public function set_armed(b:Boolean):void
{
	// if no change, exit immediately
	if( ap.armed == b )
		return;

	ap.armed = b;
}

// ---------------------------------------------
public function set_auto_armed(b:Boolean):void
{
	// if no change, exit immediately
	if( ap.auto_armed == b )
		return;

	ap.auto_armed = b;
}

// ---------------------------------------------
public function set_simple_mode(b:Boolean):void
{
	if(ap.simple_mode != b){
		ap.simple_mode = b;
	}
}

// ---------------------------------------------
public function set_failsafe_radio(mode:Boolean):void
{
	// only act on changes
	// -------------------
	if(ap.failsafe_radio != mode) {

		// store the value so we don't trip the gate twice
		// -----------------------------------------------
		ap.failsafe_radio = mode;

		if (ap.failsafe_radio == false) {
			// We've regained radio contact
			// ----------------------------
			failsafe_radio_off_event();
		}else{
			// We've lost radio contact
			// ------------------------
			failsafe_radio_on_event();
		}
	}
}


// ---------------------------------------------
public function set_low_battery(b:Boolean):void
{
	ap.low_battery = b;
}


// ---------------------------------------------
public function set_failsafe_gps(b:Boolean)
{
	ap.failsafe_gps = b;
}

// ---------------------------------------------
public function set_takeoff_complete(b:Boolean):void
{
	// if no change, exit immediately
	if( ap.takeoff_complete == b )
		return;

	ap.takeoff_complete = b;
}

// ---------------------------------------------
public function set_land_complete(b:Boolean):void
{
	// if no change, exit immediately
	if( ap.land_complete == b )
		return;

	ap.land_complete = b;
}

// ---------------------------------------------


public function set_compass_healthy(b:Boolean):void
{
	if(ap.compass_status != b){
	}
	ap.compass_status = b;
}

public function set_gps_healthy(b:Boolean):void
{
	if(ap.gps_status != b){
	}
	ap.gps_status = b;
}

public function dump_state():void
{
	//cliSerial->printf("st: %u\n",value);
}


