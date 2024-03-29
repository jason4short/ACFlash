﻿// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

public function init_commands():void
{
	g.command_index 		= NO_COMMAND;
	command_nav_index		= NO_COMMAND;
	command_cond_index		= NO_COMMAND;
	prev_nav_index 			= NO_COMMAND;
	command_cond_queue.id 	= NO_COMMAND;
	command_nav_queue.id 	= NO_COMMAND;
}

// Getters
// -------
public function get_cmd_with_index(i:int):Location
{
	var temp:Location = new Location();

	// Find out proper location in memory by using the start_byte position + the index
	// --------------------------------------------------------------------------------
	if (i >= g.command_total) {
		// we do not have a valid command to load
		// return a WP with a "Blank" id
		temp.id = CMD_BLANK;

		// no reason to carry on
		return temp;

	}else{
		// we can load a command, we don't process it yet
		// read WP position

		temp = waypoints[i].clone();

	}

    // Add on home altitude if we are a nav command (or other command with altitude) and stored alt is relative
    //if((temp.id < MAV_CMD_NAV_LAST || temp.id == MAV_CMD_CONDITION_CHANGE_ALT) && temp.options & MASK_OPTIONS_RELATIVE_ALT){
    //temp.alt += home.alt;
    //}

    if(temp.options & WP_OPTION_RELATIVE) {
		// If were relative, just offset from home
		temp.lat	+=	home.lat;
		temp.lng	+=	home.lng;
	}

	return temp;
}

// Setters
// -------
public function set_cmd_with_index(temp:Location, i:int):void
{

	i = constrain(i, 0, g.command_total);
	//Serial.printf("set_command: %d with id: %d\n", i, temp.id);
	//trace("set_command: "+i+" with id: "+ temp.id);
	//report_wp();

	// store home as 0 altitude!!!
	// Home is always a MAV_CMD_NAV_WAYPOINT (16)
    if (i == 0) {
		temp.alt = 0;
		temp.id = MAV_CMD_NAV_WAYPOINT;
	}

	waypoints[i] = temp.clone();

	// Make sure our WP_total
	if(g.command_total < (i+1))
		g.command_total = i + 1;
}

public function get_RTL_alt():int
{
    if(g.rtl_altitude <= 0) {
		return Math.min(current_loc.alt, RTL_ALT_MAX);
    }else if (g.rtl_altitude < current_loc.alt) {
		return Math.min(current_loc.alt, RTL_ALT_MAX);
	}else{
        return g.rtl_altitude;
	}
}

// run this at setup on the ground
// -------------------------------
public function init_home():void
{
    set_home_is_set(true);
	home.id 	= MAV_CMD_NAV_WAYPOINT;
	home.lng 	= 0; //g_gps.longitude;				// Lon * 10**7
	home.lat 	= 0; //g_gps.latitude;				// Lat * 10**7
	home.alt 	= 0;							// Home is always 0

	// Save Home to EEPROM
	// -------------------
	// no need to save this to EPROM
	set_cmd_with_index(home, 0);

    // set inertial nav's home position
    inertial_nav.set_current_position(g_gps.longitude, g_gps.latitude);

    //if (g.log_bitmask & MASK_LOG_CMD)
    //    Log_Write_Cmd(0, home);

    // update navigation scalers.  used to offset the shrinking longitude as we go towards the poles
    scaleLongDown = longitude_scale(home);
    scaleLongUp   = 1.0/scaleLongDown;
}



