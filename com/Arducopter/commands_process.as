/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// For changing active command mid-mission
//----------------------------------------
public function change_command(cmd_index:int):void
{
    //cliSerial->printf("change_command: %d\n",cmd_index );
	// limit range
	cmd_index = Math.min(g.command_total - 1, cmd_index);

	// load command
	var temp:Location  = get_cmd_with_index(cmd_index);

    //cliSerial->printf("loading cmd: %d with id:%d\n", cmd_index, temp.id);

	// verify it's a nav command
    if(temp.id > MAV_CMD_NAV_LAST) {
		//gcs_send_text_P(SEVERITY_LOW,PSTR("error: non-Nav cmd"));

    }else{
		// clear out command queue
		init_commands();

		// copy command to the queue
		command_nav_queue		= temp;
		command_nav_index 		= cmd_index;
		execute_nav_command();
	}
}

// update_commands - initiates new navigation commands if we have completed the previous command
// called by 10 Hz loop
// --------------------
public function update_commands():void
{
    //cliSerial->printf("update_commands: %d\n",increment );
	// A: if we do not have any commands there is nothing to do
	// B: We have completed the mission, don't redo the mission
	// XXX debug
	//uint8_t tmp = g.command_index;
    //cliSerial->printf("command_index %u \n", tmp);

	if(g.command_total <= 1 || g.command_index >= 255)
		return;

	if(command_nav_queue.id == NO_COMMAND){
		trace('Our queue is empty' ,command_nav_index);
		// Our queue is empty
		// fill command queue with a new command if available, or exit mission
		// -------------------------------------------------------------------

		// find next nav command
		var tmp_index:int;

		if(command_nav_index < g.command_total){

			// what is the next index for a nav command?
			tmp_index = find_next_nav_index(command_nav_index + 1);

			if(tmp_index == -1){
				exit_mission();
				return;
			}else{
				trace("found next nav command", tmp_index);
				command_nav_index = tmp_index;
				command_nav_queue = get_cmd_with_index(command_nav_index);
				execute_nav_command();
			}

		}else{
			// we are out of commands
			exit_mission();
			return;
		}
	}

    if(command_cond_queue.id == NO_COMMAND) {
		// Our queue is empty
		// fill command queue with a new command if available, or do nothing
		// -------------------------------------------------------------------

		// no nav commands completed yet
        if(prev_nav_index == NO_COMMAND)
			return;

        if(command_cond_index >= command_nav_index) {
			// don't process the fututre
			return;

        }else if(command_cond_index == NO_COMMAND) {
			// start from scratch
			// look at command after the most recent completed nav
			command_cond_index = prev_nav_index + 1;

		}else{
			// we've completed 1 cond, look at next command for another
			command_cond_index++;
		}

        if(command_cond_index < (g.command_total -2)) {
			// we're OK to load a new command (last command must be a nav command)
			command_cond_queue = get_cmd_with_index(command_cond_index);

            if(command_cond_queue.id > MAV_CMD_CONDITION_LAST) {
				// this is a do now command
				process_now_command();

				// clear command queue
				command_cond_queue.id = NO_COMMAND;

            }else if(command_cond_queue.id > MAV_CMD_NAV_LAST) {
				// this is a conditional command
				process_cond_command();

			}else{
				// this is a nav command, don't process
				// clear the command conditional queue and index
				prev_nav_index			= NO_COMMAND;
				command_cond_index 		= NO_COMMAND;
				command_cond_queue.id 	= NO_COMMAND;
			}

		}
	}
}

// execute_nav_command - performs minor initialisation and logging before next navigation command in the queue is executed
public function execute_nav_command():void
{
	// This is what we report to MAVLINK
	g.command_index  = command_nav_index;

	// Save CMD to Log
	//if (g.log_bitmask & MASK_LOG_CMD)
	Log_Write_Cmd(g.command_index, command_nav_queue);

	// clear navigation prameters
	reset_nav_params();

	// Act on the new command
	process_nav_command();

	// clear May indexes to force loading of more commands
	// existing May commands are tossed.
	command_cond_index	= NO_COMMAND;
}

// verify_commands - high level function to check if navigation and conditional commands have completed
// called after GPS navigation update - not constantly
public function verify_commands():void
{
	if(verify_must()){
        //cliSerial->printf("verified must cmd %d\n" , command_nav_index);
		trace("verified must cmd", command_nav_index);
		command_nav_queue.id 	= NO_COMMAND;

		// store our most recent executed nav command
		prev_nav_index 			= command_nav_index;

		// Wipe existing conditionals
		command_cond_index 		= NO_COMMAND;
		command_cond_queue.id 	= NO_COMMAND;

	}else{
        //cliSerial->printf("verified must false %d\n" , command_nav_index);
	}

    if(verify_may()) {
        //cliSerial->printf("verified may cmd %d\n" , command_cond_index);
		command_cond_queue.id = NO_COMMAND;
	}
}

// Finds the next navgation command in EEPROM
public function find_next_nav_index(search_index:int):int
{
	var tmp:Location;
	while(search_index < g.command_total){
		tmp = get_cmd_with_index(search_index);
        if(tmp.id <= MAV_CMD_NAV_LAST) {
			return search_index;
		}else{
			search_index++;
		}
	}
	return -1;
}


public function exit_mission():void
{
	trace("exit_mission");
	// we are out of commands
	g.command_index = 255;

	// if we are on the ground, enter stabilize, else Land
    if(ap.land_complete) {
		// we will disarm the motors after landing.
    }else{
		// If the approach altitude is valid (above 1m), do approach, else land
        if(g.rtl_alt_final == 0) {
			set_mode(LAND);
		}else{
			set_mode(LOITER);
            wp_nav.set_desired_alt(g.rtl_alt_final);
		}
	}

}

