/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

public var log_counter_inav:int = 0;

// read_inertia - read inertia in from accelerometers
public function read_inertia():void
{
    //static uint8_t log_counter_inav = 0;

    // inertial altitude estimates
    inertial_nav.update(G_Dt);

    if( motors.armed() && g.log_bitmask & MASK_LOG_INAV ) {
        log_counter_inav++;
        if( log_counter_inav >= 10 ) {
            log_counter_inav = 0;
            //Log_Write_INAV();
        }
    }
}

// read_inertial_altitude - pull altitude and climb rate from inertial nav library
public function read_inertial_altitude():void
{
    // with inertial nav we can update the altitude and climb rate at 50hz
    current_loc.alt = inertial_nav.get_altitude();
    climb_rate = inertial_nav.get_velocity_z();
}