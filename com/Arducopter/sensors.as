﻿// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

public function ReadSCP1000():void
{
}

public function init_sonar():void
{
}

public function init_barometer():void
{
    //barometer.calibrate();
    barometer
    ahrs.set_barometer(barometer);
    trace("barometer calibration complete");
}

// return barometric altitude in centimeters
public function read_barometer():int
{
    //barometer.read();
    //return barometer.get_altitude() * 100.0;
	return 0;
}

// return sonar altitude in centimeters
public function read_sonar():int
{
    // exit immediately if sonar is disabled
    if( !g.sonar_enabled ) {
        sonar_alt_health = 0;
        return 0;
    }

    var temp_alt:int = sonar.read();

    if (temp_alt >= sonar.min_distance && temp_alt <= sonar.max_distance * 0.70) {
        if ( sonar_alt_health < SONAR_ALT_HEALTH_MAX ) {
            sonar_alt_health++;
        }
    }else{
        sonar_alt_health = 0;
    }

    return temp_alt;
}

public function init_compass():void
{
}

public function init_optflow():void
{
}

// read_battery - check battery voltage and current and invoke failsafe if necessary
// called at 10hz
public const BATTERY_FS_COUNTER:int = 100;     // 100 iterations at 10hz is 10 seconds
public function read_battery():void
{
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
public function read_receiver_rssi():void
{
    //rssi_analog_source->set_pin(g.rssi_pin);
    //float ret = rssi_analog_source->read_latest();
    //receiver_rssi = constrain(ret, 0, 255);
}
