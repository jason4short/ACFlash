// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
public const AP_INTERTIALNAV_GRAVITY:Number		= 9.80665;

// update_navigation - invokes navigation routines
// called at 10hz
public var nav_last_update:int = 0;        // the system time of the last time nav was run update
public function update_navigation():void
{

    // check for inertial nav updates
    if( inertial_nav.position_ok() ) {

        // calculate time since nav controllers last ran
        dTnav = Number((clock.millis() - nav_last_update))/ 1000.0;
        nav_last_update = clock.millis();

        // prevent runnup in dTnav value
        dTnav = Math.min(dTnav, 1.0);

        // run the navigation controllers
        update_nav_mode();

        // update log
        if (g.log_bitmask & MASK_LOG_NTUN && motors.armed()) {
            Log_Write_Nav_Tuning();
        }
    }
}

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
public function run_nav_updates():void
{
    // fetch position from inertial navigation
    calc_position();

    // calculate distance and bearing for reporting and autopilot decisions
    calc_distance_and_bearing();

    // run autopilot to make high level decisions about control modes
    run_autopilot();
}

// calc_position - get lat and lon positions from inertial nav library
public function calc_position():void
{
    if( inertial_nav.position_ok() ) {
        // pull position from interial nav library
        current_loc.lng = inertial_nav.get_longitude();
        current_loc.lat = inertial_nav.get_latitude();
    }
}

// calc_distance_and_bearing - calculate distance and direction to waypoints for reporting and autopilot decisions
public function calc_distance_and_bearing():void
{
    // get current position in CM
    var curr:Vector3D = inertial_nav.get_position();
	
    // get target from loiter or wpinav controller
    if( nav_mode == NAV_LOITER || nav_mode == NAV_CIRCLE ) {
        wp_distance = wp_nav.get_distance_to_target();
        wp_bearing = wp_nav.get_bearing_to_target();
    }else if( nav_mode == NAV_WP ) {
        wp_distance = wp_nav.get_distance_to_destination();
        wp_bearing = wp_nav.get_bearing_to_destination();
    }else{
        wp_distance = 0;
        wp_bearing = 0;
    }

    // calculate home distance and bearing
    if(ap.home_is_set){
        home_distance = safe_sqrt(curr.x*curr.x + curr.y*curr.y);
        // To-Do: change this to more efficient calculation
        home_bearing = pv_get_bearing_cd(curr,new Vector3D(0,0,0));

        // update super simple bearing (if required) because it relies on home_bearing
        update_super_simple_bearing();
    }else{
        home_distance = 0;
        home_bearing = 0;
    }
}

// run_autopilot - highest level call to process mission commands
public function run_autopilot():void
{
    switch( control_mode ) {
        case AUTO:
            // majority of command logic is in commands_logic.pde
            verify_commands();
            break;
        case GUIDED:
            // no need to do anything - wp_nav should take care of getting us to the desired location
            break;
        case RTL:
            verify_RTL();
            break;
    }
}

// set_nav_mode - update nav mode and initialise any variables as required
public function set_nav_mode(new_nav_mode:int):Boolean
{
    // boolean to ensure proper initialisation of nav modes
    var nav_initialised:Boolean = false;

    // return immediately if no change
    if( new_nav_mode == nav_mode ) {
        return true;
    }

    switch( new_nav_mode ) {

        case NAV_NONE:
        	trace("set_nav_mode, NAV_NONE")
            nav_initialised = true;
            break;

        case NAV_CIRCLE:
        	trace("set_nav_mode, NAV_CIRCLE")
            // set center of circle to current position
            circle_set_center(inertial_nav.get_position(), ahrs.yaw);
            nav_initialised = true;
            break;

        case NAV_LOITER:
        	trace("set_nav_mode, NAV_LOITER")
            // set target to current position
            wp_nav.set_loiter_target_vel(inertial_nav.get_position(), inertial_nav.get_velocity());
            nav_initialised = true;
            break;

        case NAV_WP:
        	trace("set_nav_mode, NAV_WP")
            nav_initialised = true;
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( nav_initialised ) {
        nav_mode = new_nav_mode;
    }

    // return success or failure
    return nav_initialised;
}

// update_nav_mode - run navigation controller based on nav_mode
public function update_nav_mode():void
{
    switch( nav_mode ) {

        case NAV_NONE:
            // do nothing
            break;

        case NAV_CIRCLE:
            // call circle controller which in turn calls loiter controller
            update_circle(dTnav);
            break;

        case NAV_LOITER:
            // call loiter controller
            wp_nav.update_loiter();
            break;

        case NAV_WP:
            // call waypoint controller
            wp_nav.update_wpnav();
            break;
    }

    /*
    // To-Do: check that we haven't broken toy mode
    case TOY_A:
    case TOY_M:
        set_nav_mode(NAV_NONE);
        update_nav_wp();
        break;
    }
    */
}

public function check_missed_wp():Boolean
{
    var temp:int;
    temp = wp_bearing - original_wp_bearing;
    temp = wrap_180(temp);
    if(Math.abs(temp) > 9000)
    	trace("MISSED WP!!!");
    return (Math.abs(temp) > 9000);         // we passed the waypoint by 90 degrees
}

// Keeps old data out of our calculation / logs
public function reset_nav_params():void
{
    // Will be set by new command
    wp_bearing                      = 0;

    // Will be set by new command
    wp_distance                     = 0;

    // Will be set by nav or loiter controllers
    lon_error                       = 0;
    lat_error                       = 0;
    nav_roll 						= 0;
    nav_pitch 						= 0;
}

public function wrap_360(error:int):int
{
    if (error > 36000) error -= 36000;
    if (error < 0) error += 36000;
    return error;
}

public function wrap_180(error:int):int
{
    if (error > 18000) error -= 36000;
    if (error < -18000) error += 36000;
    return error;
}

// get_yaw_slew - reduces rate of change of yaw to a maximum
// assumes it is called at 100hz so centi-degrees and update rate cancel each other out
public function get_yaw_slew(current_yaw:int, desired_yaw:int, deg_per_sec:int):int
{
    return wrap_360(current_yaw + constrain(wrap_180(desired_yaw - current_yaw), -deg_per_sec, deg_per_sec));
}

// valid_waypoint - checks if a waypoint has been initialised or not
public function waypoint_valid(wp:Location):Boolean
{
     if( wp.lat != 0 || wp.lng != 0 ) {
         return true;
     }else{
         return true;
     }
}

////////////////////////////////////////////////////
// Loiter controller using inertial nav
////////////////////////////////////////////////////

public const MAX_LOITER_POS_VEL_VELOCITY:int = 1000
// loiter_set_pos_from_velocity - loiter velocity controller with desired velocity provided in front/right directions in cm/s
public function
loiter_set_pos_from_velocity(vel_forward_cms:int, vel_right_cms:int, dt:Number):void
{
    var vel_lat:int;
    var vel_lon:int;
    var vel_total:int;

	// rotate input
    //vel_lat = vel_forward_cms * cos_yaw - vel_right_cms * sin_yaw;
    //vel_lon = vel_forward_cms * sin_yaw + vel_right_cms * cos_yaw;

    vel_lat = vel_forward_cms;
    vel_lon = vel_right_cms;
    // constrain the velocity vector and scale if necessary
    vel_total = safe_sqrt(vel_lat * vel_lat + vel_lon * vel_lon);

    if( vel_total > MAX_LOITER_POS_VEL_VELOCITY ) {
        vel_lat = MAX_LOITER_POS_VEL_VELOCITY * vel_lat/vel_total;
        vel_lon = MAX_LOITER_POS_VEL_VELOCITY * vel_lon/vel_total;
    }
}

//////////////////////////////////////////////////////////
// circle navigation controller
//////////////////////////////////////////////////////////

// circle_set_center -- set circle controller's center position and starting angle
public function
circle_set_center(current_position:Vector3D, heading_in_radians:Number):void
{
    // set circle center to circle_radius ahead of current position
    circle_center.x = current_position.x + Number(g.circle_radius) * 100 * sin_yaw_y;
    circle_center.y = current_position.y + Number(g.circle_radius) * 100 * cos_yaw_x;

    // if we are doing a panorama set the circle_angle to the current heading
    if( g.circle_radius == 0 ) {
        circle_angle = heading_in_radians;
    }else{
    // set starting angle to current heading - 180 degrees
        circle_angle = heading_in_radians - radians(180);
		if( circle_angle > 180 ) {
			circle_angle -= 180;
		}
		if( circle_angle < -180 ) {
			circle_angle -= 180;
		}
    }

    // initialise other variables
    circle_angle_total = 0;
}

// update_circle - circle position controller's main call which in turn calls loiter controller with updated target position
public function
update_circle(dt:Number):void
{
	circle_rate = 200;
    var cir_radius:Number  		= g.circle_radius * 100;
    var angle_delta:Number  	= (circle_rate * dt)/ cir_radius;
    var circle_target:Vector3D 	= new Vector3D();

    // update the target angle
    circle_angle += angle_delta;
    if( circle_angle > 180 ) {
        circle_angle -= 360;
    }
    if( circle_angle <= -180 ) {
        circle_angle += 360;
    }

    // update the total angle travelled
    circle_angle_total += angle_delta;

    // if the circle_radius is zero we are doing panorama so no need to update loiter target
    if( g.circle_radius != 0.0 ) {
    // calculate target position
        circle_target.x = circle_center.x + cir_radius * Math.sin(1.57 - circle_angle);
        circle_target.y = circle_center.y + cir_radius * Math.cos(1.57 - circle_angle);

    // re-use loiter position controller
        wp_nav.set_loiter_target(circle_target);
    }

    // call loiter controller
    wp_nav.update_loiter();
}