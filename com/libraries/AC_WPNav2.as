package com.libraries {

	import flash.geom.Vector3D;
	import flash.geom.Point;
	import com.libraries.AP_AHRS;
	import com.libraries.AC_PID;
	import com.Sim.Clock;
	import com.Parameters;

	public class AC_WPNav2
		{
		public var g								:Parameters;

		public var clock:Clock;
		// maximum velocity that our position controller will request.  should be 1.5 ~ 2.0 times the pilot input's max velocity.  To - Do: make consistent with maximum velocity requested by pilot input to loiter
		public const MAX_LOITER_POS_VELOCITY         :int = 1200   	
		
		// defines the velocity vs distant curve.  maximum acceleration in cm/s/s that loiter position controller asks for from acceleration controller
		public const MAX_LOITER_POS_ACCEL            :int = 250
		
		// max acceleration in cm / s that the loiter velocity controller will ask from the lower accel controller.
		public const MAX_LOITER_VEL_ACCEL            :int = 450
		
		// should be 1.5 times larger than MAX_LOITER_POS_ACCEL.
		// max acceleration = max lean angle * 980 * pi / 180.  i.e. 23deg * 980 * 3.141 / 180 = 393 cm/s/s
		
		public const MAX_LOITER_OVERSHOOT            :int = 531        // maximum distance (in cm) that we will allow the target loiter point to be from the current location when switching into loiter
		//public const WPINAV_MAX_POS_ERROR            :Number = 531.25     // maximum distance (in cm) that the desired track can stray from our current location.
		public const WPINAV_MAX_POS_ERROR            :Number = 1000.00     // maximum distance (in cm) that the desired track can stray from our current location.
		public const MAX_LEAN_ANGLE                  :int = 4500        // default maximum lean angle
		public const MAX_CLIMB_VELOCITY              :int = 125         // maximum climb velocity - ToDo: pull this in from main code
		public const WPINAV_MAX_ALT_ERROR            :Number = 100.0;      // maximum distance (in cm) that the desired track can stray from our current location.
		public const AP_INTERTIALNAV_GRAVITY:Number		= 9.80665;

		// pointers to inertial nav library
		public var _inav			:AP_InertialNav;

		// pointers to pid controllers
		public var _pid_pos_lat		:AC_PID;
		public var _pid_pos_lon		:AC_PID;
		public var _pid_rate_lat	:AC_PID;
		public var _pid_rate_lon	:AC_PID;

		// parameters
		//public var _speed_cms		:Number;    // default horizontal speed in cm / s
		public var _speedz_cms		:Number;    // max vertical climb rate in cm / s.  To - Do: rename or pull this from main code
		//public var _wp_radius_cm	:Number;      // distance from a waypoint in cm that, when crossed, indicates the wp has been reached
		public var _last_update		:int;		// time of last update call
		public var _cos_yaw			:Number;    // short - cut to save on calcs required to convert roll - pitch frame to lat - lon frame
		public var _sin_yaw			:Number;
		public var  _cos_roll		:Number;

		// output from controller
		public var _desired_roll		:int;   // fed to stabilize controllers at 50hz
		public var _desired_pitch		:int;   // fed to stabilize controllers at 50hz
		public var _desired_altitude	:int;   // fed to alt hold controller at 50hz

		public var _lean_angle_max	:int;        // maximum lean angle.  can we set from main code so that throttle controller can stop leans that cause copter to lose altitude

		// internal variables
		public var _target			:Vector3D;  // loiter's target location in cm from home
		public var _target_vel		:Vector3D;  // loiter
		public var _vel_last		:Vector3D;  // previous iterations velocity in cm / s
		public var _origin			:Vector3D;  // starting point of trip to next waypoint in cm from home (equivalent to next_WP)
		public var _prev			:Vector3D;  // starting point of trip to next waypoint in cm from home (equivalent to next_WP)
		public var _destination		:Vector3D;  // target destination in cm from home (equivalent to next_WP)
		public var _pos_delta_unit  :Vector3D;// each axis's percentage of the total track from origin to destination

		public var _track_length		:Number;    // distance in cm between origin and destination
		public var _track_desired		:Number;    // our desired distance along the track in cm
		public var _vert_track_scale	:Number;    // horizontal component of track to next waypoint
		public var _reached_destination	:Boolean;    // vertical component of track to next waypoint

		public var _wp_distance			:Number;    // distance to loiter target

		public var p_loiter_lon_rate		:Number = 0;
		public var i_loiter_lon_rate		:Number = 0;
		public var d_loiter_lon_rate		:Number = 0;

		public var p_loiter_lat_rate		:Number = 0;
		public var i_loiter_lat_rate		:Number = 0;
		public var d_loiter_lat_rate		:Number = 0;
		
		// added by Jason
		
		public var _original_wp_bearing		:int = 0;
		public var _wp_bearing				:int = 0;
		public var _fast_corner				:Boolean;
		
		public var desired_speed			:Number = 0;
		private var dt						:Number = 0;
		public var crosstrack_error			:int = 0;
		
		public var N_target_speed			:int = 0;
		public var E_target_speed			:int = 0;
		public var N_rate_error				:int = 0;
		public var E_rate_error				:int = 0;
		public var nav_lon					:int = 0;
		public var nav_lat					:int = 0;
		
		public var desired_accel			:Point;
		public var desired_vel				:Point;
		public var dist_error				:Point;
		
	    // constrain target position to within reasonable distance of current location
    	public var curr_pos					:Vector3D;

		
		public var p_nav_rate				:Number = 0;
		public var i_nav_rate				:Number = 0;
		public var d_nav_rate				:Number = 0;
		public const WAYPOINT_SPEED_MIN		:int = 150 // cm/s
		public const RADX100				:Number = 0.000174532925;
		public const DEGX100				:Number = 5729.57795;
		private var max_speed_old			:int;

		
		// pilot inputs for loiter
		public var _pilot_vel_forward_cms	:int;
		public var _pilot_vel_right_cms		:int;

		public function AC_WPNav2(inav:AP_InertialNav, pid_pos_lat:AC_PID, pid_pos_lon:AC_PID, pid_rate_lat:AC_PID, pid_rate_lon:AC_PID) 
		{
			clock			= Clock.getInstance();
			_inav 			= inav;
			_pid_pos_lat 	= pid_pos_lat;
			_pid_pos_lon 	= pid_pos_lon;
			_pid_rate_lat 	= pid_rate_lat;
			_pid_rate_lon 	= pid_rate_lon;
			_speedz_cms 	= MAX_CLIMB_VELOCITY;
			_lean_angle_max = MAX_LEAN_ANGLE;
		
			_target			= new Vector3D();
			_target_vel		= new Vector3D();
			_vel_last		= new Vector3D();
			_origin			= new Vector3D();
			_destination	= new Vector3D();
			_pos_delta_unit	= new Vector3D();
			_prev			= new Vector3D();
			desired_accel	= new Point();
			desired_vel		= new Point();
			dist_error		= new Point();
			g = Parameters.getInstance();
			
		}

		// header functions

    /// get_loiter_target - get loiter target as position vector (from home in cm)
    public function get_loiter_target():Vector3D { return _target; }

    /// get_target_alt - get loiter's target altitude
    public function get_target_alt():Number { return _target.z; }

    /// set_loiter_target in cm from home
    public function set_loiter_target(position:Vector3D):void { 
    	_target = position;
    }


    /// set_angle_limit - limits maximum angle in centi-degrees the copter will lean
    public function set_angle_limit(lean_angle:int):void { _lean_angle_max = lean_angle; }

    /// clear_angle_limit - reset angle limits back to defaults
    public function clear_angle_limit():void { _lean_angle_max = MAX_LEAN_ANGLE; }
    
    /// get_angle_limit - retrieve maximum angle in centi-degrees the copter will lean
    public function get_angle_limit():int { return _lean_angle_max; }


		
	/// get_destination waypoint using position vector (distance from home in cm)
	public function get_destination():Vector3D { return _destination; }

	/// get_destination_alt - get target altitude above home in cm
	public function get_destination_alt():Number { return _destination.z; }

		/// set_destination_alt - set target altitude above home in cm
	public function set_destination_alt(altitude_in_cm:Number){ _destination.z = altitude_in_cm; }
	
	public function get_desired_roll():int { return _desired_roll; };
	public function get_desired_pitch():int { return _desired_pitch; };

    /// get_desired_alt - get desired altitude (in cm above home) from loiter or wp controller which should be fed into throttle controller
    public function get_desired_alt():int { return _target.z; }


    /// set_desired_alt - set desired altitude (in cm above home)
    public function set_desired_alt(desired_alt:Number):void {
    	//trace(_target.z);
    	_target.z = desired_alt; 
    }

	/// set_cos_sin_yaw - short-cut to save on calculations to convert from roll-pitch frame to lat-lon frame
	public function set_cos_sin_yaw(cos_yaw:Number, sin_yaw:Number, cos_roll:Number):void
	{
		_cos_yaw = cos_yaw;
		_sin_yaw = sin_yaw;
		_cos_roll = cos_roll;
	}

	/// set_climb_velocity - allows main code to pass max climb velocity to wp navigation
	public function set_climb_velocity(velocity_cms:Number):void { _speedz_cms = velocity_cms; };



// cpp functions


/// set_loiter_target - set initial loiter target based on current position and velocity
public function set_loiter_target_vel(position:Vector3D, velocity:Vector3D):void
{}

/// move_loiter_target - move loiter target by velocity provided in front/right directions in cm/s
public function move_loiter_target(control_roll:Number, control_pitch:Number, dt:Number):void
{
    // convert pilot input to desired velocity in cm/s
    _pilot_vel_forward_cms = -control_pitch * MAX_LOITER_POS_VELOCITY / 4500.0;
    _pilot_vel_right_cms = control_roll * MAX_LOITER_POS_VELOCITY / 4500.0;
}

/// translate_loiter_target_movements - consumes adjustments created by move_loiter_target
public function translate_loiter_target_movements(nav_dt:Number):void
{
    var target_vel_adj:Point = new Point();
    var vel_delta_total:Number;
    var vel_max:Number;
    var vel_total:Number;
    // range check nav_dt
    if(nav_dt < 0){
        return;
    }

    // rotate pilot input to lat/lon frame
    target_vel_adj.x = (_pilot_vel_forward_cms * _cos_yaw - _pilot_vel_right_cms * _sin_yaw) - _target_vel.x;
    target_vel_adj.y = (_pilot_vel_forward_cms * _sin_yaw + _pilot_vel_right_cms * _cos_yaw) - _target_vel.y;

    // constrain the velocity vector and scale if necessary
    vel_delta_total = Math.sqrt(target_vel_adj.x * target_vel_adj.x + target_vel_adj.y * target_vel_adj.y);
    vel_max = MAX_LOITER_POS_ACCEL * nav_dt;
    if(vel_delta_total >  vel_max){
        target_vel_adj.x = vel_max * target_vel_adj.x / vel_delta_total;
        target_vel_adj.y = vel_max * target_vel_adj.y / vel_delta_total;
    }

    // add desired change in velocity to current target velocity
    _target_vel.x += target_vel_adj.x;
    _target_vel.y += target_vel_adj.y;

    // constrain the velocity vector and scale if necessary
    vel_total = Math.sqrt(_target_vel.x * _target_vel.x + _target_vel.y * _target_vel.y);
    if(vel_total > MAX_LOITER_POS_VELOCITY){
        _target_vel.x = MAX_LOITER_POS_VELOCITY * _target_vel.x / vel_total;
        _target_vel.y = MAX_LOITER_POS_VELOCITY * _target_vel.y / vel_total;
    }

    // update target position
    _target.x += _target_vel.x * nav_dt;
    _target.y += _target_vel.y * nav_dt;

    // constrain target position to within reasonable distance of current location
    //var curr_pos:Vector3D = _inav.get_position();
	//var distance_err:Vector3D = _target - curr_pos;
	var distance_err:Vector3D = _target.clone();
	//trace("tar", _target, "disterr", distance_err);
	distance_err = distance_err.subtract(curr_pos);
	
    var distance:Number = Math.sqrt(distance_err.x * distance_err.x + distance_err.y * distance_err.y);
    if(distance > MAX_LOITER_OVERSHOOT){
        _target.x = curr_pos.x + MAX_LOITER_OVERSHOOT * distance_err.x / distance;
        _target.y = curr_pos.y + MAX_LOITER_OVERSHOOT * distance_err.y / distance;
    }
}

/// get_distance_to_target - get horizontal distance to loiter target in cm
public function get_distance_to_target():Number
{
    return _wp_distance;
}

/// get_bearing_to_target - get bearing to loiter target in centi-degrees
public function get_bearing_to_target():int
{
    return get_bearing_cd(_inav.get_position(), _target);
}

/// update_loiter - run the loiter controller - should be called at 10hz
public function update_loiter():void
{
    var now:int = clock.millis();
    var dt:Number = (now - _last_update) / 1000.0;
    _last_update = now;

    // catch if we've just been started
    if(dt >= 1.0){
        dt = 0.0;
        reset_I();
        _target_vel.x = 0;
        _target_vel.y = 0;
    }
    
    curr_pos = _inav.get_position();
    // translate any adjustments from pilot to loiter target
    translate_loiter_target_movements(dt);
    calc_location_error();
    // run loiter position controller
    //get_loiter_pos_lat_lon(_target.x, _target.y, dt);
    calc_loiter_velocity();

	calc_rate_error();
	calc_nav_rate();
	calc_pitch_roll();	
}


/// update_wpnav - run the wp controller - should be called at 10hz
public function update_wpnav():void
{
    var now:int = clock.millis();
    dt = (now - _last_update) / 1000.0;
    _last_update = now;

    // catch if we've just been started
    if(dt >= 1.0){
        dt = 0.0;
        reset_I();
    }
	curr_pos = _inav.get_position();    
    _reached_destination = did_reached_destination();
    //update_crosstrack();
	desired_speed = get_desired_speed(g.waypoint_speed_max);
	//trace("desired_speed", desired_speed)

    /// run position controller
    calc_nav_velocity();
	calc_rate_error();
	calc_nav_rate();
	calc_pitch_roll();
	//trace("lon_target_speed", lon_target_speed.toFixed(0), "lat_target_speed", lat_target_speed.toFixed(0),
	//"nav_lon", nav_lon, "nav_lat", nav_lat);

	var curr_vel:Vector3D = _inav.get_velocity();	
	var curr_speed:Number = Math.sqrt(curr_vel.x * curr_vel.x + curr_vel.y * curr_vel.y);
	//trace("curr_speed" , curr_speed.toFixed(0), curr_vel.y.toFixed(0), curr_vel.x.toFixed(0));
}

///
/// waypoint navigation
///

/// set_destination - set destination using cm from home
public function set_destination(destination:Vector3D):void
{
	_prev = _destination.clone();
	_prev.z = _inav.get_position().z;
    // To-Do: use projection of current position & velocity to set origin
    //set_origin_and_destination(_inav.get_position(), destination);
    set_origin_and_destination(_prev, destination);    
}

/// set_origin_and_destination - set origin and destination using lat/lon coordinates
public function set_origin_and_destination(origin:Vector3D, destination:Vector3D):void
{
	trace("set_origin_and_destination", origin, destination)
	_origin = origin.clone();
    _destination = destination.clone();	
    _reached_destination = false;
    // initialise original_wp_bearing which is used to check if we have missed the waypoint
    _wp_bearing = get_bearing_to_destination2();
    _original_wp_bearing = _wp_bearing;
}

public function did_reached_destination():Boolean
{
    // check if we've reached the waypoint
    if(_reached_destination){
    	return true;
    }else{
    	//var curr_pos:Vector3D = _inav.get_position();
		var dist_to_dest:Vector3D = curr_pos.subtract(_destination);
		//dist_to_dest.z *= _vert_track_scale;
		return (dist_to_dest.length <= g._wp_radius_cm)
    }
}


///
/// shared methods
///

public function calc_location_error():void
{
	/*
	Becuase we are using lat and lon to do our distance errors here's a quick chart:
	100 	= 1m
	1000 	= 11m	 = 36 feet
	1800 	= 19.80m = 60 feet
	3000 	= 33m
	10000 	= 111m
	*/

    // calculate distance error
    dist_error.x = _target.x - curr_pos.x;
    dist_error.y = _target.y - curr_pos.y;
}

public function calc_loiter_velocity():void
{
    var curr:Vector3D = _inav.get_position();
    var dist_error_total:Number;
    var vel_sqrt:Number;
    var vel_total:Number;

    var linear_distance:Number;      // the distace we swap between linear and sqrt.

    linear_distance = MAX_LOITER_POS_ACCEL / (2 * _pid_pos_lat.kP() * _pid_pos_lat.kP()); // 125 with kp = 1
    
    //_distance_to_target = linear_distance;      // for reporting purposes
    _wp_distance = linear_distance;

    dist_error_total = Math.sqrt(dist_error.x * dist_error.x + dist_error.y * dist_error.y);
    
    if(dist_error_total > 2 * linear_distance){
        vel_sqrt = constrain(Math.sqrt(2 * MAX_LOITER_POS_ACCEL * (dist_error_total - linear_distance)), 0, 1000);
        desired_vel.x = vel_sqrt * dist_error.x / dist_error_total;
        desired_vel.y = vel_sqrt * dist_error.y / dist_error_total;
        //trace("desired_vel", desired_vel.x.toFixed(1), desired_vel.y.toFixed(1));
    }else{
        desired_vel.x = _pid_pos_lat.get_p(dist_error.x);
        desired_vel.y = _pid_pos_lon.get_p(dist_error.y);
    }

    vel_total = Math.sqrt(desired_vel.x * desired_vel.x + desired_vel.y * desired_vel.y);
    if(vel_total > MAX_LOITER_POS_VELOCITY){
        desired_vel.x = MAX_LOITER_POS_VELOCITY * desired_vel.x / vel_total;
        desired_vel.y = MAX_LOITER_POS_VELOCITY * desired_vel.y / vel_total;
    }
	//trace("des vel", desired_vel.x.toFixed(2), desired_vel.y.toFixed(2));
   // get_loiter_vel_lat_lon(desired_vel.x, desired_vel.y, dt);
}



// get_loiter_pos_lat_lon - loiter position controller
//     converts desired position provided as distance from home in lat/lon directions to desired velocity
public function calc_nav_velocity():void
{
	var temp, temp_x, temp_y:Number

	var cross_speed:Number = crosstrack_error * -g.crosstrack_gain;
	
	cross_speed	= constrain(cross_speed, -350, 350);
	
	//trace("_wp_bearing",  _wp_bearing);
	
	// rotate by 90 to deal with trig functions
	temp 			= (9000 - _wp_bearing) * RADX100;
	temp_x 			= Math.cos(temp);
	temp_y 			= Math.sin(temp);

	// rotate desired spped vector:
	desired_vel.y 	= desired_speed * temp_x - cross_speed   * temp_y;
	desired_vel.x 	= cross_speed   * temp_x + desired_speed * temp_y;
	//trace("t speed", E_target_speed.toFixed(0), N_target_speed.toFixed(0));
}

public function calc_rate_error():void
{
    var vel_curr:Vector3D = _inav.get_velocity();  // current velocity in cm / s
	E_rate_error		= desired_vel.y - vel_curr.y;
	N_rate_error		= desired_vel.x - vel_curr.x;
	//trace("speed error", E_rate_error.toFixed(0), N_rate_error.toFixed(0));
}

// get_loiter_vel_lat_lon - loiter velocity controller
//    converts desired velocities in lat/lon frame to accelerations in lat/lon frame
public function calc_nav_rate():void
{
	var constr:Number = 1000;
    var accel_total:Number;                          // total acceleration in cm/s/s

    // reset last velocity if this controller has just been engaged or dt is zero
    if(dt == 0.0){
        desired_accel.x = 0;
        desired_accel.y = 0;
    } else {
        // feed forward desired acceleration calculation
        desired_accel.x = (desired_vel.x - _vel_last.x) / dt;
        desired_accel.y = (desired_vel.y - _vel_last.y) / dt;
    }

    // store this iteration's velocities for the next iteration
    _vel_last.x = desired_vel.x;
    _vel_last.y = desired_vel.y;


	E_rate_error 	= constrain(E_rate_error, -constr, constr);
	N_rate_error 	= constrain(N_rate_error, -constr, constr);	// added a rate error limit to keep pitching down to a minimum

	p_loiter_lon_rate = _pid_rate_lon.get_p(E_rate_error);
	i_loiter_lon_rate = _pid_rate_lon.get_i(E_rate_error, dt);
	d_loiter_lon_rate = _pid_rate_lon.get_d(E_rate_error, dt);

	p_loiter_lat_rate = _pid_rate_lat.get_p(N_rate_error);
	i_loiter_lat_rate = _pid_rate_lat.get_i(N_rate_error, dt);
	d_loiter_lat_rate = _pid_rate_lat.get_d(N_rate_error, dt);

	desired_accel.y	+= p_loiter_lon_rate + i_loiter_lon_rate + d_loiter_lon_rate;
	desired_accel.x += p_loiter_lat_rate + i_loiter_lat_rate + d_loiter_lat_rate;
	
    // scale desired acceleration if it's beyond acceptable limit
    accel_total = Math.sqrt(desired_accel.x * desired_accel.x + desired_accel.y * desired_accel.y);
    if(accel_total > MAX_LOITER_VEL_ACCEL){
        desired_accel.x = MAX_LOITER_VEL_ACCEL * desired_accel.x / accel_total;
        desired_accel.y = MAX_LOITER_VEL_ACCEL * desired_accel.y / accel_total;
    }	
}

// get_loiter_accel_lat_lon - loiter acceration controller
//    converts desired accelerations provided in lat/lon frame to roll/pitch angles
public function calc_pitch_roll():void
{
    var z_accel_meas:Number = -AP_INTERTIALNAV_GRAVITY * 100;    // gravity in cm / s / s
    var accel_forward:Number;
    var accel_right:Number;

    // To-Do: add 1hz filter to accel_lat, accel_lon

    // rotate accelerations into body forward-right frame
    accel_forward =  desired_accel.x * _cos_yaw + desired_accel.y * _sin_yaw;
    accel_right   = -desired_accel.x * _sin_yaw + desired_accel.y * _cos_yaw;
    // update angle targets that will be passed to stabilize controller

	_desired_roll = constrain((accel_right / (-z_accel_meas)) * (18000 / Math.PI), -_lean_angle_max, _lean_angle_max);
	_desired_pitch = constrain((-accel_forward / (-z_accel_meas * _cos_roll)) * (18000 / Math.PI), -_lean_angle_max, _lean_angle_max);
}


public function get_desired_speed(max_speed:Number):Number
{
    /*
    Based on Equation by Bill Premerlani & Robert Lefebvre
    	(sq(V2)-sq(V1))/2 = A(X2-X1)
        derives to:
        V1 = sqrt(sq(V2) - 2*A*(X2-X1))
     */

    if(false/*ap.fast_corner*/) {
        // don't slow down
    }else{
        if(_wp_distance < 20000){ // limit the size of numbers we're dealing with to avoid overflow
            // go slower
    	 	var temp:int 	= 2 * 100 * (_wp_distance - g.waypoint_radius * 100);
    	 	var s_min:int 	= WAYPOINT_SPEED_MIN;
    	 	temp 			+= s_min * s_min;
            if( temp < 0 ) temp = 0;                // check to ensure we don't try to take the sqrt of a negative number
    		max_speed 		= Math.sqrt(Number(temp));
            max_speed 		= Math.min(max_speed, g.waypoint_speed_max);
        }
    }

    max_speed 		= Math.min(max_speed, max_speed_old + (100 * dt));// limit going faster
    max_speed 		= Math.max(max_speed, WAYPOINT_SPEED_MIN); 	// don't go too slow
    max_speed_old 	= max_speed;
    return max_speed;
}

public function update_crosstrack():void
{
	_wp_bearing = get_bearing_to_destination();
	
    // Crosstrack Error
    // ----------------
    if (_wp_distance >= 500 &&
        Math.abs(wrap_180(_wp_bearing - _original_wp_bearing)) < 4500) {
	    var temp:Number = (_wp_bearing - _original_wp_bearing) * RADX100;
    	crosstrack_error = Math.sin(temp) * _wp_distance;          // Meters we are off track line
	    //trace("temp", temp, "crosstrack_error", crosstrack_error);
    }else{
        // fade out crosstrack
        crosstrack_error >>= 1;
    }
}


///
/// utilties
///

public function check_missed_wp():Boolean
{
    var temp:int;
    temp = _wp_bearing - _original_wp_bearing;
    temp = wrap_180(temp);
    if(Math.abs(temp) > 9000)
    	trace("MISSED WP!!!");
    return (Math.abs(temp) > 9000);         // we passed the waypoint by 90 degrees
}

/// get_distance_to_destination - get horizontal distance to destination in cm
public function get_distance_to_destination():Number
{
    // get current location
    var curr:Vector3D = _inav.get_position();
    //_wp_distance = Vector3D.distance(_destination, curr);
    var deltaX = curr.x - _destination.x; 
    var deltaY = curr.y - _destination.y; 
    _wp_distance = Math.sqrt(deltaY * deltaY + deltaX * deltaX);
    //trace("_wp_distance",_wp_distance)
    return _wp_distance;
    //return pythagorous2(_destination.x-curr.x,_destination.y-curr.y);
}

/// get_bearing_to_destination - get bearing to next waypoint in centi-degrees
public function get_bearing_to_destination():int
{
	_wp_bearing = get_bearing_cd(_inav.get_position(), _destination);
    return _wp_bearing;
}

public function get_bearing_to_destination2():int
{
	_wp_bearing = get_bearing_cd(_prev, _destination);
    return _wp_bearing;
}

// get_bearing_cd - return bearing in centi-degrees between two positions
// To-Do: move this to math library
public function get_bearing_cd(origin:Vector3D, destination:Vector3D):Number
{
    var bearing:Number = 9000 + Math.atan2(-(destination.x - origin.x), destination.y - origin.y) * 5729.57795;
    
    if(bearing < 0){
        bearing += 36000;
    }
	//trace(bearing.toFixed(0), ": ", destination.y.toFixed(0), origin.y.toFixed(0), " | ", destination.x.toFixed(0), origin.x.toFixed(0));
    return bearing;
}

/// reset_I - clears I terms from loiter PID controller
public function reset_I():void
{
    _pid_pos_lon.reset_I();
    _pid_pos_lat.reset_I();
    _pid_rate_lon.reset_I();
    _pid_rate_lat.reset_I();

    // set last velocity to current velocity
    _vel_last = _inav.get_velocity();
}

public function constrain(val:Number, min:Number, max:Number){
	val = Math.max(val, min);
	val = Math.min(val, max);
	return val;
}

public function wrap_180(error:int):int
{
    if (error > 18000) error -= 36000;
    if (error < -18000) error += 36000;
    return error;
}



	}
}

