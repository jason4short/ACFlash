package com.libraries {

	import flash.geom.Vector3D;
	import flash.geom.Point;
	import com.libraries.AP_AHRS;
	import com.libraries.AC_PID;
	import com.Sim.Clock;
	import com.Parameters;

	public class AC_WPNav
		{
		public var g								:Parameters;

		public var clock:Clock;
		// maximum velocity that our position controller will request.  should be 1.5 ~ 2.0 times the pilot input's max velocity.  To - Do: make consistent with maximum velocity requested by pilot input to loiter
		public const MAX_LOITER_POS_VELOCITY        :int = 500   	
		// defines the velocity vs distant curve.  maximum acceleration in cm/s/s that loiter position controller asks for from acceleration controller
		public const MAX_LOITER_POS_ACCEL           :int = 250
		// max acceleration in cm / s that the loiter velocity controller will ask from the lower accel controller.
		public const MAX_LOITER_VEL_ACCEL           :int = 800
		// should be 1.5 times larger than MAX_LOITER_POS_ACCEL.
		// max acceleration = max lean angle * 980 *pi / 180.  i.e. 23deg * 980 * 3.141 / 180 = 393 cm/s/s
		public const MAX_LEAN_ANGLE                 :int = 4500        	// default maximum lean angle
		
		public const MAX_LOITER_OVERSHOOT           :int = 531        	// maximum distance (in cm) that we will allow the target loiter point to be from the current location when switching into loiter
		//public const WPINAV_MAX_POS_ERROR         :Number = 531.25    // maximum distance (in cm) that the desired track can stray from our current location.
		public const WPINAV_MAX_POS_ERROR           :Number = 531.25    // maximum distance (in cm) that the desired track can stray from our current location.

		public const MAX_CLIMB_VELOCITY             :int = 125         	// maximum climb velocity - ToDo: pull this in from main code
		public const WPINAV_MAX_ALT_ERROR           :Number = 100.0;    // maximum distance (in cm) that the desired track can stray from our current location.
		public const AP_INTERTIALNAV_GRAVITY		:Number	= 9.80665;

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
		public var _loiter_last_update		:int;		// time of last update call
		public var _wpnav_last_update		:int;		// time of last update call
		public var _cos_yaw			:Number;    // short - cut to save on calcs required to convert roll - pitch frame to lat - lon frame
		public var _sin_yaw			:Number;
		public var  _cos_roll		:Number;

		// output from controller
		public var _desired_roll		:int;   // fed to stabilize controllers at 50hz
		public var _desired_pitch		:int;   // fed to stabilize controllers at 50hz
		public var _desired_altitude	:int;   // fed to alt hold controller at 50hz

		public var dist_error 			:Point;                // distance error calculated by loiter controller
		public var desired_vel			:Point;               // loiter controller desired velocity
		public var desired_accel		:Point;             // the resulting desired acceleration
    

		public var _lean_angle_max	:int;        // maximum lean angle.  can we set from main code so that throttle controller can stop leans that cause copter to lose altitude

		// internal variables
		public var _target			:Vector3D;  // loiter's target location in cm from home
		public var _target_vel		:Vector3D;  // loiter
		public var _vel_last		:Vector3D;  // previous iterations velocity in cm / s
		public var _origin			:Vector3D;  // starting point of trip to next waypoint in cm from home (equivalent to next_WP)
		public var _destination		:Vector3D;  // target destination in cm from home (equivalent to next_WP)
		public var _pos_delta_unit  :Vector3D;// each axis's percentage of the total track from origin to destination

		public var _track_length		:Number;    // distance in cm between origin and destination
		public var _track_desired		:Number;    // our desired distance along the track in cm
		public var _distance_to_target	:Number;    // distance to loiter target
		public var _vert_track_scale	:Number;    // horizontal component of track to next waypoint
		public var _reached_destination	:Boolean;    // vertical component of track to next waypoint


		public var p_loiter_lon_rate		:Number = 0;
		public var i_loiter_lon_rate		:Number = 0;
		public var d_loiter_lon_rate		:Number = 0;

		public var p_loiter_lat_rate		:Number = 0;
		public var i_loiter_lat_rate		:Number = 0;
		public var d_loiter_lat_rate		:Number = 0;
		
		
		public var _prev  :Vector3D;// each axis's percentage of the total track from origin to destination
		
		// pilot inputs for loiter
		public var _pilot_vel_forward_cms	:int;
		public var _pilot_vel_right_cms	:int;

		public function AC_WPNav(inav:AP_InertialNav, pid_pos_lat:AC_PID, pid_pos_lon:AC_PID, pid_rate_lat:AC_PID, pid_rate_lon:AC_PID) 
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

			dist_error		= new Point();
			desired_vel		= new Point();
			desired_accel	= new Point();

			g = Parameters.getInstance();
		}

		// header functions

    /// get_loiter_target - get loiter target as position vector (from home in cm)
    public function get_loiter_target():Vector3D { return _target; }

    /// set_loiter_target in cm from home
    public function set_loiter_target(position:Vector3D) { _target = position.clone(); }


    /// set_angle_limit - limits maximum angle in centi-degrees the copter will lean
    public function set_angle_limit(lean_angle:int):void { _lean_angle_max = lean_angle; }

    /// clear_angle_limit - reset angle limits back to defaults
    public function clear_angle_limit():void { _lean_angle_max = MAX_LEAN_ANGLE; }
    
    /// get_angle_limit - retrieve maximum angle in centi-degrees the copter will lean
    public function get_angle_limit():int { return _lean_angle_max; }

    ///
    /// waypoint controller
    ///
		
	/// get_destination waypoint using position vector (distance from home in cm)
	public function get_destination():Vector3D { return _destination; }
	
	public function reached_destination():Boolean { return _reached_destination; }
	
    ///
    /// shared methods
    ///

	public function get_desired_roll():int { return _desired_roll; };
	public function get_desired_pitch():int { return _desired_pitch; };

    /// get_desired_alt - get desired altitude (in cm above home) from loiter or wp controller which should be fed into throttle controller
    public function get_desired_alt():int { return _target.z; }


    /// set_desired_alt - set desired altitude (in cm above home)
    public function set_desired_alt(desired_alt:Number):void {_target.z = desired_alt;}

	/// set_cos_sin_yaw - short-cut to save on calculations to convert from roll-pitch frame to lat-lon frame
	public function set_cos_sin_yaw(cos_yaw:Number, sin_yaw:Number, cos_roll:Number):void
	{
		_cos_yaw = cos_yaw;
		_sin_yaw = sin_yaw;
		_cos_roll = cos_roll;
	}

    /// set_horizontal_velocity - allows main code to pass target horizontal velocity for wp navigation
    public function set_horizontal_velocity(velocity_cms:Number) { g._speed_cms = velocity_cms; };

	/// set_climb_velocity - allows main code to pass max climb velocity to wp navigation
	public function set_climb_velocity(velocity_cms:Number):void { _speedz_cms = velocity_cms; };

    public function get_waypoint_radius():Number { return g._wp_radius_cm; }




// cpp functions


/// set_loiter_target - set initial loiter target based on current position and velocity
public function project_stopping_point(position:Vector3D, velocity:Vector3D):Vector3D
{
	
	var linear_distance	:Number;      // half the distace we swap between linear and sqrt and the distace we offset sqrt.
	var linear_velocity	:Number;      // the velocity we swap between linear and sqrt.
	var vel_total		:Number;
	var target_dist		:Number;
	var target			:Vector3D = new Vector3D();

    // avoid divide by zero
    if(_pid_pos_lat.kP() <= 0.1){
        return(position);
    }

    // calculate point at which velocity switches from linear to sqrt
    linear_velocity = MAX_LOITER_POS_ACCEL / _pid_pos_lat.kP();

    // calculate total current velocity
    vel_total = Math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y);

    // calculate distance within which we can stop
    if(vel_total < linear_velocity){
        target_dist = vel_total / _pid_pos_lat.kP();
    } else {
        linear_distance = MAX_LOITER_POS_ACCEL / (2 * _pid_pos_lat.kP() * _pid_pos_lat.kP());
        target_dist = linear_distance + (vel_total * vel_total) / (2 * MAX_LOITER_POS_ACCEL);
    }
    

    target_dist = constrain(target_dist, 0, MAX_LOITER_OVERSHOOT);

	// Avoid div/0!
	if(vel_total == 0) vel_total = 1; 


    target.x = position.x + (target_dist * velocity.x / vel_total);
    target.y = position.y + (target_dist * velocity.y / vel_total);
    target.z = position.z;
    return target;
}

/// set_loiter_target - set initial loiter target based on current position and velocity
public function set_loiter_target_vel(position:Vector3D, velocity:Vector3D):void
{
	var target:Vector3D = project_stopping_point(position, velocity);
    _target.x = target.x;
    _target.y = target.y;
}

/// move_loiter_target - move loiter target by velocity provided in front/right directions in cm/s
public function move_loiter_target(control_roll:Number, control_pitch:Number, dt:Number):void
{
    // convert pilot input to desired velocity in cm/s
    _pilot_vel_forward_cms  = -control_pitch * MAX_LOITER_POS_VELOCITY / 4500.0;
    _pilot_vel_right_cms    = control_roll   * MAX_LOITER_POS_VELOCITY / 4500.0;
}

/// translate_loiter_target_movements - consumes adjustments created by move_loiter_target
public function translate_loiter_target_movements(nav_dt:Number):void
{
    var target_vel_adj:Point = new Point();    // make 2d vector?
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
    var curr_pos:Vector3D = _inav.get_position();
	//var distance_err:Vector3D = _target - curr_pos;
	var distance_err:Vector3D = _target.clone();
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
    return _distance_to_target;
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
    var dt:Number = (now - _loiter_last_update) / 1000.0;
    _loiter_last_update = now;

    // catch if we've just been started
    if(dt >= 1.0){
        dt = 0.0;
        reset_I();
    }

    // translate any adjustments from pilot to loiter target
    translate_loiter_target_movements(dt);
    
    // run loiter position controller
    get_loiter_position_to_velocity(dt);
}

///
/// waypoint navigation
///

/// set_destination - set destination using cm from home
public function set_destination(destination:Vector3D):void
{	
	var origin:Vector3D;
    // if waypoint controlls is active and copter has reached the previous waypoint use it for the origin
    if(_reached_destination && ((clock.millis() - _wpnav_last_update) < 1000)){
        origin = _destination.clone();
    }else{
        // otherwise calculate origin from the current position and velocity
        origin = project_stopping_point(_inav.get_position(), _inav.get_velocity());
    }

    // set origin and destination
    set_origin_and_destination(origin, destination);
}

/// set_origin_and_destination - set origin and destination using lat/lon coordinates
public function set_origin_and_destination(origin:Vector3D, destination:Vector3D):void
{
	_origin = origin.clone();
    _destination = destination.clone();
	
    _vert_track_scale = WPINAV_MAX_POS_ERROR / WPINAV_MAX_ALT_ERROR;
    var pos_delta:Vector3D = _destination.subtract(_origin);

    pos_delta.z = pos_delta.z * _vert_track_scale;
    _track_length = pos_delta.length;
    //_pos_delta_unit = pos_delta / _track_length;

    _pos_delta_unit = pos_delta.clone();
	_pos_delta_unit.scaleBy(1/_track_length);

    _track_desired = 0;
    _reached_destination = false;
}

/// advance_target_along_track - move target location along track from origin to destination
public function advance_target_along_track(velocity_cms:Number, dt:Number):void
{
    var track_covered			:Number;
    var track_error				:Number;
    var track_desired_max		:Number;
    var track_desired_temp		:Number = _track_desired;
    var track_extra_max			:Number;
    var curr_delta_length		:Number;

    // get current location
    var curr_pos:Vector3D 	= _inav.get_position();
    var curr_delta:Vector3D = curr_pos.subtract(_origin);
    curr_delta.z 			= curr_delta.z * _vert_track_scale;
    curr_delta_length 		= curr_delta.length;

    track_covered = curr_delta.x * _pos_delta_unit.x + curr_delta.y * _pos_delta_unit.y + curr_delta.z * _pos_delta_unit.z;
    track_error = Math.sqrt(curr_delta_length * curr_delta_length - track_covered * track_covered);

    track_extra_max = Math.sqrt(WPINAV_MAX_POS_ERROR * WPINAV_MAX_POS_ERROR - track_error * track_error);

    // we could save a sqrt by doing the following and not assigning track_error
    // track_extra_max = safe_sqrt(WPINAV_MAX_POS_ERROR*WPINAV_MAX_POS_ERROR - (curr_delta_length*curr_delta_length - track_covered*track_covered));

    track_desired_max = track_covered + track_extra_max;

    // advance the current target
    track_desired_temp += velocity_cms * dt;

    // constrain the target from moving too far
    if(track_desired_temp > track_desired_max){
        track_desired_temp = track_desired_max;
    }
    // do not let desired point go past the end of the segment
    track_desired_temp = constrain(track_desired_temp, 0, _track_length);
    _track_desired = Math.max(_track_desired, track_desired_temp);

    // recalculate the desired position
    _target.x = _origin.x + _pos_delta_unit.x * _track_desired;
    _target.y = _origin.y + _pos_delta_unit.y * _track_desired;
    _target.z = _origin.z + (_pos_delta_unit.z * _track_desired) / _vert_track_scale;

    // check if we've reached the waypoint
    if(!_reached_destination){
        if(_track_desired >= _track_length){
            var dist_to_dest:Vector3D = curr_pos.subtract(_destination);
            dist_to_dest.z *= _vert_track_scale;
            if(dist_to_dest.length <= g._wp_radius_cm){
                _reached_destination = true;
            }
        }
    }
}

/// get_distance_to_destination - get horizontal distance to destination in cm
public function get_distance_to_destination():Number
{
    // get current location
    var curr:Vector3D = _inav.get_position();
    return Vector3D.distance(_destination, curr); 
}

/// get_bearing_to_destination - get bearing to next waypoint in centi-degrees
public function get_bearing_to_destination():int
{
    return get_bearing_cd(_inav.get_position(), _destination);
}

/// update_wpnav - run the wp controller - should be called at 10hz
public function update_wpnav():void
{
    var now:int = clock.millis();
    var dt:Number = (now - _wpnav_last_update) / 1000.0;
    _wpnav_last_update = now;

    // catch if we've just been started
    if(dt >= 1.0){
        dt = 0.0;
        reset_I();
    }else{
        // advance the target if necessary
        advance_target_along_track(g._speed_cms, dt);
    }

    // run loiter position controller
    get_loiter_position_to_velocity(dt);
}

///
/// shared methods
///

// get_loiter_pos_lat_lon - loiter position controller
//     converts desired position provided as distance from home in lat/lon directions to desired velocity
public function get_loiter_position_to_velocity(dt:Number):void
{
	var curr:Vector3D				= _inav.get_position();
    var dist_error_total:Number;

    var vel_sqrt:Number;
    var vel_total:Number;

    var linear_distance:Number;      // the distace we swap between linear and sqrt.

    // calculate distance error
    dist_error.x = _target.x - curr.x;
    dist_error.y = _target.y - curr.y;

    linear_distance = MAX_LOITER_POS_ACCEL / (2 * _pid_pos_lat.kP() * _pid_pos_lat.kP()); // 125 with kp = 1
    
    _distance_to_target = linear_distance;      // for reporting purposes

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
    get_loiter_velocity_to_acceleration(desired_vel.x, desired_vel.y, dt);
}

// get_loiter_vel_lat_lon - loiter velocity controller
//    converts desired velocities in lat/lon frame to accelerations in lat/lon frame
public function get_loiter_velocity_to_acceleration(vel_lat:Number, vel_lon:Number, dt:Number):void
{
    var vel_curr:Vector3D = _inav.get_velocity();  // current velocity in cm / s
    var vel_error:Vector3D = new Vector3D();       // The velocity error in cm / s.
    var accel_total:Number;                        // total acceleration in cm / s / s

	//trace("vel_lon:", vel_lon, "vel_lat:", vel_lat);
	
    // reset last velocity if this controller has just been engaged or dt is zero
    if(dt == 0.0){
        desired_accel.x = 0;
        desired_accel.y = 0;
    } else {
        // feed forward desired acceleration calculation
        desired_accel.x = (vel_lat - _vel_last.x) / dt;
        desired_accel.y = (vel_lon - _vel_last.y) / dt;
       // trace("desired_accel", desired_accel.x.toFixed(2), "desired_vel", vel_lat.toFixed(2), "last" , _vel_last.x.toFixed(2));
    }
    
    // store this iteration's velocities for the next iteration
    _vel_last.x = vel_lat;
    _vel_last.y = vel_lon;

    // calculate velocity error
    vel_error.x = vel_lat - vel_curr.x;
    vel_error.y = vel_lon - vel_curr.y;
	//trace("vel_error", vel_error, vel_curr)
    // combine feed foward accel with PID outpu from velocity error

    p_loiter_lon_rate = _pid_rate_lon.get_p(vel_error.y);
    i_loiter_lon_rate = _pid_rate_lon.get_i(vel_error.y, dt);
    d_loiter_lon_rate = _pid_rate_lon.get_d(vel_error.y, dt);

    p_loiter_lat_rate = _pid_rate_lat.get_p(vel_error.x);
    i_loiter_lat_rate = _pid_rate_lat.get_i(vel_error.x, dt);
    d_loiter_lat_rate = _pid_rate_lat.get_d(vel_error.x, dt);

    desired_accel.x += (p_loiter_lat_rate + i_loiter_lat_rate + d_loiter_lat_rate);
    desired_accel.y += (p_loiter_lon_rate + i_loiter_lon_rate + d_loiter_lon_rate);
    
	//var temp:Point = desired_accel.clone();

    // scale desired acceleration if it's beyond acceptable limit
    accel_total = Math.sqrt(desired_accel.x * desired_accel.x + desired_accel.y * desired_accel.y);
    if(accel_total > MAX_LOITER_VEL_ACCEL){
        desired_accel.x = MAX_LOITER_VEL_ACCEL * desired_accel.x / accel_total;
        desired_accel.y = MAX_LOITER_VEL_ACCEL * desired_accel.y / accel_total;
    }
	//trace("clip", desired_accel.x.toFixed(2), temp.x.toFixed(2), desired_accel.y.toFixed(2), temp.y.toFixed(2));
	//trace("noclip", desired_accel.x.toFixed(2), desired_accel.y.toFixed(2));
	
    // call accel based controller with desired acceleration
    get_loiter_acceleration_to_lean_angles(desired_accel.x, desired_accel.y);
}

// get_loiter_accel_lat_lon - loiter acceration controller
//    converts desired accelerations provided in lat/lon frame to roll/pitch angles
public function get_loiter_acceleration_to_lean_angles(accel_lat:Number, accel_lon:Number):void
{
    var z_accel_meas:Number = -AP_INTERTIALNAV_GRAVITY * 100;    // gravity in cm / s / s
    var accel_forward:Number;
    var accel_right:Number;

    // To-Do: add 1hz filter to accel_lat, accel_lon

    // rotate accelerations into body forward-right frame
    accel_forward =  accel_lat * _cos_yaw + accel_lon * _sin_yaw;
    accel_right   = -accel_lat * _sin_yaw + accel_lon * _cos_yaw;
    
	//trace("get_loiter_accel_lat_lon", accel_right, accel_forward);
    // update angle targets that will be passed to stabilize controller
    _desired_roll = constrain((accel_right / (-z_accel_meas)) * (18000 / Math.PI), -_lean_angle_max, _lean_angle_max);
    _desired_pitch = constrain((-accel_forward / (-z_accel_meas * _cos_roll)) * (18000 / Math.PI), -_lean_angle_max, _lean_angle_max);
}

// get_bearing_cd - return bearing in centi-degrees between two positions
// To-Do: move this to math library
public function get_bearing_cd(origin:Vector3D, destination:Vector3D):Number
{
    var bearing:Number = 9000 + Math.atan2(-(destination.x - origin.x), destination.y - origin.y) * 5729.57795;
    if(bearing < 0){
        bearing += 36000;
    }
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

	}
}

