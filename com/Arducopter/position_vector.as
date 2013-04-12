// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// position_vector.pde related utility functions

// position vectors are Vector2f
//    .x = latitude from home in cm
//    .y = longitude from home in cm

// pv_latlon_to_vector - convert lat/lon coordinates to a position vector
public function pv_latlon_to_vector(lat:int, lon:int, alt:int):Vector3D
{
    var tmp:Vector3D = new Vector3D(lat-home.lat * LATLON_TO_CM, lon-home.lng * LATLON_TO_CM * scaleLongDown, alt);
    return tmp;
}

// pv_latlon_to_vector - convert lat/lon coordinates to a position vector
public function pv_location_to_vector(loc:Location):Vector3D
{
    var tmp:Vector3D  = new Vector3D((loc.lat-home.lat) * LATLON_TO_CM, (loc.lng-home.lng) * LATLON_TO_CM * scaleLongDown, loc.alt);
    return tmp;
}

// pv_get_lon - extract latitude from position vector
public function pv_get_lat(pos_vec:Vector3D):int
{
    return home.lat + (pos_vec.x / LATLON_TO_CM);
}

// pv_get_lon - extract longitude from position vector
public function pv_get_lon(pos_vec:Vector3D):int
{
    return home.lng + (pos_vec.y / LATLON_TO_CM * scaleLongUp);
}

// pv_get_distance_cm - return distance between two positions in cm
public function pv_get_distance_cm(origin:Vector3D, destination:Vector3D):Number
{
	return Vector3D.distance(origin, destination);
    //return pythagorous2(dist.x,dist.y);
}

// pv_get_bearing_cd - return bearing in centi-degrees between two locations
public function pv_get_bearing_cd2(origin:Vector3D, destination:Vector3D):Number
{
    var dist:Point  = new Point(destination.x - origin.x, destination.y - origin.y);
    var bearing:int = 9000 + Math.atan2(dist.x, dist.y) * 5729.57795;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}


public function pv_get_bearing_cd(origin:Vector3D, destination:Vector3D):Number
{
    var bearing:int = 9000 + Math.atan2(-(destination.x-origin.x), destination.y-origin.y) * 5729.57795;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}