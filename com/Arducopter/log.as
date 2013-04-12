// ----------------------------------------
// Loging
// ----------------------------------------
private function Log_Write_Nav_Tuning():void
{
	//			  wp_distance, (nav_bearing/100), lon_error, lat_error, nav_lon, nav_lat, x_actual_speed, y_actual_speed, g.pid_nav_lon.get_integrator(), g.pid_nav_lat.get_integrator()
	trace("NTUN, "+ wp_distance +","+ (wp_bearing/100)+","+  lon_error+","+  lat_error+","+  nav_lon.toFixed(0)+","+  nav_lat.toFixed(0)+","+  copter.velocity.x.toFixed(0)+","+  copter.velocity.y.toFixed(0)+","+  g.pid_loiter_rate_lon.get_integrator()+","+  g.pid_loiter_rate_lat.get_integrator());

}

private function Log_Write_Control_Tuning():void
{
	trace("CTUN,"+ g.rc_3.control_in +","+ 0 +","+  baro_alt  +","+  next_WP.alt +","+ nav_throttle +","+ angle_boost +","+ g.rc_3.servo_out +","+ climb_rate +","+ copter.throttle +","+ g.pi_alt_hold.get_integrator() +","+ g.pid_throttle.get_integrator());
}
private function Log_Write_Attitude():void
{
	trace("ATT", ahrs.roll_sensor+","+  ahrs.pitch_sensor+","+  ahrs.yaw_sensor);
}
private function Log_Write_GPS():void
{
	trace("GPS, 0, 0, " + ((wp_manager.lat_offset + current_loc.lat)/10000000) +", "+ ((wp_manager.lng_offset + current_loc.lng)/10000000) +", "+ (wp_manager.alt_offset + current_loc.alt/100) + ", 0, 0, 0");  //+", "+ current_loc.alt/100 +", "+ copter.velocity.x +", "+ wp_bearing);
}

public function Log_Write_Cmd(index:int, cmd:Location)
{
	trace("CMD, " + g.command_total+", "+ index + ", " + cmd.id + ", " + cmd.options + ", " + cmd.p1 + ", " + cmd.alt + ", " + (cmd.lat+ wp_manager.lat_offset) + ", " + (cmd.lng + wp_manager.lng_offset));
}



// Wrote an event packet
public function Log_Write_Event(id:int):void
{
	trace("id",id);
}

// Wrote an event packet
public function Log_Write_Error(sub_system:int, error_code:int):void
{
	trace("error",sub_system, error_code);
}

