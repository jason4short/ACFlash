/*
	RC_Channel.cpp - Radio library for Arduino
	Code by Jason Short. DIYDrones.com

	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

*/

package com.Sim
{
	import flash.display.MovieClip;
    import flash.events.*;
    import com.Main;
    import com.libraries.Location;
    import com.UI.*;

	public class WaypointManager extends MovieClip
	{
		// GUI
		public var waypoints			:Array;
		public var controller			:Main;
		public var lat_offset			:int = 0;
		public var lng_offset			:int = 0;
		public var alt_offset			:int = 0;

		public function WaypointManager()
		{
			addEventListener(Event.ADDED_TO_STAGE, addedToStage);
			visible = false;
		}

	    public function addedToStage(even:Event):void
		{

			for(var i:int = 0; i <= 12; i++){
				populateMenus(this["waypoint_"+i].id_menu);
			}

		}

	    public function populateMenus(m:QuickPopupMenu):void
		{
			// Stability
			m.addItem(new QuickMenuItem("Takeoff",			"22"));
			m.addItem(new QuickMenuItem("WP Nav",			"16"));
			m.addItem(new QuickMenuItem("Land",				"21"));
		}

	    public function set_waypoint_array(wp:Array):void
		{
			//trace("wp", wp, wp.length);
			waypoints = wp;
			this.update();
		}

		public function clearWaypoints()
		{
			controller.g.command_total = 1;
		}

		public function setHome(lat:int, lng:int)
		{
			var tmp:Location = new Location();
			tmp.id = 16;
			tmp.options = 0;
			tmp.p1 = 0;
			tmp.alt = 0;
			tmp.lat = lat;
			tmp.lng = lng;
			controller.set_cmd_with_index(tmp, 0);
		}

		public function addWaypoint(id:int, options:int, p1:int, alt:int, lat:int, lng:int)
		{
			// use home as an offset
			lat = lat - lat_offset;
			lng = lng - lng_offset;
			var tmp:Location = new Location();
			tmp.id = id;
			tmp.options = options;
			tmp.p1 = p1;
			tmp.alt = alt;
			tmp.lat = lat;
			tmp.lng = lng;
			controller.set_cmd_with_index(tmp, controller.g.command_total);
		}


	    public function update():void
		{
			for(var i:int = 0; i < waypoints.length; i++){
				//trace("i ", i)
				this["waypoint_"+i].id_menu.setSelectedItemByCode(waypoints[i].id.toFixed(0));
				this["waypoint_"+i].p1_BI.setNumber(waypoints[i].p1);
				this["waypoint_"+i].alt_BI.setNumber(waypoints[i].alt);
				this["waypoint_"+i].lat_BI.setNumber(waypoints[i].lat);
				this["waypoint_"+i].lng_BI.setNumber(waypoints[i].lng);
			}

		}


	}
}
