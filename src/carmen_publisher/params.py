#!/usr/bin/env python
# -*- coding: utf-8 -*-

old_laser_params = {"front_laser_dev" 			: None,
					"rear_laser_dev" 			: None,
					"laser3_dev" 				: None,
					"laser4_dev" 				: None,
					"laser5_dev" 				: None,
					"front_laser_type" 			: None,
					"front_laser_resolution" 	: None,
					"front_laser_use_remission" : None,
					"front_laser_fov" 			: None,
					"front_laser_baud" 			: None,
					"front_laser_flipped" 		: None,
					"rear_laser_type" 			: None,
					"rear_laser_resolution" 	: None,
					"rear_laser_use_remission" 	: None,
					"rear_laser_fov" 			: None,
					"rear_laser_baud" 			: None,
					"rear_laser_flipped" 		: None,
					"laser3_type" 				: None,
					"laser3_resolution" 		: None,
					"laser3_use_remission" 		: None,
					"laser3_fov" 				: None,
					"laser3_baud" 				: None,
					"laser3_flipped" 			: None,
					"laser4_type" 				: None,
					"laser4_resolution" 		: None,
					"laser4_use_remission" 		: None,
					"laser4_fov" 				: None,
					"laser4_baud" 				: None,
					"laser4_flipped" 			: None,
					"laser5_type" 				: None,
					"laser5_resolution" 		: None,
					"laser5_use_remission" 		: None,
					"laser5_fov" 				: None,
					"laser5_baud" 				: None,
					"laser5_flipped" 			: None}
					
new_laser_params = {"laser1_flipped" 			: None,
					"laser5_flipped" 			: None,
					"laser5_flipped" 			: None,
					"laser5_flipped" 			: None,
					"laser5_flipped" 			: None,
					"laser5_flipped" 			: None,
					"laser5_flipped" 			: None,
					"laser5_flipped" 			: None,
					"laser5_flipped" 			: None}
					
robot_params = {"allow_rear_motion" 					: None,
				"rectangular" 							: None,
				"use_laser" 							: None,
				"use_sonar" 							: None,
				"converge" 								: None,
				"timeout" 								: None,
				"sensor_timeout"						: None,
				"collision_avoidance" 					: None,
				"collision_avoidance_frequency" 		: None,
				"laser_bearing_skip_rate" 				: None,
				"laser_bearing_skip_rate" 				: None,
				"turn_before_driving_if_heading_bigger" : None,
				"backwards" 							: None,
				"length" 								: None,
				"width" 								: None,
				"frontlaser_use"						: None,
				"frontlaser_id"							: None,
				"frontlaser_offset" 					: None,
				"frontlaser_side_offset"				: None,
				"frontlaser_angular_offset"				: None,
				"rearlaser_use"							: None,
				"rearlaser_id"							: None,
				"rearlaser_offset" 						: None,
				"rearlaser_side_offset"					: None,
				"rearlaser_angular_offset"				: None,
				"front_laser_max" 						: None,
				"min_approach_dist" 					: None,
				"min_side_dist" 						: None,
				"acceleration" 							: None,
				"deceleration" 							: None,
				"reaction_time" 						: None,
				"max_t_vel" 							: None,
				"max_r_vel" 							: None,
				"curvature" 							: None,
				"theta_gain" 							: None,
				"displacement_gain" 					: None,
				"displacement_gain" 					: None,
				"displacement_gain" 					: None,
				"displacement_gain" 					: None,
				"displacement_gain" 					: None,
				"displacement_gain" 					: None,
				"use_bumper" 							: None,
				"bumper_offsets" 						: None,
				"odometry_inverted" 					: None}
				
gps_params = {"nmea_dev" 				: None,
			  "nmea_baud" 				: None,
			  "originlat" 				: None,
			  "originlon" 				: None,
			  "integrate_with_odometry" : None,
			  "initialtheta" 			: None,
			  "initialthetastd" 		: None,
			  "odomdiststdper1m" 		: None,
			  "odomthetastdper1m" 		: None,
			  "odomthetastdper1rad" 	: None,
			  "gpsxystdper1precdil" 	: None}
			  
base_params = {"type" 					 : None,
			   "model" 					 : None,
			   "motion_timeout"			 : None,
			   "dev" 					 : None,
			   "use_hardware_integrator" : None,
			   "relative_wheelsize" 	 : None,
			   "relative_wheelbase" 	 : None}
			   
arm_params = {"num_joints"   : None,
			  "joint_types"	 : None,
			  "dev" 		 : None}
			  
segway_params = {"accel_limit"   : None,
				 "torque_limit"	 : None,
				 "gain_schedule" : None}
					
def laserId2ParamPrefix(laser_id):
	
	if laser_id == "FLASER":
		return "front_laser"
	elif laser_id == "RLASER":
		return "rear_laser"
	elif laser_id == "LASER3" or laser_id == "RAWLASER3":
		return "laser3"
	elif laser_id == "LASER4" or laser_id == "RAWLASER4":
		return "laser4"
	elif laser_id == "LASER5" or laser_id == "RAWLASER5":
		return "laser5"
	elif laser_id == "RAWLASER1":
		return "laser1"
	elif laser_id == "RAWLASER2":
		return "laser2"
