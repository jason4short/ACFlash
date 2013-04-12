package com.libraries
{
	public class AP_MotorsQuad extends AP_MotorsMatrix
	{

		public function AP_MotorsQuad(rc_roll:RC_Channel, rc_pitch:RC_Channel, rc_throttle:RC_Channel, rc_yaw:RC_Channel) {
			super(rc_roll, rc_pitch, rc_throttle, rc_yaw);

			setup_motors();
		}

		override public function setup_motors():void
		{
			remove_all_motors();

				// hard coded config for supported frames
			if( _frame_orientation == AP_MOTORS_PLUS_FRAME ) {
				//trace("plus frame")
				// plus frame set-up
				add_motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_MOTOR_CCW, 2);
				add_motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_MOTOR_CCW, 4);
				add_motor(AP_MOTORS_MOT_3,   0, AP_MOTORS_MATRIX_MOTOR_CW,  1);
				add_motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_MOTOR_CW,  3);
			}else{
				// X frame set-up
				//trace("X frame")
				add_motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_MOTOR_CCW, 1);
				add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_MOTOR_CCW, 3);
				add_motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_MOTOR_CW,  4);
				add_motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_MOTOR_CW,  2);
			}
		}
	}
};

