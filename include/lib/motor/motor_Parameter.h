#ifndef _MOTOR_PARAMETER_H
#define _MOTOR_PARAMETER_H

#if defined(CONFIG_MOTOR_SUPER_ABZHALL_400W)

#define MOTOR_PAIRS		    (5.0f)
#define MOTOR_RS		    (0.005f)
#define MOTOR_LS		    (0.005f)
#define MOTOR_DEBUG_IQ		    (0.02f)
#define MOTOR_DEBUG_SELFANGLE	    (0.2f)
#define SCETION_3_BASEANGLE	    (286.00f)
#define SCETION_2_BASEANGLE	    (346.00f)
#define SCETION_6_BASEANGLE	    (046.00f)
#define SCETION_4_BASEANGLE	    (106.00f)
#define SCETION_5_BASEANGLE	    (166.00f)
#define SCETION_1_BASEANGLE	    (226.00f)
#define HALL_SENSOR_POSITIVE_OFFSET (-30.0f)	   /* 霍尔传感器正方向偏移量 (度) */
#define HALL_SENSOR_NEGATIVE_OFFSET (0.0f)	   /* 霍尔传感器负方向偏移量 (度) */
#define ABZ_ENCODER_LINES	    (4096)	   // 编码器线数
#define ABZ_ENCODER_RESOLUTION	    (0.087890625f) // 360.0f/ABZ_ENCODER_LINES

#define TOQREMODE_D_KP	      (0.08f)
#define TOQREMODE_D_KI	      (0.006f)
#define TOQREMODE_D_KC	      (0.5f)
#define TOQREMODE_D_MAX_LIMIT (12.0f)
#define TOQREMODE_D_MIN_LIMIT (-TOQREMODE_D_MAX_LIMIT)

#define TOQREMODE_Q_KP	      (0.08f)
#define TOQREMODE_Q_KI	      (0.006f)
#define TOQREMODE_Q_KC	      (0.5f)
#define TOQREMODE_Q_MAX_LIMIT (12.0f)
#define TOQREMODE_Q_MIN_LIMIT (-TOQREMODE_Q_MAX_LIMIT)

#define SPEEDMODE_SPEED_KP	  (0.0125f)
#define SPEEDMODE_SPEED_KI	  (0.0083f)
#define SPEEDMODE_SPEED_KC	  (0.5f)
#define SPEEDMODE_SPEED_MAX_LIMIT (48.0f)
#define SPEEDMODE_SPEED_MIN_LIMIT (-TOQREMODE_D_MAX_LIMIT)

#define SPEEDPLAN_MAX_A	 (100.00f)
#define SPEEDPLAN_MIN_A	 (0.0f)
#define SPEEDPLAN_MAX_JA (300.0f)
#define SPEEDPLAN_MIN_JA (0.0f)

#define POSMODE_POS_KP	      (10.0f)
#define POSMODE_POS_KI	      (0.0001f)
#define POSMODE_POS_KC	      (0.5f)
#define POSMODE_POS_MAX_LIMIT (2000.0f)
#define POSMODE_POS_MIN_LIMIT (-POSMODE_POS_MAX_LIMIT)

#define POSPLAN_MAX_SPEED (1400.0F)
#define POSPLAN_MAX_ACC	  (3000.0f)
#define POSPLAN_MAX_JACC  (15000.0f)

#elif defined(CONFIG_MOTOR_ELUR_HALL)

#elif defined(CONFIG_ENCODER_TYPE_TLE5012B)
#define MOTOR_PAIRS	      (15.0f)
#define MOTOR_DEBUG_IQ	      (0.02f)
#define MOTOR_DEBUG_SELFANGLE (0.008f)

#define TOQREMODE_D_KP	      (0.08f)
#define TOQREMODE_D_KI	      (0.006f)
#define TOQREMODE_D_KC	      (0.5f)
#define TOQREMODE_D_MAX_LIMIT (12.0f)
#define TOQREMODE_D_MIN_LIMIT (-TOQREMODE_D_MAX_LIMIT)

#define TOQREMODE_Q_KP	      (0.08f)
#define TOQREMODE_Q_KI	      (0.006f)
#define TOQREMODE_Q_KC	      (0.5f)
#define TOQREMODE_Q_MAX_LIMIT (12.0f)
#define TOQREMODE_Q_MIN_LIMIT (-TOQREMODE_Q_MAX_LIMIT)

#define SPEEDMODE_SPEED_KP	  (0.0125f)
#define SPEEDMODE_SPEED_KI	  (0.0083f)
#define SPEEDMODE_SPEED_KC	  (0.5f)
#define SPEEDMODE_SPEED_MAX_LIMIT (48.0f)
#define SPEEDMODE_SPEED_MIN_LIMIT (-TOQREMODE_D_MAX_LIMIT)

#define SPEEDPLAN_MAX_A	 (100.00f)
#define SPEEDPLAN_MIN_A	 (0.0f)
#define SPEEDPLAN_MAX_JA (300.0f)
#define SPEEDPLAN_MIN_JA (0.0f)

#define POSMODE_POS_KP	      (10.0f)
#define POSMODE_POS_KI	      (0.0001f)
#define POSMODE_POS_KC	      (0.5f)
#define POSMODE_POS_MAX_LIMIT (2000.0f)
#define POSMODE_POS_MIN_LIMIT (-POSMODE_POS_MAX_LIMIT)

#define POSPLAN_MAX_SPEED (1400.0F)
#define POSPLAN_MAX_ACC	  (3000.0f)
#define POSPLAN_MAX_JACC  (15000.0f)

#elif defined(CONFIG_ENCODER_TYPE_AS5047)
#define MOTOR_DEBUG_IQ	      (0.02f)
#define MOTOR_DEBUG_SELFANGLE (0.2f)
#else
#error "No valid board configuration selected!"
#endif

#endif
