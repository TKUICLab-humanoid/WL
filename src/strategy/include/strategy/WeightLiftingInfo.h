#pragma once
#ifndef WEIGHTLIFTINGINFO_H
#define WEIGHTLIFTINGINFO_H

//=========================================================================== Shengru
#define InitialHeadX 511
#define InitialHeadY 350

enum ACTION_STATE{

	headdown,PickUp,Check,Near,tweak,FirstLifting,SecondLifting,checkmidline,WalkingOverTape
};
//===========================================================================

class WeightLiftingInfo
{
	public:
		int BodyState;
		int HeadPostitionX;
		int HeadPostitionY;
		float imu_initial;
		bool DIO_fake;
		int etLine;
		int LineLabel2;//red
		int LineLabel;
//===========================================================================
		int Firstlifting_sector;
		int Firstlifting_sup_sector;
		int Secondlifting_sector;
		int Secondlifting_sup_sector;
		int continuous_speed;
		int continuous_Y;
		int continuous_theta;
		int continuous_imu;
		int continuous_stopdistance;
		int tweak_stopdistance;
		int Near_left;
		int tweak_range;
		int turn_left_X; 
		int turn_left_Y; 
		int turn_left_tweak; 
		int turn_right_X; 
		int turn_right_Y; 
		int turn_right_tweak;
		int turn_tweak_imu;
		int tweak_stra_step_X;
		int tweak_stra_step_Y;
		int tweak_stra_theta;
		int left_tweak_x_large;
		int left_tweak_y_large;
		int left_tweak_theta_large;
		int left_tweak_large_imu;
		int right_tweak_x_large;
		int right_tweak_y_large;
		int right_tweak_theta_large;
		int right_tweak_large_imu;
		int tweak_delay;
		int FirstLifting_motion_delay;
		int SecondLifting_delay;
		int speed;
		int first_speed;
		int speed_up;
		int ccount;
//===========================================================================
		int white[4][4]={{0}};
		int red[4][4]={{0}};
		
		int count;
		double time_start;
		double time_end;

		bool stand_flag=false;
		bool finallookline_flag;
		bool closeimage;

		WeightLiftingInfo(void);
};

extern WeightLiftingInfo* weightlifting_info;
#endif

