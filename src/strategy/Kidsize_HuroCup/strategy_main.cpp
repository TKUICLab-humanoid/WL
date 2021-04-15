#include "strategy/strategy_main.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kidsize");
	ros::NodeHandle nh;
	WeightLifting WeightLifting(nh);
	ros::Rate loop_rate(30);
	WeightLifting.initparameterpath();
	while (nh.ok()) 
	{	
		WeightLifting.strategyMain();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void WeightLifting::strategyMain()
{
	if(strategy_info->getStrategyStart())
	{	
		weightlifting_info->stand_flag = false;
		strategy_info->get_image_flag = true;
		ros_com->drawImageFunction(1,  DrawMode::DrawLine, 0, 320, 120, 120, 152, 245, 255);
		ros_com->drawImageFunction(2,  DrawMode::DrawLine, 160, 160, 0, 240, 152, 245, 255);
		ros_com->drawImageFunction(3,  DrawMode::DrawObject, 80, 240, 60, 180, 125, 38, 205);
		ros_com->drawImageFunction(4,  DrawMode::DrawObject, strategy_info->color_mask_subject[6][0].XMin, strategy_info-> color_mask_subject[6][0].XMax, strategy_info-> color_mask_subject[6][0].YMin, strategy_info-> color_mask_subject[6][0].YMax, 255, 165, 0);
		ros_com->drawImageFunction(5,  DrawMode::DrawObject, strategy_info->color_mask_subject[5][0].XMin, strategy_info-> color_mask_subject[5][0].XMax, strategy_info-> color_mask_subject[5][0].YMin, strategy_info-> color_mask_subject[5][0].YMax, 20, 165, 50);
		strategy_info->get_label_model_flag = true;
		if(weightlifting_info->BodyState == headdown||weightlifting_info->BodyState == PickUp||weightlifting_info->BodyState== Check||weightlifting_info->BodyState == tweak)
		{
			strategyClassify2();
		}
		else
		{
			//if(!weightlifting_info->closeimage)
				
		}
		strategyBody();
		strategyClassify();
		ROS_INFO("MASK = %d",strategy_info-> color_mask_subject[6][0].YMax);
		
	}
	else
	{	
		loadParameter();
		initialparameter();
		if(continuous_flag == true)
		{
			ros_com->sendBodyAuto(0,0,0,0,WalkingMode::ContinuousStep,SensorMode::None);
			ROS_INFO("Stop");
			tool->Delay(2000);
			continuous_flag = false;
		}
		ros_com->sendHeadMotor(HeadMotorID::VerticalID,1330,100);
		tool->Delay(300);
		weightlifting_info->BodyState = headdown;
		weightlifting_info->speed = weightlifting_info->continuous_speed;
		
		if(weightlifting_info->stand_flag == false)
		{
			ros_com->sendBodySector(29);
			ROS_INFO("already stand");
			tool->Delay(500);
			weightlifting_info->stand_flag = true;
			tool->Delay(500);
		}
	}
}
void WeightLifting::initialparameter(void)
{
	
		strategy_info->get_image_flag = true;
		strategy_info->get_label_model_flag = true;
		weightlifting_info->etLine =  (int)LabelModel::Orange;
		weightlifting_info->LineLabel = (int)LabelMark::OrangeLabel;
		weightlifting_info->BodyState = 0;
		weightlifting_info->HeadPostitionX = 2022;
		weightlifting_info->HeadPostitionY = 1400;
		weightlifting_info->etLine =  (int)LabelModel::Orange;
		weightlifting_info->LineLabel = (int)LabelMark::OrangeLabel;
		weightlifting_info->white[4][4] = 0;
		weightlifting_info->red[4][4] = 0;

}
void WeightLifting::initparameterpath()
{
	while(parameter_path == "N")
	{
		parameter_path = tool->getPackagePath("strategy");
	}
	printf("parameter_path is %s\n", parameter_path.c_str());
}

void WeightLifting::loadParameter(void)	//read file
{
	fstream fin;
	string sTmp;
	char line[100];
	char path[200];
	strcpy(path, parameter_path.c_str());         //copy string
	strcat(path, "/quickchage.ini");   //Connected to path
	fin.open(path, ios::in);
	//fin.open(("/home/andrew/Desktop/WL/src/strategy/Parameter/quickchage.ini"), ios::in);
	try
	{ 
		weightlifting_info->Firstlifting_sector = tool->readvalue(fin, "Firstlifting_sector", 1);		// Firstlifting motion's sector
		weightlifting_info->Firstlifting_sup_sector = tool->readvalue(fin, "Firstlifting_sup_sector", 1);	// Firstlifting support motion's sector
		weightlifting_info->Secondlifting_sector = tool->readvalue(fin, "Secondlifting_sector", 1);
		weightlifting_info->Secondlifting_sup_sector = tool->readvalue(fin, "Secondlifting_sup_sector", 1);
		 
		weightlifting_info->continuous_speed = tool->readvalue(fin, "continuous_speed", 1);	//continuous's start speed
		weightlifting_info->continuous_Y = tool->readvalue(fin, "continuous_Y", 1);		//continuous's Y value
		weightlifting_info->continuous_theta = tool->readvalue(fin, "continuous_theta", 1);	//continuous's theta value
		weightlifting_info->continuous_imu = tool->readvalue(fin, "continuous_imu", 1);		//continuous's imu value

		weightlifting_info->continuous_stopdistance = tool->readvalue(fin, "continuous_stopdistance", 1);		//continuous's stop position
		weightlifting_info->tweak_stopdistance = tool->readvalue(fin, "tweak_stopdistance", 1);				//tweak's stop position

		
		weightlifting_info->Near_left = tool->readvalue(fin, "Near_left", 1);			//Middle direction of Near
		weightlifting_info->tweak_range = tool->readvalue(fin, "tweak_range", 1);		//middle range of tweak
		weightlifting_info->turn_left_X = tool->readvalue(fin, "turn_left_X", 1);		//turn left's X value
		weightlifting_info->turn_left_Y = tool->readvalue(fin, "turn_left_Y", 1);
		weightlifting_info->turn_left_tweak = tool->readvalue(fin, "turn_left_tweak", 1);
		weightlifting_info->turn_right_X = tool->readvalue(fin, "turn_right_X", 1);
		weightlifting_info->turn_right_Y = tool->readvalue(fin, "turn_right_Y", 1);
		weightlifting_info->turn_right_tweak = tool->readvalue(fin, "turn_right_tweak", 1);
		weightlifting_info->turn_tweak_imu = tool->readvalue(fin, "turn_tweak_imu", 1);

		weightlifting_info->tweak_stra_step_X = tool->readvalue(fin, "tweak_stra_step_X", 1);	//straight's step of tweak (Second approach)
		weightlifting_info->tweak_stra_step_Y = tool->readvalue(fin, "tweak_stra_step_Y", 1);	
		weightlifting_info->tweak_stra_theta = tool->readvalue(fin, "tweak_stra_theta", 1);	//straight's theta of tweak

		weightlifting_info->left_tweak_x_large = tool->readvalue(fin, "left_tweak_x_large", 1);		//Large left tweak's X value
		weightlifting_info->left_tweak_y_large = tool->readvalue(fin, "left_tweak_y_large", 1);
		weightlifting_info->left_tweak_theta_large = tool->readvalue(fin, "left_tweak_theta_large", 1);
		weightlifting_info->left_tweak_large_imu = tool->readvalue(fin, "left_tweak_large_imu", 1);
		weightlifting_info->right_tweak_x_large = tool->readvalue(fin, "right_tweak_x_large", 1);	//Large right tweak's X value
		weightlifting_info->right_tweak_y_large = tool->readvalue(fin, "right_tweak_y_large", 1);
		weightlifting_info->right_tweak_theta_large = tool->readvalue(fin, "right_tweak_theta_large", 1);
		weightlifting_info->right_tweak_large_imu = tool->readvalue(fin, "right_tweak_large_imu", 1);
		weightlifting_info->speed_up = tool->readvalue(fin, "speed_up", 1);
		weightlifting_info->ccount = tool->readvalue(fin, "ccount", 1);   //

		weightlifting_info->tweak_delay = tool->readvalue(fin, "tweak_delay", 1);		//Delay between each tweak's motion

		weightlifting_info->FirstLifting_motion_delay = tool->readvalue(fin, "FirstLifting_motion_delay", 1);		//

		weightlifting_info->SecondLifting_delay = tool->readvalue(fin, "SecondLifting_delay", 1);			//
	}
	catch (exception e)
	{
		ROS_INFO("READ ERROR\n");
	}
}
bool WeightLifting::strategyClassify(void)
{
	ROS_INFO("strategyClassify");

	for (int j = 0 ; j < strategy_info-> color_mask_subject_cnts[6] ; j++)//數有幾個物件，從左上到右下（底層）, color_mask_subject_cnts[6]==orange;
	{

		if (strategy_info-> color_mask_subject[6][j].size > 500)//判斷物件的面積是否大於某值 
		{			
			weightlifting_info->white[0][0] = strategy_info-> color_mask_subject[6][j].XMin;
			weightlifting_info->white[1][0] = strategy_info-> color_mask_subject[6][j].XMax;
			weightlifting_info->white[0][1] = strategy_info-> color_mask_subject[6][j].YMin;
			weightlifting_info->white[1][1] = strategy_info-> color_mask_subject[6][j].YMax;
		}
	}
	ROS_INFO("White YMax = %d", weightlifting_info->white[1][1]);
	return true;
}
bool WeightLifting::strategyClassify2(void)//test
{
	ROS_INFO("strategyClassify2");
	for (int j = 0 ; j < strategy_info-> color_mask_subject_cnts[5] ; j++)//數有幾個物件,從左上到右下（底層）, color_mask_subject_cnts[5]==red;
	{	ROS_INFO(" SIZE : %d ",strategy_info-> color_mask_subject[5][j].size);
		ROS_INFO("J = %d",j);
		if (strategy_info-> color_mask_subject[5][j].size > 200)//判斷物件的面積有否大於某值
		{			
			weightlifting_info->red[0][0] = strategy_info-> color_mask_subject[5][j].XMin;
			weightlifting_info->red[1][0] = strategy_info-> color_mask_subject[5][j].XMax;
			weightlifting_info->red[0][1] = strategy_info-> color_mask_subject[5][j].YMin;
			weightlifting_info->red[1][1] = strategy_info-> color_mask_subject[5][j].YMax;
		}
	}
	ROS_INFO(" Red YMax = %d ", weightlifting_info->red[1][1]);
	return true;
}

void WeightLifting::firstSpeedControl(void)		//control continuous_speed
{		
	int con_fix = 0;
	ROS_INFO(" YMAX = %d ",(weightlifting_info->red[1][1]));
	if((weightlifting_info->red[1][1])>= weightlifting_info->continuous_stopdistance){
		weightlifting_info->BodyState = Check;	
	}
	else if((weightlifting_info->red[1][1]) <= 100){	//Boost
		weightlifting_info->speed = weightlifting_info->speed+100;
	}
	else if	(weightlifting_info->speed > 1800 && weightlifting_info->red[1][1] > 120){
		weightlifting_info->speed = weightlifting_info->speed - 200;	//Slow down
		tool->Delay(40);
	}
	else{
		
	}
	if	(weightlifting_info->speed > 2000)
	{	//Speed limit
		weightlifting_info->speed = 2000;
		tool->Delay(300);
	}
	ROS_INFO(" imu_initial %f",weightlifting_info->imu_initial);
	ros::spinOnce();
	ROS_INFO(" Yaw = %f ",strategy_info->getIMUValue().Yaw);
	if(strategy_info->getIMUValue().Yaw - weightlifting_info->imu_initial <= -1)
	{
		ROS_INFO("add theta");
		con_fix = 1;
	}
	else if(strategy_info->getIMUValue().Yaw - weightlifting_info->imu_initial >= 1)
	{
		ROS_INFO("sub theta");
		con_fix = -1;
	}
	else
	{
		con_fix = 0;
	}
	ROS_INFO(" Speed = %d ",weightlifting_info->speed);
	ros_com->sendContinuousValue(weightlifting_info->speed,weightlifting_info->continuous_Y,0,weightlifting_info->continuous_theta + con_fix,SensorMode(weightlifting_info->continuous_imu));	
}

bool  WeightLifting::strategyBody(void)
{
	
	switch (weightlifting_info->BodyState)
	{
		case headdown:
			ros_com->sendHeadMotor(HeadMotorID::VerticalID,1350,100);
			//ros_com->sendBodySector(20);
			//move head's motor2 to 1450 with speed 100
			tool->Delay(1000);
			ros::spinOnce();
			weightlifting_info->imu_initial=strategy_info->getIMUValue().Yaw;
			ROS_INFO("%f",weightlifting_info->imu_initial);

			weightlifting_info->BodyState = PickUp;			
			break;

		case PickUp:
			ROS_INFO(" PICK UP ");
			if(continuous_flag == false)
			{
				ros_com->sendBodyAuto(0,0,0,0, WalkingMode::ContinuousStep, SensorMode::None);
				tool->Delay(3000);
				/*ros_com->sendBodyAuto(weightlifting_info->speed,weightlifting_info->continuous_Y,0,weightlifting_info->continuous_theta,WalkingMode::ContinuousStep,SensorMode(weightlifting_info->continuous_imu));*/
				continuous_flag = true;
			}	
			if(continuous_flag == true)
			{
				firstSpeedControl();
			}
			break;

		case Check:
			ROS_INFO(" Check ");
			if ((weightlifting_info->red[1][1]) >= weightlifting_info->continuous_stopdistance-40 && (weightlifting_info->red[1][1]) <= 240)//Ymax
			{
				if(continuous_flag == true)
				{
					ros_com->sendContinuousValue(0,0,0,0,SensorMode::None);
					ros_com->sendHeadMotor(HeadMotorID::VerticalID,1180,100);
			        tool->Delay(1000);
					ROS_INFO(" Stop ");//走到200時
					weightlifting_info->BodyState = tweak;

				}
				
			}	
			break;
		case tweak:
			
			ROS_INFO(" Tweak ");
			ROS_INFO(" initial %f",weightlifting_info->imu_initial);
			ROS_INFO("tweak_stopdistancellllllllllllllllll %d",weightlifting_info->red[1][1]);
			ROS_INFO("tweak_range-8 OOOOOOOOO %d",(((weightlifting_info->red[0][0])+(weightlifting_info->red[1][0]))/2));
			ros::spinOnce();
			
			ROS_INFO("%f",strategy_info->getIMUValue().Yaw);
			
			strategyClassify2();
			
			ROS_INFO(" XMIN = %d , XMAX = %d ",(weightlifting_info->red[0][0]),(weightlifting_info->red[1][0]));

			if(strategy_info->getIMUValue().Yaw - weightlifting_info->imu_initial <= weightlifting_info->Near_left-1||strategy_info->getIMUValue().Yaw - weightlifting_info->imu_initial >=weightlifting_info->Near_left+1)
			{

				if(strategy_info->getIMUValue().Yaw - weightlifting_info->imu_initial <= weightlifting_info->Near_left-1)
				{
					ROS_INFO("Turn Left");
					ros_com->sendContinuousValue(weightlifting_info->turn_left_X,weightlifting_info->turn_left_Y,0,weightlifting_info->turn_left_tweak,SensorMode(weightlifting_info->turn_tweak_imu));
				}
				else if(strategy_info->getIMUValue().Yaw - weightlifting_info->imu_initial >= weightlifting_info->Near_left+1)
				{
					ROS_INFO("Turn Right");
					ros_com->sendContinuousValue(weightlifting_info->turn_right_X,weightlifting_info->turn_right_Y,0,weightlifting_info->turn_right_tweak,SensorMode(weightlifting_info->turn_tweak_imu));
				}
			}
			else
			{
				ROS_INFO(" BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB ");
				if (((weightlifting_info->red[0][0])+(weightlifting_info->red[1][0]))/2 >= weightlifting_info->tweak_range-12&& ((weightlifting_info->red[0][0])+(weightlifting_info->red[1][0]))/2 <= weightlifting_info->tweak_range+12)//直走,tweak_range中心=tweak_range,看情況增減
				{	
					ROS_INFO("CCCCCCCCCCCCCCCCCCCCCCCCCC");	
					if((weightlifting_info->red[1][1])<weightlifting_info->tweak_stopdistance&&strategy_info->getStrategyStart())
					{
						ROS_INFO("DDDDDDDDDDDDDDDDDD");
						ros_com->sendContinuousValue(weightlifting_info->tweak_stra_step_X,weightlifting_info->tweak_stra_step_Y,0,weightlifting_info->tweak_stra_theta,SensorMode(weightlifting_info->left_tweak_large_imu));
						ROS_INFO("forward");
						tool->Delay(1000);
					}
					else
					{
						ROS_INFO("EEEEEEEEEEEEEEEEEEE");
						ros_com->sendBodyAuto(0,0,0,0, WalkingMode::ContinuousStep, SensorMode::None);
						tool->Delay(3000);
						continuous_flag = false;
						weightlifting_info->BodyState = FirstLifting;
						tool->Delay(500);
					}	
				}
				else if(((weightlifting_info->red[0][0])+(weightlifting_info->red[1][0]))/2 < weightlifting_info->tweak_range-8)//大左移
				{   
					ROS_INFO("FFFFFFFFFFFFFFFFF");
					ROS_INFO(" Large Left ");
					ROS_INFO("%d",((weightlifting_info->red[0][0])+(weightlifting_info->red[1][0]))/2);
					ros_com->sendContinuousValue(weightlifting_info->left_tweak_x_large,weightlifting_info->left_tweak_y_large,0,weightlifting_info->left_tweak_theta_large,SensorMode(weightlifting_info->left_tweak_large_imu));
					tool->Delay(500);
				}	
				else if(((weightlifting_info->red[0][0])+(weightlifting_info->red[1][0]))/2 > weightlifting_info->tweak_range+8)//大右移
				{  
					ROS_INFO("GGGGGGGGGGGGGGGGGGGGGGGGGG");
					ROS_INFO(" Large Right ");
					ROS_INFO("%d",((weightlifting_info->red[0][0])+(weightlifting_info->red[1][0]))/2);
					ros_com->sendContinuousValue(weightlifting_info->right_tweak_x_large,weightlifting_info->right_tweak_y_large,0,weightlifting_info->right_tweak_theta_large,SensorMode(weightlifting_info->right_tweak_large_imu));
					tool->Delay(500);
				}	
				else 
				{	
					ROS_INFO(" Remeasure range ");
					ros_com->drawImageFunction(5,  DrawMode::DrawObject, (weightlifting_info->red[0][0]), (weightlifting_info->red[1][0]), (weightlifting_info->red[0][1]), (weightlifting_info->red[1][1]), 180, 68, 5);
					weightlifting_info->BodyState = Check;
				}
			}
			break;

		case FirstLifting: //拿起竿子
			ros_com->sendHeadMotor(HeadMotorID::VerticalID,1350,100);	//move head's motor2 to 1360 with speed 100
			tool->Delay(2500);
			ros_com->sendBodySector(weightlifting_info->Firstlifting_sector);	
			ROS_INFO(" FirstLifting ");
			tool->Delay(8000);
			ros_com->sendBodySector(weightlifting_info->Firstlifting_sup_sector);	
			tool->Delay(10000);
			if(continuous_flag == false)
			{	 
				ros_com->sendBodyAuto(weightlifting_info->speed,weightlifting_info->continuous_Y,0,weightlifting_info->continuous_theta, WalkingMode::ContinuousStep,SensorMode(weightlifting_info->continuous_imu));
				continuous_flag = true;
				tool->Delay(4000);
			}
			ros_com->sendHeadMotor(HeadMotorID::VerticalID,1200,100);
			tool->Delay(2500);
			weightlifting_info->BodyState = checkmidline;
			weightlifting_info->time_start = ros::WallTime::now().toSec()*1000;
			weightlifting_info->time_end = ros::WallTime::now().toSec()*1000;
			break;
		
		case checkmidline: //判斷中線距離
			ROS_INFO("flaggggggggggggggggggggggggggggg = %d",weightlifting_info->finallookline_flag);
			int check_fix ;
			ROS_INFO(" YMAX = %d ",(weightlifting_info->white[1][1]));
			weightlifting_info->time_end = ros::WallTime::now().toSec()*1000;
			ROS_INFO("NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN = %f",weightlifting_info->time_end - weightlifting_info->time_start);
			
			ROS_INFO(" imu_initial %f",weightlifting_info->imu_initial);
			ros::spinOnce();
			ROS_INFO(" Yaw = %f",strategy_info->getIMUValue().Yaw);
			ROS_INFO(" ccount = %d",weightlifting_info->ccount);
			if(strategy_info->getIMUValue().Yaw - weightlifting_info->imu_initial <= -1)
			{
				ROS_INFO("add theta");
				check_fix = 2;
			}
			else if(strategy_info->getIMUValue().Yaw - weightlifting_info->imu_initial >= 1)
			{
				ROS_INFO("sub theta");
				check_fix = -1;
			}
			else
			{
				check_fix = 0;
			}
			if((weightlifting_info->white[1][1]) > 160 && weightlifting_info->finallookline_flag)
			{
				ROS_INFO("testthreeeeeeeeeeeeeeeeeeeeee");
				weightlifting_info->closeimage = true;
				if(weightlifting_info->time_end - weightlifting_info->time_start > 90 && weightlifting_info->speed > 1000)
				{
					weightlifting_info->speed = weightlifting_info->speed - 40;
					weightlifting_info->time_start = ros::WallTime::now().toSec()*1000;
					ros_com->sendContinuousValue(weightlifting_info->speed,weightlifting_info->continuous_Y,0,weightlifting_info->continuous_theta + check_fix,SensorMode(weightlifting_info->continuous_imu));
				}
				else if (weightlifting_info->time_end - weightlifting_info->time_start > 90 && weightlifting_info->speed > 0)
				{
					weightlifting_info->speed = weightlifting_info->speed - 150;
					if(weightlifting_info->speed < 0)
					{
						weightlifting_info->speed = 0;
					}
						
					weightlifting_info->time_start = ros::WallTime::now().toSec()*1000;	
					ros_com->sendContinuousValue(weightlifting_info->speed,weightlifting_info->continuous_Y,0,weightlifting_info->continuous_theta + check_fix,SensorMode(weightlifting_info->continuous_imu));
				}
				else if (weightlifting_info->time_end - weightlifting_info->time_start > 90&& weightlifting_info->speed <= 0)
				{
					weightlifting_info->BodyState = SecondLifting;
					weightlifting_info->closeimage = false;
					weightlifting_info->time_end = ros::WallTime::now().toSec()*1000;
					weightlifting_info->time_start = ros::WallTime::now().toSec()*1000;
				}
			}
			else if(weightlifting_info->white[1][1] <= 80&& weightlifting_info->white[1][1] > 60)
			{
				ROS_INFO("testoneOOOOOOOOOOOOOOOOOOOOOOOOOO");
				weightlifting_info->finallookline_flag = true;
				if(weightlifting_info->time_end - weightlifting_info->time_start > 90 && weightlifting_info->speed < 1200)
				{
					weightlifting_info->speed = weightlifting_info->speed + 50;
					weightlifting_info->time_start = ros::WallTime::now().toSec()*1000;
					ros_com->sendContinuousValue(weightlifting_info->speed,weightlifting_info->continuous_Y,0,weightlifting_info->continuous_theta + check_fix,SensorMode(weightlifting_info->continuous_imu));
				}
				else if(weightlifting_info->time_end - weightlifting_info->time_start > 90)
				{
					ros_com->sendContinuousValue(weightlifting_info->speed,weightlifting_info->continuous_Y,0,weightlifting_info->continuous_theta + check_fix,SensorMode(weightlifting_info->continuous_imu));
					weightlifting_info->time_start = ros::WallTime::now().toSec()*1000;
				}
			}
			else
			{
				weightlifting_info->ccount = weightlifting_info->ccount + 1;
				if(weightlifting_info->ccount > 1)
				{
					ROS_INFO("testotwo1111111111111111111111111111");
					if(weightlifting_info->time_end - weightlifting_info->time_start > 90 && weightlifting_info->speed < 1200)
					{
						weightlifting_info->speed = weightlifting_info->speed + 50;
						weightlifting_info->time_start = ros::WallTime::now().toSec()*1000;
						ros_com->sendContinuousValue(weightlifting_info->speed,weightlifting_info->continuous_Y,0,weightlifting_info->continuous_theta + check_fix,SensorMode(weightlifting_info->continuous_imu));
					}
					else if(weightlifting_info->time_end - weightlifting_info->time_start > 90)
					{
						ros_com->sendContinuousValue(weightlifting_info->speed,weightlifting_info->continuous_Y,0,weightlifting_info->continuous_theta + check_fix,SensorMode(weightlifting_info->continuous_imu));
						weightlifting_info->time_start = ros::WallTime::now().toSec()*1000;
					}
				}
			}
			ROS_INFO(" Speed = %d ",weightlifting_info->speed);

			break;

		case SecondLifting://Lifting_up 二舉
			
			if(continuous_flag == true){
				ros_com->sendBodyAuto(-200,50,0,0,WalkingMode::ContinuousStep, SensorMode::None);
				ROS_INFO(" Stop ");
				tool->Delay(3000);
				continuous_flag = false;
			}			
				ros_com->sendBodySector(weightlifting_info->Secondlifting_sector);	
				ROS_INFO(" SecondLifting ");
				tool->Delay(5000);
				ros_com->sendBodySector(weightlifting_info->Secondlifting_sup_sector);	
				tool->Delay(5000);		
			weightlifting_info->finallookline_flag = false;
			weightlifting_info->BodyState = WalkingOverTape;
			break;

		case WalkingOverTape:  //離開SecondLifting line
			
			int final_fix ;
			ROS_INFO(" initial %f",weightlifting_info->imu_initial);
			ros::spinOnce();
			ROS_INFO("%f",strategy_info->getIMUValue().Yaw);
			if(strategy_info->getIMUValue().Yaw - weightlifting_info->imu_initial <= -1)
			{
				ROS_INFO("add theta");
				final_fix = 1;
			}
			else if(strategy_info->getIMUValue().Yaw - weightlifting_info->imu_initial >= 1)
			{
				ROS_INFO("sub theta");
				final_fix = -1;
			}
			else
			{
				final_fix = 0;
			}

			if(continuous_flag == false)
			{
				ros_com->sendBodyAuto(weightlifting_info->continuous_speed,weightlifting_info->continuous_Y,0,weightlifting_info->continuous_theta+ final_fix, WalkingMode::ContinuousStep,SensorMode(weightlifting_info->continuous_imu));//
				tool->Delay(200);
				continuous_flag = true;
			}
			if(weightlifting_info->white[1][1] >= 180 && weightlifting_info->white[1][1] <= 240 && weightlifting_info->finallookline_flag) //finallookline_flag 開啟後就不再修正, 直接衝線
			{
				while(strategy_info->getStrategyStart() && ros::ok())
				{	
					ROS_INFO("Speed Up");
					ROS_INFO("continuous_speed = %d",weightlifting_info->continuous_speed);
					if(weightlifting_info->continuous_speed<weightlifting_info->speed_up)
					{
						weightlifting_info->continuous_speed = weightlifting_info->continuous_speed+100;

					}
					else if(weightlifting_info->continuous_speed>=weightlifting_info->speed_up)
					{
						weightlifting_info->continuous_speed=weightlifting_info->speed_up;
					}
					ros_com->sendContinuousValue(weightlifting_info->continuous_speed,weightlifting_info->continuous_Y,0,weightlifting_info->continuous_theta + final_fix,SensorMode(weightlifting_info->continuous_imu));
					ros::spinOnce();
					ros::WallDuration(0.1).sleep();
					//ROS_INFO("goal");
				}
			}
			else if(weightlifting_info->white[1][1] < 180 && weightlifting_info->white[1][1] >= 0)
			{
				weightlifting_info->finallookline_flag = true;
				ros_com->sendContinuousValue(weightlifting_info->continuous_speed,weightlifting_info->continuous_Y,0,weightlifting_info->continuous_theta+final_fix,SensorMode(weightlifting_info->continuous_imu));
			}
		break;

	}
	return true;
}
