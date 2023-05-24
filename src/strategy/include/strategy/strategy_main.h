#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <stdio.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
//----------------libs----------------
#include "tku_libs/strategy_info.h"
#include "tku_libs/TKU_tool.h"
#include "tku_libs/RosCommunication.h"
//---------------header---------------
#include "strategy/WeightLiftingInfo.h"

using namespace std;

class WeightLifting
{
	public:
		RosCommunication* ros_com ;
		StrategyInfo* strategy_info;
		Tool* tool;
		WeightLifting(ros::NodeHandle &nh)
		{
			strategy_info = StrategyInfoInstance::getInstance();
			tool = ToolInstance::getInstance();
			ros_com = RosCommunicationInstance::getInstance();
		};
		~WeightLifting(){};
		void strategyMain();
		void firstSpeedControl(void);
		bool strategyClassify(void);
		bool strategyClassify2(void);
		bool strategyBody(void);
		void initparameterpath();
		void loadParameter(void);
		void initialparameter(void);
		string parameter_path="N";
		bool continuous_flag =false;
};
