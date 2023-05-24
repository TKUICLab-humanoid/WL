#include "strategy/WeightLiftingInfo.h"
#include "tku_libs/strategy_info.h"

WeightLiftingInfo* weightlifting_info = new WeightLiftingInfo();

WeightLiftingInfo::WeightLiftingInfo(void)
{
	DIO_fake = 0;
	BodyState = 0;
	HeadPostitionX = 511;
	HeadPostitionY = 350;
	etLine = (int)LabelModel::Orange;
	LineLabel = (int)LabelMark::OrangeLabel;
	LineLabel2 = (int)LabelMark::RedLabel;
	//===========================================================================
	//point_pos[4][4] = 0;


	count = 0;
	finallookline_flag = false;
	closeimage = false;
}
