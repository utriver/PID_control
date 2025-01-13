/*
 * SensorConv.cpp
 *
 *  Created on: 2020. 10. 16.
 *      Author: MesAbay
 */

#include "Controllers.h"

using namespace std;

void Controllers::joint_to_encoder(JointVec joint, INT32 *enc)
{
  	int axis=0;
	for(axis =0; axis<NUM_AXIS; axis++)
	{
			enc[axis]=joint(axis,0)*ECAT2RAD;		
	}

}
void Controllers::encoder_to_joint(INT32 *enc, JointVec joint)
{
  

}