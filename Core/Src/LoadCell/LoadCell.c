/*
 * LoadCell.c
 *
 *  Created on: 07-Aug-2020
 *      Author: shreyasnr
 */

#include "LoadCell/LoadCell.h"

void ReadLoadcell()
{
	  if(BlowerFunctionalTime == 0)
	  {
		  DryIceLoad = HX711_valueUnits(NumOfSamples,DRY_ICE_LOAD);
		  BlowerOnOff = 0;
	  }
	  else
	  {
		  BlowerOnOff = 5;
	  }
	  SalineOneLoad = HX711_valueUnits(NumOfSamples,PERFUSION_FLUID_LOAD);
	  SalineTwoLoad = HX711_valueUnits(NumOfSamples,WASTE_FLUID_LOAD);

	  if(DryIceLoad < 0)
		  DryIceLoad = 0;
	  if(SalineOneLoad < 0)
		  SalineOneLoad = 0;
	  if(SalineTwoLoad < 0)
		  SalineTwoLoad = 0;
}
