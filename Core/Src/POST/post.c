/*
 * post.c
 *
 *  Created on: Aug 24, 2020
 *      Author: shreyasnr
 */

#include "POST/post.h"

void PowerOnSelfTest() {
	switch (RxOptionData[0]) {
	case '1':
		StartBlower();
		break;
	case '2':
		StopBlower();
		break;
	case '3':
		StartExhaustFan();
		break;
	case '4':
		StopExhaustFan();
		break;
	case '5':
		OpenSolenoidValve();
		break;
	case '6':
		CloseSolenoidValve();
		break;
	case '7':
		ReadTemperatureSensorOne();
		break;
	case '8':
		ReadTemperatureSensorTwo();
		break;
	case '9':
		ReadHeartTemperatureSensor();
		break;
	case 'a':
		ReadHumidtySensor();
		break;
	case 'b':
		ReadLoadcell();
		break;
	case 'c':
		StartPumpOne();
		break;
	case 'd':
		StopPumpOne();
		break;
	case 'e':
		ReadEncoder();
		break;
	case 'f':
		StartPumpTwo();
		break;
	case 'g':
		StopPumpTwo();
		break;
	case 'h':
		ReadPressureSensor();
		break;
	case 'i':
		ReadPHSensor();
		break;
	case 'j':
		ReadFlowSensor();
		break;
	case 'k':
		TestCommunication();
		break;
	case 'p':
		StartCoolingChamber();
		break;
	case 's':
		StopCoolingChamber();
		break;
	case 'x':
		ExitMenu();
		break;
	default:
		break;
	}
}
