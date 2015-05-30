/*
 * BlueBerry battery driver based on EC
 *
 * Copyright (C) 2013-2013 Paul Ma <magf@bitland.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Trademarks are the property of their respective owners.
 */

#ifndef _BLUEBERRY_BAT_H
#define _BLUEBERRY_BAT_H

/* EC register base address */
#define regBatBase						0x30		/* 0x30 to 0x7f is battery data in EC space */

/* reg address for blueberry_bat */
#define regBatControl					0x00		/* _Control 0x00 */
#define regBatAtRate 					0x02		/* _AtRate  0x02 */
#define regBatAtRateTimeToEmpty			0x04		/* _AtRateTimeToEmpty 0x04 */
#define regBatTemperature				0x06		/* _Temperature 0x06 */
#define regBatVoltage					0x08		/* _Voltage 0x08 */
#define regBatFlags						0x0A		/* _Flags 0x0A */
#define regBatNominalAvailableCapacity	0x0C		/* _NominalAvailableCapacity 0x0C */
#define regBatFullAvailableCapacity		0x0E		/* _FullAvailableCapacity 0x0E */
#define regBatRemainingCapacity			0x10		/* _RemainingCapacity 0x10 */
#define regBatFullChargeCapacity		0x12		/* _FullChargeCapacity 0x12 */
#define regBatAverageCurrent			0x14		/* _AverageCurrent 0x14 */
#define regBatTimeToEmpty				0x16		/* _TimeToEmpty 0x16 */
#define regBatTimeToFull				0x18		/* _TimeToFull 0x18 */
#define regBatStandbyCurrent			0x1A		/* _StandbyCurrent 0x1A */
#define regBatStandbyTimeToEmpty		0x1C		/* _StandbyTimeToEmpty 0x1C */
#define regBatMaxLoadCurrent			0x1E		/* _MaxLoadCurrent 0x1E */
#define regBatMaxLoadToEmpty			0x20		/* _MaxLoadToEmpty 0x20 */
#define regBatAvailableEnergy			0x22		/* _AvailableEnerty 0x22 */
#define regBatAveragePower				0x24		/* _AveragePower 0x24 */
#define regBatTTEatConstantPower		0x26		/* _TTEatConstantPower 0x26 TimeToEmptyAtConstantPower */
#define regBatInternalTemp				0x28		/* _InternalTemp 0x28 */
#define regBatCycleCount				0x2A		/* _CycleCount 0x2A */
#define regBatStateOfCharge				0x2C		/* _StateOfCharge 0x2C */
#define regBatStateOfHealth				0x2E		/* _StateOfHealth 0x2E */
#define regBatPassedCharge				0x30		/* _PassedCharge 0x34 */
#define regBatDOD0						0x32		/* _DOD0 0x36 */
#define regBatPackConfig				0x34		/* _PackConfig 0x38 */
#define regBatDesignCapacity			0x36		/* _DesignCapacity 0x3c */
#define regBatDeviceNameLength			0x38		/* byte length of device name */
/* the following is 16 bytes buffer of EC */
/* unsigned char NameBuffer[16];          */
#define regBatACDCState					0x4A		/* bit 0: dc state, bit 1: ac state */

/* BatFlags flag */
#define BLUEBERRY_BAT_FLAG_DSC        	BIT(0)
#define BLUEBERRY_BAT_FLAG_CHGS       	BIT(8)
#define BLUEBERRY_BAT_FLAG_FC         	BIT(9)
#define BLUEBERRY_FLAG_OTD        		BIT(14)
#define BLUEBERRY_FLAG_OTC        		BIT(15)

/* ACDCState */
#define BLUEBERRY_DC_PRESENT			BIT(0)
#define BLUEBERRY_AC_PRESNET			BIT(1)

#endif
