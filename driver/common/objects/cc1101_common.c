/**
*  ----------------------------------------------------------------------------
*  Copyright (c) 2012-13, Anaren Microwave, Inc.
*
*  ----------------------------------------------------------------------------
*
*  CC1101.c - CC110x/2500 device driver.
*
*  @version	2.0.0
*  @date	   15 Jan 2013
*  @author	Kieron Gillespie, kgillespie@anaren.com 
* 		BPB, air@anaren.com
*
*/
#include "cc1101.h"

#ifndef NULL
#define NULL  (void*)0
#endif

#ifndef ST
#define ST(X) do { X } while (0)
#endif

#ifndef ABS
#define ABS(a) ((a) < 0 ? -(a) : (a))
#endif

unsigned char CC1101_GetChipPartnum()
{
	return CC1101_GetRegister(CC1101_PARTNUM);
}

unsigned char CC1101_GetChipVersion()
{
	return CC1101_GetRegister(CC1101_VERSION);
}


static bool bCC1101Sleep = false;

void CC1101_Read(unsigned char address, unsigned char *data, unsigned int count)
{
	//void AIR_SPI_Read(int cspin, unsigned char *writeBytes, unsigned int numWriteBytes);
}

void CC1101_Write(unsigned char address, unsigned char *data, unsigned int count)
{
	//void AIR_SPI_Write(int cspin, unsigned char *writeBytes, unsigned int numWriteBytes);
}


bool CC1101_GetSleepState()
{
	return bCC1101Sleep;
}

unsigned char CC1101_GetRxFifoCount()
{
	return CC1101_GetRegister(CC1101_RXBYTES);
}

unsigned char CC1101_GetTxFifoCount()
{
	return CC1101_GetRegister(CC1101_TXBYTES);
}

enum CC1101_MarcState CC1101_GetMarcState()
{
	(enum CC1101_MarcState)CC1101_GetRegister(CC1101_MARCSTATE);
}

unsigned char CC1101_GetRssi()
{
	return CC1101_GetRegister(CC1101_RSSI);
}

unsigned char CC1101_GetLqi()
{
	return ((CC1101_GetRegister(CC1101_LQI)) & CC1101_LQI_EST);
}

unsigned char CC1101_GetCrc()
{
	return ((CC1101_GetRegister(CC1101_LQI)) & CC1101_CRC_OK);
}

void CC1101_Reset()
{
	CC1101_Write((CC1101_SRES & 0xBF), NULL, 0);
}

void CC1101_EnableFrequencySynthesizer()
{
	CC1101_Write((CC1101_SFSTXON & 0xBF), NULL, 0);
}

void CC1101_ReceiverOn()
{
	CC1101_Write((CC1101_SRX & 0xBF), NULL, 0);
}

void CC1101_Transmit()
{
	CC1101_Write((CC1101_STX & 0xBF), NULL, 0);
}

void CC1101_Idle()
{
	CC1101_Write((CC1101_SIDLE & 0xBF), NULL, 0);
}

void CC1101_StartWakeOnRadio()
{
	CC1101_Write((CC1101_SWOR & 0xBF), NULL, 0);
}

void CC1101_FlushRxFifo()
{
	CC1101_Write((CC1101_SFRX & 0xBF), NULL, 0);
}

void CC1101_FlushTxFifo()
{
	CC1101_Write((CC1101_SFTX & 0xBF), NULL, 0);
}

void CC1101_ResetWakeOnRadio()
{
	CC1101_Write((CC1101_SWORRST & 0xBF), NULL, 0);
}

void CC1101_Nop()
{
	CC1101_Write((CC1101_SNOP & 0xBF), NULL, 0);
}

void CC1101_WaitForSyncSent()
{
	return;
}

void CC1101_WaitForTransmitComplete()
{
	return;
}

unsigned char CC1101_GetRegisterWithSpiSyncProblem( unsigned char address)
{
	unsigned char i;
	unsigned char state[4];

	for (i = 0; i < 4; i++)
	{
		CC1101_Read(address, (unsigned char *)(state + i), 1);
		// If two consecutive reads yield the same result, then we are guaranteed
		// that the value is valid; no need to continue further...
		if ((i > 0) && (state[i] == state[i-1]))
		{
			break;
		}
	}

	return state[i];
}

bool CC1101_TimeoutEvent(unsigned int *tick)
{
	if (*tick < CC1101_MAX_TIMEOUT)
	{
		(*tick)++;
		return false;
	}

	return true;
}

bool CC1101_SetAndVerifyState(unsigned char command, enum CC1101_MarcState state)
{
	unsigned int tick = 0;

	//CC1101_Strobe(command);
	CC1101_Write((command & 0xBF), NULL, 0);
	while (CC1101_GetMarcState() != state)
	{
		if (CC1101_TimeoutEvent(&tick))
		{
			// TODO: Add a reset-radio routine. Some of the operational states may
			// cause a irreversable state.
			return false;
		}
	}

	return true;
}

void CC1101_Init()
{
	bCC1101Sleep = false;
}

bool CC1101_Configure(const struct sCC1101 *config)
{
	/**
	*  Make sure the radio is in an IDLE state before attempting to configure any
	*  packet handling registers. See Table 28 in the CC1101 User's Guide for
	*  more information (swrs061g).
	*/
	if (!CC1101_SetAndVerifyState(CC1101_SIDLE, CC1101_MarcStateIdle))
	{
		return false;
	}

	CC1101_Write(0x00, (unsigned char *)((struct sCC1101*)config), sizeof(struct sCC1101)/sizeof(unsigned char));

	return true;
}

unsigned char CC1101_GetRegister(unsigned char address)
{
	switch (address)
	{
		/**
		*  Note: In order to allow for efficient continuous reading of the RSSI
		*  value, the CC1101_RSSI case has been commented out. RSSI is not
		*  protected by the fix for the Errata Notes issue. A fix must be performed
		*  outside of this device driver (such as averaging).
		*/
		case CC1101_RSSI:
		case CC1101_FREQEST:
		case CC1101_MARCSTATE:
		case CC1101_RXBYTES:
		case CC1101_TXBYTES:
		case CC1101_WORTIME1:
		case CC1101_WORTIME0:
			return CC1101_GetRegisterWithSpiSyncProblem(address);
			break;
			
		default:
		{
			unsigned char value;
			CC1101_Read(address, &value, 1);
			return value;
			break;
		}
	}
}

void CC1101_SetRegister(unsigned char address, unsigned char value)
{
	CC1101_Write(address, &value, 1);
}

void CC1101_ReadRegisters(unsigned char address, unsigned char *buffer, unsigned char count)
{
	if (count == 1)
	{
		*buffer = CC1101_GetRegister(address);
	}
	else if (count > 1)
	{
		CC1101_Read(address, buffer, count);
	}
}

void CC1101_WriteRegisters(unsigned char address, unsigned char *buffer, unsigned char count)
{
	CC1101_Write(address, buffer, count);
}

enum CC1101_Chip CC1101_GetChip()
{
	unsigned char partNum = CC1101_GetChipPartnum();
	unsigned char version = CC1101_GetChipVersion();


	if (partNum == CC1101_CHIPPARTNUM && version == CC1101_CHIPVERSION)
		return CC1101_Chip1101;
	else if (partNum == CC2500_CHIPPARTNUM && version == CC2500_CHIPVERSION)
		return CC1101_Chip2500;
	else if (partNum == CC110L_CHIPPARTNUM && version == CC110L_CHIPVERSION)
		return CC1101_Chip110L;
	else
		return CC1101_ChipUnknown;
}

unsigned char CC1101_ReadRxFifo(unsigned char *buffer, unsigned char count)
{
	unsigned char rxBytes = CC1101_GetRxFifoCount();

	if (rxBytes < count && rxBytes > 0)
	{
		CC1101_Read(CC1101_RXFIFO, buffer, rxBytes);
		return rxBytes;
	}
	else if (rxBytes > 0)
	{
		CC1101_Read(CC1101_RXFIFO, buffer, count);
		return count;
	}
	
	return 0;
}

void CC1101_WriteTxFifo( unsigned char *buffer, unsigned char count)
{
	CC1101_Write(CC1101_TXFIFO, buffer, count);
}

bool CC1101_TurnOffCrystalOscillator()
{
	// CC1101 must be in an IDLE state before powering down. Please see section
	// 19.2 - "Crystal Control" for more inforamtion.
	if (!CC1101_SetAndVerifyState(CC1101_SIDLE, CC1101_MarcStateIdle))
	{
		return false;
	}

	CC1101_Write((CC1101_SXOFF & 0xBF), NULL, 0);
	
	return true;
}

bool CC1101_Calibrate()
{
	// Calibrate once radio is in IDLE state.
	if (!CC1101_SetAndVerifyState(CC1101_SIDLE, CC1101_MarcStateIdle))
	{
		return false;
	}

	CC1101_Write((CC1101_SCAL & 0xBF), NULL, 0);
	
	return true;
}

bool CC1101_Sleep()
{
	if (!bCC1101Sleep)
	{
		// CC1101 must be in an IDLE state before powering down. Please see section
		// 19.2 - "Crystal Control" for more inforamtion.
		if (!CC1101_SetAndVerifyState(CC1101_SIDLE, CC1101_MarcStateIdle))
		{
			return false;
		}

		/**
		*  Once the radio is asleep, any CSn toggling will wake the radio up. We have
		*  to assume the radio is going to sleep and must therefore set the state
		*  without the use of CC1101GetMarcState().
		*/
		CC1101_Write((CC1101_SPWD & 0xBF), NULL, 0);
		bCC1101Sleep = true;
	}

	return true;
}

void CC1101_Wakeup(const unsigned char agctest, const unsigned char test[3], const unsigned char *paTable, unsigned char paTableSize)
{
	/**
	*  Assumes that the SPI Read/Write implementation automatically handles
	*  asserting CSn for this radio (active low) and has waited for the SPI MISO
	*  pin to go low (CHIP_RDYn). If this is not done, this procedure WILL NOT
	*  WORK CORRECTLY.
	*/
	if (bCC1101Sleep)
	{
		// If the radio is coming out of a sleep state, perform a wake up routine to
		// reestablish the initial radio state.
		CC1101_Write((CC1101_SIDLE & 0xBF), NULL, 0);
		bCC1101Sleep = false;

		/**
		*  The last valid calibration results are maintained so calibration is
		*  still valid after waking up from a low power state. The TX/RX FIFOs have
		*  been flushed. If IOCFGx.GDOx_CFG setting is less that 0x20 and
		*  IOCFGx.GDOx_INV is 0(1), GDO0 and GDO2 pins are hardwired to 0(1), and
		*  GDO1 is hardwired to 1(0), until CSn goes low. The following registers
		*  are not retained and must be rewritten: AGCTEST, TEST2, TEST1, TEST0,
		*  PATABLE(contents of PATABLE are lost except the first byte).
		*/
		CC1101_Write(CC1101_REG_AGCTEST, &agctest, 1);
		CC1101_Write(CC1101_REG_TEST2, test, 3);
		CC1101_Write(CC1101_PATABLE, paTable, paTableSize);
	}
}
