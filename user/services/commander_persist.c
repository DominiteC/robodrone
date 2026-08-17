/*
 * commander_persist.c
 * EEPROM 模式持久化：attitude / servo 模式掉电保存和恢复。
 * 本文件不依赖控制算法，仅通过 AT24Cxx 操作外部 EEPROM。
 */
#include "commander.h"
#include "AT24Cxx.h"

#define EEPROM_MODE_MAGIC      0xA5U
#define EEPROM_ADDR_MAGIC      0x00U
#define EEPROM_ADDR_ATTI_MODE  0x01U
#define EEPROM_ADDR_SERVO_MODE 0x02U

static void CommanderPersist_EnsureMagic(void)
{
	uint8_t magic = 0;
	if (AT24Cxx_Read_nByteBuf(EEPROM_ADDR_MAGIC, &magic, 1) != 0 || magic != EEPROM_MODE_MAGIC)
	{
		magic = EEPROM_MODE_MAGIC;
		(void)AT24Cxx_Write_nByte_In_One_Block(EEPROM_ADDR_MAGIC, &magic, 1);
	}
}

bool CommanderPersist_LoadModes(uint8_t *attitudeMode, uint8_t *servoMode)
{
	uint8_t magic = 0;
	uint8_t at = 0;
	uint8_t sv = 0;

	if (AT24Cxx_Read_nByteBuf(EEPROM_ADDR_MAGIC, &magic, 1) != 0
		|| AT24Cxx_Read_nByteBuf(EEPROM_ADDR_ATTI_MODE, &at, 1) != 0
		|| AT24Cxx_Read_nByteBuf(EEPROM_ADDR_SERVO_MODE, &sv, 1) != 0)
	{
		return false;
	}

	if (magic != EEPROM_MODE_MAGIC || at > MODE_WALK_45 || sv > 2U)
	{
		return false;
	}

	*attitudeMode = at;
	*servoMode = sv;
	return true;
}

void CommanderPersist_SaveModes(uint8_t attitudeMode, uint8_t servoMode)
{
	CommanderPersist_EnsureMagic();
	(void)AT24Cxx_Write_nByte_In_One_Block(EEPROM_ADDR_ATTI_MODE, &attitudeMode, 1);
	(void)AT24Cxx_Write_nByte_In_One_Block(EEPROM_ADDR_SERVO_MODE, &servoMode, 1);
}

void CommanderPersist_SaveAttitudeMode(uint8_t attitudeMode)
{
	CommanderPersist_EnsureMagic();
	(void)AT24Cxx_Write_nByte_In_One_Block(EEPROM_ADDR_ATTI_MODE, &attitudeMode, 1);
}

void CommanderPersist_SaveServoMode(uint8_t servoMode)
{
	CommanderPersist_EnsureMagic();
	(void)AT24Cxx_Write_nByte_In_One_Block(EEPROM_ADDR_SERVO_MODE, &servoMode, 1);
}
