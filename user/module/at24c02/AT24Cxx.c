#include "AT24Cxx.h"

static uint8_t AT24Cxx_Write_nByte_In_One_Page(uint16_t WriteMemAddr, uint8_t *WriteDataBuf, uint16_t WriteDataSize);

/**
 * @function: static uint8_t AT24Cxx_Write_nByte_In_One_Page(uint16_t WriteMemAddr, uint8_t *WriteDataBuf, uint16_t WriteDataSize)
 * @description: 在AT24Cxx的一页中写入n个byte数据(AT24C01/02:n<=8、04/08/16:n<=16)
 * @param {uint16_t} WriteMemAddr待写入数据的内存首地址
 * @param {uint8_t} *WriteDataBuf待写入的数据
 * @param {uint16_t} WriteDataSize待写入的数据大小(byte)
 * @return {0} 写入成功
 * @return {1} 写入失败
 */
static uint8_t AT24Cxx_Write_nByte_In_One_Page(uint16_t WriteMemAddr, uint8_t *WriteDataBuf, uint16_t WriteDataSize)
{
#if AT24Xxx_USING_HARDWARE_I2C
  if (HAL_I2C_Mem_Write(&I2Cx, AT24CXX_ADDRESS, WriteMemAddr, I2C_MEMADD_SIZE_8BIT, WriteDataBuf, WriteDataSize, AT24CXX_MAX_TIMEOUT) == HAL_OK)
  {
    while (HAL_I2C_GetState(&I2Cx) != HAL_I2C_STATE_READY)
    {
    }
    /* Check if the EEPROM is ready for a new operation */
    while (HAL_I2C_IsDeviceReady(&I2Cx, AT24CXX_ADDRESS, AT24CXX_MAX_TRIALS, AT24CXX_MAX_TIMEOUT) == HAL_TIMEOUT)
    {
    }
    /* Wait for the end of the transfer */
    while (HAL_I2C_GetState(&I2Cx) != HAL_I2C_STATE_READY)
    {
    }
    return 0;
  }
  else
    return 1;
#endif
#if AT24Xxx_USING_SOFTWARE_I2C
  Software_I2C_Start();
  Software_I2C_WriteByte(AT24CXX_ADDRESS & 0xFE);
  if (Software_I2C_WaitACK())
  {
    Software_I2C_Stop();
    return 1;
  }
  Software_I2C_WriteByte(WriteMemAddr);
  if (Software_I2C_WaitACK())
  {
    Software_I2C_Stop();
    return 1;
  }
  while (WriteDataSize--)
  {
    Software_I2C_WriteByte(*WriteDataBuf++);
    if (Software_I2C_WaitACK())
    {
      Software_I2C_Stop();
      return 1;
    }
  }
  Software_I2C_Stop();
  AT24Cxx_Delay_ms(5);
  return 0;
#endif
}

/**
 * @function: uint8_t AT24Cxx_Write_nByte_In_One_Block(uint16_t WriteMemAddr, uint8_t *WriteDataBuf, uint16_t WriteDataSize)
 * @description: 在AT24Cxx的一个数据块中写入n个byte数据(AT24C01/02/04/08/16:n<=256)
 * @param {uint16_t} WriteMemAddr待写入数据的内存首地址
 * @param {uint8_t} *WriteDataBuf待写入的数据
 * @param {uint16_t} WriteDataSize待写入的数据大小(byte)
 * @return {0} 写入成功
 * @return {1} 写入失败
 */
uint8_t AT24Cxx_Write_nByte_In_One_Block(uint16_t WriteMemAddr, uint8_t *WriteDataBuf, uint16_t WriteDataSize)
{
  uint8_t NumOfPage = 0, NumOfSingle = 0, AddressAligned = 0, FreeNumOfPage = 0;
  AddressAligned = WriteMemAddr % AT24CXX_PAGE_SIZE;  //地址刚好与每页可存入数据大小对齐后的余数(判断传入地址是不是每页首地址)
  FreeNumOfPage = AT24CXX_PAGE_SIZE - AddressAligned; //从内存地址到一页末尾剩余的地址空间
  //从页中开始写
  //传入内存地址不是每页的首地址，AddressAligned!=0(先填满该页，再按页写入，再写入剩下不满一页的)
  if (AddressAligned != 0)
  {
    //写入数据个数小于等于传入地址后该页剩余的空间，则将数据直接写入，写入WriteDataSize个数据
    if (WriteDataSize <= FreeNumOfPage)
    {
      if (AT24Cxx_Write_nByte_In_One_Page(WriteMemAddr, WriteDataBuf, WriteDataSize))
        return 1;
      else
        return 0;
    }
    else //传入地址该页剩余的空间小于数据个数，则将该页填满，写入FreeNumOfPage个数据
    {
      if (AT24Cxx_Write_nByte_In_One_Page(WriteMemAddr, WriteDataBuf, FreeNumOfPage))
        return 1;
      //内存首地址/待写入数据进行偏移
      WriteMemAddr += FreeNumOfPage;
      WriteDataBuf += FreeNumOfPage;
      WriteDataSize -= FreeNumOfPage;
    }
  }
  //从页首开始写
  NumOfPage = WriteDataSize / AT24CXX_PAGE_SIZE;   //可以写满一页的次数
  NumOfSingle = WriteDataSize % AT24CXX_PAGE_SIZE; //写不满一页的数量
  //传入的数据个数小于等于一页，则将数据直接写入，写入WriteDataSize个数据
  if (WriteDataSize <= AT24CXX_PAGE_SIZE)
  {
    if (AT24Cxx_Write_nByte_In_One_Page(WriteMemAddr, WriteDataBuf, WriteDataSize))
      return 1;
    else
      return 0;
  }
  //待写入的数据超一页，按页写入具体要写入的页数，写入NumOfPage个页
  while (NumOfPage--)
  {
    if (AT24Cxx_Write_nByte_In_One_Page(WriteMemAddr, WriteDataBuf, AT24CXX_PAGE_SIZE))
      return 1;
    WriteMemAddr += AT24CXX_PAGE_SIZE;
    WriteDataBuf += AT24CXX_PAGE_SIZE;
  }
  //剩下写不满一页的数据，直接写入NumOfSingle个数据
  if (NumOfSingle != 0)
  {
    if (AT24Cxx_Write_nByte_In_One_Page(WriteMemAddr, WriteDataBuf, NumOfSingle))
      return 1;
    else
      return 0;
  }
  return 0;
}

/**
 * @function: uint8_t AT24Cxx_Read_nByteBuf(uint16_t ReadMemAddr, uint8_t *ReadBuf, uint16_t ReadDataSize)
 * @description: AT24Cxx读取n个byte的数据
 * @param {uint16_t} ReadMemAddr读取数据的内存首地址
 * @param {uint8_t} *ReadBuf存放数据的数组
 * @param {uint16_t} ReadDataSize读取的数据数量(byte)
 * @return {0} 写入成功
 * @return {1} 写入失败
 */
uint8_t AT24Cxx_Read_nByteBuf(uint16_t ReadMemAddr, uint8_t *ReadBuf, uint16_t ReadDataSize)
{
#if AT24Xxx_USING_HARDWARE_I2C
  while (HAL_I2C_GetState(&I2Cx) != HAL_I2C_STATE_READY)
  {
  }
  /* Check if the EEPROM is ready for a new operation */
  while (HAL_I2C_IsDeviceReady(&I2Cx, AT24CXX_ADDRESS, AT24CXX_MAX_TRIALS, AT24CXX_MAX_TIMEOUT) == HAL_TIMEOUT)
  {
  }
  /* Wait for the end of the transfer */
  while (HAL_I2C_GetState(&I2Cx) != HAL_I2C_STATE_READY)
  {
  }
  if (HAL_I2C_Mem_Read(&I2Cx, AT24CXX_ADDRESS, ReadMemAddr, I2C_MEMADD_SIZE_8BIT, ReadBuf, ReadDataSize, AT24CXX_MAX_TIMEOUT) == HAL_OK)
    return 0;
  else
    return 1;
#endif
#if AT24Xxx_USING_SOFTWARE_I2C
  Software_I2C_Start();
  Software_I2C_WriteByte(AT24CXX_ADDRESS & 0xFE);
  if (Software_I2C_WaitACK())
  {
    Software_I2C_Stop();
    return 1;
  }
  Software_I2C_WriteByte(ReadMemAddr);
  if (Software_I2C_WaitACK())
  {
    Software_I2C_Stop();
    return 1;
  }
  Software_I2C_Start();
  Software_I2C_WriteByte(AT24CXX_ADDRESS | 0x01);
  Software_I2C_WaitACK();
  if (Software_I2C_WaitACK())
  {
    Software_I2C_Stop();
    return 1;
  }
  for (uint16_t i = 0; i < ReadDataSize; i++)
  {
    *ReadBuf = Software_I2C_ReadByte();
    if (ReadDataSize == 1)
      Software_I2C_NACK();
    else
      Software_I2C_ACK();
    ReadBuf++;
  }
  Software_I2C_Stop();
  AT24Cxx_Delay_ms(5);
  return 0;
#endif
}

#include "C_code_Log.h"
void AT24Cxx_Test(void)
{
  uint8_t write_data[5] = {1, 2, 3, 4, 5};
  AT24Cxx_Write_nByte_In_One_Block(0, write_data, 5);
  uint8_t read_data[5] = {0};
  AT24Cxx_Read_nByteBuf(0, read_data, 5);
  LOG_INFO("AT24Cxx Read Data: %d, %d, %d, %d, %d", read_data[0], read_data[1], read_data[2], read_data[3], read_data[4]);
}
