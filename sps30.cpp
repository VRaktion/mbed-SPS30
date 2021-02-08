#include "mbed.h"
#include "sps30.h"

//-----------------------------------------------------------------------------
// Constructor 

// Sps30::Sps30(PinName sda, PinName scl, int i2c_frequency)  : _i2c(sda, scl) {
//         //_i2c->frequency(i2c_frequency);
// }

Sps30::Sps30(I2C* i2c)  : _i2c(i2c) {
        //_i2c->frequency(i2c_frequency);
}

//-----------------------------------------------------------------------------
// Destructor

Sps30::~Sps30() {
}

//-----------------------------------------------------------------------------
// start auto-measurement 
//

uint8_t Sps30::StartMeasurement() 
{
    i2cbuff[0] = SPS30_CMMD_STRT_MEAS >> 8;
    i2cbuff[1] = SPS30_CMMD_STRT_MEAS & 255;
    i2cbuff[2] = SPS30_STRT_MEAS_WRITE_DATA >> 8;
    i2cbuff[3] = SPS30_STRT_MEAS_WRITE_DATA & 255;
    i2cbuff[4] = Sps30::CalcCrc2b(SPS30_STRT_MEAS_WRITE_DATA);
    int res = _i2c->write(SPS30_I2C_ADDR  << 1, i2cbuff, 5, false);
    if(res) return SPSNOACKERROR;
    return SPSNOERROR;
}

//-----------------------------------------------------------------------------
// Stop auto-measurement

uint8_t Sps30::StopMeasurement()
{
    i2cbuff[0] = SPS30_CMMD_STOP_MEAS >> 8;
    i2cbuff[1] = SPS30_CMMD_STOP_MEAS & 255;
    int res = _i2c->write(SPS30_I2C_ADDR << 1, i2cbuff, 2, false);
    if(res) return SPSNOACKERROR;
    return SPSNOERROR;
}

//-----------------------------------------------------------------------------
// Get ready status value

uint8_t Sps30::GetReadyStatus()
{
    i2cbuff[0] = SPS30_CMMD_GET_READY_STAT >> 8;
    i2cbuff[1] = SPS30_CMMD_GET_READY_STAT & 255;
    int res = _i2c->write(SPS30_I2C_ADDR << 1, i2cbuff, 2, false);
    if(res) return SPSNOACKERROR;
    
    _i2c->read(SPS30_I2C_ADDR << 1, i2cbuff, 3, false);
    uint16_t stat = (i2cbuff[0] << 8) | i2cbuff[1];
    sps_ready = stat;
    uint8_t dat = Sps30::CheckCrc2b(stat, i2cbuff[2]);
    
    if(dat == SPSCRCERROR) return SPSCRCERROR;
    return SPSNOERROR;
}

//-----------------------------------------------------------------------------
// Get all the measurement values, stick them into the array

uint8_t Sps30::ReadMeasurement()   
{
    i2cbuff[0] = SPS30_CMMD_READ_MEAS >> 8;
    i2cbuff[1] = SPS30_CMMD_READ_MEAS & 255;
    int res = _i2c->write(SPS30_I2C_ADDR << 1, i2cbuff, 2, false);
    if(res) return SPSNOACKERROR;
    
    _i2c->read(SPS30_I2C_ADDR << 1, i2cbuff, 60, false);
    
    uint16_t stat = (i2cbuff[0] << 8) | i2cbuff[1];
    mass_1p0_m = stat;
    uint8_t dat = Sps30::CheckCrc2b(stat, i2cbuff[2]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;
    
    stat = (i2cbuff[3] << 8) | i2cbuff[4];
    mass_1p0_l = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[5]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;


    
    stat = (i2cbuff[6] << 8) | i2cbuff[7];
    mass_2p5_m = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[8]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;
    
    stat = (i2cbuff[9] << 8) | i2cbuff[10];
    mass_2p5_l = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[11]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;


    
    stat = (i2cbuff[12] << 8) | i2cbuff[13];
    mass_4p0_m = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[14]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;
    
    stat = (i2cbuff[15] << 8) | i2cbuff[16];
    mass_4p0_l = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[17]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;



    stat = (i2cbuff[18] << 8) | i2cbuff[19];
    mass_10p0_m = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[20]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;
    
    stat = (i2cbuff[21] << 8) | i2cbuff[22];
    mass_10p0_l = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[23]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;



    stat = (i2cbuff[24] << 8) | i2cbuff[25];
    num_0p5_m = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[26]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;
    
    stat = (i2cbuff[27] << 8) | i2cbuff[28];
    num_0p5_l = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[29]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;


    stat = (i2cbuff[30] << 8) | i2cbuff[31];
    num_1p0_m = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[32]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;
    
    stat = (i2cbuff[33] << 8) | i2cbuff[34];
    num_1p0_l = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[35]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;


    
    stat = (i2cbuff[36] << 8) | i2cbuff[37];
    num_2p5_m = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[38]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;
    
    stat = (i2cbuff[39] << 8) | i2cbuff[40];
    num_2p5_l = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[41]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;


    
    stat = (i2cbuff[42] << 8) | i2cbuff[43];
    num_4p0_m = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[44]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;
    
    stat = (i2cbuff[45] << 8) | i2cbuff[46];
    num_4p0_l = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[47]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;


    stat = (i2cbuff[48] << 8) | i2cbuff[49];
    num_10p0_m = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[50]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;
    
    stat = (i2cbuff[51] << 8) | i2cbuff[52];
    num_10p0_l = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[53]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;


    stat = (i2cbuff[54] << 8) | i2cbuff[55];
    typ_pm_size_m = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[56]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;
    
    stat = (i2cbuff[57] << 8) | i2cbuff[58];
    typ_pm_size_l = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[59]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;
    
    mass_1p0_i = (mass_1p0_m << 16) | mass_1p0_l;
    mass_2p5_i = (mass_2p5_m << 16) | mass_2p5_l;
    mass_4p0_i = (mass_4p0_m << 16) | mass_4p0_l;
    mass_10p0_i = (mass_10p0_m << 16) | mass_10p0_l;

    num_0p5_i = (num_0p5_m << 16) | num_0p5_l;
    num_1p0_i = (num_1p0_m << 16) | num_1p0_l;
    num_2p5_i = (num_2p5_m << 16) | num_2p5_l;
    num_4p0_i = (num_4p0_m << 16) | num_4p0_l;
    num_10p0_i = (num_10p0_m << 16) | num_10p0_l;          

    typ_pm_size_i = (typ_pm_size_m << 16) | typ_pm_size_l;          

    mass_1p0_f = *(float*)&mass_1p0_i;
    mass_2p5_f = *(float*)&mass_2p5_i;
    mass_4p0_f = *(float*)&mass_4p0_i;
    mass_10p0_f = *(float*)&mass_10p0_i;

    num_0p5_f = *(float*)&num_0p5_i;
    num_1p0_f = *(float*)&num_1p0_i;
    num_2p5_f = *(float*)&num_2p5_i;
    num_4p0_f = *(float*)&num_4p0_i;
    num_10p0_f = *(float*)&num_10p0_i;          

    typ_pm_size_f = *(float*)&typ_pm_size_i;
    
    return SPSNOERROR;
}
    
//-----------------------------------------------------------------------------
// Calculate the CRC of a 2 byte value using the SPS30 CRC polynomial

uint8_t Sps30::CalcCrc2b(uint16_t seed)
{
  uint8_t bit;                  // bit mask
  uint8_t crc = SPS30_CRC_INIT; // calculated checksum
  
  // calculates 8-Bit checksum with given polynomial

    crc ^= (seed >> 8) & 255;
    for(bit = 8; bit > 0; --bit)
    {
      if(crc & 0x80) crc = (crc << 1) ^ SPS30_POLYNOMIAL;
      else           crc = (crc << 1);
    }

    crc ^= seed & 255;
    for(bit = 8; bit > 0; --bit)
    {
      if(crc & 0x80) crc = (crc << 1) ^ SPS30_POLYNOMIAL;
      else           crc = (crc << 1);
    }
    
  return crc;
}

//-----------------------------------------------------------------------------
// Compare the CRC values

uint8_t Sps30::CheckCrc2b(uint16_t seed, uint8_t crc_in)
{
    uint8_t crc_calc = Sps30::CalcCrc2b(seed);
    if(crc_calc != crc_in) return SPSCRCERROR;
    return SPSNOERROR;
}

//-----------------------------------------------------------------------------
// Get Sps30 serial number

uint8_t Sps30::GetSerialNumber()
{
    i2cbuff[0] = SPS30_CMMD_READ_SERIALNBR >> 8;
    i2cbuff[1] = SPS30_CMMD_READ_SERIALNBR & 255;
    int res = _i2c->write(SPS30_I2C_ADDR << 1, i2cbuff, 2, false);
    if(res) return SPSNOACKERROR;
    
    int i = 0;
    for(i = 0; i < sizeof(sn); i++) sn[i] = 0;
    for(i = 0; i < sizeof(i2cbuff); i++) i2cbuff[i] = 0;
    
    _i2c->read(SPS30_I2C_ADDR << 1, i2cbuff, SPS30_SN_SIZE, false);
    int t = 0;
    for(i = 0; i < SPS30_SN_SIZE; i +=3) {
        uint16_t stat = (i2cbuff[i] << 8) | i2cbuff[i + 1];
        sn[i - t] = stat >> 8;
        sn[i - t + 1] = stat & 255;
        uint8_t dat = Sps30::CheckCrc2b(stat, i2cbuff[i + 2]);
        t++;
        if(dat == SPSCRCERROR) return SPSCRCERROR;
        if(stat == 0) break;
    }

    return SPSNOERROR;
}

//-----------------------------------------------------------------------------
// Read Auto Cleaning Interval on the SPS30

uint8_t Sps30::ReadAutoCleanInterval()
{   
    i2cbuff[0] = SPS30_CMMD_AUTO_CLEAN_INTV >> 8;
    i2cbuff[1] = SPS30_CMMD_AUTO_CLEAN_INTV & 255;
    
    int res = _i2c->write(SPS30_I2C_ADDR << 1, i2cbuff, 2, false);
    if(res) return SPSNOACKERROR;
    
    _i2c->read(SPS30_I2C_ADDR << 1, i2cbuff, 6, false);
    
    uint16_t stat = (i2cbuff[0] << 8) | i2cbuff[1];
    clean_interval_m = stat;
    uint8_t dat = Sps30::CheckCrc2b(stat, i2cbuff[2]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;
    
    stat = (i2cbuff[3] << 8) | i2cbuff[4];
    clean_interval_l = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[5]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;
    
    clean_interval_i = (clean_interval_m << 16) | clean_interval_l;
    
    return SPSNOERROR;
}

//-----------------------------------------------------------------------------
// Set Auto Cleaning Interval on the SPS30

uint8_t Sps30::SetAutoCleanInterval(uint32_t set_interval)
{
    uint16_t set_interval_m = set_interval >> 16;
    uint16_t set_interval_l = set_interval & 65535;
    
    i2cbuff[0] = SPS30_CMMD_AUTO_CLEAN_INTV >> 8;
    i2cbuff[1] = SPS30_CMMD_AUTO_CLEAN_INTV & 255;
    
    i2cbuff[2] = set_interval_m >> 8;
    i2cbuff[3] = set_interval_m & 255;
    i2cbuff[4] = Sps30::CalcCrc2b(set_interval_m);
    
    i2cbuff[5] = set_interval_l >> 8;
    i2cbuff[6] = set_interval_l & 255;
    i2cbuff[7] = Sps30::CalcCrc2b(set_interval_l);
    
    int res = _i2c->write(SPS30_I2C_ADDR << 1, i2cbuff, 8, false);
    if(res) return SPSNOACKERROR;
    
    _i2c->read(SPS30_I2C_ADDR << 1, i2cbuff, 6, false);
    
    uint16_t stat = (i2cbuff[0] << 8) | i2cbuff[1];
    clean_interval_m = stat;
    uint8_t dat = Sps30::CheckCrc2b(stat, i2cbuff[2]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;
    
    stat = (i2cbuff[3] << 8) | i2cbuff[4];
    clean_interval_l = stat;
    dat = Sps30::CheckCrc2b(stat, i2cbuff[5]);
    if(dat == SPSCRCERROR) return SPSCRCERROR;
    
    clean_interval_i = (clean_interval_m << 16) | clean_interval_l;
    
    return SPSNOERROR;
}

//-----------------------------------------------------------------------------
// Perform manual fan cleaning

uint8_t Sps30::StartFanClean()
{
    i2cbuff[0] = SPS30_CMMD_START_FAN_CLEAN >> 8;
    i2cbuff[1] = SPS30_CMMD_START_FAN_CLEAN & 255;
    int res = _i2c->write(SPS30_I2C_ADDR << 1, i2cbuff, 2, false);
    if(res) return SPSNOACKERROR;
    return SPSNOERROR;
}

//-----------------------------------------------------------------------------
// Perform a soft reset on the SPS30

uint8_t Sps30::SoftReset()
{
    i2cbuff[0] = SPS30_CMMD_SOFT_RESET >> 8;
    i2cbuff[1] = SPS30_CMMD_SOFT_RESET & 255;
    int res = _i2c->write(SPS30_I2C_ADDR << 1, i2cbuff, 2, false);
    if(res) return SPSNOACKERROR;
    return SPSNOERROR;
}

/**************************   PUBLIC METHODS   **********************************/

//-----------------------------------------------------------------------------
// Initialise SPS30

uint8_t Sps30::InitSPS30()
{
    uint8_t dat = Sps30::GetSerialNumber();
    if (dat == SPSNOACKERROR){
        return DISCONNECTED;
    } 

    dat = Sps30::StartMeasurement();
    if (dat == SPSNOACKERROR){
        return DISCONNECTED;
    }

    return CONNECTED;
}

//-----------------------------------------------------------------------------
// Poll SPS30

uint8_t Sps30::PollSPS30()
{
    uint8_t dat = Sps30::GetReadyStatus();
    if (dat == SPSNOACKERROR) return DISCONNECTED;
    
    if (sps_ready == SPSISREADY)
    {
        uint8_t crcc = Sps30::ReadMeasurement();
        if (crcc == SPSNOERROR)
        {
            return DATAREADY;
        }
        else if (crcc == SPSNOACKERROR)
        {
            return DISCONNECTED;
        }
        else
        {
            return DATAERROR;
        }
    }
    else
    {
        return DATANOTREADY;
    }
}
