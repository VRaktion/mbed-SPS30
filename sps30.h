/*******************************************************************************
 * Copyright (c) 2018-2019, Sensors and IoT Capability Centre (SIOT) at GovTech.
 *
 * Contributor(s):
 *    Yap Zi Qi     yap_zi_qi@tech.gov.sg
 *******************************************************************************/

#ifndef SPS30_H
#define SPS30_H

#define SPS30_I2C_ADDR 0x69

#define SPS30_CMMD_STRT_MEAS 0x0010
#define SPS30_CMMD_STOP_MEAS 0x0104
#define SPS30_CMMD_GET_READY_STAT 0x0202
#define SPS30_CMMD_READ_MEAS 0x0300

#define SPS30_CMMD_AUTO_CLEAN_INTV 0x8004
#define SPS30_CMMD_START_FAN_CLEAN 0x5607

#define SPS30_CMMD_SOFT_RESET 0xD304

#define SPS30_CMMD_READ_SERIALNBR 0xD033
#define SPS30_CMMD_READ_ARTICLECODE 0xD025

#define SPS30_STRT_MEAS_WRITE_DATA 0x0300

#define SPS30_POLYNOMIAL 0x31 // P(x) = x^8 + x^5 + x^4 + 1 = 100110p01
#define SPS30_CRC_INIT 0xff

#define SPS30_SN_SIZE 33 // size of the s/n ascii string + CRC values

#define MASS_MAX 1000.00f
#define MASS_MIN 0.00f
#define NUM_MAX 3000.00f
#define NUM_MIN 0.00f

#define I2C_FREQUENCY_STD 100000 // SPS30 uses 100MHz for I2C communication

/** Create SPS30 controller class
 *
 * @param sps30 class
 *
 */
class Sps30
{

public:
    enum SPSStatus
    {
        DISCONNECTED,
        CONNECTED,
        DATANOTREADY,
        DATAREADY,
        DATAERROR,
        DATAOOR,
        DATAOK,
    } SPSStatus;

    uint16_t sps_ready;        /**< 1 = ready, 0 = busy */
    uint32_t clean_interval_i; /** 32 unsigned bit in seconds */

    float mass_1p0_f;  /**< float of Mass Conc of PM1.0 */
    float mass_2p5_f;  /**< float of Mass Conc of PM2.5 */
    float mass_4p0_f;  /**< float of Mass Conc of PM4.0 */
    float mass_10p0_f; /**< float of Mass Conc of PM10 */

    float num_0p5_f;  /**< float of Number Conc of PM0.5 */
    float num_1p0_f;  /**< float of Number Conc of PM1.0 */
    float num_2p5_f;  /**< float of Number Conc of PM2.5 */
    float num_4p0_f;  /**< float of Number Conc of PM4.0 */
    float num_10p0_f; /**< float of Number Conc of PM10 */

    float typ_pm_size_f; /**< float of Typical Particle Size */

    uint8_t sn[33]; /**< ASCII Serial Number */

    /** Create a SPS30 object using the specified I2C object
     * @param sda - mbed I2C interface pin
     * @param scl - mbed I2C interface pin
     * @param I2C Frequency (in Hz)
     *
     * @return none
     */
    //  Sps30(PinName sda, PinName scl, int i2c_frequency);
    Sps30(I2C *i2c);

    /** Destructor
     *
     * @param --none--
     *
     * @return none
     */
    ~Sps30();

    /** Initialise SPS30
     *
     * @param --none--
     *
     * @return enum SPSStatus
     */
    uint8_t InitSPS30();

    /** Poll SPS30 Data
     *
     * @param --none--
     *
     * @return enum SPSStatus
     */
    uint8_t PollSPS30();

    /** Stop Auto-Measurement 
     *
     * @param --none--
     *
     * @return enum SPSerror
     */
    uint8_t StopMeasurement();

    /** Perform a soft reset
     *
     * @param --none--
     *
     * @return enum SPSerror
     */
    uint8_t SoftReset();

    /** Perform manual Fan Cleaning
     *
     * @param --none--
     *
     * @return enum SPSerror
     */
    uint8_t StartFanClean();

private:
    enum SPSError
    {
        SPSNOERROR,    //all ok
        SPSISREADY,    //ready status register
        SPSNOACKERROR, //no I2C ACK error
        SPSCRCERROR,   //CRC error, any
    };

    char i2cbuff[60];

    uint16_t clean_interval_m; /**< High order 16 bit word of Auto Clean Interval */
    uint16_t clean_interval_l; /**< High order 16 bit word of Auto Clean Interval */

    uint16_t mass_1p0_m;  /**< High order 16 bit word of Mass Conc of PM1.0 */
    uint16_t mass_1p0_l;  /**< Low order 16 bit word of Mass Conc of PM1.0 */
    uint16_t mass_2p5_m;  /**< High order 16 bit word of Mass Conc of PM2.5 */
    uint16_t mass_2p5_l;  /**< Low order 16 bit word of Mass Conc of PM2.5 */
    uint16_t mass_4p0_m;  /**< High order 16 bit word of Mass Conc of PM4.0 */
    uint16_t mass_4p0_l;  /**< Low order 16 bit word of Mass Conc of PM4.0 */
    uint16_t mass_10p0_m; /**< High order 16 bit word of Mass Conc of PM10 */
    uint16_t mass_10p0_l; /**< Low order 16 bit word of Mass Conc of PM10 */

    uint16_t num_0p5_m;  /**< High order 16 bit word of Number Conc of PM0.5 */
    uint16_t num_0p5_l;  /**< Low order 16 bit word of Number Conc of PM0.5 */
    uint16_t num_1p0_m;  /**< High order 16 bit word of Number Conc of PM1.0 */
    uint16_t num_1p0_l;  /**< Low order 16 bit word of Number Conc of PM1.0 */
    uint16_t num_2p5_m;  /**< High order 16 bit word of Number Conc of PM2.5 */
    uint16_t num_2p5_l;  /**< Low order 16 bit word of Number Conc of PM2.5 */
    uint16_t num_4p0_m;  /**< High order 16 bit word of Number Conc of PM4.0 */
    uint16_t num_4p0_l;  /**< Low order 16 bit word of Number Conc of PM4.0 */
    uint16_t num_10p0_m; /**< High order 16 bit word of Number Conc of PM10 */
    uint16_t num_10p0_l; /**< Low order 16 bit word of Number Conc of PM10 */

    uint16_t typ_pm_size_m; /**< High order 16 bit word of Typical Particle Size */
    uint16_t typ_pm_size_l; /**< Low order 16 bit word of Typical Particle Size */

    uint32_t mass_1p0_i;  /**< 32 bit int of Mass Conc of PM1.0 */
    uint32_t mass_2p5_i;  /**< 32 bit int of Mass Conc of PM2.5 */
    uint32_t mass_4p0_i;  /**< 32 bit int of Mass Conc of PM4.0 */
    uint32_t mass_10p0_i; /**< 32 bit int of Mass Conc of PM10 */

    uint32_t num_0p5_i;  /**< 32 bit int of Number Conc of PM0.5 */
    uint32_t num_1p0_i;  /**< 32 bit int of Number Conc of PM1.0 */
    uint32_t num_2p5_i;  /**< 32 bit int of Number Conc of PM2.5 */
    uint32_t num_4p0_i;  /**< 32 bit int of Number Conc of PM4.0 */
    uint32_t num_10p0_i; /**< 32 bit int of Number Conc of PM10 */

    uint32_t typ_pm_size_i; /**< 32 bit int of Typical Particle Size */

    /** Start Auto-Measurement 
     *
     * @param --none--
     *
     * @return enum SPSerror
     */
    uint8_t StartMeasurement();

    /** Get Ready Status register 
     *
     * @param --none--
     * @see Ready Status result
     *
     * @return enum SPSerror
     */
    uint8_t GetReadyStatus();

    /** Get all particulate matter parameters
     *
     * @param --none-
     * @see Results in Public member variables
     *
     * @return enum SPSerror
     */
    uint8_t ReadMeasurement();

    /** Calculate the SPS30 CRC value
     *
     * @param 16 bit value to perform a CRC check on
     *
     * @return 8 bit CRC value
     */
    uint8_t CalcCrc2b(uint16_t seed);

    /** Compare received CRC value with calculated CRC value
     *
     * @param 16 bit value to perform a CRC check on
     * @param 8 bit value to compare CRC values
     *
     * @return enum SPSerror
     */
    uint8_t CheckCrc2b(uint16_t seed, uint8_t crc_in);

    /** Get Serial Number
     *
     * @param --none--
     * @see ASCII Serial Number as sn[33]
     *
     * @return enum SPSerror
     */
    uint8_t GetSerialNumber();

    /** Read Auto Cleaning Interval on the SPS30
     *
     * @param --none--
     *
     * @return enum SPSerror
     */
    uint8_t ReadAutoCleanInterval();

    /** Set Auto Cleaning Interval on the SPS30
     *
     * @param Auto Cleaning Interval in seconds 
     * default is 604800s = 1 week, 0 to disable auto clean
     *
     * @return enum SPSerror
     */
    uint8_t SetAutoCleanInterval(uint32_t set_interval = 604800);

protected:
    I2C *_i2c;
};
#endif
