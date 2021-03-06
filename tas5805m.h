#pragma once

#include <Arduino.h>

#ifdef __arm__
#include <arm_math.h>
#include <core_cmInstr.h>
#endif

const byte RESET_CTRL = 0x01;
const byte DEVICE_CTRL_1 = 0x02;
const byte DEVICE_CTRL_2 = 0x03;
const byte SIG_CH_CTRL = 0x28;
const byte SAP_CTRL1 = 0x33;
const byte SAP_CTRL2 = 0x34;
const byte SAP_CTRL3 = 0x35;
const byte FS_MON = 0x37;
const byte DIG_VOL_CTL = 0x4c;
const byte ANA_CTRL = 0x53;
const byte AGAIN = 0x54; 
const byte BQ_WR_CTRL1 = 0x5c;
const byte ADR_PIN_CTRL = 0x60;
const byte ADR_PIN_CONFIG = 0x61;
const byte DSP_MISC = 0x66;
const byte AUTOMUTE_STATE = 0x69;
const byte CHAN_FAULT = 0x70;
const byte GLOBAL_FAULT1 = 0x71;
const byte GLOBAL_FAULT2 = 0x72;
const byte OT_WARNING = 0x73;
const byte PIN_CONTROL1 = 0x74;

class Tas5805m
{
public:
    enum Channel {
        BOTH,
        LEFT,
        RIGHT
    };

    Tas5805m();
    inline byte getAdr() {return _adr;};
    bool begin(byte adr, byte pinPDN, bool start = true);
    void powerDown();   // mutes, powerDown
    bool powerUp();  // powerUp
    bool unMute();
    bool loop(bool printLevels = false);

    void setAnalogGain(float gain); //< -15.5..0
    void setDigitalVolume(int gain); //< Allways both channels -103.5(MUTE)..24 DIG_VOL_CTR
    void setChannels(Channel chA, Channel chB);
    void setCoefficients(uint32_t stage, const float *coef, Channel ch);

    inline float getLevelDBLeft()  { return _dbLeft; }
    inline float getLevelDBRight() { return _dbRight; }


private:

    // inline uint32_t swap32(uint32_t val) { return (val&0xff000000)>>24 | (val&0x00ff0000)>>8 | (val&0x0000ff00)<<8 | (val&0x000000ff)<<24; }
    inline uint32_t swap32(uint32_t val) {
#ifdef __arm__ 
        return __REV(val);
#else
        return val;
#endif
    }

    bool setBookPage(byte book, byte page);
    bool write(byte reg, byte data);
    bool write(byte reg, byte *buffer, uint8_t len);
    bool write_9_23(byte reg, float val);
    byte  read(byte reg);
    int   read(byte startreg, byte *buffer, uint8_t len);
    int32_t read_32i(byte adr);
    float read_1_31f(byte adr);
    float read_5_27f(byte adr);
    float read_9_23f(byte adr);
    bool  readStatus();

    void logerror(const char*, byte code, byte adr);
    byte _adr;
    byte _online;
    byte _pinPDN;

    void setHighpass(uint32_t stage, float frequency, float q, Channel ch = BOTH);
    void setLowpass(uint32_t stage, float frequency, float q, Channel ch = BOTH);

    void writeBQ(byte page, byte adr, const float *coef);
    void readLevels(bool printLevels = false);

    float _levelLeft;
    float _levelReight;
    float _dbLeft;
    float _dbRight;

    static bool _isReset;
};
