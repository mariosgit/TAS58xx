#pragma once

#include <Arduino.h>
#include <Audio.h> // just for __REV()

const byte RESET_CTRL = 0x01;
const byte DEVICE_CTRL_1 = 0x02;
const byte DEVICE_CTRL_2 = 0x03;
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
    bool begin(byte adr, byte pinPDN);
    void loop();

    void setAnalogGain(float gain); // -15.5..0
    void setDigitalVolume(int gain); // -103.5(MUTE)..24 DIG_VOL_CTR
    void setChannels(Channel chA, Channel chB);
    void setCoefficients(uint32_t stage, float *coef, Channel ch);

private:
    bool setBookPage(byte book, byte page);

    // inline uint32_t swap32(uint32_t val) { return (val&0xff000000)>>24 | (val&0x00ff0000)>>8 | (val&0x0000ff00)<<8 | (val&0x000000ff)<<24; }
    inline uint32_t swap32(uint32_t val) { return __REV(val); }

    void write(byte reg, byte data);
    void write(byte reg, byte *buffer, uint8_t len);
    void write_9_23(byte reg, float val);
    byte  read(byte reg);
    int   read(byte startreg, byte *buffer, uint8_t len);
    float read_1_31f(byte adr);
    float read_5_27f(byte adr);
    float read_9_23f(byte adr);
    void  readStatus();

    void logerror(const char*, byte code, byte adr);
    byte _adr;

    void setHighpass(uint32_t stage, float frequency, float q, Channel ch = BOTH);
    void setLowpass(uint32_t stage, float frequency, float q, Channel ch = BOTH);

    void writeBQ(byte adr, float *coef);
    void readLevels();

    static bool _isReset;
};
