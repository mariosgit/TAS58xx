#include "tas5805m.h"

#include <Wire.h>
#include <mbLog.h>

bool Tas5805m::_isReset = false;

Tas5805m::Tas5805m()
{

}

// Give me 7bit address e.g. 0x58>>1.
// And the pin where /PDN is connected.
bool Tas5805m::begin(byte adr, byte pinPDN, bool start)
{
    bool result = true;
    _adr = adr;

    if(!_isReset)
    {
        // reset Tas5805m
        pinMode(pinPDN, OUTPUT_OPENDRAIN);
        digitalWrite(pinPDN, LOW);
        delay(500);
        digitalWrite(pinPDN, HIGH);
        delay(500);
        _isReset = true;
    }

    result |= setBookPage(0,0);
    write(DEVICE_CTRL_2, 0x08);  // mute
    write(DEVICE_CTRL_2, 0x1A);  // DSPreset + HiZ + mute
    write(RESET_CTRL, 0x11); //reset DSP and CTL
    // write(ANA_CTRL, 0x03); //175kHz Bandpass ??
    write(DSP_MISC, 0x08); //decouple BQ coefs for LR
    delay(5);
    write(DEVICE_CTRL_2, 0x0B);  // Play + Mute

    write(ADR_PIN_CTRL, 1); // enable output
    // ADR_PIN_CONFIG:
    // 00000: off (low)
    // 00011: Auto mute flag (asserted when both L and R channels are auto muted)
    // 00100: Auto mute flag for left channel 0101: Auto mute flag for right channel
    // 00110: Clock invalid flag (clock error or clock missing) 00111: Reserved
    // 01001: Reserved
    // 01011: ADR as FAULTZ output
    write(ADR_PIN_CONFIG, 0b01011);

    if(start)
        ctlPlay();
    return result;
}

void Tas5805m::ctlPlay()
{
    write(DEVICE_CTRL_2, 0x03);  // Play + UnMute
}

// These bits indicate the currently detected audio sampling rate.
// 0000: FS Error
// 0010: 8KHz, 0100: 16KHz
// 0110: 32KHz 1000: Reserved
// 1001: 48KHz 1011: 96KHz Others Reserved

byte buf[256];
void Tas5805m::loop()
{
    readStatus();

    //toggle LevelMeter to input
    setBookPage(0x8c, 0x2c);
    // write_9_23(0x0c, 1.0);
    // write_9_23(0x10, 1.0);
    // write_9_23(0x14, 0.0);
    // write_9_23(0x18, 0.0);

    readLevels();
}

void Tas5805m::setAnalogGain(float gain)
{
    setBookPage(0,0);
    if(gain > 0.0)
        gain = 0.0;
    if(gain < -15.5)
        gain = -15.5;
    gain = gain * -2.0; // map to 0..31
    // 00000: 0 dB (29.5V peak voltage)
    // 00001: -0.5db
    // 11111: -15.5 dB
    uint8_t ugain = (uint8_t)gain;
    LOG << "Tas5805m::setAGain" <<ugain <<"\n";
    ugain = ugain & 0x1f;
    write(AGAIN, ugain);
}

void Tas5805m::setDigitalVolume(int gain)
{
    byte vol = 0x30;
    if(gain < -103)
        vol = 0xff;
    else if(gain >= 24)
        vol = 0;
    else
    {
        vol = 0x30 - 2*gain;
    }
    // LOG.hex() <<"setDigitalVolume: " <<gain <<" -> " <<vol <<"\n";
    setBookPage(0,0);
    write(DIG_VOL_CTL, vol); //0x30 = 0db  adding 1 reduces gain by .5dB
}
void Tas5805m::setChannels(Channel chA, Channel chB)
{
    if(!setBookPage(0x8c,0x29))
        return;
    switch(chA)
    {
        case LEFT:
            write_9_23(0x18, 1.0); // left -> left
            write_9_23(0x1c, 0.0); // right -> left
            break;
        case RIGHT:
            write_9_23(0x18, 0.0); // left -> left
            write_9_23(0x1c, 1.0); // right -> left
            break;
        case BOTH:
            write_9_23(0x18, 1.0); // left -> left
            write_9_23(0x1c, 1.0); // right -> left
            break;
    }
    switch(chB)
    {
        case LEFT:
            write_9_23(0x20, 1.0); // left -> right
            write_9_23(0x24, 0.0); // right -> right
            break;
        case RIGHT:
            write_9_23(0x20, 0.0); // left -> right
            write_9_23(0x24, 1.0); // right -> right
            break;
        case BOTH:
            write_9_23(0x20, 1.0); // left -> right
            write_9_23(0x24, 1.0); // right -> right
            break;
    }
}

void Tas5805m::setCoefficients(uint32_t stage, float *coef, Channel ch)
{
    // LOG.dez(5);
    // LOG.bin();
    // LOG <<"[0]:" <<coef[0] <<"=\t" <<(int32_t)(coef[0]*0x8000000) <<"\n";
    // LOG <<"[1]:" <<coef[1] <<"=\t" <<(int32_t)(coef[1]*0x8000000) <<"\n";
    // LOG <<"[2]:" <<coef[2] <<"=\t" <<(int32_t)(coef[2]*0x8000000) <<"\n";
    // LOG <<"[3]:" <<coef[3] <<"=\t" <<(int32_t)(coef[3]*0x8000000) <<"\n";
    // LOG <<"[4]:" <<coef[4] <<"=\t" <<(int32_t)(coef[4]*0x8000000) <<"\n";
    // LOG.dec();

    // setBookPage(0x00, 0x00);
    // Wire.begin();
    // write(DEVICE_CTRL_2, 0x02);  // HiZ
    // delay(5);

    if(ch == BOTH || ch == LEFT)
    {
        setBookPage(0,0);
        write(BQ_WR_CTRL1,1); // Indicate the first coefficient of a BQ is starting to write.
        setBookPage(0xaa, 0x24);
        byte leftBQ1 = 0x18 + stage*20;
        writeBQ(leftBQ1, coef);
    }
    if(ch == BOTH || ch == RIGHT)
    {
        setBookPage(0,0);
        write(BQ_WR_CTRL1,1); // Indicate the first coefficient of a BQ is starting to write.
        setBookPage(0xaa, 0x26);
        byte rightBQ1 = 0x54 + stage*20;
        writeBQ(rightBQ1, coef);
    }

    // setBookPage(0x00, 0x00);
    // write(DEVICE_CTRL_2, 0x03);  // Play
}

/***************************** private ********************************/

bool Tas5805m::setBookPage(byte book, byte page)
{
    bool result = true;
    byte retval = 0;
    // w 58 00 00 #Go to Page0
    // w 58 7f 00 #Change the Book to 0x00
    // w 58 00 00 #Go to Page 0x00
    Wire.beginTransmission(_adr);
    Wire.write(0);
    Wire.write(0);
    retval = Wire.endTransmission();
    result |= retval;
    logerror("setBookPage1", retval, 0);
    Wire.beginTransmission(_adr);
    Wire.write(0x7f);
    Wire.write(book);
    retval = Wire.endTransmission();
    result |= retval;
    logerror("setBookPage2", retval, 0x7f);
    Wire.beginTransmission(_adr);
    Wire.write(0);
    Wire.write(page);
    retval = Wire.endTransmission();
    result |= retval;
    logerror("setBookPage3", retval, 0);
    return result;
}

void Tas5805m::write(byte reg, byte data)
{
    Wire.beginTransmission(_adr);
    Wire.write(reg);
    Wire.write(data);
    byte result = Wire.endTransmission();
    logerror("write", result, reg);
}

void Tas5805m::write(byte reg, byte *buffer, uint8_t len)
{
    Wire.beginTransmission(_adr);
    Wire.write(reg);
    Wire.write(buffer, len);
    byte result = Wire.endTransmission();
    logerror("write", result, reg);
}

void Tas5805m::write_9_23(byte reg, float val)
{
    union AllVals {
        int32_t ival;
        uint8_t buf[4];
    } uval;
    uval.ival = swap32(val * (float)(8388608.0));  // only on ARM
    write(reg, uval.buf, 4);
}

byte Tas5805m::read(byte reg)
{
    byte result = 0;
    Wire.beginTransmission(_adr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(_adr, (uint8_t)1);    // request 6 bytes from slave device #2
    while(Wire.available())    // slave may send less than requested
    {
        result = Wire.read();    // receive a byte as character
    }
    return result;
}

int Tas5805m::read(byte startreg, byte *buffer, uint8_t len)
{
    int bytesread = 0;
    Wire.beginTransmission(_adr);
    Wire.write(startreg);
    Wire.endTransmission();
    Wire.requestFrom(_adr, len);    // request 6 bytes from slave device #2
    while(Wire.available())    // slave may send less than requested
    {
        buffer[bytesread] = Wire.read();    // receive a byte as character
        bytesread++;
    }
    return bytesread;
}

float Tas5805m::read_1_31f(byte adr)
{
    byte buf[4];
    int len = read(adr, buf, 4);
    // LOG <<"levels: len:" <<len <<": ";
    // for(int i = 0; i < 4; i++)
    //     LOG.hex() <<buf[i] <<" ";
    // LOG <<"\n";
    int32_t iVal  = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
    float fVal  = (float)iVal / (float)(2147483648.0);
    return fVal;
}
float Tas5805m::read_5_27f(byte adr)
{
    byte buf[4];
    int len = read(adr, buf, 4);
    // LOG <<"levels: len:" <<len <<": ";
    // for(int i = 0; i < 4; i++)
    //     LOG.hex() <<buf[i] <<" ";
    // LOG <<"\n";
    int32_t iVal = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
    float   fVal = (float)iVal / (float)(134217728.0);
    return fVal;
}
float Tas5805m::read_9_23f(byte adr)
{
    byte buf[4];
    int len = read(adr, buf, 4);
    // LOG <<"levels: len:" <<len <<": ";
    // for(int i = 0; i < 4; i++)
    //     LOG.hex() <<buf[i] <<" ";
    // LOG <<"\n";
    int32_t iVal = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
    float   fVal = (float)iVal / (float)(8388608.0);
    return fVal;
}

void Tas5805m::readStatus()
{
    enum GC_BITS
    {
        CH1_DC_1 = 0x08,
        CH2_DC_1 = 0x04,
        CH1_OC_I = 0x02,
        CH2_OC_I = 0x01,
        GC1_OTP_CRC_ERROR = 0x80,
        GC1_BQ_WR_ERROR = 0x40,
        GC1_CLK_FAULT_I = 0x04,
        GC1_PVDD_OV_I = 0x02,
        GC1_PVDD_UV_I = 0x01,
        GC2_OTSD_I = 0x01
    };

    setBookPage(0,0);

    byte chf = read(CHAN_FAULT);
    byte gf1 = read(GLOBAL_FAULT1);
    byte gf2 = read(GLOBAL_FAULT2);
    if(chf&0xff || gf1&0xc7 || gf2&0x01)
    {
        LOG <<"ERRORS: ";
        if(chf & CH1_DC_1) { LOG <<"Left channel DC fault"; }
        if(chf & CH2_DC_1) { LOG <<"Right channel DC fault"; }
        if(chf & CH1_OC_I) { LOG <<"Left channel over current fault"; }
        if(chf & CH2_OC_I) { LOG <<"Right channel over current fault"; }
        if(gf1 & GC1_OTP_CRC_ERROR) { LOG <<"Indicate OTP CRC check error. "; }
        if(gf1 & GC1_BQ_WR_ERROR) { LOG <<"The recent BQ is written failed. "; }
        if(gf1 & GC1_CLK_FAULT_I) { LOG <<"Clock fault. "; }
        if(gf1 & GC1_PVDD_OV_I) { LOG <<"PVDD OverVoltage fault. "; }
        if(gf1 & GC1_PVDD_UV_I) { LOG <<"PVDD UnderVoltage fault. "; }
        if(gf2 & GC2_OTSD_I) { LOG <<"Over temperature shut down fault. "; }
        LOG <<"\n";
    }
    byte otw = read(OT_WARNING);
    if(otw&0x04)
    {
        LOG <<"WARNING: " <<"Over temperature warning ,135C" <<"\n";
    }

    // byte dc1 = read(DEVICE_CTRL_1);
    // byte dc2 = read(DEVICE_CTRL_2);
    // LOG.hex() <<"device ctl 1:" <<dc1 <<" 2:" <<dc2 <<"\n";

    // int result = read(FS_MON, buf, 2);
    // int bckRatio = ((buf[0] & 0x30) <<4) | buf[1];
    // result += read(CHAN_FAULT, buf+2, 4);
    // byte ams = read(AUTOMUTE_STATE);
    // LOG.hex() <<"status:" <<result <<"bytes.. bckR:" <<bckRatio <<" ams:" <<(int(ams&0xfc))  <<" chf:"<<buf[2] <<" glf1:"<<buf[3] <<" glf2:"<<buf[4] <<" otw:"<<buf[5] ;
    // switch(buf[0])
    // {
    //     case 0:
    //         LOG <<" fs error\n";
    //         break;
    //     case 0b0010:
    //         LOG <<" fs 8kHz\n";
    //         break;
    //     case 0b0100:
    //         LOG <<" fs 16kHz\n";
    //         break;
    //     case 0b0110:
    //         LOG <<" fs 32kHz\n";
    //         break;
    //     case 0b1001:
    //         LOG <<" fs 48kHz\n";
    //         break;
    //     case 0b1011:
    //         LOG <<" fs 96kHz\n";
    //         break;
    //     default:
    //         LOG <<" fs ???kHz" <<int(buf[0]) ;
    // }
    // LOG <<"\n";
}
void Tas5805m::logerror(const char* text, byte code, byte adr)
{
    switch(code)
    {
        case 0: //success
            // LOG <<"Tas5805m::logerror " <<text <<" - success\n";
            break;
        case 1: // data too long to fit in transmit buffer
            LOG <<"Tas5805m::logerror adr:" <<_adr <<" " <<text <<" - data too long to fit in transmit buffer\n";
            break;
        case 2: // received NACK on transmit of address
            LOG <<"Tas5805m::logerror adr:" <<_adr <<" " <<text <<" - received NACK on transmit of address\n";
            break;
        case 3: //received NACK on transmit of data
            LOG <<"Tas5805m::logerror adr:" <<_adr <<" " <<text <<" - received NACK on transmit of data\n";
            break;
        default: //other error
            LOG <<"Tas5805m::logerror adr:" <<_adr <<" " <<text <<" - other error\n";
            break;
    }
}

void Tas5805m::writeBQ(byte adr, float *coef)
{
    byte result = 0;
    int32_t b0 = (int32_t)(coef[0]*0x8000000);
    int32_t b1 = (int32_t)(coef[1]*0x8000000);
    int32_t b2 = (int32_t)(coef[2]*0x8000000);
    int32_t a1 = (int32_t)(coef[3]*0x8000000);
    int32_t a2 = (int32_t)(coef[4]*0x8000000);
    // ahhh it's little endian
    byte *x0 = (byte*)&b0;
    byte *x1 = (byte*)&b1;
    byte *x2 = (byte*)&b2;
    byte *x3 = (byte*)&a1;
    byte *x4 = (byte*)&a2;

    byte buf[20];
    byte *pbuf = buf;
    LOG.bin();
    for(int i =3; i >= 0; i--)
        *pbuf++ = x0[i];
    for(int i =3; i >= 0; i--)
        *pbuf++ = x1[i];
    for(int i =3; i >= 0; i--)
        *pbuf++ = x2[i];
    for(int i =3; i >= 0; i--)
        *pbuf++ = x3[i];
    for(int i =3; i >= 0; i--)
        *pbuf++ = x4[i];
    // for(int i = 0; i < 20; i++)
    //     LOG <<buf[i] <<" ";
    // LOG.dec() <<"\n";

    // return;

    Wire.beginTransmission(_adr);
    Wire.write(adr);
    Wire.write(buf,20);
    result = Wire.endTransmission();
    logerror("writeBQ0:", result, adr);
}

void Tas5805m::readLevels()
{
    setBookPage(0x78, 0x02);
    float levelLeft   = read_1_31f(0x60);
    float levelReight = read_1_31f(0x64);
    float dbLeft  = 20.0*log10f(levelLeft);
    float dbRight = 20.0*log10f(levelReight);
    // LOG.dez(0) <<"level l:" <<dbLeft <<"db r:" <<dbRight <<"db\n";
    int dbRestLeft  = 100+dbLeft;
    int dbRestRight = 100+dbRight;
    if(dbRestLeft < 0)
        dbRestLeft = 0;
    if(dbRestLeft > 99)
        dbRestLeft = 100;
    if(dbRestRight > 99)
        dbRestRight = 100;
    for(int i = 0; i < dbRestLeft/4; i++)
        LOG <<"*";
    LOG <<"\n";
    for(int i = 0; i < dbRestRight/4; i++)
        LOG <<"*";
    LOG <<"\n";

    // setBookPage(0x8c, 0x29);
    // float mixLeftLeft     = read_9_23f(0x18);
    // float mixReightReight = read_9_23f(0x24);
    // LOG <<"mixll:" <<mixLeftLeft <<" mixrr:" <<mixReightReight <<"\n";

#if 0
    setBookPage(0x8c,0x2c);  // xbar
    float xb1     = read_9_23f(0x1c);
    float xb2     = read_9_23f(0x20);
    float xb3     = read_9_23f(0x28);
    float xb4     = read_9_23f(0x2c);
    LOG <<"xbar:" <<xb1 <<" " <<xb2 <<" " <<xb3 <<" " <<xb4 <<"\n";
    float ml1     = read_9_23f(0x0c);
    float ml2     = read_9_23f(0x10);
    float ml3     = read_9_23f(0x14);
    float ml4     = read_9_23f(0x18);
    LOG <<"mix level:" <<ml1 <<" " <<ml2 <<" " <<ml3 <<" " <<ml4 <<"\n";
    {
    setBookPage(0xaa, 0x24);
    float b0 = read_5_27f(0x18);
    float b1 = read_5_27f(0x1c);
    float b2 = read_5_27f(0x20);
    float a1 = read_5_27f(0x24);
    float a2 = read_5_27f(0x28);
    LOG.dec().dez(5) <<"bq L0:";
    LOG <<"b0:" <<b0 <<"\t";
    LOG <<"b1:" <<b1 <<"\t";
    LOG <<"b2:" <<b2 <<"\t";
    LOG <<"a1:" <<a1 <<"\t";
    LOG <<"a2:" <<a2 <<"\n";
    }
    {
    setBookPage(0xaa, 0x26);
    float b0 = read_5_27f(0x54);
    float b1 = read_5_27f(0x58);
    float b2 = read_5_27f(0x5c);
    float a1 = read_5_27f(0x60);
    float a2 = read_5_27f(0x64);
    LOG.dec().dez(5) <<"bq R0:";
    LOG <<"b0:" <<b0 <<"\t";
    LOG <<"b1:" <<b1 <<"\t";
    LOG <<"b2:" <<b2 <<"\t";
    LOG <<"a1:" <<a1 <<"\t";
    LOG <<"a2:" <<a2 <<"\n";
    }
#endif
}

