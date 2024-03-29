#include "tas5805m.h"

#include <Wire.h>
#include <mbLog.h>

bool Tas5805m::_isReset = false;

Tas5805m::Tas5805m() :
    _adr(0),
    _online(false),
    _pinPDN(0),
    _levelLeft(0.0f),
    _levelReight(0.0f),
    _dbLeft(-80.0f),
    _dbRight(-80.0f)
{
}

// Give me 7bit address e.g. 0x58>>1.
// And the pin where /PDN is connected.
bool Tas5805m::begin(byte adr, byte pinPDN, bool start)
{
    _adr = adr;
    _pinPDN = pinPDN;

#ifdef ESP32
    pinMode(_pinPDN, OUTPUT);
#else
    pinMode(_pinPDN, OUTPUT_OPENDRAIN);
#endif

    powerDown();
    if(start)
    {
        delay(500);
        powerUp();
    }

    return setBookPage(0,0);
}

void Tas5805m::powerDown()
{
    digitalWrite(_pinPDN, LOW);

    LOG <<LOG.dec <<"Tas5805m::powerDown..." <<LOG.endl;
}

bool Tas5805m::powerUp()
{
    bool result = true;

    LOG <<LOG.dec <<"Tas5805m::powerUp..." <<LOG.endl;

    // Once power supplies are stable,bring up PDN to High and wait 5ms at least,then start SCLK,LRCLK.
    digitalWrite(_pinPDN, HIGH);
    delay(500);
    _isReset = true;

    // soft reset !? when no PDN is connected
    result &= write(DEVICE_CTRL_2, 0x1A);  // DSP reset + HiZ + Mute
    delay(100);
    result &= write(DEVICE_CTRL_2, 0x0A);  // DSP run + HiZ + Mute

    result &= setBookPage(0,0);
    if(!result)
        return result;
    
    LOG <<LOG.dec <<"Tas5805m::powerUp... init: " <<result <<LOG.endl;

    result &= write(DEVICE_CTRL_2, 0x08);  // mute
    LOG <<LOG.dec <<"Tas5805m::powerUp... init 0: " <<result <<LOG.endl;

    result &= write(DEVICE_CTRL_2, 0x1A);  // DSPreset + HiZ + mute
    LOG <<LOG.dec <<"Tas5805m::powerUp... init 1: " <<result <<LOG.endl;

    delay(5);
    result &= write(RESET_CTRL, 0x11); //reset DSP and CTL
    LOG <<LOG.dec <<"Tas5805m::powerUp... init 2: " <<result <<LOG.endl;

    delay(5);
    result &= write(SAP_CTRL1, 0x02); //0x02 = 24bit I2S, 0x00 = 16 bit i2s, eigentlich egal weil left aligned
    
    // write(ANA_CTRL, 0x03); //175kHz Bandpass ??
    result &= write(DSP_MISC, 0x08); //decouple BQ coefs for LR
    delay(5);
    result &= write(DEVICE_CTRL_2, 0x0B);  // Play + Mute
    LOG <<LOG.dec <<"Tas5805m::powerUp... init 3: " <<result <<LOG.endl;

    delay(5);
    result &= write(ADR_PIN_CTRL, 1); // enable output
    LOG <<LOG.dec <<"Tas5805m::powerUp... init 4: " <<result <<LOG.endl;
    // ADR_PIN_CONFIG:
    // 00000: off (low)
    // 00011: Auto mute flag (asserted when both L and R channels are auto muted)
    // 00100: Auto mute flag for left channel 0101: Auto mute flag for right channel
    // 00110: Clock invalid flag (clock error or clock missing) 00111: Reserved
    // 01001: Reserved
    // 01011: ADR as FAULTZ output
    delay(5);
    result &= write(ADR_PIN_CONFIG, 0b01011);

    // set level meter in/out-puts, default 0,0,1,1
    // setBookPage(0x8c,0x2c);
    // result &= write_9_23(0x0c, 1.0); //li
    // result &= write_9_23(0x10, 0.0); //ri
    // result &= write_9_23(0x14, 1.0); //lo
    // result &= write_9_23(0x18, 0.0); //ro

    // toggle LevelMeter to input
    // setBookPage(0x8c, 0x2c);
    // write_9_23(0x0c, 1.0);
    // write_9_23(0x10, 1.0);
    // write_9_23(0x14, 0.0);
    // write_9_23(0x18, 0.0);

    return result;
}

bool Tas5805m::unMute()
{
    LOG <<LOG.dec <<"Tas5805m::unMute" <<LOG.endl;
    return write(DEVICE_CTRL_2, 0x03);  // Play + UnMute
}

// These bits indicate the currently detected audio sampling rate.
// 0000: FS Error
// 0010: 8KHz, 0100: 16KHz
// 0110: 32KHz 1000: Reserved
// 1001: 48KHz 1011: 96KHz Others Reserved

bool Tas5805m::loop(bool printLevels)
{
    bool result = readStatus();

    if(result)
    {
        readLevels(printLevels);
    }
    else
    {
        _online = false;
        LOG <<LOG.hex <<"Tas5805m::loop OFFLINE adr:" <<_adr <<LOG.endl;
    }
    return _online;
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
    LOG <<LOG.bin << "Tas5805m::setAGain " <<ugain <<"\n";
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
    // LOG <<LOG.hex <<"setDigitalVolume: " <<gain <<" -> " <<vol <<"\n";
    setBookPage(0,0);
    write(DIG_VOL_CTL, vol); //0x30 = 0db  adding 1 reduces gain by .5dB
}
void Tas5805m::setChannels(Channel chA, Channel chB)
{
    // return;  /// hier is was faul !!! Wenn ich das mach wirds leise !!!
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
            write_9_23(0x18, 0.7); // left -> left
            write_9_23(0x1c, 0.7); // right -> left
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
            write_9_23(0x20, 0.7); // left -> right
            write_9_23(0x24, 0.7); // right -> right
            break;
    }
}

//adau hat 5.23 format, tas hat 5.27
// a1 and a2 must be negated
void Tas5805m::setCoefficients(uint32_t stage, const float *coef, Channel ch)
{
    if(stage < 0 || stage > 14)
    {
        // LOG <<"Tas5805::setCoef.. stage out of range [0..14]" <<stage <<"\n";
        return;
    }
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

    byte _bqBookL[16];
    byte _bqBookR[16];
    byte _bqAddrL[16];
    byte _bqAddrR[16];

    _bqBookL[ 0] = 0x24;    _bqAddrL[ 0] = 0x18;    _bqBookR[ 0] = 0x26;    _bqAddrR[ 0] = 0x54; //1
    _bqBookL[ 1] = 0x24;    _bqAddrL[ 1] = 0x2C;    _bqBookR[ 1] = 0x26;    _bqAddrR[ 1] = 0x68; //2
    _bqBookL[ 2] = 0x24;    _bqAddrL[ 2] = 0x40;    _bqBookR[ 2] = 0x26;    _bqAddrR[ 2] = 0x7C; //ahh 0x7C
    _bqBookL[ 3] = 0x24;    _bqAddrL[ 3] = 0x54;    _bqBookR[ 3] = 0x27;    _bqAddrR[ 3] = 0x18; //4
    _bqBookL[ 4] = 0x24;    _bqAddrL[ 4] = 0x68;    _bqBookR[ 4] = 0x27;    _bqAddrR[ 4] = 0x2C;
    _bqBookL[ 5] = 0x24;    _bqAddrL[ 5] = 0x7C;    _bqBookR[ 5] = 0x27;    _bqAddrR[ 5] = 0x40; //ahh
    _bqBookL[ 6] = 0x25;    _bqAddrL[ 6] = 0x18;    _bqBookR[ 6] = 0x27;    _bqAddrR[ 6] = 0x54;
    _bqBookL[ 7] = 0x25;    _bqAddrL[ 7] = 0x2C;    _bqBookR[ 7] = 0x27;    _bqAddrR[ 7] = 0x68;
    _bqBookL[ 8] = 0x25;    _bqAddrL[ 8] = 0x40;    _bqBookR[ 8] = 0x27;    _bqAddrR[ 8] = 0x7C; //ahh
    _bqBookL[ 9] = 0x25;    _bqAddrL[ 9] = 0x54;    _bqBookR[ 9] = 0x28;    _bqAddrR[ 9] = 0x18;
    _bqBookL[10] = 0x25;    _bqAddrL[10] = 0x68;    _bqBookR[10] = 0x28;    _bqAddrR[10] = 0x2C;
    _bqBookL[11] = 0x25;    _bqAddrL[11] = 0x7C;    _bqBookR[11] = 0x28;    _bqAddrR[11] = 0x40; //ahh
    _bqBookL[12] = 0x26;    _bqAddrL[12] = 0x18;    _bqBookR[12] = 0x28;    _bqAddrR[12] = 0x54; //13
    _bqBookL[13] = 0x26;    _bqAddrL[13] = 0x2C;    _bqBookR[13] = 0x28;    _bqAddrR[13] = 0x68; //14
    _bqBookL[14] = 0x26;    _bqAddrL[14] = 0x40;    _bqBookR[14] = 0x28;    _bqAddrR[14] = 0x7C; //ahh

    if(ch == BOTH || ch == LEFT)
    {
        if(!setBookPage(0,0))
            return;
        write(BQ_WR_CTRL1,1); // Indicate the first coefficient of a BQ is starting to write.
        byte leftBQp =  _bqBookL[stage];
        byte leftBQ1 =  _bqAddrL[stage];
        writeBQ(leftBQp, leftBQ1, coef);
    }
    if(ch == BOTH || ch == RIGHT)
    {
        if(!setBookPage(0,0))
            return;
        write(BQ_WR_CTRL1,1); // Indicate the first coefficient of a BQ is starting to write.
        byte rightBQp = _bqBookR[stage];
        byte rightBQ1 = _bqAddrR[stage];
        writeBQ(rightBQp, rightBQ1, coef);
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
    if(retval != 0)
       result = false;
    _online &= result;
    if(!result)
        return result;
    // logerror("setBookPage1", retval, 0);
    Wire.beginTransmission(_adr);
    Wire.write(0x7f);
    Wire.write(book);
    retval = Wire.endTransmission();
    if(retval != 0)
       result = false;
    _online &= result;
    if(!result)
        return result;
    // logerror("setBookPage2", retval, 0x7f);
    Wire.beginTransmission(_adr);
    Wire.write(0);
    Wire.write(page);
    retval = Wire.endTransmission();
    logerror("setBookPage", retval, _adr);
    _online &= result;
    return result;
}

bool Tas5805m::write(byte reg, byte data)
{
    Wire.beginTransmission(_adr);
    Wire.write(reg);
    Wire.write(data);
    byte retval = Wire.endTransmission();
    if(retval != 0)
       return false;
    return true;
}

bool Tas5805m::write(byte reg, byte *buffer, uint8_t len)
{
    Wire.beginTransmission(_adr);
    Wire.write(reg);
    Wire.write(buffer, len);
    byte retval = Wire.endTransmission();
    if(retval != 0)
       return false;
    return true;
}

bool Tas5805m::write_9_23(byte reg, float val)
{
    int32_t ival;
    uint8_t buf[4];

    // beim read gehts ja...
    // int32_t iVal  = buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3];
    ival = swap32(val * (float)(8388608.0));  // swap only on ARM !
    // LOG << LOG.hex <<"write_9_23: " <<val <<" int:" <<ival <<LOG.endl;

    buf[0] = (ival >> 24) & 0xff;
    buf[1] = (ival >> 16) & 0xff;
    buf[2] = (ival >>  8) & 0xff;
    buf[3] = (ival      ) & 0xff;

    return write(reg, buf, 4);
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

int32_t Tas5805m::read_32i(byte adr)
{
    byte buf[4];
    int len = read(adr, buf, 4);
    // LOG <<"levels: len:" <<len <<": ";
    // for(int i = 0; i < 4; i++)
    //     LOG.hex() <<buf[i] <<" ";
    // LOG <<"\n";
    int32_t iVal  = (buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3]);
    return iVal;
}
float Tas5805m::read_1_31f(byte adr)
{
    byte buf[4];
    int len = read(adr, buf, 4);
    // LOG <<"levels: len:" <<len <<": ";
    // for(int i = 0; i < 4; i++)
    //     LOG.hex() <<buf[i] <<" ";
    // LOG <<"\n";
    int32_t iVal  = (buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3]);
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
    int32_t iVal = (buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3]);
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
    int32_t iVal = (buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3]);
    float   fVal = (float)iVal / (float)(8388608.0);
    return fVal;
}

bool Tas5805m::readStatus()
{
    bool result = true;
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

    result &= setBookPage(0,0);
    if(!result)
    {
        LOG <<"Tas5805m:readStatus:setBookPage FAIL" <<LOG.endl;
        return result;
    }

    byte chf = read(CHAN_FAULT);
    byte gf1 = read(GLOBAL_FAULT1);
    byte gf2 = read(GLOBAL_FAULT2);
    if(chf&0xff || gf1&0xc7 || gf2&0x01)
    {
        LOG <<"Tas5805m:readStatus:ERRORS: ";
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
        LOG <<"Tas5805m:readStatus:WARNING: " <<"Over temperature warning ,135C" <<"\n";
    }

    return result; //skip the details


    byte dc1 = read(DEVICE_CTRL_1);
    byte dc2 = read(DEVICE_CTRL_2);
    LOG <<LOG.hex <<"Tas5805m:readStatus:device ctl 1:" <<dc1 <<" 2:" <<dc2 <<"\n";

    byte buf[6];
    int resultFS = read(FS_MON, buf, 2);
    int bckRatio = ((buf[0] & 0x30) <<4) | buf[1];
    resultFS += read(CHAN_FAULT, buf+2, 4);
    byte ams = read(AUTOMUTE_STATE);
    LOG <<LOG.hex <<"Tas5805m:readStatus:" <<resultFS <<"bytes.. bckR:" <<LOG.dec <<bckRatio <<LOG.hex <<" ams:" <<(int(ams&0xfc))  <<" chf:"<<buf[2] <<" glf1:"<<buf[3] <<" glf2:"<<buf[4] <<" otw:"<<buf[5] ;
    switch(buf[0])
    {
        case 0:
            LOG <<" fs error";
            break;
        case 0b0010:
            LOG <<" fs 8kHz";
            break;
        case 0b0100:
            LOG <<" fs 16kHz";
            break;
        case 0b0110:
            LOG <<" fs 32kHz";
            break;
        case 0b1001:
            LOG <<" fs 48kHz";
            break;
        case 0b1011:
            LOG <<" fs 96kHz";
            break;
        default:
            LOG <<" fs ???kHz" <<int(buf[0]) ;
    }
    LOG <<LOG.endl;
    // byte sap1 = read(SAP_CTRL1);
    // byte sap2 = read(SAP_CTRL2);
    // byte sap3 = read(SAP_CTRL3);
    // LOG <<LOG.hex <<"   SAP_CTRL1:" <<sap1 <<"   SAP_CTRL2:"<<sap2 <<"   SAP_CTRL3:"<<sap3 <<LOG.endl;
    return result;
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

void Tas5805m::writeBQ(byte page, byte adr, const float *coef)
{
    byte result = 0;
    int32_t b0 = (int32_t)(coef[0]*0x8000000);
    int32_t b1 = (int32_t)(coef[1]*0x8000000);
    int32_t b2 = (int32_t)(coef[2]*0x8000000);
    int32_t a1 = (int32_t)(-1*coef[3]*0x8000000);
    int32_t a2 = (int32_t)(-1*coef[4]*0x8000000);
    // ahhh it's little endian
    byte *x0 = (byte*)&b0;
    byte *x1 = (byte*)&b1;
    byte *x2 = (byte*)&b2;
    byte *x3 = (byte*)&a1;
    byte *x4 = (byte*)&a2;

    byte buf[20];
    byte *pbuf = buf;
    LOG <<LOG.bin;
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
    setBookPage(0xaa, page);
    if(adr != 0x7c)
    {
        Wire.beginTransmission(_adr);
        Wire.write(adr);
        Wire.write(buf,20);
        result = Wire.endTransmission();
    }
    else // pffff... we must switch pages after first coeff
    {
        Wire.beginTransmission(_adr);
        Wire.write(adr);
        Wire.write(buf,4);
        result = Wire.endTransmission();

        setBookPage(0xaa, page+1);

        Wire.beginTransmission(_adr);
        Wire.write(0x18); // next allway at 0x18
        Wire.write(buf+4,16);
        result = Wire.endTransmission();
    }
    logerror("writeBQ0:", result, adr);
}

void Tas5805m::readLevels(bool printLevels)
{
    setBookPage(0x78, 0x02);
    _levelLeft   = read_1_31f(0x60);
    _levelReight = read_1_31f(0x64);
    _dbLeft  = 20.0*log10f(_levelLeft);
    _dbRight = 20.0*log10f(_levelReight);
    // LOG.dez(0) <<"level l:" <<_dbLeft <<"db r:" <<_dbRight <<"db\n";
    int dbRestLeft  = 100+_dbLeft;
    int dbRestRight = 100+_dbRight;
    if(dbRestLeft < 0)
        dbRestLeft = 0;
    if(dbRestLeft > 99)
        dbRestLeft = 100;
    if(dbRestRight > 99)
        dbRestRight = 100;

    if(printLevels)
    {
        LOG <<LOG.hex; // << "\033[22;34mHello, world!\033[0m";
        LOG <<"--------- AMP ----  0x" <<_adr <<LOG.dec <<" l:"<<_dbLeft <<" r:" <<_dbRight <<"\n";
        for(int i = 0; i < 25; i++)
            LOG <<".";
        LOG <<"\n";
        for(int i = 0; i < dbRestLeft/4; i++)
            LOG <<"*";
        LOG <<"\n";
        for(int i = 0; i < dbRestRight/4; i++)
            LOG <<"*";
        LOG <<"\n";
    }

    // setBookPage(0x8c, 0x29);
    // float mixLeftLeft     = read_9_23f(0x18);
    // float mixReightReight = read_9_23f(0x24);
    // LOG <<"mixll:" <<mixLeftLeft <<" mixrr:" <<mixReightReight <<"\n";

    return;

    setBookPage(0x8c,0x29);
    // int32_t xii1   = read_32i(0x18); // left -> left
    // int32_t xii2   = read_32i(0x1c); // left -> right
    // int32_t xii3   = read_32i(0x20); // right -> left
    // int32_t xii4   = read_32i(0x24); // right -> right
    // LOG <<LOG.hex <<"input xbar:\t\t" <<xii1 <<" " <<xii2 <<" " <<xii3 <<" " <<xii4 <<"\n";
    float xi1     = read_9_23f(0x18); // left -> left
    float xi2     = read_9_23f(0x1c); // right -> left
    float xi3     = read_9_23f(0x20); // left -> right
    float xi4     = read_9_23f(0x24); // right -> right
    LOG <<LOG.dec <<"input xbar:\t\t" <<xi1 <<" " <<xi2 <<" " <<xi3 <<" " <<xi4 <<"\n";

    setBookPage(0x8c,0x2c);  // output xbar
    float xb1     = read_9_23f(0x1c);
    float xb2     = read_9_23f(0x20);
    float xb3     = read_9_23f(0x28);
    float xb4     = read_9_23f(0x2c);
    LOG <<"output xbar:\t\t" <<xb1 <<" " <<xb2 <<" " <<xb3 <<" " <<xb4 <<"\n";

    float ml1     = read_9_23f(0x0c);   // Level Meter Left gain from Left Input
    float ml2     = read_9_23f(0x10);   // Level Meter Left gain from Right Input
    float ml3     = read_9_23f(0x14);   // Level Meter Left gain from Left Output
    float ml4     = read_9_23f(0x18);   // Level Meter Left gain from Right Output
    LOG <<"meter mix level:\t" <<ml1 <<" " <<ml2 <<" " <<ml3 <<" " <<ml4 <<"\n";
#if 0
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

