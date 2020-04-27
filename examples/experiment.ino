/* platformio.ini
[env:teensy40]
platform = teensy
board = teensy40
framework = arduino
build_flags = -D TEENSY_OPT_SMALLEST_CODE -D USB_MIDI_AUDIO_SERIAL -Wunused-variable
lib_deps =
    12
*/

#include <Arduino.h>
#include <Wire.h>   
#include <mbLog.h>
#include <elapsedMillis.h>
#include <ClickEncoder.h>  // it's on github
#include <SPI.h>

#define WITH_JSON
#ifdef WITH_JSON   // ~4k flash
#include <ArduinoJson.h>
#endif

#include <mbConfig.h>
#define AUDIO_SAMPLE_RATE_EXACT 96000
// #define AUDIO_SAMPLE_RATE_EXACT 88200
#include <mbPageEQ.h>

#include "tas5805m.h"

#include <usb_names.h>

// #include "./Fonts/Org_01.h"
// #define FontType Org_01 // oh schick, leicht spaceisch //baseline +5


// Encoders
const byte pinROT1L = 3;
const byte pinROT1R = 2;
const byte pinROT1C = 4;

const byte pinSWRU = 5;
const byte pinSWRD = 6;

const byte pinLED = 13; //conflict with SPI
const byte TasPDN = 9;

const byte midiChannel = 1;

void setCoefficientsCB(uint32_t stage, const float *coefficients);
DisplayType _display;
mbPageEQ    _pageEQ(&setCoefficientsCB);

ClickEncoder _encoder1(pinROT1R, pinROT1L, pinROT1C, 4, LOW); // Bourns, STEC12E08(Alps plastik)

IntervalTimer encoderTimer;
elapsedMillis timerLED;
elapsedMillis timerAMP;
elapsedMillis timerInput;
elapsedMillis timerRestart;
elapsedMillis timerUSBMidi;

Tas5805m _amp0;
Tas5805m _amp2;

void setHighpass(Tas5805m &unit, uint32_t stage, float frequency, float q, Tas5805m::Channel ch);
void setLowpass (Tas5805m &unit, uint32_t stage, float frequency, float q, Tas5805m::Channel ch);

bool led = true;
float   again = -15.0;  //-15.5..0 dB determines max output, should fit the speaker
int16_t gain = -20; // initial volume starts low... move to EEPROM...
bool someOnline = true;

#ifdef WITH_JSON
DynamicJsonDocument docReceived(256);
DynamicJsonDocument docToSend(1024);
#endif

class PageCover : public PageType
{
  public:
    PageCover() : PageType(),
        _volume("vol", 0, 127),
        _func  ("vis", 0, 3),
        _horst ("pups", 0, 127)
    {
        _params[_paramCount++] = &_volume;
        _params[_paramCount++] = &_func;
        _params[_paramCount++] = &_horst;
    }

    inline void redraw() override
    {
        display().clearDisplay();
        // display().fillRect(5,5,20,20,1);
    }

    inline void setLevels( float l0, float r0, float l2, float r2 )
    {
        display().fillRect(0, 0, 48, 64, 0);
        display().fillRect( 0, l0/-2, 8, 2, 1);
        display().fillRect(12, r0/-2, 8, 2, 1);
        display().fillRect(24, l2/-2, 8, 2, 1);
        display().fillRect(36, r2/-2, 8, 2, 1);
        // LOG <<"display: " <<l0 <<" " <<r0 <<"\n";
    }

    inline void update(bool force = false) override
    {
        return PageType::update(force);
    }

private:

    class VolDispMapper
    {
    public:
        int16_t operator()(uint8_t value)
        {
            // -103.5(MUTE)..24
            int16_t val = -104;
            return val+value;
        }
    };

    class TriggerVol
    {
    public:
        void operator()(uint8_t value)
        {
            int16_t val = -104;
            val += value;
            _amp0.setDigitalVolume(val);
            _amp2.setDigitalVolume(val);

        }
    };

    mbParameterRB<PAGES_POS_X2, PAGES_POS_Y0, PAGES_WIDTH_X1, PAGES_HEIGHT_X1, VolDispMapper, TriggerVol> _volume;
    mbParameterRB<PAGES_POS_X2, PAGES_POS_Y1, PAGES_WIDTH_X1, PAGES_HEIGHT_X1> _func;
    mbParameterRB<PAGES_POS_X2, PAGES_POS_Y2, PAGES_WIDTH_X1, PAGES_HEIGHT_X1> _horst;

} _pageCover;

void setCoefficientsCB(uint32_t stage, const float *coefficients)
{
    if(stage < 0 || stage > 3)
        return;
    
    LOG <<"setCoefficientsCB " <<stage;
    for(int i = 0; i < 5; i++)
        LOG <<" " <<coefficients[i];
    LOG <<"\n";

    _amp0.setCoefficients(stage, coefficients, Tas5805m::BOTH);
    _amp2.setCoefficients(stage, coefficients, Tas5805m::BOTH);
}

void encoderServiceFunc()
{
    _encoder1.service();
}

void startAmps()
{
    if(_amp0.begin((0x58+0)>>1, TasPDN, LOW))
    {
        _amp0.setChannels(Tas5805m::LEFT, Tas5805m::BOTH);
        _amp0.setAnalogGain(again);
        setLowpass (_amp0, 13, 250, 0.7, Tas5805m::RIGHT);
        setHighpass(_amp0, 13, 250, 0.7, Tas5805m::LEFT);
        _amp0.setDigitalVolume(gain);
        _amp0.ctlPlay();
    }
    else
    {
        LOG <<LOG.hex <<"no TAS5805m found @0x" <<_amp0.getAdr();
    }
    if(_amp2.begin((0x58+2)>>1, TasPDN, LOW))
    {
        _amp2.setChannels(Tas5805m::RIGHT, Tas5805m::BOTH);
        _amp2.setAnalogGain(again);
        setLowpass (_amp2, 13, 250, 0.7, Tas5805m::RIGHT);
        setHighpass(_amp2, 13, 250, 0.7, Tas5805m::LEFT);
        _amp2.setDigitalVolume(gain);
        _amp2.ctlPlay();
    }
    else
    {
        LOG <<LOG.hex <<"no TAS5805m found @0x" <<_amp2.getAdr();
    }
}

void handleNoteOn(uint8_t channel, uint8_t note, uint8_t velocity)
{
    LOG <<"note on: " <<LOG.hex <<note <<"\n";
}

void handleNoteOff(uint8_t channel, uint8_t note, uint8_t velocity)
{
    LOG <<"note offç: " <<LOG.hex <<note <<"\n";
}

void setup()
{
    pinMode(pinROT1L, INPUT_PULLUP);
    pinMode(pinROT1R, INPUT_PULLUP);
    pinMode(pinROT1C, INPUT_PULLUP);
    pinMode(pinSWRU,  INPUT_PULLUP);
    pinMode(pinSWRD,  INPUT_PULLUP);
    pinMode(TasPDN, OUTPUT);
    
    // _encoder1.setDoubleClickTime(250); // long times delay the clicked detection !?
    encoderTimer.begin(encoderServiceFunc, 1000);  // run every 1 miliseconds

    SPI.begin();
    // SPI.setClockDivider(SPI_CLOCK_DIV2);

    Serial.begin(115200);
    //Wait for console...
    // while (!Serial);
    // Serial.println("setup - start");

    Wire.begin();
    // Wire.setClock(400000); // mhm does not work properly

    // _display.get().setFont(&FontType);  // reduced 778byte oder so
    _display.addPage(&_pageCover);
    _display.addPage(&_pageEQ);
    _display.begin();

    startAmps();

    usbMIDI.setHandleNoteOn(handleNoteOn);
    usbMIDI.setHandleNoteOff(handleNoteOff);

    mbStorage::the()->dump();
    // mbStorage::the()->restore();
    // _display.restore();
}

void loop()
{
    // LOG <<"loop\n";

    if(timerInput > 20)
    {
        timerInput = 0;

        auto button = _encoder1.getButton();
        if(button == ClickEncoder::Clicked)
        {
            _display.changeActiveParam(1);
        }
        else if(button == ClickEncoder::DoubleClicked)
        {
            _display.changeCurrentPage(1);
        }

        int16_t val = _encoder1.getValue();
        if(val)
        {
            _display.getPage().encoderValue(val);

            // gain += val;
            // // LOG.dec() <<"encoder:" <<val <<" gain:" <<gain <<"\n";
            // _amp0.setDigitalVolume(gain);
            // _amp2.setDigitalVolume(gain);
        }

    }

    if(timerUSBMidi > 20)
    {
        timerUSBMidi = 0;
        if(usbMIDI.read(midiChannel))
        {
            if(usbMIDI.getType() == 0xF0)
            {
                int midiSysLength = usbMIDI.getSysExArrayLength();
                if(midiSysLength > 3)
                {
                    // LOG <<LOG.dec <<"ohhh sysex !! len:" <<midiSysLength <<" type:" <<LOG.hex <<usbMIDI.getType() <<"\n";
                    uint8_t *sysex = usbMIDI.getSysExArray();
                    sysex[midiSysLength-1] = 0; // add a string end
                    sysex++;

                    LOG <<"sysex: \"" <<(char*)(sysex) <<"\" hex:";
                    for(int i = 0; i < midiSysLength; i++)
                        LOG <<LOG.hex <<" 0x" <<sysex[i];
                    LOG <<"\n";

#ifdef WITH_JSON

                    // parse..
                    deserializeJson(docReceived, sysex);
                    const char* name = docReceived["name"];
                    if(name)
                    {
                        LOG <<"deserialize:" <<name <<"\n";
                    }
                    else
                    {
                        LOG <<"deserialization... no name\n";
                    }
                    const char* cmd = docReceived["cmd"];
                    if(cmd)
                    {
                        LOG <<"deser: got cmd " <<cmd <<"\n";
                        if(strcmp(cmd, "disp") == 0)
                        {
                            docToSend.clear();
                            docToSend["pages"] = 3;
                        }
                        else
                        {
                            // #define USB_MIDI_SYSEX_MAX 290 // steht in usb_midi.h
                            docToSend["msg"] = "Hallo SysEx !!!";
                            docToSend["time"]   = 1351824120;
                            docToSend["data"][0] = 48.756080;
                            docToSend["data"][1] = 2.302038;
                        }
                    }
                    uint8_t data[256];
                    size_t len = serializeJson(docToSend, data, 256);
                    usbMIDI.sendSysEx(len, data);
                    // This prints:
                    // {"sensor":"gps","time":1351824120,"data":[48.756080,2.302038]}
#endif
                }
            }
        }
    }
    // blink...
    if(timerLED > 50)
    {
        led = !led;
        digitalWrite(pinLED, led);
        // LOG <<"led an " <<led <<"\n";
        timerLED = 0;

        _pageCover.setLevels(
            _amp0.getLevelDBLeft(),
            _amp0.getLevelDBReight(),
            _amp2.getLevelDBLeft(),
            _amp2.getLevelDBReight() );

        _display.update();
    }
    
    if(timerAMP > 100 && someOnline)
    {
        timerAMP = 0;
        if(someOnline)
        {
            someOnline = false;
            someOnline |= _amp0.loop(false);
            someOnline |= _amp2.loop(false);
        }
    }
    if(timerRestart > 3000)
    {
        timerRestart = 0;
        if(!someOnline)
        {
            LOG <<"restarting amps !\n";
            startAmps();
            someOnline = true;
        }

        // mbStorage::the()->store();  // hängt sich auf !
    }

}

/* ADAU1704
Numerical Format: 5.23
Linear range: −16.0 to (+16.0 − 1 LSB)
ADAU1701
  with a range of 1.0 (minus 1 LSB) to −1.0.
 Figure 29 shows the maximum signal levels at each point in the data flow in both binary and decibel levels.
4-BIT SIGN EXTENSION
 Examples: 
  1000 0000   0000 0000 0000 0000 0000 = −16.0
  1110 0000   0000 0000 0000 0000 0000 = −4.0
  1111 1000   0000 0000 0000 0000 0000 = −1.0
  1111 1110   0000 0000 0000 0000 0000 = −0.25
  1111 1111   0011 0011 0011 0011 0011 = −0.1
  1111 1111   1111 1111 1111 1111 1111 = (1 LSB below 0.0)
  0000 0000   0000 0000 0000 0000 0000 = 0.0
  0000 0000   1100 1100 1100 1100 1101 = 0.1
  0000 0010   0000 0000 0000 0000 0000 = 0.25
  0000 1000   0000 0000 0000 0000 0000 = 1.0
  0010 0000   0000 0000 0000 0000 0000 = 4.0
  0111 1111   1111 1111 1111 1111 1111 = (16.0 − 1 LSB).
*/

//adau hat 5.23 format, tas hat 5.27

// a1 and a2 must be negated


void setHighpass(Tas5805m &unit, uint32_t stage, float frequency, float q, Tas5805m::Channel ch) {
    float coef[5];
    double w0 = frequency * (2.0 * 3.141592654 / 48000.0);
    double sinW0 = sin(w0);
    double alpha = sinW0 / ((double)q * 2.0);
    double cosW0 = cos(w0);
    double scale = 1.0 / (1.0 + alpha);
    /* b0 */ coef[0] = ((1.0 + cosW0) / 2.0) * scale;
    /* b1 */ coef[1] = -(1.0 + cosW0) * scale;
    /* b2 */ coef[2] = coef[0];
    /* a1 */ coef[3] = 1.0* (-2.0 * cosW0) * scale;
    /* a2 */ coef[4] = 1.0* (1.0 - alpha) * scale;
    unit.setCoefficients(stage, coef, ch);
}

void setLowpass(Tas5805m &unit, uint32_t stage, float frequency, float q, Tas5805m::Channel ch)
{
    float coef[5];
    double w0 = frequency * (2.0 * 3.141592654 / 88200.0);
    double sinW0 = sin(w0);
    double alpha = sinW0 / ((double)q * 2.0);
    double cosW0 = cos(w0);
    double scale = 1.0 / (1.0 + alpha);   // sollte sein beim ADAU:0x800000, beim TAS:0x8000000  ???? Original:1073741824.0
    /* b0 */ coef[0] = ((1.0 - cosW0) / 2.0) * scale;
    /* b1 */ coef[1] = (1.0 - cosW0) * scale;
    /* b2 */ coef[2] = coef[0];
    /* a1 */ coef[3] = 1.0* (-2.0 * cosW0) * scale;
    /* a2 */ coef[4] = 1.0* (1.0 - alpha) * scale;
    unit.setCoefficients(stage, coef, ch);
}
