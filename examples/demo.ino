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
#include <mbLog.h>
#include <elapsedMillis.h>
#include <ClickEncoder.h>  // it's on github
#include <SPI.h>

#include "tas5805m.h"

// Reserved Pins...

// Audio           |  I2C
// MCLK = 23       | SCL 19
// BCLK = 21       | SDA 18
// LRCLK = 20
// DIN 8  (MISO)
// DOUT 7 (MOSI)

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioSynthWaveformSine   sine1;          //xy=130,269
AudioSynthWaveformPWM    pwm1;           //xy=226,367
AudioInputUSB            usb1;           //xy=259,165
AudioInputI2S            i2s2;           //xy=268,91
AudioSynthNoiseWhite     noise1;         //xy=286,313
AudioMixer4              mixer1;         //xy=525,127
AudioMixer4              mixer4;         //xy=533,403
AudioMixer4              mixer3;         //xy=534,328
AudioMixer4              mixer2;         //xy=537,217
AudioOutputI2S           i2s1;           //xy=703,363
AudioOutputUSB           usb2;           //xy=713,170
AudioAnalyzePeak         peak1;          //xy=713,324
AudioAnalyzePeak         peak2;          //xy=714,439
AudioAnalyzeRMS          rms1;           //xy=715,284
AudioAnalyzeRMS          rms2;           //xy=715,402
AudioConnection          patchCord1(sine1, pwm1);
AudioConnection          patchCord2(pwm1, 0, mixer3, 3);
AudioConnection          patchCord3(pwm1, 0, mixer4, 3);
AudioConnection          patchCord4(usb1, 0, mixer3, 0);
AudioConnection          patchCord5(usb1, 0, mixer1, 0);
AudioConnection          patchCord6(usb1, 1, mixer4, 0);
AudioConnection          patchCord7(usb1, 1, mixer2, 0);
AudioConnection          patchCord8(i2s2, 0, mixer1, 1);
AudioConnection          patchCord9(i2s2, 0, mixer3, 1);
AudioConnection          patchCord10(i2s2, 1, mixer2, 1);
AudioConnection          patchCord11(i2s2, 1, mixer4, 1);
AudioConnection          patchCord12(noise1, 0, mixer3, 2);
AudioConnection          patchCord13(noise1, 0, mixer4, 2);
AudioConnection          patchCord14(mixer1, 0, usb2, 0);
AudioConnection          patchCord15(mixer4, 0, i2s1, 1);
AudioConnection          patchCord16(mixer4, rms2);
AudioConnection          patchCord17(mixer4, peak2);
AudioConnection          patchCord18(mixer3, 0, i2s1, 0);
AudioConnection          patchCord19(mixer3, peak1);
AudioConnection          patchCord20(mixer3, rms1);
AudioConnection          patchCord21(mixer2, 0, usb2, 1);
// GUItool: end automatically generated code


// Encoders
const byte pinROT1L = 2;
const byte pinROT1R = 3;
const byte pinROT1C = 4;

const byte pinSWRU = 5;
const byte pinSWRD = 6;

const byte pinLED = 13; //conflict with SPI
const byte TasPDN = 9;

ClickEncoder _encoder1(pinROT1R, pinROT1L, pinROT1C, 4, LOW); // Bourns, STEC12E08(Alps plastik)

IntervalTimer encoderTimer;
elapsedMillis timerLED;
elapsedMillis timerDISP;
elapsedMillis timerAMP;
elapsedMillis timerInput;

Tas5805m _amp0;
Tas5805m _amp2;

void setHighpass(Tas5805m &unit, uint32_t stage, float frequency, float q, Tas5805m::Channel ch);
void setLowpass (Tas5805m &unit, uint32_t stage, float frequency, float q, Tas5805m::Channel ch);

void encoderServiceFunc()
{
    _encoder1.service();
}

void setup()
{
    pinMode(pinROT1L, INPUT_PULLUP);
    pinMode(pinROT1R, INPUT_PULLUP);
    pinMode(pinROT1C, INPUT_PULLUP);
    pinMode(pinSWRU,  INPUT_PULLUP);
    pinMode(pinSWRD,  INPUT_PULLUP);
    pinMode(pinLED, OUTPUT);
    
    // _encoder1.setDoubleClickTime(250); // long times delay the clicked detection !?
    encoderTimer.begin(encoderServiceFunc, 1000);  // run every 1 miliseconds

    // SPI.begin();
    // SPI.setClockDivider(SPI_CLOCK_DIV2);

    Serial.begin(115200);
    //Wait for console...
    while (!Serial);
    Serial.println("setup - start");

    Wire.begin();
    // Wire.setClock(400000);

    // _display.begin();
    // _display.addPage(&_pageTest);

    // mbStorage::the()->dump();
    // mbStorage::the()->restore();

    // _display.restore();

    float again = -15.0;
    if(_amp0.begin((0x58+0)>>1, TasPDN))
    {
        _amp0.setChannels(Tas5805m::LEFT, Tas5805m::BOTH);
        _amp0.setAnalogGain(again);
        setLowpass (_amp0, 0, 250, 0.7, Tas5805m::RIGHT);
        setHighpass(_amp0, 0, 250, 0.7, Tas5805m::LEFT);
    }
    else
    {
        LOG.hex() <<"no TAS5805m found @0x" <<_amp0.getAdr();
    }
    if(_amp2.begin((0x58+2)>>1, TasPDN))
    {
        _amp2.setChannels(Tas5805m::RIGHT, Tas5805m::BOTH);
        _amp2.setAnalogGain(again);
        setLowpass (_amp2, 0, 250, 0.7, Tas5805m::RIGHT);
        setHighpass(_amp2, 0, 250, 0.7, Tas5805m::LEFT);
    }
    else
    {
        LOG.hex() <<"no TAS5805m found @0x" <<_amp2.getAdr();
    }

    AudioMemory(250); // with mbSynth
    AudioMemoryUsageMaxReset();
    AudioNoInterrupts();
    //Audio
    sine1.amplitude(0.2);
    sine1.frequency(3);
    sine1.phase(0);
    pwm1.amplitude(0.7);
    pwm1.frequency(220);

    mixer1.gain(0, 1);
    mixer1.gain(1, 1);
    mixer1.gain(2, 1);
    mixer1.gain(3, 1);

    mixer2.gain(0, 1);
    mixer2.gain(1, 1);
    mixer2.gain(2, 1);
    mixer2.gain(3, 1);

    float mastervol = 1.0;
    mixer3.gain(0, mastervol * 1.0); // usb
    mixer3.gain(1, mastervol * 0.0); // i2s
    mixer3.gain(2, mastervol * 0.0); // nix
    mixer3.gain(3, mastervol * 0.0); // synth

    mixer4.gain(0, mastervol * 1.0); // usb
    mixer4.gain(1, mastervol * 0.0); // i2s
    mixer4.gain(2, mastervol * 0.0); // nix
    mixer4.gain(3, mastervol * 0.0); // synth

    AudioInterrupts();
}

bool led = true;
int16_t gain = 0;
void loop()
{
    // LOG <<"loop\n";

    if(timerInput > 20)
    {
        int16_t val = _encoder1.getValue();
        if(val)
        {
            gain += val;
            // LOG.dec() <<"encoder:" <<val <<" gain:" <<gain <<"\n";
            _amp0.setDigitalVolume(gain);
            _amp2.setDigitalVolume(gain);
        }
    }

    // blink...
    if(timerLED > 200)
    {
        led = !led;
        digitalWrite(pinLED, led);
        // LOG <<"led an " <<led <<"\n";
        timerLED = 0;
    }
    
    // if(timerDISP > 500)
    // {
    //     timerDISP = 0;

    //     _pageTest.setLevels(rms1.read(), rms2.read(), peak1.read(), peak2.read());
    //     _display.update();

    //     // LOG << "bla\n";
    // }

    if(timerAMP > 100)
    {
        timerAMP = 0;
        LOG <<"--------- AMP 0 ---------\n";
        _amp0.loop();
        LOG <<"--------- AMP 2 ---------\n";
        _amp2.loop();
        LOG <<"\n";

        // http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0491i/CJAEEGCE.html
        // uint32_t swap1 = 0x11223344;
        // uint32_t swap2 = __REV(swap1);
        // LOG.hex() <<"swap: " <<swap1 <<" -> " <<swap2 <<"\n";
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
    /* a1 */ coef[3] = -1.0* (-2.0 * cosW0) * scale;
    /* a2 */ coef[4] = -1.0* (1.0 - alpha) * scale;
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
    /* a1 */ coef[3] = -1.0* (-2.0 * cosW0) * scale;
    /* a2 */ coef[4] = -1.0* (1.0 - alpha) * scale;
    unit.setCoefficients(stage, coef, ch);
}
