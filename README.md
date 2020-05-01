# Control a TAS5805M

or similar devices ? A look at the datasheet and process flow document is strongly encuraged.

A breakout board can be found in my mbAMP2 project or eventually one day on [Tindie](https://www.tindie.com/stores/majobecher/).

# Attention! Achtung! Pozor! 留神！

Do not use with your overexpensive audio equipment, this is DIY ! Aeehh I mean, you could **damage** drivers and ears while playing with this ! Also the AMP can handle quite some Amps, be aware of sparking shorts and magic smoke. You been warned.

# Howto

## Run Examples

See examples/demo.ino. Never tested with Arduino GUI, I use PlatformIO, create an empty project, add this lib and mbLog, then include the demo.ino in main.cpp. That should do it.

```
// src/main.cpp
#include "../lib/TAS58xx/examples/demo.ino"
```

This example boots 2 of these chipies with addresses 0x58 (4.7k @adr) and 0x5a (15k @adr). I used a Teensy4.0 to control ~~and also pipe usb audio to the chipies~~  or TeensyLC.

It initializes a crossover setup where left channel is sub <80Hz and right channel is wideband 80Hz++. Other biquad formulas can be borrowed from TeensyAudio library, just the fixed point format needs adjustment and negate the a[12] coefficients.

It also uses an encoder at pins 2,3 to control volume. And the terminal will print a level meter :)

The demo starts with minimal gain and volume settings, see setAnalogGain() and setDigitalVolume() functions.

## Connect Hardware

Audio input can be almost any I2S capable device which can run as a master. E.g Teensy3.2 and up, some PCM1808(needs clock), SPDIF/bluetooth/HDMI receiver boards from China, etc...

I recommend laptop power bricks from the junkyard as power supply. No truly, these are often thrown away for no reason, please recycle !

# Todo

* Power Up Sequence fails sometimes !
* Power down detect, could trigger a mute before caps run out.
