# Control a TAS5805M

or similar devices ?

# Howto

See examples/dome.ino.

This boots 2 of these chipies with addresses 0x58 and 0x5a. I used an Teensy4.0 to control and also pipe usb audio to the chipies.

It initializes a crossover setup where left channel is sub <80Hz> and right channel is wideband 80Hz++.

It also uses an encoder at pins 2,3 to control volume.
