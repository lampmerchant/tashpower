# TashPower

## Elevator Pitch

It's firmware for a companion product to give soft power capability to a Macintosh that doesn't have it.  It imitates the PowerKey Classic (model PK-1) by Sophisticated Circuits and can power the Macintosh on by a press of a connected ADB keyboard's power key and power it off when it shuts down as well as powering the Macintosh on and off on a user-defined schedule.

It also features a PS/2 keyboard interface that allows an independently powered PS/2 keyboard to power the Macintosh on and later makes it appear to the Macintosh as an ADB keyboard.


## Technical Details

### Connections

```
                        .--------.                     
                Supply -|01 \/ 08|- Ground             
          ADB <--> RA5 -|02    07|- RA0 <--- PS/2 Data 
ADB Power Key ---> RA4 -|03    06|- RA1 <--- PS/2 Clock
   Line Clock ---> RA3 -|04    05|- RA2 ---> Relay     
                        '--------'                     
```

The relay control line is active low.

External resistors should be used for the PS/2 lines (10 kΩ to 1 kΩ).

The ADB power key and line clock inputs have internal weak pullup resistors. 


### Building Firmware

Building the firmware requires Microchip MPASM, which is included with their development environment, MPLAB. Note that you **must** use MPLAB X version 5.35 or earlier or MPLAB 8 as later versions of MPLAB X have removed MPASM.
