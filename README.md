# TashPower

## Elevator Pitch

It's firmware for a companion product to give soft power capability to a Macintosh which doesn't have it.  It imitates the PowerKey by Sophisticated Circuits and can power the Macintosh on by a press of the keyboard's power key and power it off when it shuts down, as well as powering the Macintosh on and off on a user-defined schedule.


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

The relay control line is active high.

The inputs all have internal weak pullup resistors enabled, but stronger external resistors should be used for the PS/2 lines (10kohm or stronger).


### Building Firmware

Building the firmware requires Microchip MPASM, which is included with their development environment, MPLAB. Note that you **must** use MPLAB X version 5.35 or earlier or MPLAB 8 as later versions of MPLAB X have removed MPASM.
