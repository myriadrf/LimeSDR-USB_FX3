/**
 * ----------------------------------------------------------------------------
 * FILE:	Si5351_config_map.h
 * DESCRIPTION:	Si5351 configuration file
 * DATE:	2016.10.27
 * AUTHOR(s):	Lime Microsystems
 * REVISION: v0r1
 * ----------------------------------------------------------------------------
 */

/*
Si5351 configuration:

CLK0 = 27 MHz
CLK1 = 27 MHz
CLK2 = disabled
CLK3 = disabled
CLK4 = disabled
CLK5 = disabled
CLK6 = disabled
CLK7 = disabled

Full report from Si5351C clock builder:

#XTAL (MHz) = 25.000000000
#Mode = Automatic
#PLL A
# Input Frequency (MHz) = 25.000000000
# F divider = 1
# PFD (MHz) = 25.000000000
# VCO Frequency (MHz) =  810.000000000
# Feedback Divider = 32  2/5
# Internal Load Cap (pf) = 10
# SSC disabled
#PLL B
# Input Frequency (MHz) = 0.0
# VCO Frequency (MHz) =  0.0
# Pull Range (ppm) = 0.0
#Output Clocks
#Channel 0
# Output Frequency (MHz) = 27.000000000
# Multisynth Output Frequency (MHz) = 27.000000000
# Multisynth Divider = 30
# R Divider = 1
# PLL source = PLLA
# Initial phase offset (ns) = 0.000
# Error (ppm) = 0.0000
# Powered = On
# Inverted = No
# Drive Strength = b11
# Disable State = Low
# Clock Source = b11
#Channel 1
# Output Frequency (MHz) = 27.000000000
# Multisynth Output Frequency (MHz) = 27.000000000
# Multisynth Divider = 30
# R Divider = 1
# PLL source = PLLA
# Initial phase offset (ns) = 0.000
# Error (ppm) = 0.0000
# Powered = On
# Inverted = No
# Drive Strength = b11
# Disable State = Low
# Clock Source = b11
#Channel 2
# Powered = Off
#Channel 3
# Powered = Off
#Channel 4
# Powered = Off
#Channel 5
# Powered = Off
#Channel 6
# Powered = Off
#Channel 7
# Powered = Off
#
*/

//Registers values generated with Si5351C clock builder (registers addresses are removed)
uint8_t Si5351_config_map[] = {
	0x00,
	0x4F,
	0x4F,
	0x83,
	0x83,
	0x83,
	0x83,
	0x83,
	0x83,
	0x00,
	0x00,
	0x00,
	0x05,
	0x00,
	0x0E,
	0x33,
	0x00,
	0x00,
	0x01,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x01,
	0x00,
	0x0D,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x01,
	0x00,
	0x0D,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
};
