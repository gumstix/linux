/* OV7692: linux kernel driver for the OmniVision OV7692 Image Sensor 
*
*  Copyright (C) 2016, Adam YH Lee <adam@gumstix.com> 
*  
*  This is a derived work from the following:
*  
*  		-  OV9650/OV9652 by Sylwester Nawrocki
*		-  MT9V032 by Laurent Pinchart
*
*  This is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 2 of the License, or
*  (at your option) any later version.
*  
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*  
*  You should have received a copy of the GNU General Public License
*  along with OV7692. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _MEDIA_OV7692_H
#define _MEDIA_OV7692_H

struct ov7692_platform_data {
	unsigned int clk_pol:1;
	const s64 *link_freqs;
	s64 link_def_freq;
};

#endif
