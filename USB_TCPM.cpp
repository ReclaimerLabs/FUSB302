/*
  USB_TCPM.cpp - Library for interacting with the FUSB302B chip.
  Copyright 2010 The Chromium OS Authors
  Copyright 2017 Jason Cerundolo
  Released under an MIT license. See LICENSE file. 
*/

#include "USB_TCPM.h"

int USB_TCPM::get_num_bytes(uint16_t header)
{
    int rv;

    /* Grab the Number of Data Objects field.*/
    rv = PD_HEADER_CNT(header);

    /* Multiply by four to go from 32-bit words -> bytes */
    rv *= 4;

    /* Plus 2 for header */
    rv += 2;

    return rv;
}
