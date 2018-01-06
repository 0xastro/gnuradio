/* -*- c++ -*- */
/*
 * Copyright 2007,2008,2010,2011 Free Software Foundation, Inc.
 * 
 * This file is part of GNU Radio
 * 
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <digital_ofdm_sampler_snroncp.h>
#include <gr_io_signature.h>
#include <gr_expj.h>
#include <cstdio>
#include <iostream>

#define VERBOSE 0

digital_ofdm_sampler_snroncp_sptr
digital_make_ofdm_sampler_snroncp (unsigned int fft_length, 
			   unsigned int symbol_length,
			   unsigned int downsampling,
			   unsigned int timeout)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_sampler_snroncp (fft_length, symbol_length, downsampling, timeout));
}

digital_ofdm_sampler_snroncp::digital_ofdm_sampler_snroncp (unsigned int fft_length, 
					    unsigned int symbol_length,
					    unsigned int downsampling,
					    unsigned int timeout)
  : gr_block ("ofdm_sampler_snroncp",
	      gr_make_io_signature2 (2, 2, sizeof (gr_complex), sizeof(char)),
	      gr_make_io_signature3 (3, 3, sizeof (gr_complex)*fft_length, sizeof(char)*fft_length, sizeof(float))),
    d_state(STATE_NO_SIG), d_timeout_max(timeout), d_fft_length(fft_length), d_symbol_length(symbol_length)
{
  d_downsampling = downsampling;
  std::cout<<"d_downsampling " << d_downsampling << "\n";
  set_relative_rate((double) d_downsampling/((double) (fft_length)));   // buffer allocator hint
  
}

void
digital_ofdm_sampler_snroncp::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  // FIXME do we need more
  //int nreqd  = (noutput_items-1) * d_symbol_length + d_fft_length;
  int nreqd  = (d_symbol_length + d_fft_length)*d_downsampling;
  unsigned ninputs = ninput_items_required.size ();
  for (unsigned i = 0; i < ninputs; i++)
    ninput_items_required[i] = nreqd;
}


int
digital_ofdm_sampler_snroncp::general_work (int noutput_items,
				    gr_vector_int &ninput_items,
				    gr_vector_const_void_star &input_items,
				    gr_vector_void_star &output_items)
{
  const gr_complex *iptr = (const gr_complex *) input_items[0];
  const char *trigger = (const char *) input_items[1];

  gr_complex *optr = (gr_complex *) output_items[0];
  char *outsig = (char *) output_items[1];
  float *outsnr = (float *) output_items[2];

  //FIXME: we only process a single OFDM symbol at a time; after the preamble, we can 
  // process a few at a time as long as we always look out for the next preamble.

  unsigned int index=(d_fft_length*d_downsampling);  // start one fft length into the input so we can always look back this far

  outsig[0] = 0; // set output to no signal by default

  // Search for a preamble trigger signal during the next symbol length
  while((d_state != STATE_PREAMBLE) && (index <= (d_symbol_length+d_fft_length)*d_downsampling )) {
    if(trigger[index]) {
      outsig[0] = 1; // tell the next block there is a preamble coming
      d_state = STATE_PREAMBLE;
    }
    else {
      index++;
    }
  }
  
  unsigned int i, pos, ret;
  int pos2 = d_fft_length*d_downsampling;
  unsigned int cp_length = d_symbol_length-d_fft_length;
  float num=0.0;
  float den=0.0;
  std::vector<gr_complex> OFDMsymbol;
  
  if (VERBOSE){
    std::cout<< "d_symbol_length " << d_symbol_length << "\n";
    std::cout<< "d_fft_length " << d_fft_length << "\n";
    std::cout<< "cp_length " << cp_length << "\n";
  }
  
  switch(d_state) {
  case(STATE_PREAMBLE):
    //std::cerr << "STATE PREAMBLE\n";
    // When we found a preamble trigger, get it and set the symbol boundary here
    for(i = (index - (d_fft_length)*d_downsampling +d_downsampling); i <= index; i+=d_downsampling) {
      *optr++ = iptr[i];
    }
    
    d_timeout = d_timeout_max; // tell the system to expect at least this many symbols for a frame
    d_state = STATE_FRAME;
    consume_each(index - (d_fft_length)*d_downsampling+d_downsampling); // consume up to one fft_length away to keep the history
    ret = 1;
    break;
    
  case(STATE_FRAME):
    // use this state when we have processed a preamble and are getting the rest of the frames
    //FIXME: we could also have a power squelch system here to enter STATE_NO_SIG if no power is received

    // skip over fft length history and cyclic prefix
    pos = d_symbol_length*d_downsampling;         // keeps track of where we are in the input buffer
    while(pos < (d_symbol_length + d_fft_length)*d_downsampling ) {
      float real = 0.0;
      float imag = 0.0;
      for (unsigned int d=0; d<d_downsampling;d++){
	real += iptr[pos+d].real();
	imag += iptr[pos+d].imag();
      }
      real/=float(d_downsampling);
      imag/=float(d_downsampling);
      gr_complex tmp = gr_complex(real,imag);
      *optr++ = iptr[pos];
      pos+=d_downsampling;
    }
    
    
    // compute SNR
    pos2 = d_fft_length*d_downsampling;
    cp_length = d_symbol_length-d_fft_length;
    num=0.0;
    den=0.0;
    OFDMsymbol.resize(d_symbol_length);
    for(i=0;i<d_symbol_length;i++){
      pos2+=d_downsampling;
      OFDMsymbol[i] = iptr[pos2];
    }
    outsnr[0] = 1;
    
    if(d_timeout-- == 0) {
      printf("TIMEOUT\n");
      d_state = STATE_NO_SIG;
    }

    consume_each(d_symbol_length*d_downsampling); // jump up by 1 fft length and the cyclic prefix length
    ret = 1;
    break;

  case(STATE_NO_SIG):
  default:
    consume_each(index-(d_fft_length*d_downsampling)); // consume everything we've gone through so far leaving the fft length history
    ret = 0;
    break;
  }

  return ret;
}
