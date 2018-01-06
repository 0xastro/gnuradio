/* -*- c++ -*- */
/*
 * Copyright 2007,2010-2012 Free Software Foundation, Inc.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <digital_ofdm_cyclic_prefixer_80211n.h>
#include <gr_io_signature.h>
#include <stdexcept>
#include <iostream>
#include <string.h>

#define VERBOSE 0

digital_ofdm_cyclic_prefixer_80211n_sptr
digital_make_ofdm_cyclic_prefixer_80211n(int fft_length, int cp_length, int symbol_length,
				     unsigned int id_rx,
				  const std::vector<std::vector<gr_complex> > &preamble)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_cyclic_prefixer_80211n(fft_length, cp_length, symbol_length, id_rx, preamble ));
}

digital_ofdm_cyclic_prefixer_80211n::digital_ofdm_cyclic_prefixer_80211n
       (int fft_length, int cp_length, int symbol_length,
	unsigned int id_rx,  const std::vector<std::vector<gr_complex> > &preamble)
  : gr_sync_interpolator("ofdm_cyclic_prefixer_80211n",
			  gr_make_io_signature (1, 1, fft_length*sizeof(gr_complex)),
			  gr_make_io_signature (1, 1, sizeof(gr_complex)),
			  symbol_length),
    d_fft_length(fft_length),
    d_cp_length(cp_length),
    d_symbol_length(symbol_length),
    d_preamble(preamble),
    d_id_rx(id_rx)
{
    d_type_preamble=0;


}


digital_ofdm_cyclic_prefixer_80211n::~digital_ofdm_cyclic_prefixer_80211n()
{
}



int
digital_ofdm_cyclic_prefixer_80211n::work(int noutput_items,
				      gr_vector_const_void_star &input_items,
				      gr_vector_void_star &output_items)
{
  const gr_complex *in_sym = (const gr_complex *) input_items[0];

  gr_complex *out_sym = (gr_complex *) output_items[0];
  
  int i = 0;
  int j;
    /* d_type_preamble = 0 --> first short preamble
     * d_type_preamble = 1 --> second short preamble
     * d_type_preamble = 2 --> first long preamble
     * d_type_preamble = 3 --> second long preamble
     */
  
    if(VERBOSE){
	  std::cout<<"ofdm_cyclic_prefixer_80211n: d_type_preamble "<<d_type_preamble<<"\n";
	}
    
    if ( d_type_preamble==0 ){ 
	
	// check if first short
	gr_complex sum = gr_complex(0.0,0.0);
	for (i = 0 ; i < d_fft_length ; i++){
	  sum.real() += abs((d_preamble[0][i].real() - in_sym[i].real()));
	  sum.imag() += abs((d_preamble[0][i].imag() - in_sym[i].imag()));
	}
 	if (VERBOSE){
	  if (d_id_rx == 0){
	    std::cout<<"ofdm_cyclic_prefixer_80211n: 0 abs(sum) "<<abs(sum)<<"\n";
	  } else {
	    std::cerr<<"ofdm_cyclic_prefixer_80211n: 1 abs(sum) "<<abs(sum)<<"\n";	    
	  }
	}
	
	if (VERBOSE){
	    if (d_id_rx == 0){
	      std::cout<<"ofdm_cyclic_prefixer_80211n: 0 d_preamble ";
	      for (int i = 0 ; i < d_fft_length ; i++){
		std::cout<<d_preamble[0][i]<<" ";
	      }
	      std::cout<<"\n";
	      std::cout<<"ofdm_cyclic_prefixer_80211n: 0 insym ";
	      for (int i = 0 ; i < d_fft_length ; i++){
		std::cout<<in_sym[i]<<" ";
	      }
	      std::cout<<"\n";
	    } else {
	      std::cerr<<"ofdm_cyclic_prefixer_80211n: 1 d_preamble ";
	      for (int i = 0 ; i < d_fft_length ; i++){
		std::cerr<<d_preamble[0][i]<<" ";
	      }
	      std::cerr<<"\n";
	      std::cerr<<"ofdm_cyclic_prefixer_80211n: 1 insym ";
	      for (int i = 0 ; i < d_fft_length ; i++){
		std::cerr<<in_sym[i]<<" ";
	      }
	      std::cerr<<"\n";
	  }
	}
	
	// is the first short
	if ( abs(sum) == 0 ){
	  d_type_preamble++;
	  if(VERBOSE){
	    if (d_id_rx == 0)std::cout<<"ofdm_cyclic_prefixer_80211n: I'm going to copy cp for the first short -> ";
	    else std::cerr<<"ofdm_cyclic_prefixer_80211n: I'm going to copy cp for the first short -> ";
	  }
	}
    

	j = d_fft_length - d_cp_length;
	for(i=0; i < d_cp_length; i++, j++) {
	    out_sym[i] = in_sym[j];
	}
	j = d_cp_length;
	for(i=0; i < d_fft_length; i++,j++) {
	    out_sym[j] = in_sym[i];
	}
	if(VERBOSE){
	 if (d_id_rx == 0)std::cout<<"ofdm_cyclic_prefixer_80211n: cp copied\n";
	 else std::cerr<<"ofdm_cyclic_prefixer_80211n: cp copied\n";
	}
	  
    } else if ( d_type_preamble==1 ){
	// second short
	j = d_fft_length - d_cp_length;
	for(i=0; i < d_cp_length; i++, j++) {
	    out_sym[i] = in_sym[j];
	}
	j = d_cp_length;
	for(i=0; i < d_fft_length; i++,j++) {
	    out_sym[j] = in_sym[i];
	}
	if(VERBOSE){
	  std::cout<<"ofdm_cyclic_prefixer_80211n: cp copied for the second short\n";
	}

	d_type_preamble++;
    } else if ( d_type_preamble == 2 ){
	// modify first long
	std::vector<gr_complex> tmp;
	for (i = 0; i < d_fft_length ; i++){
	      tmp.push_back(in_sym[i]);
	}
	// copy last cp_length*2 in the output
	j=0;
	for (i = (d_fft_length-d_cp_length*2) ; i < d_fft_length ; i++){
	      out_sym[j] = tmp[i];
	      j++;
	}
	for (i = 0 ; i < (d_fft_length-d_cp_length) ; i++){
	  out_sym[j] = tmp[i];
	  j++;
	}
	
	if (VERBOSE){
	  std::cout<<"ofdm_cyclic_prefixer_80211n: first long ";
	  for (i = 0 ; i < d_fft_length+d_cp_length ; i++){
	      std::cout<<out_sym[i]<<" ";
	  }
	  std::cout<<"\n";
	}
	
	d_type_preamble++;
    } else if ( d_type_preamble == 3 ){
	// modify second long
	std::vector<gr_complex> tmp;
	for (i = 0; i < d_fft_length ; i++){
	      tmp.push_back(in_sym[i]);
	}
	// copy last cp_length in the output
	j=0;
	for (i = (d_fft_length-d_cp_length) ; i < d_fft_length ; i++){
	      out_sym[j] = tmp[i];
	      j++;
	}
	for (i = 0 ; i < d_fft_length ; i++){
	  out_sym[j] = tmp[i];
	  j++;
	}
	if (VERBOSE){
	  std::cout<<"ofdm_cyclic_prefixer_80211n: second long ";
	  for (i = 0 ; i < d_fft_length +d_cp_length; i++){
	      std::cout<<out_sym[i]<<" ";
	  }
	  std::cout<<"\n";
	}
	
	d_type_preamble=0;
    }
	
      
    return d_symbol_length;

}
