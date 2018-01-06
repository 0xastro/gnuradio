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

#include <digital_ofdm_insert_pilot_80211n.h>
#include <gr_io_signature.h>
#include <stdexcept>
#include <iostream>
#include <string.h>

#define VERBOSE 0

digital_ofdm_insert_pilot_80211n_sptr
digital_make_ofdm_insert_pilot_80211n(int fft_length, int zeros_on_left, int zeros_on_right,
				  const std::vector<gr_complex> &pilot,int first_pilot,int space_pilot,int ntx_antennas)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_insert_pilot_80211n(fft_length, zeros_on_left, zeros_on_right,
								     pilot,first_pilot,space_pilot,
								     ntx_antennas ));
}

digital_ofdm_insert_pilot_80211n::digital_ofdm_insert_pilot_80211n
       (int fft_length, int zeros_on_left, int zeros_on_right,
	const std::vector<gr_complex> &pilot,int first_pilot,int space_pilot, int ntx_antennas)
  : gr_block("ofdm_insert_pilot_80211n",
	     gr_make_io_signature2(2, 2,
				   sizeof(gr_complex)*fft_length,
				   sizeof(char)),
	     gr_make_io_signature2(1, 2,
				   sizeof(gr_complex)*fft_length,
				   sizeof(char))),
    d_fft_length(fft_length),
    d_zeros_on_left(zeros_on_left),
    d_zeros_on_right(zeros_on_right),
    d_pilot(pilot),
    d_first_pilot(first_pilot),
    d_space_pilot(space_pilot),
    d_ntx_antennas(ntx_antennas)
{
    d_type_preamble=0;

}


digital_ofdm_insert_pilot_80211n::~digital_ofdm_insert_pilot_80211n()
{
}



int
digital_ofdm_insert_pilot_80211n::general_work(int noutput_items,
					   gr_vector_int &ninput_items_v,
					   gr_vector_const_void_star &input_items,
					   gr_vector_void_star &output_items)
{
  const gr_complex *in_sym = (const gr_complex *) input_items[0];
  const unsigned char *in_flag = (const unsigned char *) input_items[1];

  gr_complex *out_sym = (gr_complex *) output_items[0];
  const unsigned char *out_flag = 0;
  

  //int pilot_length = d_pilot.size();
  int i = 0, j = 0, id_pilot=0, id_output=0, id_tmp;
  
  std::vector<gr_complex> tmp, tmp_out;
  tmp_out.resize(d_fft_length);
 
      /* d_type_preamble = 0 --> first short preamble
     * d_type_preamble = 1 --> second short preamble
     * d_type_preamble = 2 --> first long preamble
     * d_type_preamble = 3 --> second long preamble
     * d_type_preamble = 4 --> first sig preamble
     * d_type_preamble = 5 --> second sig preamble
     * d_type_preamble = 6 --> third long preamble if nrx_antennas = 2
     */
  

      if ( (in_flag[0] && d_type_preamble==0) || d_type_preamble==1 || d_type_preamble==2 || d_type_preamble==3 || 
	d_type_preamble==6){
	  if (d_type_preamble==6) d_type_preamble=0;
	  else d_type_preamble++;
    
	  for (i = 0 ; i < d_fft_length ; i++){
	      out_sym[i] = in_sym[i];
	  }
	  
      } else if ( (!in_flag[0] && d_type_preamble==0) || d_type_preamble==4 || d_type_preamble==5 ){
	  if ( d_type_preamble==4 ) d_type_preamble++;
	  else if (d_type_preamble==5 && d_ntx_antennas == 2) d_type_preamble=6;
	  else if (d_type_preamble==5 && d_ntx_antennas == 1) d_type_preamble=0;
	  
	  id_output = 0;
	  id_pilot = 0;
	  id_tmp = 0;
    
	  for (i = (d_zeros_on_left+(d_pilot.size()/2) ) ; i < (d_fft_length - d_zeros_on_right - (d_pilot.size()/2)) ; i++){
	      if (VERBOSE) {
		std::cout << "digital_ofdm_insert_pilot_80211n: i= "<< i <<"\n";
	      }
	      tmp.push_back(in_sym[i]);
	  }
    
	  for (i = 0 ; i < d_fft_length ; i++){
	    if ( i < d_zeros_on_left || i > (d_fft_length - d_zeros_on_right) ){
	      tmp_out[i] = in_sym[i];
	    }
	  }
	  id_pilot = d_zeros_on_left + d_first_pilot;
	  for (i = 0 ; i < d_pilot.size() ; i++){
	    if (VERBOSE) {
	      std::cout<< "digital_ofdm_insert_pilot_80211n: id_pilot= " << id_pilot << "\n";
	    }
	    tmp_out[id_pilot] = d_pilot[i]; 
	    id_pilot += d_space_pilot;
      
	  }
	  id_output = d_zeros_on_left;
	  for (i = 0 ; i < tmp.size() ; i++){
	    if ( (id_output-(d_zeros_on_left+d_first_pilot)) % d_space_pilot == 0)  {
	      if (VERBOSE) {
		std::cout<< "digital_ofdm_insert_pilot_80211n: pilot= " << id_output-(d_zeros_on_left+d_first_pilot) <<  " d_space_pilot " << d_space_pilot << "\n";
	      }
	      id_output++;
	      i--;
	    } else {
	      tmp_out[id_output] = tmp[i];
	      id_output++;
	    }
	  }
  
	  for (i = 0 ; i < d_fft_length ; i++){
	    out_sym[i] = tmp_out[i];
	  }
      } 
  
      if (VERBOSE){
	  for (i = 0 ; i < d_fft_length ; i++){
	    std::cout <<"digital_ofdm_insert_pilot_80211n: "<< i << " " << out_sym[i] << "\n";
	  }
      }
  
      out_flag = in_flag;
      if (VERBOSE){
	if (in_flag[0]){
	  std::cout<< "digital_ofdm_insert_pilot_80211n: in_flag[0]=1 d_type_preamble= " << d_type_preamble <<"\n";
	} else {
	  std::cout<< "digital_ofdm_insert_pilot_80211n: in_flag[0]=0 d_type_preamble= " << d_type_preamble <<"\n";
	}
      }
      
      consume_each(1);
      return 1;

}
