/* -*- c++ -*- */
/*
 * Copyright 2006-2008,2010,2011 Free Software Foundation, Inc.
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

#include <digital_ofdm_frame_acquisition_80211n.h>
#include <gr_io_signature.h>
#include <gr_expj.h>
#include <gr_math.h>
#include <cstdio>
#include <iostream>
#include <iomanip>

#define VERBOSE 0
#define M_TWOPI (2*M_PI)
#define MAX_NUM_SYMBOLS 1000
#define REQ_PKTS 100
#define EMPTYCARRIERS 0
#define HDSIZE 4
#define CRCSIZE 4
#define TRAILERSIZE 0

int count_ofdm = 0;

digital_ofdm_frame_acquisition_80211n_sptr
digital_make_ofdm_frame_acquisition_80211n (int ntx_antennas,
					    int nrx_antennas,
					    unsigned int occupied_carriers,
					    unsigned int fft_length, 
					    unsigned int cplen,
					    unsigned int pilot_first, unsigned int pilot_spacing,
					    float rate,
					    unsigned int mod_rate,
					    unsigned int pkt_size,
					    unsigned int headeronoff,
					    const std::vector<gr_complex> &known_symbol,
					    const std::vector<gr_complex> &known_pilot,
					    unsigned int max_fft_shift_len)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_frame_acquisition_80211n (
									ntx_antennas, nrx_antennas,
									occupied_carriers, fft_length, cplen, 
									pilot_first, pilot_spacing,
									rate, mod_rate, pkt_size, headeronoff,
									known_symbol, known_pilot, max_fft_shift_len));
}

digital_ofdm_frame_acquisition_80211n::digital_ofdm_frame_acquisition_80211n (
								int ntx_antennas, int nrx_antennas,
								unsigned occupied_carriers,
								unsigned int fft_length, 
								unsigned int cplen,
								unsigned int pilot_first, unsigned int pilot_spacing,
								float rate,
								unsigned int mod_rate,
								unsigned int pkt_size,
								unsigned int headeronoff,
								const std::vector<gr_complex> &known_symbol,
								const std::vector<gr_complex> &known_pilot,
								unsigned int max_fft_shift_len)
  : gr_block ("ofdm_frame_acquisition_80211n",
	      gr_make_io_signature2 (2, 2, sizeof(gr_complex)*fft_length, sizeof(char)*fft_length),
	      gr_make_io_signature2 (2, 2, sizeof(gr_complex)*occupied_carriers, sizeof(char))),
    d_occupied_carriers(occupied_carriers),
    d_fft_length(fft_length),
    d_cplen(cplen),
    d_pilot_first(pilot_first),
    d_pilot_spacing(pilot_spacing),
    d_freq_shift_len(max_fft_shift_len),
    d_known_symbol(known_symbol),
    d_known_pilot(known_pilot),
    d_angle_pilot(0.0),
    d_coarse_freq(0),
    d_phase_count(0),
    d_rate(rate),
    d_mod_rate(mod_rate),
    d_pkt_size(pkt_size),
    d_ntx_antennas(ntx_antennas),
    d_nrx_antennas(nrx_antennas)
{

  unsigned int i, j;
  
  d_type_preamble=0;
  
  d_pilot_lenght = d_known_pilot.size();
  
  d_symbol_phase_diff.resize(d_fft_length);
  d_known_phase_diff.resize(d_occupied_carriers+d_pilot_lenght);
  d_hestimate.resize(d_occupied_carriers+d_pilot_lenght);
  
  int unoccupied_carriers = d_fft_length - (d_occupied_carriers + d_pilot_lenght);
  int zeros_on_left = (int)ceil(unoccupied_carriers/2.0);
  
  if (headeronoff == 1){
      d_snr_num_symbols = ceil( (( (d_rate * (d_pkt_size+CRCSIZE) ) + HDSIZE + TRAILERSIZE )*8 ) / float(d_mod_rate * (d_occupied_carriers-EMPTYCARRIERS)) );
  } else{
      d_snr_num_symbols = ceil( (( (d_rate * (d_pkt_size+CRCSIZE) ) + TRAILERSIZE )*8 ) / float(d_mod_rate * (d_occupied_carriers-EMPTYCARRIERS)) );
  }
  d_max_snr_total = floor(REQ_PKTS/d_snr_num_symbols) * d_snr_num_symbols;
  
  if (VERBOSE){
    std::cout <<"d_pkt_size " << d_pkt_size << "\n";
    std::cout <<"d_rate " << d_rate << "\n";
    std::cout <<"d_mod_rate " << d_mod_rate << "\n";
    std::cout <<"d_snr_num_symbols " <<  d_snr_num_symbols << "\n";
    std::cout <<"d_max_snr_total " << d_max_snr_total << "\n";
    std::cout <<"d_pilot_first " << d_pilot_first << "\n";
    std::cout <<"d_pilot_spacing " << d_pilot_spacing << "\n";
  }
  
  i = 0; j = 0;
  
  if (zeros_on_left % 2 == 0){ // even
    std::fill(d_known_phase_diff.begin(), d_known_phase_diff.end(), 0);
    for(i = 0; i < d_known_symbol.size()-2; i+=2) {
      d_known_phase_diff[i] = norm(d_known_symbol[i] - d_known_symbol[i+2]);
    }
  } else { // odd
    std::fill(d_known_phase_diff.begin(), d_known_phase_diff.end(), 0);
    d_known_phase_diff[0]=0;
    for(i = 1; i < d_known_symbol.size()-1; i+=2) {
      d_known_phase_diff[i] = norm(d_known_symbol[i] - d_known_symbol[i+2]);
    }
  }
  
  d_phase_lut = new gr_complex[(2*d_freq_shift_len+1) * MAX_NUM_SYMBOLS];
  for(i = 0; i <= 2*d_freq_shift_len; i++) {
    for(j = 0; j < MAX_NUM_SYMBOLS; j++) {
      d_phase_lut[j + i*MAX_NUM_SYMBOLS] =  gr_expj(-M_TWOPI*d_cplen/d_fft_length*(i-d_freq_shift_len)*j);
    }
  }
}

digital_ofdm_frame_acquisition_80211n::~digital_ofdm_frame_acquisition_80211n(void)
{
  delete [] d_phase_lut;
}

void
digital_ofdm_frame_acquisition_80211n::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  unsigned ninputs = ninput_items_required.size ();
  for (unsigned i = 0; i < ninputs; i++)
    ninput_items_required[i] = 1;
}

gr_complex
digital_ofdm_frame_acquisition_80211n::coarse_freq_comp(int freq_delta, int symbol_count)
{

  return gr_expj(-M_TWOPI*freq_delta*d_cplen/d_fft_length*symbol_count);

}

void
digital_ofdm_frame_acquisition_80211n::correlate(const gr_complex *symbol, int zeros_on_left)
{
  unsigned int i,j;

  
  std::fill(d_symbol_phase_diff.begin(), d_symbol_phase_diff.end(), 0);
  if (zeros_on_left % 2 == 0){ // even
    for(i = 0; i < d_fft_length-2; i++) {
	d_symbol_phase_diff[i] = norm(symbol[i] - symbol[i+2]);
    }
  } else { //odd
    for(i = 1; i < d_fft_length-1; i++) {
	d_symbol_phase_diff[i] = norm(symbol[i] - symbol[i+2]);
    }
  }

  // sweep through all possible/allowed frequency offsets and select the best
  int index = 0;
  float max = 0, sum=0;
  for(i =  zeros_on_left - d_freq_shift_len; i < zeros_on_left + d_freq_shift_len; i++) {
    sum = 0;
    for(j = 0; j < (d_occupied_carriers+d_pilot_lenght); j++) {
      sum += (d_known_phase_diff[j] * d_symbol_phase_diff[i+j]);
    }
    if(sum > max) {
      max = sum;
      index = i;
    }
  }
  
  // set the coarse frequency offset relative to the edge of the occupied tones
  d_coarse_freq = index - zeros_on_left;
  
  if(1) 
    fprintf(stdout, "d_coarse_freq= %d\n",d_coarse_freq);
}

void
digital_ofdm_frame_acquisition_80211n::correlate2(const gr_complex *symbol, int zeros_on_left)
{
  unsigned int i,j;
  
  std::vector<gr_complex> Coarse;
  Coarse.resize(d_freq_shift_len*2);
  std::fill(Coarse.begin(), Coarse.end(), gr_complex(0,0));

  unsigned int coarse_id=0;
  for(i =  zeros_on_left - d_freq_shift_len; i < zeros_on_left + d_freq_shift_len; i++) {
    for(j = 0; j < d_fft_length; j++) {
      Coarse[coarse_id] += d_known_symbol[j] * symbol[i];
    }
    coarse_id++;
  }
  
  float max = 0; 
  unsigned int id_max=0;
  for(j = 0; j < d_freq_shift_len*2; j++) {
    if (abs(Coarse[j]) > max ){
      max = abs(Coarse[j]);
      id_max=j;
    }
  }
   
  
  // set the coarse frequency offset relative to the edge of the occupied tones
  d_coarse_freq = id_max - d_freq_shift_len;
  d_coarse_freq = 0;
}


void
digital_ofdm_frame_acquisition_80211n::correlate_pilot(const gr_complex *symbol, int zeros_on_left)
{
  unsigned int i,j;
  
  std::vector<gr_complex> Coarse;
  Coarse.resize(d_freq_shift_len*2);
  std::fill(Coarse.begin(), Coarse.end(), gr_complex(0,0));

  unsigned int pilot_id; 
  unsigned int coarse_id=0;
  for(i =  zeros_on_left - d_freq_shift_len; i < zeros_on_left + d_freq_shift_len; i++) {
    pilot_id = d_pilot_first;
    for(j = 0; j < d_pilot_lenght; j++) {
      Coarse[coarse_id] += d_known_pilot[j] * symbol[i+pilot_id];
      pilot_id += d_pilot_spacing;
    }
    coarse_id++;
  }
  
  float max = 0; 
  unsigned int id_max=0;
  for(j = 0; j < d_freq_shift_len*2; j++) {
    if (abs(Coarse[j]) > max ){
      max = abs(Coarse[j]);
      id_max=j;
    }
  }
   
  
  // set the coarse frequency offset relative to the edge of the occupied tones
  d_coarse_freq = id_max - d_freq_shift_len;
  if(VERBOSE) 
    fprintf(stdout, "pilot d_coarse_freq= %d\n",d_coarse_freq);
}


void
digital_ofdm_frame_acquisition_80211n::calculate_equalizer(const gr_complex *symbol, int zeros_on_left)
{
  unsigned int 	i=0, 
		tocorrect; 
 
  d_hestimate.clear();
  d_hestimate.resize(d_occupied_carriers+d_pilot_lenght);
  
  tocorrect = (d_occupied_carriers+d_pilot_lenght)/2;
  if ((zeros_on_left %2 ) == 0){
    for(i = 0; i < (d_occupied_carriers + d_pilot_lenght); i++) {
      d_hestimate[i] = d_known_symbol[i+zeros_on_left] / (coarse_freq_comp(d_coarse_freq,1)*(symbol[i+zeros_on_left+d_coarse_freq])); 
    }
    d_hestimate[tocorrect] = d_hestimate[tocorrect-1];
    
  } else {
     d_hestimate[1] = d_known_symbol[1] / 
     (coarse_freq_comp(d_coarse_freq,1)*symbol[zeros_on_left+d_coarse_freq+1]);
    d_hestimate[0]=d_hestimate[1];
    for(i = 3; i < (d_occupied_carriers + d_pilot_lenght); i+=2) {
       d_hestimate[i] = d_known_symbol[i] / 
       (coarse_freq_comp(d_coarse_freq,1)*(symbol[i+zeros_on_left+d_coarse_freq]));
      d_hestimate[i-1] = (d_hestimate[i] + d_hestimate[i-2]) / gr_complex(2.0, 0.0);    
    }

    // with even number of carriers; last equalizer tap is wrong
    if(!((d_occupied_carriers+d_pilot_lenght) & 1)) {
      d_hestimate[(d_occupied_carriers+d_pilot_lenght)-1] = d_hestimate[(d_occupied_carriers+d_pilot_lenght)-2];
    }
  
    tocorrect = (d_fft_length/2)-(EMPTYCARRIERS-1); 
    d_hestimate[tocorrect] = d_hestimate[tocorrect-1];
  }
  if(VERBOSE) {
    fprintf(stdout, "Equalizer setting:\n");
    for(i = 0; i < (d_occupied_carriers+d_pilot_lenght); i++) {
      gr_complex sym = coarse_freq_comp(d_coarse_freq,1)*symbol[i+zeros_on_left+d_coarse_freq];
      gr_complex output = sym * d_hestimate[i];
      fprintf(stdout, "sym: %+.4f + j%+.4f  ks: %+.4f + j%+.4f  eq: %+.4f + j%+.4f  ==>  %+.4f + j%+.4f\n", 
	      sym .real(), sym.imag(),
	      d_known_symbol[i].real(), d_known_symbol[i].imag(),
	      d_hestimate[i].real(), d_hestimate[i].imag(),
	      output.real(), output.imag());
    }
    fprintf(stdout, "\n");
  }

}


// compute H on pilots
void 
digital_ofdm_frame_acquisition_80211n::calculate_eq_pilots(const gr_complex *symbol, int zeros_on_left){
  
  unsigned int id_pilot= d_pilot_first;
  unsigned int i, j;
  
  std::vector<gr_complex> hest_test;
  hest_test.resize(d_pilot_lenght);
  d_hestPilot.clear();
  d_hestPilot.resize(d_occupied_carriers+d_pilot_lenght);
  
  // compute h for pilots
  for(i = 0; i < d_pilot_lenght; i++) {
      hest_test[i] = d_known_pilot[i] * (symbol[id_pilot+zeros_on_left+d_coarse_freq]);
      id_pilot = id_pilot + d_pilot_spacing;
  }
  
  gr_complex hest_empty = gr_complex(0.0, 0.0);

  if (VERBOSE){
    for(j = 0; j < d_pilot_lenght; j++) {
      std::cout << "channelEstimation["<<j<<"]= "<< hest_test[j] << "\n";
    }
    std::cout << "-------------------------------------------------------\n";
  }
  

  // copy h for values before first pilot
  for (j = 0; j < d_pilot_first; j++){
      d_hestPilot[j] = hest_test[0];
  }

  // copy h for values after pilot
  for(j = 0; j < d_pilot_lenght-1; j++) {
     for(i = (d_pilot_spacing*j)+d_pilot_first; i < (d_pilot_spacing*j)+d_pilot_first+d_pilot_spacing; i++) {
        d_hestPilot[i] = hest_test[j];
     }
  }
  
  // copy h for values after last pilot
  for (j = i; j < d_occupied_carriers+d_pilot_lenght; j++){
    d_hestPilot[j] = hest_test[d_pilot_lenght-1]; 
  }
  
  if (VERBOSE){
    for(j = 0; j < d_occupied_carriers+d_pilot_lenght; j++) {
      std::cout << "d_hestPilot["<<j<<"]= "<< d_hestPilot[j] << "\n";
    }
  }  
}


// used when H is computed on preamble
void
digital_ofdm_frame_acquisition_80211n::calculate_angle(const gr_complex *symbol, int zeros_on_left)
{
  int i=0, id_pilot=0;
  std::vector<gr_complex> d_hest_pilot;
  d_hest_pilot.resize(d_pilot_lenght);
  
  id_pilot= d_pilot_first;
  for(i = 0; i < d_pilot_lenght; i++) {
    d_hest_pilot[i] = d_known_pilot[i] / 
      (coarse_freq_comp(d_coarse_freq,1)*(symbol[id_pilot+zeros_on_left+d_coarse_freq]));
      id_pilot = id_pilot + d_pilot_spacing;
  }

  // calculate angle from pilot
  gr_complex tmp = gr_complex(0.0, 0.0);
  id_pilot= d_pilot_first;
  for(i = 0; i < d_pilot_lenght; i++) {
    tmp += conj(d_known_pilot[i]) * coarse_freq_comp(d_coarse_freq,1) * 
	  d_hest_pilot[i] * symbol[id_pilot+zeros_on_left+d_coarse_freq];
    id_pilot = id_pilot + d_pilot_spacing;
  }
  d_angle_pilot = arg(tmp);
}

// used when H is computed on preamble
void
digital_ofdm_frame_acquisition_80211n::correct_phase(const gr_complex *symbol, int zeros_on_left)
{
  int i=0, id_carrier=0, tocorrect;
  d_phase_carriers.clear();
  d_phase_carriers.resize(d_occupied_carriers+d_pilot_lenght);
  
  d_corrected_symbol.clear();
  d_corrected_symbol.resize(d_occupied_carriers+d_pilot_lenght);
  
  
  if(zeros_on_left%2==0){//even
    d_phase_carriers[0] = d_known_symbol[0] /
	(coarse_freq_comp(d_coarse_freq,1)*symbol[zeros_on_left+d_coarse_freq]);
    for(i = 2; i < (d_occupied_carriers + d_pilot_lenght); i+=2) {
	d_phase_carriers[i] = d_known_symbol[i] / 
	  (coarse_freq_comp(d_coarse_freq,1)*(symbol[i+zeros_on_left+d_coarse_freq]));
	//instead of interpolating, empty carrers get the previous value
	d_phase_carriers[i-1] = d_phase_carriers[i-2];    
      }

      // with even number of carriers; last equalizer tap is wrong
      if(!((d_occupied_carriers+d_pilot_lenght) & 1)) {
	  d_phase_carriers[(d_occupied_carriers+d_pilot_lenght)-1] = d_phase_carriers[(d_occupied_carriers+d_pilot_lenght)-2];
      }
  
      tocorrect = (d_fft_length/2)-(EMPTYCARRIERS-1); 
      d_phase_carriers[tocorrect] = d_phase_carriers[tocorrect-1];
  }

  else{
   d_phase_carriers[1] = d_known_symbol[1] / 
      (coarse_freq_comp(d_coarse_freq,1)*(symbol[1+zeros_on_left+d_coarse_freq]));
   d_phase_carriers[0] = d_phase_carriers[1] ;
    
   for(i = 3; i < d_occupied_carriers+d_pilot_lenght; i+=2) {
    d_phase_carriers[i] = d_known_symbol[i] / 
      (coarse_freq_comp(d_coarse_freq,1)*(symbol[i+zeros_on_left+d_coarse_freq]));
    d_phase_carriers[i-1] = (d_phase_carriers[i] + d_phase_carriers[i-2]) / gr_complex(2.0, 0.0);     
   }

      tocorrect = (d_fft_length/2)-(EMPTYCARRIERS-1); 
      d_phase_carriers[tocorrect] = d_phase_carriers[tocorrect-1];
  }
  // calculate angle from preamble
  gr_complex menoJ = gr_complex(0.0, -1.0);
  for(i = 0; i < d_occupied_carriers+d_pilot_lenght; i++) {
    d_corrected_symbol[i] = symbol[i+zeros_on_left+d_coarse_freq] * coarse_freq_comp(d_coarse_freq,1) * exp(- menoJ * arg(d_phase_carriers[i]));
  }
}



int
digital_ofdm_frame_acquisition_80211n::general_work(int noutput_items,
					     gr_vector_int &ninput_items,
					     gr_vector_const_void_star &input_items,
					     gr_vector_void_star &output_items)
{
  const gr_complex *symbol = (const gr_complex *)input_items[0];
  const char *signal_in = (const char *)input_items[1];

  gr_complex *out = (gr_complex *) output_items[0];
  char *signal_out = (char *) output_items[1];
  
  int unoccupied_carriers = d_fft_length - (d_occupied_carriers + d_pilot_lenght);
  int zeros_on_left = (int)ceil(unoccupied_carriers/2.0);
  unsigned int i, j, id_pilot;
  
  if (VERBOSE){
    std::cout <<"ofdm_frame_acquisition_80211n: d_pkt_size " << d_pkt_size << "\n";
    std::cout <<"ofdm_frame_acquisition_80211n: d_rate " << d_rate << "\n";
    std::cout <<"ofdm_frame_acquisition_80211n: d_mod_rate " << d_mod_rate << "\n";
    std::cout <<"ofdm_frame_acquisition_80211n: d_snr_num_symbols " <<  d_snr_num_symbols << "\n";
    std::cout <<"ofdm_frame_acquisition_80211n: d_max_snr_total " << d_max_snr_total << "\n";
    std::cout <<"ofdm_frame_acquisition_80211n: d_pilot_first " << d_pilot_first << "\n";
    std::cout <<"ofdm_frame_acquisition_80211n: d_pilot_spacing " << d_pilot_spacing << "\n";
  }
  
  // variables used to remove pilots
  std::vector<gr_complex> tmp_all;
  tmp_all.resize(d_occupied_carriers+d_pilot_lenght);
  gr_complex tmp = gr_complex(0.0, -1.0);
  
  /* d_type_preamble = 0 --> first short preamble
     * d_type_preamble = 1 --> second short preamble
     * d_type_preamble = 2 --> first long preamble
     * d_type_preamble = 3 --> second long preamble
     * d_type_preamble = 4 --> first sig preamble
     * d_type_preamble = 5 --> second sig preamble
     * d_type_preamble = 6 --> fake long preamble
  */
  

  if (signal_in[0] && d_type_preamble == 0){
    count_ofdm = 1;
    if (VERBOSE){
	std::cout<< "ofdm_frame_acquisition_80211n: signal_in[0] 1 d_type_preamble= " <<d_type_preamble<<" count_ofdm " <<count_ofdm <<"\n";
    }
    d_type_preamble++;
    d_type_preamble++;
    signal_out[0] = 1;
    
    
    if (VERBOSE){
	std::cout<< "ofdm_frame_acquisition_80211n: d_coarse_freq= "<<d_coarse_freq<<"\n";
    }
    d_angle_pilot = 0.0;
    
    for(i = 0; i < d_occupied_carriers+d_pilot_lenght; i++) {
      tmp_all[i] = symbol[i+zeros_on_left+d_coarse_freq];  
    }

  } else if (d_type_preamble == 1){
    count_ofdm++;
    if (VERBOSE) {
      if (signal_in[0])
	std::cout<< "ofdm_frame_acquisition_80211n: signal_in[0] = 1 d_type_preamble= " <<d_type_preamble<<" count_ofdm " <<count_ofdm <<"\n";
      else 
	std::cout<< "ofdm_frame_acquisition_80211n: signal_in[0] = 0 d_type_preamble= " <<d_type_preamble<<" count_ofdm " <<count_ofdm <<"\n";
    }
    d_type_preamble++;
    signal_out[0] = 1;
    
    for(i = 0; i < d_occupied_carriers+d_pilot_lenght; i++) {
      tmp_all[i] = symbol[i+zeros_on_left+d_coarse_freq];  
    }
    
  } else if (d_type_preamble == 2){
    count_ofdm++;
    if (VERBOSE) {
      if (signal_in[0])
	std::cout<< "ofdm_frame_acquisition_80211n: signal_in[0] = 1 d_type_preamble= " <<d_type_preamble<<" count_ofdm " <<count_ofdm <<"\n";
      else 
	std::cout<< "ofdm_frame_acquisition_80211n: signal_in[0] = 0 d_type_preamble= " <<d_type_preamble<<" count_ofdm " <<count_ofdm <<"\n";
    }
    d_type_preamble++;
    signal_out[0] = 1;
    // channel estimation based on long preamble    
    for(i = 0; i < d_occupied_carriers+d_pilot_lenght; i++) {
      tmp_all[i] = symbol[i+zeros_on_left+d_coarse_freq];    
    }
    
  } else if (d_type_preamble == 3){
    count_ofdm++;
    if (VERBOSE) {
      if (signal_in[0])
	std::cout<< "ofdm_frame_acquisition_80211n: signal_in[0] = 1 d_type_preamble= " <<d_type_preamble<<" count_ofdm " <<count_ofdm <<"\n";
      else 
	std::cout<< "ofdm_frame_acquisition_80211n: signal_in[0] = 0 d_type_preamble= " <<d_type_preamble<<" count_ofdm " <<count_ofdm <<"\n";
    }
    d_type_preamble++;
    signal_out[0] = 1;
    correlate2(symbol, zeros_on_left); // compute coarse
    d_phase_count = 1;
    calculate_equalizer(symbol, zeros_on_left);
    for(i = 0; i < d_occupied_carriers+d_pilot_lenght; i++) {    
     tmp_all[i] = symbol[i+zeros_on_left+d_coarse_freq] * d_hestimate[i] 
       		* coarse_freq_comp(d_coarse_freq,d_phase_count) ;
    
       if (VERBOSE){
	 std::cout << "long2: d_hestimate["<<i<<"] = "<< d_hestimate[i] 
	           << " symbol["<<i+zeros_on_left+d_coarse_freq<<"] = "<<symbol[i+zeros_on_left+d_coarse_freq] 
	           << " tmp_all["<<i<<"] = "<<tmp_all[i]
	           << " coarse_freq_comp = "<<coarse_freq_comp(d_coarse_freq,d_phase_count)<<"\n";
       }
    }
  } else if (d_type_preamble == 4){
    count_ofdm++;
    if (VERBOSE) {
      if (signal_in[0])
	std::cout<< "ofdm_frame_acquisition_80211n: signal_in[0] = 1 d_type_preamble= " <<d_type_preamble<<" count_ofdm " <<count_ofdm <<"\n";
      else 
	std::cout<< "ofdm_frame_acquisition_80211n: signal_in[0] = 0 d_type_preamble= " <<d_type_preamble<<" count_ofdm " <<count_ofdm <<"\n";
    }
    d_type_preamble++;
    signal_out[0] = 1;
    for(i = 0; i < d_occupied_carriers+d_pilot_lenght; i++) {    
     tmp_all[i] = symbol[i+zeros_on_left+d_coarse_freq] * d_hestimate[i] 
       		* coarse_freq_comp(d_coarse_freq,d_phase_count) ;
    
       if (VERBOSE){
	 std::cout << "sig1: d_hestimate["<<i<<"] = "<< d_hestimate[i] 
	           << " symbol["<<i+zeros_on_left+d_coarse_freq<<"] = "<<symbol[i+zeros_on_left+d_coarse_freq] 
	           << " tmp_all["<<i<<"] = "<<tmp_all[i]<<"\n";
       }
    }
    
  } else if (d_type_preamble == 5){
    count_ofdm++;
    if (VERBOSE) {
      if (signal_in[0])
	std::cout<< "ofdm_frame_acquisition_80211n: signal_in[0] = 1 d_type_preamble= " <<d_type_preamble<<" count_ofdm " <<count_ofdm <<"\n";
      else 
	std::cout<< "ofdm_frame_acquisition_80211n: signal_in[0] = 0 d_type_preamble= " <<d_type_preamble<<" count_ofdm " <<count_ofdm <<"\n";
    }
    d_type_preamble++;
    signal_out[0] = 1;
    for(i = 0; i < d_occupied_carriers+d_pilot_lenght; i++) {    
     tmp_all[i] = symbol[i+zeros_on_left+d_coarse_freq] * d_hestimate[i] 
       		* coarse_freq_comp(d_coarse_freq,d_phase_count) ;
    
       if (VERBOSE){
	 std::cout << "sig2: d_hestimate["<<i<<"] = "<< d_hestimate[i] 
	           << " symbol["<<i+zeros_on_left+d_coarse_freq<<"] = "<<symbol[i+zeros_on_left+d_coarse_freq] 
	           << " tmp_all["<<i<<"] = "<<tmp_all[i]<<"\n";
       }
    }
    
  } else if (d_type_preamble == 6){
    count_ofdm++;
    if (VERBOSE) {
      if (signal_in[0])
	std::cout<< "ofdm_frame_acquisition_80211n: signal_in[0] = 1 d_type_preamble= " <<d_type_preamble<<" count_ofdm " <<count_ofdm <<"\n";
      else 
	std::cout<< "ofdm_frame_acquisition_80211n: signal_in[0] = 0 d_type_preamble= " <<d_type_preamble<<" count_ofdm " <<count_ofdm <<"\n";
    }
    d_type_preamble = 0;
    signal_out[0] = 1;
    
    for(i = 0; i < d_occupied_carriers+d_pilot_lenght; i++) {    
     tmp_all[i] = symbol[i+zeros_on_left+d_coarse_freq] * d_hestimate[i] 
       		* coarse_freq_comp(d_coarse_freq,d_phase_count) ;
    }  
    
  } else if (!signal_in[0] && d_type_preamble == 0){
    count_ofdm++;
    if (VERBOSE) {
      if (signal_in[0])
	std::cout<< "ofdm_frame_acquisition_80211n: signal_in[0] = 1 d_type_preamble= " <<d_type_preamble<<" count_ofdm " <<count_ofdm <<"\n";
      else 
	std::cout<< "ofdm_frame_acquisition_80211n: signal_in[0] = 0 d_type_preamble= " <<d_type_preamble<<" count_ofdm " <<count_ofdm <<"\n";
    }
    d_type_preamble = 0;
    signal_out[0] = 0;
    
    calculate_angle(symbol, zeros_on_left);
    
    /*COMPUTE H ON PILOTS for each OFDM symbol*/
    calculate_eq_pilots(symbol, zeros_on_left);
  
    for(i = 0; i < d_occupied_carriers+d_pilot_lenght; i++) {
         /*COMPUTE H ON PILOTS*/
      gr_complex inv = gr_complex(d_hestPilot[i].real()/norm(d_hestPilot[i]), -d_hestPilot[i].imag()/norm(d_hestPilot[i]));
      tmp_all[i] = inv * symbol[i+zeros_on_left+d_coarse_freq];   
	     
  
       if (VERBOSE){
	 std::cout << "data: d_hestPilot["<<i<<"] = "<< d_hestPilot[i] 
	           << " symbol["<<i+zeros_on_left+d_coarse_freq<<"] = "<<symbol[i+zeros_on_left+d_coarse_freq] 
	           << " tmp_all["<<i<<"] = "<<tmp_all[i]
	           << " d_coarse_freq "<<d_coarse_freq
	           << " d_phase_count "<<d_phase_count
	           << " coarse_freq_comp = "<<coarse_freq_comp(d_coarse_freq,d_phase_count) 
		   << " exp "<<exp(- tmp * d_angle_pilot)<<"\n";
       }  
    }
	
  } else {
      if (VERBOSE){
	if (signal_in[0]) 
	  std::cout<< "ofdm_frame_acquisition_80211n: ERROR signal_in[0] 1 d_type_preamble= " <<d_type_preamble<<"\n";
	else
	  std::cout<< "ofdm_frame_acquisition_80211n: ERROR signal_in[0] 0 d_type_preamble= " <<d_type_preamble<<"\n";
      }
    }
  
  std::vector<gr_complex> tmp_out;
  tmp_out.resize(d_occupied_carriers);
  // remove pilots from output symbol
  id_pilot = d_pilot_first;
  j = 0;
  for(i = 0; i < (d_occupied_carriers+d_pilot_lenght); i++) {
      if (i == id_pilot) { id_pilot = id_pilot + d_pilot_spacing; continue;}
      tmp_out[j] = tmp_all[i];
      j++;
  }
  
  // copy corrected symbol to the output
  for(i = 0; i < d_occupied_carriers; i++) {
      out[i] = tmp_out[i];
  }
  
  /*Correct phase going to the next OFDM symbol when H is computed ON PREAMBLE*/
  d_phase_count++;
  if(d_phase_count == MAX_NUM_SYMBOLS) {
    d_phase_count = 1;
  }
  
  consume_each(1);
  return 1;
}
