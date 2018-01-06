/* -*- c++ -*- */
/*
 * Copyright 2010 Free Software Foundation, Inc.
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

#include <digital_ofdm_frame_acquisition_multiplexing_80211n.h>
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
#define EMPTYCARRIERS 1
#define HDSIZE 4
#define CRCSIZE 4
#define TRAILERSIZE 0

int count_ofdm_multiplexing = 0;

digital_ofdm_frame_acquisition_multiplexing_80211n_sptr
digital_make_ofdm_frame_acquisition_multiplexing_80211n (int ntx_antennas,
						int nrx_antennas,
						unsigned occupied_carriers,
						unsigned int fft_length, 
						unsigned int cplen,
						unsigned int pilot_first, unsigned int pilot_spacing,
						float rate,
						unsigned int mod_rate,
						unsigned int pkt_size,
						unsigned int headeronoff,
						const std::vector<gr_complex> &known_symbol0,
						const std::vector<gr_complex> &known_pilot0,
						const std::vector<gr_complex> &known_pilot1,
						unsigned int max_fft_shift_len)
{
  return digital_ofdm_frame_acquisition_multiplexing_80211n_sptr 
    (new digital_ofdm_frame_acquisition_multiplexing_80211n (ntx_antennas,nrx_antennas,occupied_carriers, fft_length,  cplen,
						  pilot_first, pilot_spacing, rate, mod_rate, pkt_size,
						  headeronoff, known_symbol0, known_pilot0, known_pilot1, max_fft_shift_len));
}

digital_ofdm_frame_acquisition_multiplexing_80211n::digital_ofdm_frame_acquisition_multiplexing_80211n (
									int ntx_antennas, 
									int nrx_antennas,
									unsigned occupied_carriers,
									unsigned int fft_length, 
									unsigned int cplen,
									unsigned int pilot_first, unsigned int pilot_spacing,
									float rate,
									unsigned int mod_rate,
									unsigned int pkt_size,
									unsigned int headeronoff,
									const std::vector<gr_complex> &known_symbol0,
									const std::vector<gr_complex> &known_pilot0,
									const std::vector<gr_complex> &known_pilot1,
									unsigned int freq_shift_len
									)

  : gr_block ("ofdm_frame_acquisition_multiplexing_80211n",
	      gr_make_io_signature3 (3, 3, sizeof(char)*fft_length, sizeof(gr_complex)*fft_length, sizeof(gr_complex)*fft_length),
	      gr_make_io_signature3 (3, 3, sizeof(gr_complex)*occupied_carriers, sizeof(gr_complex)*occupied_carriers, sizeof(char))),
    d_occupied_carriers(occupied_carriers),
    d_fft_length(fft_length),
    d_cplen(cplen),
    d_pilot_first(pilot_first),
    d_pilot_spacing(pilot_spacing),
    d_freq_shift_len(freq_shift_len),
    d_known_symbol0(known_symbol0),
    d_known_pilot0(known_pilot0),
    d_known_pilot1(known_pilot1),
    d_rate(rate),
    d_mod_rate(mod_rate),
    d_pkt_size(pkt_size),
    d_ntx_antennas(ntx_antennas),
    d_nrx_antennas(nrx_antennas)
{
  
  if( d_nrx_antennas != 2)
    throw std::out_of_range("ofdm_frame_acquisition_multiplexing_80211n: Multiplexing Frame Acquisition: Can only have 2 antenna inputs.");
  
  
  unsigned int i = 0, j = 0;
  
  d_type_preamble=0;
  
  d_pilot_length = d_known_pilot0.size();
  
  int unoccupied_carriers = d_fft_length - (d_occupied_carriers + d_pilot_length);
  int zeros_on_left = (int)ceil(unoccupied_carriers/2.0);
  
  d_hestimate00.resize(d_occupied_carriers+d_pilot_length);
  d_hestimate01.resize(d_occupied_carriers+d_pilot_length);
  d_hestimate10.resize(d_occupied_carriers+d_pilot_length);
  d_hestimate11.resize(d_occupied_carriers+d_pilot_length);
  d_known_phase_diff.resize(d_occupied_carriers+d_pilot_length);
  d_coarse_freq.resize(nrx_antennas);
  d_symbol_phase_diff0.resize(d_fft_length);

  if(VERBOSE){
    std::cout<<"ofdm_frame_acquisition_multiplexing_80211n: d_known_symbol0.size()"<<d_known_symbol0.size()<<"\n";
  }
   if (zeros_on_left % 2 == 0){
    std::fill(d_known_phase_diff.begin(), d_known_phase_diff.end(), 0);
    for(i = 0; i < d_known_symbol0.size()-2; i+=2) {
      d_known_phase_diff[i] = norm(d_known_symbol0[i] - d_known_symbol0[i+2]);
    }
  } else {
    std::fill(d_known_phase_diff.begin(), d_known_phase_diff.end(), 0);
    for(i = 1; i < d_known_symbol0.size()-1; i+=2) {
      d_known_phase_diff[i] = norm(d_known_symbol0[i] - d_known_symbol0[i+2]);
    }
  }
  

 
  if (VERBOSE){
    std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_occupied_carriers " << d_occupied_carriers << "\n";
    std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_fft_length " << d_fft_length << "\n";
    std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_cplen " << d_cplen << "\n";
    std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_pilot_first " << d_pilot_first << "\n";
    std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_pilot_spacing " << d_pilot_spacing << "\n";
    std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_freq_shift_len " << d_freq_shift_len << "\n";
    std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_known_symbol0 " << d_known_symbol0.size() <<" ";
    for (int i=0; i < d_known_symbol0.size(); i++)
      std::cout << d_known_symbol0[i] << " ";
    std::cout<<"\n";
    std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_known_pilot0" ;
    for (int i=0; i < d_known_pilot0.size(); i++)
      std::cout << d_known_pilot0[i] << " ";
    std::cout<<"\n";
    std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_known_pilot1" ;
    for (int i=0; i < d_known_pilot1.size(); i++)
      std::cout << d_known_pilot1[i] << " ";
    std::cout<<"\n";
    std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_coarse_freq[0] " << d_coarse_freq[0] << "\n";
    std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_coarse_freq[1] " << d_coarse_freq[1] << "\n";
    std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_rate " << d_rate << "\n";
    std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_mod_rate " << d_mod_rate << "\n";
    std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_pkt_size " << d_pkt_size << "\n";
    std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_ntx_antennas " << d_ntx_antennas << "\n";
    std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_nrx_antennas " << d_nrx_antennas << "\n";
  }

}

digital_ofdm_frame_acquisition_multiplexing_80211n::~digital_ofdm_frame_acquisition_multiplexing_80211n(void)
{

}


int
digital_ofdm_frame_acquisition_multiplexing_80211n::correlate(unsigned int channel, const gr_complex *symbol, int zeros_on_left)
{
  unsigned int i,j;
  int coarse_freq=0;
  
  if (channel == 0) {
    std::fill(d_symbol_phase_diff0.begin(), d_symbol_phase_diff0.end(), 0);
    if (zeros_on_left % 2 == 0){ // zeros_on_left even
      for(i = 0; i < d_fft_length-2; i++) {
	d_symbol_phase_diff0[i] = norm(symbol[i] - symbol[i+2]);
      }
    } else { // zeros_on_left odd
      for(i = 1; i < d_fft_length-1; i++) {
	d_symbol_phase_diff0[i] = norm(symbol[i] - symbol[i+2]);
      }
    }
    // sweep through all possible/allowed frequency offsets and select the best
    int index = 0;
    float max = 0, sum=0;
    for(i =  zeros_on_left - d_freq_shift_len; i < zeros_on_left + d_freq_shift_len; i++) {
      sum = 0;
      for(j = 0; j < (d_occupied_carriers+d_pilot_length); j++) {
	sum += (d_known_phase_diff[j] * d_symbol_phase_diff0[i+j]);
      }
      if(sum > max) {
	max = sum;
	index = i;
      }
    }
    // set the coarse frequency offset relative to the edge of the occupied tones
    coarse_freq = index - zeros_on_left;
    if (VERBOSE){
      std::cout<<"ofdm_frame_acquisition_multiplexing_80211n: "<<channel<<" coarse_freq " << coarse_freq << "\n";
    }
    return coarse_freq;
    
  } else {

    std::fill(d_symbol_phase_diff0.begin(), d_symbol_phase_diff0.end(), 0);
    if (zeros_on_left % 2 == 0){ // zeros_on_left even
      for(i = 0; i < d_fft_length-2; i++) {
	d_symbol_phase_diff0[i] = norm(symbol[i] - symbol[i+2]);
      }
    } else { // zeros_on_left odd
      for(i = 1; i < d_fft_length-1; i++) {
	d_symbol_phase_diff0[i] = norm(symbol[i] - symbol[i+2]);
      }
    }
    // sweep through all possible/allowed frequency offsets and select the best
    int index = 0;
    float max = 0, sum=0;
    for(i =  zeros_on_left - d_freq_shift_len; i < zeros_on_left + d_freq_shift_len; i++) {
      sum = 0;
      for(j = 0; j < (d_occupied_carriers+d_pilot_length); j++) {
	sum += (d_known_phase_diff[j] * d_symbol_phase_diff0[i+j]);
      }
      if(sum > max) {
	max = sum;
	index = i;
      }
    }
    // set the coarse frequency offset relative to the edge of the occupied tones
    coarse_freq = index - zeros_on_left;
    if (VERBOSE){
      std::cout<<"ofdm_frame_acquisition_multiplexing_80211n: "<<channel<<" coarse_freq " << coarse_freq << "\n";
    }
    return coarse_freq;
  }
}


void
digital_ofdm_frame_acquisition_multiplexing_80211n::calculate_eq_pilots(int zeros_on_left,const gr_complex *symbol0,const gr_complex *symbol1,
								 const char *preamble)
{
  unsigned int i=0,j;
  std::vector<gr_complex> h_test00;
  std::vector<gr_complex> h_test01;
  std::vector<gr_complex> h_test10;
  std::vector<gr_complex> h_test11;
  h_test00.resize(d_pilot_length/2);
  h_test01.resize(d_pilot_length/2);
  h_test10.resize(d_pilot_length/2);
  h_test11.resize(d_pilot_length/2);
  
  j=d_pilot_first;
  if (VERBOSE){
    std::cout<<"ofdm_frame_acquisition_multiplexing_80211n: zeros_on_left "<<zeros_on_left<< "\n";
    std::cout<<"ofdm_frame_acquisition_multiplexing_80211n: d_coarse_freq[0] "<<d_coarse_freq[0]<< "\n";
    std::cout<<"ofdm_frame_acquisition_multiplexing_80211n: d_coarse_freq[1] "<<d_coarse_freq[1]<< "\n";
    std::cout<<"ofdm_frame_acquisition_multiplexing_80211n: d_pilot_first "<<d_pilot_first<< "\n";
    std::cout<<"ofdm_frame_acquisition_multiplexing_80211n: d_pilot_length "<<d_pilot_length<< "\n";
    std::cout<<"ofdm_frame_acquisition_multiplexing_80211n: d_pilot_spacing "<<d_pilot_spacing<< "\n";
  }
  for(i=0; i < (d_pilot_length/2); i++){
    h_test00[i] = (symbol0[j+zeros_on_left+d_coarse_freq[0]] + symbol0[j+d_pilot_spacing+zeros_on_left+d_coarse_freq[0]]);
    h_test00[i].real() = h_test00[i].real()*0.5;
    h_test00[i].imag() = h_test00[i].imag()*0.5;
    h_test01[i] = (symbol0[j+zeros_on_left+d_coarse_freq[0]] - symbol0[j+d_pilot_spacing+zeros_on_left+d_coarse_freq[0]]);
    h_test01[i].real() = h_test01[i].real()*0.5;
    h_test01[i].imag() = h_test01[i].imag()*0.5;
    h_test10[i] = (symbol1[j+zeros_on_left+d_coarse_freq[1]] + symbol1[j+d_pilot_spacing+zeros_on_left+d_coarse_freq[1]]);
    h_test10[i].real() = h_test10[i].real()*0.5;
    h_test10[i].imag() = h_test10[i].imag()*0.5;
    h_test11[i] = (symbol1[j+zeros_on_left+d_coarse_freq[1]] - symbol1[j+d_pilot_spacing+zeros_on_left+d_coarse_freq[1]]);
    h_test11[i].real() = h_test11[i].real()*0.5;
    h_test11[i].imag() = h_test11[i].imag()*0.5;
    j+=(d_pilot_spacing*2);
  }
  
  // copy h for values before first pilot
  for (j = 0; j < d_pilot_first; j++){
    d_hestimate00[j] = h_test00[0];
    d_hestimate01[j] = h_test01[0]; 
    d_hestimate10[j] = h_test10[0]; 
    d_hestimate11[j] = h_test11[0]; 
  }

  // copy h for values after pilot
  for(j = 0; j < (d_pilot_length/2)-1; j++) {
     for(i = (d_pilot_spacing*2*j)+d_pilot_first; i < (d_pilot_spacing*2*j)+d_pilot_first+d_pilot_spacing*2; i++) {
	d_hestimate00[i] = h_test00[j];
	d_hestimate01[i] = h_test01[j];
	d_hestimate10[i] = h_test10[j];
	d_hestimate11[i] = h_test11[j];
     }
  }
  
  // copy h for values after last pilot
  for (j = i; j < d_occupied_carriers+d_pilot_length; j++){
    d_hestimate00[j] = h_test00[(d_pilot_length/2)-1]; 
    d_hestimate01[j] = h_test01[(d_pilot_length/2)-1]; 
    d_hestimate10[j] = h_test10[(d_pilot_length/2)-1]; 
    d_hestimate11[j] = h_test11[(d_pilot_length/2)-1]; 
  }
  
  if (VERBOSE){
    for (j = 0; j < d_occupied_carriers+d_pilot_length; j++){
      std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_hestimate00["<<j<<"] "<< d_hestimate00[j] <<" d_hestimate01["<<j<<"] "<< d_hestimate01[j] <<"\n";
      std::cout <<"ofdm_frame_acquisition_multiplexing_80211n: d_hestimate10["<<j<<"] "<< d_hestimate10[j] <<" d_hestimate11["<<j<<"] "<< d_hestimate11[j]<<"\n";
    }
  }
  
  //compute channel correlation
  unsigned int id_pilot = d_pilot_first;
  float cond;
  for(i = 0; i < d_pilot_length; i++) {
//      cond=0.0;
      
      //compute inverse H
      gr_complex det = d_hestimate00[id_pilot]*d_hestimate11[id_pilot]-d_hestimate01[id_pilot]*d_hestimate10[id_pilot];
      gr_complex detinv = gr_complex(det.real()/norm(det),-det.imag()/norm(det));
      gr_complex h00inv = detinv*d_hestimate11[id_pilot];
      gr_complex h01inv = -detinv*d_hestimate01[id_pilot];
      gr_complex h10inv = -detinv*d_hestimate10[id_pilot];
      gr_complex h11inv = detinv*d_hestimate00[id_pilot];
      
      // compute Condition number of H
      // ||H||_2 = sqrt(max eigenvalue(H^H*H))
      // ||H^(-1)||_2 = sqrt(max eigenvalue((H^(-1))^H*H^(-1)))
      // cond = ||H||_2 * ||H^(-1)||_2
      
      // compute H^H
      // h00H h01H h10H h11H
      gr_complex h00H = conj(d_hestimate00[id_pilot]);
      gr_complex h01H = conj(d_hestimate10[id_pilot]);
      gr_complex h10H = conj(d_hestimate01[id_pilot]);
      gr_complex h11H = conj(d_hestimate11[id_pilot]);
      
      // compute (H^(-1))^H
      // h00H h01H h10H h11H
      gr_complex h00invH = conj(h00inv);
      gr_complex h01invH = conj(h10inv);
      gr_complex h10invH = conj(h01inv);
      gr_complex h11invH = conj(h11inv);
      
      // compute H^H*H
      gr_complex p00H = h00H * d_hestimate00[id_pilot] + h01H * d_hestimate10[id_pilot];
      gr_complex p01H = h00H * d_hestimate01[id_pilot] + h01H * d_hestimate11[id_pilot];
      gr_complex p10H = h10H * d_hestimate00[id_pilot] + h11H * d_hestimate10[id_pilot];
      gr_complex p11H = h10H * d_hestimate01[id_pilot] + h11H * d_hestimate11[id_pilot];
      
      // compute (H^(-1))^H*H^(-1))
      gr_complex p00invH = h00invH * h00inv + h01invH * h10inv;
      gr_complex p01invH = h00invH * h01inv + h01invH * h11inv;
      gr_complex p10invH = h10invH * h00inv + h11invH * h10inv;
      gr_complex p11invH = h10invH * h01inv + h11invH * h11inv;
      
      gr_complex t = p00H*p11H - p01H*p10H; 
      gr_complex tmp1H= p00H+p11H + sqrt( (p00H+p11H) * (p00H+p11H) -gr_complex(4*t.real(), 4*t.imag()) );
      gr_complex eig1H = gr_complex(tmp1H.real()/2.0, tmp1H.imag()/2.0); 
      gr_complex tmp2H= p00H+p11H - sqrt( (p00H+p11H) * (p00H+p11H) -gr_complex(4*t.real(), 4*t.imag()) );
      gr_complex eig2H= gr_complex(tmp2H.real()/2.0, tmp2H.imag()/2.0); 
      
      t = p00invH*p11invH - p01invH*p10invH;
      gr_complex tmp1invH= p00invH+p11invH + sqrt( (p00invH+p11invH) * (p00invH+p11invH) -gr_complex(4*t.real(), 4*t.imag()) );
      gr_complex eig1invH= gr_complex(tmp1invH.real()/2.0, tmp1invH.imag()/2.0); 
      gr_complex tmp2invH= p00invH+p11invH - sqrt( (p00invH+p11invH) * (p00invH+p11invH) -gr_complex(4*t.real(), 4*t.imag()) );
      gr_complex eig2invH= gr_complex(tmp2invH.real()/2.0, tmp2invH.imag()/2.0); 
      
      float maxH = eig1H.real(); 
      if (maxH < eig2H.real() ) maxH = eig2H.real();
      float maxinvH = eig1invH.real(); 
      if (maxinvH < eig2invH.real() ) maxinvH = eig2invH.real();
      
      float sqrtH = sqrt(maxH);
      float sqrtinvH = sqrt(maxinvH);
      cond = sqrtH * sqrtinvH;
      
  id_pilot = id_pilot + d_pilot_spacing; 
  }
}

int
digital_ofdm_frame_acquisition_multiplexing_80211n::general_work(int noutput_items,
						 gr_vector_int &ninput_items,
						 gr_vector_const_void_star &input_items,
						 gr_vector_void_star &output_items)
{
    const char *signal_in = (const char *)input_items[0];
    const gr_complex *symbol0 = (const gr_complex *)input_items[1];
    const gr_complex *symbol1 = (const gr_complex *)input_items[2];

    gr_complex *out0 = (gr_complex *) output_items[0];
    gr_complex *out1 = (gr_complex *) output_items[1];
    char *signal_out0 = (char *) output_items[2];

    int syms_processed = 0;
    int unoccupied_carriers = d_fft_length - (d_occupied_carriers+d_pilot_length);
    int zeros_on_left = (int)ceil(unoccupied_carriers/2.0);
    if (VERBOSE){
      std::cout<<"digital_ofdm_frame_acquisition_multiplexing_80211n::general_work\n";
      std::cout<<"digital_ofdm_frame_acquisition_multiplexing_80211n: zeros_on_left= "<< zeros_on_left<<"\n";
    }
    unsigned int i, id_pilot, j;
       
    d_coarse_freq.clear();
    d_coarse_freq.resize(2);
    
    if(VERBOSE){
      for(i = 0; i < d_fft_length; i++) {
	std::cout<<"ofdm_frame_acquisition_multiplexing_80211n: symbol0["<<i<<"] " << symbol0[i] <<" symbol1["<<i<<"] " << symbol1[i] << "\n"; 
      }
    }
       
    // variables used to remove pilots
    std::vector<gr_complex> sig0;
    std::vector<gr_complex> sig1;
    sig0.resize(d_occupied_carriers+d_pilot_length);
    sig1.resize(d_occupied_carriers+d_pilot_length);
  
    /* d_type_preamble = 0 --> first short preamble
     * d_type_preamble = 1 --> second short preamble
     * d_type_preamble = 2 --> first long preamble
     * d_type_preamble = 3 --> second long preamble
     * d_type_preamble = 4 --> first sig preamble
     * d_type_preamble = 5 --> second sig preamble
     * d_type_preamble = 6 --> third long preamble if nrx_antennas = 2
    */
  
    if (signal_in[0] && d_type_preamble == 0){
      count_ofdm_multiplexing=0;
      if (VERBOSE){
	std::cout<< "ofdm_frame_acquisition_multiplexing_80211n: signal_in[0] 1 d_type_preamble= " <<d_type_preamble<<" count_ofdm_multiplexing " <<count_ofdm_multiplexing <<"\n";
      }
      d_type_preamble++;
      d_type_preamble++;
      signal_out0[0] = 1;
      // correlate on first antenna
      d_coarse_freq[0] = correlate(0, &symbol0[0], zeros_on_left);
      if (VERBOSE){
	std::cout<< "ofdm_frame_acquisition_multiplexing_80211n: d_coarse_freq[0] " << d_coarse_freq[0] << "\n";
      }
      // correlate on second antenna as the first
      d_coarse_freq[1] = correlate(1, &symbol1[0], zeros_on_left);
      if (VERBOSE){
	std::cout<< "ofdm_frame_acquisition_multiplexing_80211n: d_coarse_freq[1] " << d_coarse_freq[1] << "\n";
      }
      for(i = 0; i < d_occupied_carriers+d_pilot_length; i++) {
	sig0[i] = symbol0[i+zeros_on_left+d_coarse_freq[0]]; 
	sig1[i] = symbol1[i+zeros_on_left+d_coarse_freq[1]];    
      }

    } else if (d_type_preamble == 1){
      count_ofdm_multiplexing++;
      if (VERBOSE){
	std::cout<< "ofdm_frame_acquisition_multiplexing_80211n: d_type_preamble= " <<d_type_preamble<<" count_ofdm_multiplexing " <<count_ofdm_multiplexing <<"\n";
      }
      d_type_preamble++;
      signal_out0[0] = 1;
    
      for(i = 0; i < d_occupied_carriers+d_pilot_length; i++) {
	sig0[i] = symbol0[i+zeros_on_left+d_coarse_freq[0]]; 
	sig1[i] = symbol1[i+zeros_on_left+d_coarse_freq[1]];    
      }
    
    } else if (d_type_preamble == 2){
      count_ofdm_multiplexing++;
      if (VERBOSE){
	std::cout<< "ofdm_frame_acquisition_multiplexing_80211n: d_type_preamble= " <<d_type_preamble<<" count_ofdm_multiplexing " <<count_ofdm_multiplexing <<"\n";
      }
      d_type_preamble++;
      signal_out0[0] = 1;
      // channel estimation based on long preamble
    
      for(i = 0; i < d_occupied_carriers+d_pilot_length; i++) {
	sig0[i] = symbol0[i+zeros_on_left+d_coarse_freq[0]]; 
	sig1[i] = symbol1[i+zeros_on_left+d_coarse_freq[1]];    
      }
    
    } else if (d_type_preamble == 3){
      count_ofdm_multiplexing++;
      if (VERBOSE){
	std::cout<< "ofdm_frame_acquisition_multiplexing_80211n: d_type_preamble= " <<d_type_preamble<<" count_ofdm_multiplexing " <<count_ofdm_multiplexing <<"\n";
      }
      d_type_preamble++;
      signal_out0[0] = 1;

      for(i = 0; i < d_occupied_carriers+d_pilot_length; i++) {
	sig0[i] = symbol0[i+zeros_on_left+d_coarse_freq[0]]; 
	sig1[i] = symbol1[i+zeros_on_left+d_coarse_freq[1]];    
      }
    
    } else if (d_type_preamble == 4){
      count_ofdm_multiplexing++;
      if (VERBOSE){
	std::cout<< "ofdm_frame_acquisition_multiplexing_80211n: d_type_preamble= " <<d_type_preamble<<" count_ofdm_multiplexing " <<count_ofdm_multiplexing <<"\n";
      }
      d_type_preamble++;
      signal_out0[0] = 1;
    
      for(i = 0; i < d_occupied_carriers+d_pilot_length; i++) {
	sig0[i] = symbol0[i+zeros_on_left+d_coarse_freq[0]]; 
	sig1[i] = symbol1[i+zeros_on_left+d_coarse_freq[1]];    
      }
    } else if (d_type_preamble == 5){
      count_ofdm_multiplexing++;
      if (VERBOSE){
	std::cout<< "ofdm_frame_acquisition_multiplexing_80211n: d_type_preamble= " <<d_type_preamble<<" count_ofdm_multiplexing " <<count_ofdm_multiplexing <<"\n";;
      }
      d_type_preamble++;
      signal_out0[0] = 1;
    
      for(i = 0; i < d_occupied_carriers+d_pilot_length; i++) {
	sig0[i] = symbol0[i+zeros_on_left+d_coarse_freq[0]]; 
	sig1[i] = symbol1[i+zeros_on_left+d_coarse_freq[1]];    
      }
    } else if (d_type_preamble == 6){
      count_ofdm_multiplexing++;
      if (VERBOSE){
	std::cout<< "ofdm_frame_acquisition_multiplexing_80211n: d_type_preamble= " <<d_type_preamble<<" count_ofdm_multiplexing " <<count_ofdm_multiplexing <<"\n";
      }
      d_type_preamble = 0;
      signal_out0[0] = 1;
      // channel estimation based on long preamble
    
      for(i = 0; i < d_occupied_carriers+d_pilot_length; i++) {
	sig0[i] = symbol0[i+zeros_on_left+d_coarse_freq[0]]; 
	sig1[i] = symbol1[i+zeros_on_left+d_coarse_freq[1]];    
      }
    
    } else if (!signal_in[0] && d_type_preamble == 0){
      count_ofdm_multiplexing++;
      if (VERBOSE){
	std::cout<< "ofdm_frame_acquisition_multiplexing_80211n: signal_in[0] 0 d_type_preamble= " <<d_type_preamble<<" count_ofdm_multiplexing " <<count_ofdm_multiplexing <<"\n";
      }
      d_type_preamble = 0;
      signal_out0[0] = 0;
    
      // Compute H based on PILOTS for each OFDM symbol
      calculate_eq_pilots(zeros_on_left, &symbol0[0], &symbol1[0],&signal_in[0]);

      float cond;
      // Spatial Multiplexing to reconstruct two different packets
      for(i = 0; i < (d_occupied_carriers+d_pilot_length); i++) {
      
	//compute inverse H
	gr_complex det = d_hestimate00[i]*d_hestimate11[i]-d_hestimate01[i]*d_hestimate10[i];
	gr_complex detinv = gr_complex(det.real()/norm(det),-det.imag()/norm(det));
	gr_complex h00inv = detinv*d_hestimate11[i];
	gr_complex h01inv = -detinv*d_hestimate01[i];
	gr_complex h10inv = -detinv*d_hestimate10[i];
	gr_complex h11inv = detinv*d_hestimate00[i];
      
	// compute Condition number of H
	// ||H||_2 = sqrt(max eigenvalue(H^H*H))
	// ||H^(-1)||_2 = sqrt(max eigenvalue((H^(-1))^H*H^(-1)))
	// cond = ||H||_2 * ||H^(-1)||_2
      
	// compute H^H
	// h00H h01H h10H h11H
	gr_complex h00H = conj(d_hestimate00[i]);
	gr_complex h01H = conj(d_hestimate10[i]);
	gr_complex h10H = conj(d_hestimate01[i]);
	gr_complex h11H = conj(d_hestimate11[i]);
      
	// compute (H^(-1))^H
	// h00H h01H h10H h11H
	gr_complex h00invH = conj(h00inv);
	gr_complex h01invH = conj(h10inv);
	gr_complex h10invH = conj(h01inv);
	gr_complex h11invH = conj(h11inv);
      
	// compute H^H*H
	gr_complex p00H = h00H * d_hestimate00[i] + h01H * d_hestimate10[i];
	gr_complex p01H = h00H * d_hestimate01[i] + h01H * d_hestimate11[i];
	gr_complex p10H = h10H * d_hestimate00[i] + h11H * d_hestimate10[i];
	gr_complex p11H = h10H * d_hestimate01[i] + h11H * d_hestimate11[i];
      
	// compute (H^(-1))^H*H^(-1))
	gr_complex p00invH = h00invH * h00inv + h01invH * h10inv;
	gr_complex p01invH = h00invH * h01inv + h01invH * h11inv;
	gr_complex p10invH = h10invH * h00inv + h11invH * h10inv;
	gr_complex p11invH = h10invH * h01inv + h11invH * h11inv;
      
	gr_complex t = p00H*p11H - p01H*p10H; 
	gr_complex tmp1H= p00H+p11H + sqrt( (p00H+p11H) * (p00H+p11H) -gr_complex(4*t.real(), 4*t.imag()) );
	gr_complex eig1H = gr_complex(tmp1H.real()/2.0, tmp1H.imag()/2.0); 
	gr_complex tmp2H= p00H+p11H - sqrt( (p00H+p11H) * (p00H+p11H) -gr_complex(4*t.real(), 4*t.imag()) );
	gr_complex eig2H= gr_complex(tmp2H.real()/2.0, tmp2H.imag()/2.0); 
      
	t = p00invH*p11invH - p01invH*p10invH;
	gr_complex tmp1invH= p00invH+p11invH + sqrt( (p00invH+p11invH) * (p00invH+p11invH) -gr_complex(4*t.real(), 4*t.imag()) );
	gr_complex eig1invH= gr_complex(tmp1invH.real()/2.0, tmp1invH.imag()/2.0); 
	gr_complex tmp2invH= p00invH+p11invH - sqrt( (p00invH+p11invH) * (p00invH+p11invH) -gr_complex(4*t.real(), 4*t.imag()) );
	gr_complex eig2invH= gr_complex(tmp2invH.real()/2.0, tmp2invH.imag()/2.0); 
      
	float maxH = eig1H.real(); 
	if (maxH < eig2H.real() ) maxH = eig2H.real();
	float maxinvH = eig1invH.real(); 
	if (maxinvH < eig2invH.real() ) maxinvH = eig2invH.real();
      
	float sqrtH = sqrt(maxH);
	float sqrtinvH = sqrt(maxinvH);
	cond = sqrtH * sqrtinvH;
      
	if (VERBOSE){
	  std::cout<<"ofdm_frame_acquisition_multiplexing_80211n: h00= "<< d_hestimate00[i] <<" h01= "<< d_hestimate01[i] 
		    <<" h10= "<< d_hestimate10[i] << " h11= "<< d_hestimate11[i];
	  std::cout<<" cond " << cond<<"\n";
	  std::cout<<"ofdm_frame_acquisition_multiplexing_80211n: eig1H= "<< eig1H <<" eig2H= "<< eig2H <<"\n";
	  std::cout<<"ofdm_frame_acquisition_multiplexing_80211n: eig1invH= "<< eig1invH <<" eig2invH= "<< eig2invH <<"\n";
	}
      
	if (cond < 15){
	  // compute separate streams
	  sig0[i] = h00inv * symbol0[i+zeros_on_left+d_coarse_freq[0]] + h01inv * symbol1[i+zeros_on_left+d_coarse_freq[0]];
	  sig1[i] = h10inv * symbol0[i+zeros_on_left+d_coarse_freq[0]] + h11inv * symbol1[i+zeros_on_left+d_coarse_freq[0]];
	  if (VERBOSE) {
	    std::cout<<"ofdm_frame_acquisition_multiplexing_80211n: sig0["<<i<<"] " << sig0[i] <<"\n";
	    std::cout<<"ofdm_frame_acquisition_multiplexing_80211n: sig1["<<i<<"] " << sig1[i] <<"\n";
	  }
	} else {
	  // maximum likehood
	  if (VERBOSE) std::cout<<"ofdm_frame_acquisition_multiplexing_80211n: maximum likehood cond= "<<cond<<"\n";
	  std::vector<float>x0; x0.push_back(1.0);x0.push_back(-1.0);x0.push_back(1.0);x0.push_back(-1.0); 
	  std::vector<float>x1; x1.push_back(1.0);x1.push_back(1.0);x1.push_back(-1.0);x1.push_back(-1.0);
	  std::vector<gr_complex> test_y0;
	  std::vector<gr_complex> test_y1;
	  test_y0.resize(4);
	  test_y1.resize(4);
	  for (unsigned int g=0; g<4;g++){
	    test_y0[g] = symbol0[i+zeros_on_left+d_coarse_freq[0]] - d_hestimate00[i] * x0[g] - d_hestimate01[i] * x1[g];
	    test_y1[g] = symbol1[i+zeros_on_left+d_coarse_freq[0]] - d_hestimate10[i] * x0[g] - d_hestimate11[i] * x1[g];
	  }
	  std::vector<float> metric;
	  metric.resize(4);
	  for (unsigned int g=0; g<4;g++){
	    metric[g] = test_y0[g].real()*test_y0[g].real() + test_y0[g].imag()*test_y0[g].imag() +  
		      test_y1[g].real()*test_y1[g].real() + test_y1[g].imag()*test_y1[g].imag();
	  }
	  // find min of metric
	  float min = metric[0];
	  unsigned int index = 0;
	  for (unsigned int g=1; g<4;g++){
	    if (metric[g] < min) { min = metric[g]; index = g;}
	  }
	  sig0[i] = gr_complex(x0[index],0);
	  sig1[i] = gr_complex(x1[index],0);
	}
      }
	
    } else {
      if (VERBOSE){
	  if (signal_in[0]) 
	      std::cout<< "ofdm_frame_acquisition_multiplexing_80211n: ERROR signal_in[0] 1 d_type_preamble= " <<d_type_preamble<<"\n";
	  else
	    std::cout<< "ofdm_frame_acquisition_multiplexing_80211n: ERROR signal_in[0] 0 d_type_preamble= " <<d_type_preamble<<"\n";
      }
    } 
    // remove pilots 
    id_pilot= d_pilot_first;
    j=0;
    for(i = 0; i < (d_occupied_carriers+d_pilot_length); i++) {
	if (i==id_pilot) { 
	  id_pilot = id_pilot + d_pilot_spacing; 
	  continue;
	}
	out0[j] = sig0[i];
	out1[j] = sig1[i];
	if(VERBOSE){
	  std::cout<<i<<"ofdm_frame_acquisition_multiplexing_80211n: out0["<<j<<"] " << out0[j] <<"\n";
	  std::cout<<i<<"ofdm_frame_acquisition_multiplexing_80211n:  out1["<<j<<"] " << out1[j] <<"\n";
	}
	j++;
     }

  consume_each(1);
  return 1;
}
