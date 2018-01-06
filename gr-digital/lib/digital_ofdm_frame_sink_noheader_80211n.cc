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

#include <digital_ofdm_frame_sink_noheader_80211n.h>
#include <gr_io_signature.h>
#include <gr_expj.h>
#include <gr_math.h>
#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <iostream>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#define VERBOSE 0
#define INFNEG -10000
#define REQ_PKTS 100
#define EMPTYCARRIERS 1
#define HDSIZE 4
#define CRCSIZE 4
#define TRAILERSIZE 0




void digital_ofdm_frame_sink_noheader_80211n::compute_snr_sc(){

  unsigned int i, count, tmp, count_sum;
  gr_complex sum_est, sum_true, avg_est, avg_true;
  float variance_est, variance_true, tmpsnr;
  std::vector<gr_complex>::iterator jt;
  
  if  (d_count_snr < (int) (d_max_snr_total/d_snr_num_symbols) ){ 
    d_count_snr++;
    return;
  } else {
    d_count_snr++;
  } 
   
   // compute est snr
   for (i=0; i< (d_occupied_carriers-EMPTYCARRIERS); i++){
     
      // compute average estimated snr
      sum_est = gr_complex(0,0);
      jt = distance_sc_est[i].begin();
      count = 0;
      count_sum = 0;
      for (; jt != distance_sc_est[i].end(); ++jt){
	  if ( (i >= (d_occupied_carriers-EMPTYCARRIERS-d_diff_symbols)) && ( (count % d_snr_num_symbols ) == (d_snr_num_symbols-1)) ) { 
	    count++;
	  } else {
	    sum_est += (*jt);
	    count++;
	    count_sum++;
	  }
      }
      avg_est = sum_est / float(count_sum) ;
      
      // compute variance estimated snr
      variance_est = 0;
      jt = distance_sc_est[i].begin();
      count = 0;
      count_sum = 0;
      for (; jt != distance_sc_est[i].end(); ++jt){
	  if ( (i >= (d_occupied_carriers-EMPTYCARRIERS-d_diff_symbols)) && ( (count % d_snr_num_symbols ) == (d_snr_num_symbols-1)) ) { 
	    count++;
	  } else {
	    variance_est += norm( (*jt) ); 
	    count++;
	    count_sum++;
	  }
      }
      variance_est = variance_est / float(count_sum);
      tmpsnr = d_rate / (variance_est * log(d_sym_position.size())/log(2) );
      
      fprintf(filesnr_sc_est_nh, "%f ", tmpsnr);
      d_snr_result_est[i].push_back(tmpsnr);
   }
   fprintf(filesnr_sc_est_nh, "\n");
    
    // compute average true snr
    for (i=0; i< (d_occupied_carriers-EMPTYCARRIERS); i++){
      sum_true = gr_complex(0,0);
      jt = distance_sc_true[i].begin();
      count = 0;
      count_sum = 0;
      for (; jt != distance_sc_true[i].end(); ++jt){
	  if ( (i >= (d_occupied_carriers-EMPTYCARRIERS-d_diff_symbols)) && ( (count % d_snr_num_symbols ) == (d_snr_num_symbols-1)) ) { 
	    count++;
	  } else {
	    sum_true += (*jt);
	    count++;
	    count_sum++;
	  }
      }
      avg_true = sum_true / float(count_sum);

      // compute variance true snr
      variance_true=0;
      jt = distance_sc_true[i].begin();
      count = 0;
      count_sum = 0;
      for (; jt != distance_sc_true[i].end(); ++jt){
	  if ( (i >= (d_occupied_carriers-EMPTYCARRIERS-d_diff_symbols)) && ( (count % d_snr_num_symbols ) == (d_snr_num_symbols-1)) ) { 
	    count++;
	  } else {
	    variance_true += norm((*jt));
	    count++;
	    count_sum++;
	  }
      }
      variance_true = variance_true /  float(count_sum);
      tmpsnr =  d_rate / ( variance_true * log(d_sym_position.size())/log(2) );
      
      fprintf(filesnr_sc_true_nh, "%f ", tmpsnr);
      d_snr_result_true[i].push_back(tmpsnr);
    }
    fprintf(filesnr_sc_true_nh, "\n");
    
    // clear
    distance_sc_est.clear();
    distance_sc_true.clear();
    d_count_snr=0;

}


snr_results_nh_80211n_secondshort digital_ofdm_frame_sink_noheader_80211n::compute_snr(){
  
  unsigned int 	j, d,
		count, count_sum, i;
  // variance
  gr_complex sum_est, sum_true;
  gr_complex mean_est, mean_true;
  float variance_est = 0.0, variance_true = 0.0;
  float tmpsnr;

  snr_results_nh_80211n_secondshort result;
  
  std::vector<gr_complex>::iterator it, jt;

  // compute ESTIMATED SNR
  distance_est.clear();
  distance_est.resize((d_occupied_carriers-EMPTYCARRIERS)*d_snr_num_symbols);
  for (j = 0; j < d_sym_position.size(); j++){
    it = rx_symbols_est[j].begin();
    d=0;
    for (; it != rx_symbols_est[j].end(); it++, d++){
	if ( (*it).real() == INFNEG ) continue;
	// euclidean distance between the received symbol and the mapped constellation symbol
	distance_est.at(d) = (*it) - d_sym_position[j];
	distance_sc_est[d%(d_occupied_carriers-EMPTYCARRIERS)].push_back( (*it) - d_sym_position[j] );
    }
  }

  // compute estimated snr on the packet
  // compute average estimated snr
  jt = distance_est.begin();
  sum_est = gr_complex(0,0);
  count=0;
  count_sum=0;
  // sum all the distances across the subcarriers
  for (i=0; jt != distance_est.end(); ++jt){
      if ( (i >= (d_occupied_carriers-EMPTYCARRIERS-d_diff_symbols)) && ( (count % d_snr_num_symbols ) == (d_snr_num_symbols-1)) ) { 
	  count++;
      } else {
	  sum_est += (*jt);
	  count++;
	  count_sum++;
      }
      i++;
      if (i == (d_occupied_carriers-EMPTYCARRIERS) ) i = 0;
   }
   mean_est = sum_est / float(count_sum);
    
   // compute variance estimated snr
   variance_est = 0.0;
   count = 0;
   count_sum = 0;
   jt = distance_est.begin();
   for (i = 0; jt != distance_est.end(); ++jt){
      if ( (i >= (d_occupied_carriers-EMPTYCARRIERS-d_diff_symbols)) && ( (count % d_snr_num_symbols ) == (d_snr_num_symbols-1)) ) { 
	  count++;
      } else {
	  variance_est += norm( (*jt));
	  count++;
	  count_sum++;
      }
      i++;
      if (i == (d_occupied_carriers-EMPTYCARRIERS) ) i = 0;
   }
   variance_est = variance_est / float(count_sum);    
   tmpsnr =  d_rate / ( variance_est * log(d_sym_position.size())/log(2) );
   result.snr_est = tmpsnr;
   SNR_est_total.push_back(tmpsnr);
	    
   // compute true SNR
   distance_true.clear();
   distance_true.resize((d_occupied_carriers-EMPTYCARRIERS)*d_snr_num_symbols);
   it = rx_symbols_true.begin();
   d = 0;
   for (; it != rx_symbols_true.end(); it++, d++){
      if ( (*it).real() == INFNEG ) { abort();}
      distance_true.at(d) = (*it) - tx_symbols[d];
      distance_sc_true[d%(d_occupied_carriers-EMPTYCARRIERS)].push_back( (*it) - tx_symbols[d] );
   }
   
   // compute average true snr
   jt = distance_true.begin();
   sum_true= gr_complex(0,0);
   count = 0;
   count_sum = 0;
   for (i=0; jt != distance_true.end(); jt++){
	if ( (i >= (d_occupied_carriers-EMPTYCARRIERS-d_diff_symbols)) && ( (count % d_snr_num_symbols ) == (d_snr_num_symbols-1)) ) { 
	  count++;
      } else {
	  sum_true += (*jt);
	  count++;
	  count_sum++;
      }
      i++;
      if (i == (d_occupied_carriers-EMPTYCARRIERS) ) i = 0;
   }
   mean_true = sum_true / float(count_sum);
    
   // compute variance true snr
   variance_true = 0.0;
   count = 0;
   count_sum = 0;
   jt = distance_true.begin();
   for (i=0; jt != distance_true.end(); jt++, i++){
	if ( (i >= (d_occupied_carriers-EMPTYCARRIERS-d_diff_symbols)) && ( (count % d_snr_num_symbols ) == (d_snr_num_symbols-1)) ) { 
	  count++;
      } else {
	  variance_true += norm( (*jt) ); 
	  count++;
	  count_sum++;
      }
      i++;
      if (i == (d_occupied_carriers-EMPTYCARRIERS) ) i = 0;
   }
   variance_true = variance_true / float(count_sum);
        
   tmpsnr = d_rate / ( variance_true * log(d_sym_position.size())/log(2) );
   result.snr_true = tmpsnr;
   SNR_true_total.push_back(tmpsnr);
    
   return result;
}



inline void
digital_ofdm_frame_sink_noheader_80211n::enter_search()
{
  if (VERBOSE)
    std::cout<< "digital_ofdm_frame_sink_noheader_80211n: @ enter_search\n";

  d_state = STATE_SYNC_SEARCH;
  
  for (unsigned int i=0; i < d_sym_position.size(); i++){
      rx_symbols_est[i].clear();
      rx_symbols_est[i].resize((d_occupied_carriers-EMPTYCARRIERS)*d_snr_num_symbols);
  }
  std::vector<gr_complex>::iterator it;
  for (unsigned int i=0; i < d_sym_position.size(); i++){
    it = rx_symbols_est[i].begin();
    for (; it != rx_symbols_est[i].end(); ++it){
	(*it).real() = INFNEG;
    }
  }

  rx_symbols_true.clear();
  rx_symbols_true.resize((d_occupied_carriers-EMPTYCARRIERS)*d_snr_num_symbols);

  it = rx_symbols_true.begin();
  for (; it != rx_symbols_true.end(); ++it){
    (*it).real() = INFNEG;
  }
    
  d_symbol_count = 0;
  
  d_packetlen = (d_pkt_size + CRCSIZE) * d_rate; // in bytes with FEC and NO TRAILER 
  d_packet_whitener_offset = 0;
  d_packetlen_cnt = 0; // start counting bytes in the packet
}


unsigned char digital_ofdm_frame_sink_noheader_80211n::slicer(const gr_complex x, unsigned int i)
{
  unsigned int table_size = d_sym_value_out.size();
  unsigned int min_index = 0;
  unsigned int j, d;
  float min_euclid_dist = norm(x - d_sym_position[0]);
  float euclid_dist = 0;

  rx_symbols_sc_true.clear();
  rx_symbols_sc_true.resize(d_occupied_carriers-EMPTYCARRIERS);
  for (j = 0; j < d_sym_position.size(); j++){
  	rx_symbols_sc_est[j].clear();
  	rx_symbols_sc_est[j].resize(d_occupied_carriers-EMPTYCARRIERS);
        for (d = 0; d < (d_occupied_carriers-EMPTYCARRIERS); d++){
	 rx_symbols_sc_est[j].at(d).real() = INFNEG;
	 rx_symbols_sc_true.at(d).real() = INFNEG;
	}
  } 
  
  for (j = 1; j < table_size; j++){
    euclid_dist = norm(x - d_sym_position[j]);
    if (euclid_dist < min_euclid_dist){
      min_euclid_dist = euclid_dist;
      min_index = j;
    }
  }
  
  // put symbol in the map of vectors
  rx_symbols_est[min_index].at((d_occupied_carriers-EMPTYCARRIERS)*(d_symbol_count)+i) = x;
  rx_symbols_sc_est[min_index].at(i) = x;
  
  // put symbol for true snr
  rx_symbols_true.at((d_occupied_carriers-EMPTYCARRIERS)*(d_symbol_count)+i) = x;
  rx_symbols_sc_true.at(i) = x;
  
  return d_sym_value_out[min_index];
}

unsigned int digital_ofdm_frame_sink_noheader_80211n::demapper(const gr_complex *in,
					       unsigned char *out)
{
  unsigned int i = 0, bytes_produced = 0;
  unsigned char bits;
  int mask;
  float angle;
  gr_complex carrier;

  carrier=gr_expj(d_phase);

  gr_complex accum_error = 0.0;
  while(i < d_subcarrier_map.size()) {
      if(d_nresid > 0) {
	d_partial_byte |= d_resid;
	d_byte_offset += d_nresid;
	d_nresid = 0;
	d_resid = 0;
      }
    
     
      while((d_byte_offset < 8) && (i < d_subcarrier_map.size())) {
	gr_complex sigrot = in[d_subcarrier_map[i]]*carrier*d_dfe[i];
      
	if (VERBOSE){
	    std::cout<< "sigrot " << sigrot 
 		     << " in["<<i<<"] " << in[d_subcarrier_map[i]]
 		     << " carrier "<<carrier<< " d_dfe["<<i<<"]"<<d_dfe[i]
		     <<"\n";
	}
	d_derotated_output[0] = sigrot;
	bits = slicer(sigrot,i);
	gr_complex closest_sym = d_sym_position[bits];
	accum_error += sigrot * conj(closest_sym);

	// FIX THE FOLLOWING STATEMENT
	if (norm(sigrot)> 0.001) d_dfe[i] +=  d_eq_gain*(closest_sym/sigrot-d_dfe[i]);
	if(VERBOSE){
	  std::cout<<"norm(sigrot) = "<<norm(sigrot)<<" d_dfe+= "<<d_eq_gain*(closest_sym/sigrot-d_dfe[i])<<"\n";
	}
	
	i++;

	if((8 - d_byte_offset) >= d_nbits) {
	    d_partial_byte |= bits << (d_byte_offset);
	    d_byte_offset += d_nbits;
	} else {
	    d_nresid = d_nbits-(8-d_byte_offset);
	    mask = ((1<<(8-d_byte_offset))-1);
	    d_partial_byte |= (bits & mask) << d_byte_offset;
	    d_resid = bits >> (8-d_byte_offset);
	    d_byte_offset += (d_nbits - d_nresid);
	}
	
	if (VERBOSE){
	    std::cout<< "digital_ofdm_frame_sink_noheader_80211n: "
			<<  "demod symbol: " << in[i-1] <<" sigrot: "<< sigrot << " bits: " << d_sym_position[bits] 
			<< " partial_byte: "<< d_partial_byte << " byte_offset: "<< d_byte_offset 
			<< " resid: " << d_resid << " nresid: "<<d_nresid<<"\n";
	}
	
      }

    if(d_byte_offset == 8) {
      out[bytes_produced++] = d_partial_byte;
      d_byte_offset = 0;
      d_partial_byte = 0;
    }
  }
  if (VERBOSE)
      std::cout<< "digital_ofdm_frame_sink_noheader_80211n: accum_error " << accum_error << std::endl;

  angle = arg(accum_error);

  d_freq = d_freq - d_freq_gain*angle;
  d_phase = d_phase + d_freq - d_phase_gain*angle;
  if (d_phase >= 2*M_PI) d_phase -= 2*M_PI;
  if (d_phase <0) d_phase += 2*M_PI;
    
  if(VERBOSE)
    std::cerr<< "accum_error " << accum_error 
	     << " angle " << angle 
	     << " d_freq " << d_freq 
	     << " d_phase " << d_phase << "\n";
  
  // increment id of the ofdm symbol
  d_symbol_count++;
  
  return bytes_produced;
}


digital_ofdm_frame_sink_noheader_80211n_sptr
digital_make_ofdm_frame_sink_noheader_80211n(const std::vector<gr_complex> &sym_position, 
			     const std::vector<unsigned char> &sym_value_out,
			     const char* truesnr,
			     gr_msg_queue_sptr target_queue, 
			     int ntx_antennas, int nrx_antennas, 
			     unsigned int occupied_carriers, int pkt_size, float rate,
			     float phase_gain, float freq_gain, unsigned int id_rx)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_frame_sink_noheader_80211n(sym_position, sym_value_out, 
								truesnr,
								target_queue, 
								ntx_antennas, nrx_antennas, 
								occupied_carriers, pkt_size, rate,
								phase_gain, freq_gain, id_rx));
}


digital_ofdm_frame_sink_noheader_80211n::digital_ofdm_frame_sink_noheader_80211n(const std::vector<gr_complex> &sym_position, 
						 const std::vector<unsigned char> &sym_value_out,
						 const char* truesnr,
						 gr_msg_queue_sptr target_queue, 
						 int ntx_antennas, int nrx_antennas, 
						 unsigned int occupied_carriers, 
						 int pkt_size, float rate,
						 float phase_gain, float freq_gain, unsigned int id_rx)
  : gr_sync_block ("ofdm_frame_sink_noheader_80211n",
		   gr_make_io_signature2 (2, 2, sizeof(gr_complex)*occupied_carriers, sizeof(char)),
		   gr_make_io_signature (1, 1, sizeof(gr_complex)*occupied_carriers)),
    d_target_queue(target_queue), d_occupied_carriers(occupied_carriers), d_pkt_size(pkt_size), d_rate(rate), 
    d_byte_offset(0), d_partial_byte(0),
    d_resid(0), d_nresid(0),d_phase(0),d_freq(0),d_phase_gain(phase_gain),d_freq_gain(freq_gain),
    d_eq_gain(0.05),
    d_count_snr(0),
    d_id_rx(id_rx),
    d_ntx_antennas(ntx_antennas),
    d_nrx_antennas(nrx_antennas)
{
  
  unsigned int i,j,k;
  int diff;
  float real, imag;
  gr_complex tmp;
  
  d_symbol_count = 0;
 
  d_truesnr = truesnr;
  
  
  // read true SNR file
  FILE* filesnr = fopen(d_truesnr, "r");
  if ( filesnr != NULL )  {
    for (i=0; !feof(filesnr); i++){
	fscanf(filesnr, "%f %f\n", &real, &imag);
	tmp.real() = real; 
	tmp.imag()=imag;
	tx_symbols.push_back(tmp);
    }
  } else {
    std::cout<< "digital_ofdm_frame_sink_noheader_80211n: ERROR: true snr file sink"<<d_truesnr<<"\n";
  }
  fclose(filesnr);
  
  // Save snr values
  const char* filenameest;
  const char* filenametrue;
  std:: cout << "d_id_rx " << d_id_rx << "\n";
  if (d_id_rx == 0){
    filenameest = "snr_sc_est_file0";
    filenametrue = "snr_sc_true_file0";
  } else {
    filenameest = "snr_sc_est_file1";
    filenametrue = "snr_sc_true_file1";
  }
  std::cout<< "filenametrue " << filenametrue << "\n";
  filesnr_sc_est_nh = fopen(filenameest, "w+");
  if ( filesnr_sc_est_nh == NULL )  {
    std::cout << "ERROR: snr sc est file\n";
    abort();
  }
  // Save snr values
  filesnr_sc_true_nh = fopen(filenametrue, "w+");
  if ( filesnr_sc_true_nh == NULL )  {
    std::cout << "ERROR: snr sc true file\n";
    abort();
  }
  
  unsigned int empty_carriers, num_empty;
  // d_occupied_carriers EVEN
  if (!(d_occupied_carriers % 2)){//EVEN:: to be modified like for ODD 
    if (1){
      std::cout<< "digital_ofdm_frame_sink_noheader_80211n:  EVEN\n ";
    }
    d_subcarrier_map.clear();
    empty_carriers = (d_occupied_carriers/2)-(EMPTYCARRIERS/2);
    num_empty = 0;
    if (1){
      std::cout<< "digital_ofdm_frame_sink_noheader_80211n EVEN: empty_carriers " << empty_carriers <<"\n";
    }
    for(i = 0; i < d_occupied_carriers; i++) {
      if ( (i== empty_carriers) && (num_empty < EMPTYCARRIERS) ){
	empty_carriers++;
	num_empty++;
      } else {
	d_subcarrier_map.push_back(i);
      }
    }
    if(VERBOSE){
      for(i = 0; i < d_subcarrier_map.size(); i++) {
	std::cout << "d_subcarrier_map["<<i<<"] "<< d_subcarrier_map[i] << "\n";
      }
    }
  } else {//ODD
    if (1) {
      std::cout<< "digital_ofdm_frame_sink_noheader_80211n:  ODD\n ";
    }
    d_subcarrier_map.clear();
    empty_carriers = (d_occupied_carriers/2)-(EMPTYCARRIERS/2);
    num_empty = 0;
    if (1){
      std::cout<< "digital_ofdm_frame_sink_noheader_80211n ODD: empty_carriers " << empty_carriers <<"\n";
    }
    for(i = 0; i < d_occupied_carriers; i++) {
      if ( (i== empty_carriers) && (num_empty < EMPTYCARRIERS) ){
	empty_carriers++;
	num_empty++;
      } else {
	d_subcarrier_map.push_back(i);
      }
    }
    if(VERBOSE){
      for(i = 0; i < d_subcarrier_map.size(); i++) {
	std::cout << "d_subcarrier_map["<<i<<"] "<< d_subcarrier_map[i] << "\n";
      }
    }
  }
  
  // make sure we stay in the limit currently imposed by the occupied_carriers
  if(d_subcarrier_map.size() > d_occupied_carriers) {
    throw std::invalid_argument("digital_ofdm_frame_sink_noheader_80211n: subcarriers allocated exceeds size of occupied carriers");
  }

  d_bytes_out = new unsigned char[d_occupied_carriers];
  d_dfe.resize(occupied_carriers);
  fill(d_dfe.begin(), d_dfe.end(), gr_complex(1.0,0.0));

  set_sym_value_out(sym_position, sym_value_out);
  
  d_mod_rate = (int)(log(d_sym_position.size())/log(2));
  d_snr_num_symbols = ceil( (( d_rate * (d_pkt_size + CRCSIZE) + TRAILERSIZE )*8 ) / float(d_mod_rate * (d_subcarrier_map.size())) );
  d_tot_symbols = (int) ((( d_rate * (d_pkt_size + CRCSIZE) + TRAILERSIZE )*8 ) / float(d_mod_rate));
  d_diff_symbols = (int) ( d_snr_num_symbols * (d_subcarrier_map.size()) - d_tot_symbols);
  d_max_snr_total = floor(REQ_PKTS/d_snr_num_symbols) * d_snr_num_symbols;
  
  if (VERBOSE){
    std::cout<< "digital_ofdm_frame_sink_noheader_80211n: d_rate " << d_rate<<"\n";
    std::cout<< "digital_ofdm_frame_sink_noheader_80211n: d_pkt_size " << d_pkt_size<<"\n";
    std::cout<< "digital_ofdm_frame_sink_noheader_80211n: d_mod_rate "<<d_mod_rate <<"\n";
    std::cout<< "digital_ofdm_frame_sink_noheader_80211n: d_snr_num_symbols "<< d_snr_num_symbols <<"\n";
    std::cout<< "digital_ofdm_frame_sink_noheader_80211n: d_max_snr_total "<<d_max_snr_total<< "\n";
    std::cout<< "digital_ofdm_frame_sink_noheader_80211n: d_tot_symbols "<<d_tot_symbols<<"\n";
    std::cout<< "digital_ofdm_frame_sink_noheader_80211n: d_diff_symbols "<<d_diff_symbols<< "\n";
  }
  
  enter_search();
}

digital_ofdm_frame_sink_noheader_80211n::~digital_ofdm_frame_sink_noheader_80211n ()
{
  delete [] d_bytes_out;
}

bool
digital_ofdm_frame_sink_noheader_80211n::set_sym_value_out(const std::vector<gr_complex> &sym_position, 
					   const std::vector<unsigned char> &sym_value_out)
{
  if (sym_position.size() != sym_value_out.size())
    return false;

  if (VERBOSE){
      std::cout<< "digital_ofdm_frame_sink_noheader_80211n: sym_position.size()  " << sym_position.size()<< "\n";
      std::cout<< "digital_ofdm_frame_sink_noheader_80211n: sym_value_out  " << sym_value_out[0]<< " " << sym_value_out[1]<<"\n";
      for (unsigned int i=0; i< sym_position.size(); i++){
	  gr_complex tmp = sym_value_out[i];
	  std::cout<< "digital_ofdm_frame_sink_noheader_80211n: sym_value_out["<<i<<"] " << tmp.real() <<" " << tmp.imag() << "\n";
	  std::cout<< "digital_ofdm_frame_sink_noheader_80211n: sym_position["<<i<<"] " << sym_position[i] << "\n";
      }
  }
  if (sym_position.size()<1)
    return false;

  d_sym_position  = sym_position;
  d_sym_value_out = sym_value_out;
  d_nbits = (unsigned long)ceil(log10(float(d_sym_value_out.size())) / log10(2.0));

  return true;
}



int
digital_ofdm_frame_sink_noheader_80211n::work (int noutput_items,
			       gr_vector_const_void_star &input_items,
			       gr_vector_void_star &output_items)
{
  const gr_complex *in = (const gr_complex *) input_items[0];
  const char *sig = (const char *) input_items[1];
  
  unsigned int j = 0;
  unsigned int bytes=0;
  
  // If the output is connected, send it the derotated symbols
  if(output_items.size() >= 1)
    d_derotated_output = (gr_complex *)output_items[0];
  else
    d_derotated_output = NULL;
  
 
  
  if (VERBOSE){
    std::cout<< "digital_ofdm_frame_sink_noheader_80211n: Entering state machine\n";
  }
  switch(d_state) {
      
    case STATE_SYNC_SEARCH:    // Look for flag indicating beginning of pkt
      if (VERBOSE){
	if(d_id_rx==0){
	    std::cout<< "digital_ofdm_frame_sink_noheader_80211n: STATE_SYNC_SEARCH\n";
	}
      }
      if (sig[0]) {
	// delete FIRST SHORT
	// go to SECOND SHORT
	d_state = STATE_HAVE_FIRST_LONG;
	//d_state = STATE_HAVE_SECOND_SHORT;
	// clear state of demapper
	d_byte_offset = 0;
	d_partial_byte = 0;

	// Resetting PLL
	d_freq = 0.0;
	d_phase = 0.0;
	fill(d_dfe.begin(), d_dfe.end(), gr_complex(1.0,0.0));
	if (VERBOSE){
	    if(d_id_rx==0){
	      std::cout<< "digital_ofdm_frame_sink_noheader_80211n: sig[0]=1 HAVE_FIRST_SHORT\n";
	    }
	}
      }
      break;

    case STATE_HAVE_SECOND_SHORT: // OFDM symbols of the packet
	// delete SECOND SHORT
	// go to FIRST LONG
      if (VERBOSE){
	if(!sig[0])
	  std::cout<< "ERROR -- sig[0]=0 STATE_HAVE_SECOND_SHORT\n";
	else 
	  if(d_id_rx==0){
	    std::cout<< "sig[0]=1 STATE_HAVE_SECOND_SHORT\n";
	  }
      }
      d_state = STATE_HAVE_FIRST_LONG;
      break;
      
    case STATE_HAVE_FIRST_LONG:
	// delete FIRST LONG
	// go to FIRST SIG
      if (VERBOSE){
	  if(!sig[0]){
	    std::cout<< "digital_ofdm_frame_sink_noheader_80211n: ERROR -- sig[0]=0 STATE_HAVE_FIRST_LONG\n";
	  } else {
	    if(d_id_rx==0) {
	      std::cout<< "digital_ofdm_frame_sink_noheader_80211n: sig[0]=1 STATE_HAVE_FIRST_LONG\n";
	    }
	  }
      }
      d_state = STATE_HAVE_SECOND_LONG;
      break;
      
    case STATE_HAVE_SECOND_LONG: // OFDM symbols of the packet
	// delete FIRST LONG
	// go to FIRST SIG
      if (VERBOSE){
	if(!sig[0]){
	  std::cout<< "digital_ofdm_frame_sink_noheader_80211n : ERROR -- sig[0]=0 STATE_HAVE_SECOND_LONG\n";
	} else {
	 if(d_id_rx==0) {
	   std::cout<< "digital_ofdm_frame_sink_noheader_80211n : sig[0]=1 STATE_HAVE_SECOND_LONG\n";
	 }
	}
      }
      d_state = STATE_HAVE_FIRST_SIG;
      break;
      
    case STATE_HAVE_FIRST_SIG: // OFDM symbols of the packet
	// delete FIRST SIG
	// go to SECOND SIG
      if (VERBOSE){
	  if(!sig[0]){
	      std::cout<< "digital_ofdm_frame_sink_noheader_80211n: ERROR -- sig[0]=0 STATE_HAVE_FIRST_SIG\n";
	  } else {
	      if(d_id_rx==0){
		std::cout<< "digital_ofdm_frame_sink_noheader_80211n: sig[0]=1 STATE_HAVE_FIRST_SIG\n";
	      }
	  }
      }
      d_state = STATE_HAVE_SECOND_SIG;
      break;

    case STATE_HAVE_SECOND_SIG:
      // delete SECOND SIG
      // go to DATA or THIRD LONG  
      if (VERBOSE){
	    if(!sig[0]){
		std::cout<< "digital_ofdm_frame_sink_noheader_80211n: ERROR -- sig[0]=0 STATE_HAVE_SECOND_SIG\n";
	    } else {
		if(d_id_rx==0){  
		    std::cout<< "digital_ofdm_frame_sink_noheader_80211n: sig[0]=1 STATE_HAVE_SECOND_SIG\n";
		}
	    }
      }


	  d_state = STATE_HAVE_THIRD_LONG;
      break;
    case STATE_HAVE_THIRD_LONG: // OFDM symbols of the packet
      if (VERBOSE){
	if(!sig[0]){
	    std::cout<< "digital_ofdm_frame_sink_noheader_80211n: ERROR -- sig[0]=0 STATE_HAVE_SECOND_LONG\n";
	} else {
	    if(d_id_rx==0)  {
		std::cout<< "digital_ofdm_frame_sink_noheader_80211n: sig[0]=1 STATE_HAVE_SECOND_LONG\n"; 
	    }
	}
      }
      d_state = STATE_HAVE_DATA;   
      break;

    case STATE_HAVE_DATA: // OFDM symbols of the packet
      if (VERBOSE){
	if(d_id_rx==0){
	  std::cout<< "digital_ofdm_frame_sink_noheader_80211n: STATE_HAVE_DATA\n";
	}
      }
      // only demod after getting the preamble signal; otherwise, the 
      // equalizer taps will screw with the PLL performance
      bytes = demapper(&in[0], d_bytes_out);
             
      if (VERBOSE){
	std::cout<< "digital_ofdm_frame_sink_noheader_80211n: bytes " << bytes << "\n";
	std::cout<< "digital_ofdm_frame_sink_noheader_80211n: Packet Length: "<< d_packetlen<<"\n";
	if(sig[0])
	  std::cout<< "digital_ofdm_frame_sink_noheader_80211n: ERROR -- Found SYNC\n";
      }
    
      j = 0;
      while(j < bytes) {
	d_packet[d_packetlen_cnt++] = d_bytes_out[j++];
      
	// packet is filled
	if (d_packetlen_cnt == d_packetlen){
	    // compute snr when packet is complete
	    snr_results_nh_80211n_secondshort result = compute_snr();
	    if (! (result.snr_est > 0.0)){
		result.snr_est = 0.0;
	    }
	    if (! (result.snr_true > 0.0) ){
		result.snr_true = 0.0;
	    }
	    compute_snr_sc();

	    // build packet
	    gr_message_sptr msg =
	    gr_make_message(0, result.snr_true, d_id_rx, d_packetlen);
	    memcpy(msg->msg(), d_packet, d_packetlen_cnt);
	    d_target_queue->insert_tail(msg);		// send it
	    msg.reset();  				// free it up
	
	  enter_search();
	  break;
	}
      }
      break;
    
  default:
    assert(0);
    
  } // switch
  
  
  return 1;
}
