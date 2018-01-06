/* -*- c++ -*- */
/*
 * Copyright 2007,2011 Free Software Foundation, Inc.
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

#ifndef INCLUDED_DIGITAL_OFDM_FRAME_SINK_NOHEADER_80211N_SECONDSHORT_H
#define INCLUDED_DIGITAL_OFDM_FRAME_SINK_NOHEADER_80211N_SECONDSHORT_H

#include <digital_api.h>
#include <gr_sync_block.h>
#include <gr_msg_queue.h>
#include <string.h>

class digital_ofdm_frame_sink_noheader_80211n;
typedef boost::shared_ptr<digital_ofdm_frame_sink_noheader_80211n> digital_ofdm_frame_sink_noheader_80211n_sptr;

struct snr_results_nh_80211n_secondshort {
  float snr_est;
  float snr_true;
  std::vector<float> snr_sc_est;
  std::vector<float> snr_sc_true;
};

DIGITAL_API digital_ofdm_frame_sink_noheader_80211n_sptr 
digital_make_ofdm_frame_sink_noheader_80211n (const std::vector<gr_complex> &sym_position, 
			      const std::vector<unsigned char> &sym_value_out,
			      const char* truesnr,
			      gr_msg_queue_sptr target_queue, 
			      int ntx_antennas, int nrx_antennas, 
			      unsigned int occupied_tones, int pkt_size, float rate,
			      float phase_gain=0.25, float freq_gain=0.25*0.25/4.0, unsigned int id_rx=0);

/*!
 * \brief Takes an OFDM symbol in, demaps it into bits of 0's and 1's, packs
 * them into packets, and sends to to a message queue sink.
 * \ingroup sink_blk
 * \ingroup ofdm_blk
 *
 * NOTE: The mod input parameter simply chooses a pre-defined demapper/slicer. Eventually,
 * we want to be able to pass in a reference to an object to do the demapping and slicing
 * for a given modulation type.
 */
class DIGITAL_API digital_ofdm_frame_sink_noheader_80211n : public gr_sync_block
{
  friend DIGITAL_API digital_ofdm_frame_sink_noheader_80211n_sptr 
  digital_make_ofdm_frame_sink_noheader_80211n (const std::vector<gr_complex> &sym_position, 
				const std::vector<unsigned char> &sym_value_out,
				const char* truesnr,
				gr_msg_queue_sptr target_queue, 
				int ntx_antennas, int nrx_antennas, 
				unsigned int occupied_tones, int pkt_size, float rate,
				float phase_gain, float freq_gain, unsigned int id_rx);

 private:
  enum state_t {STATE_SYNC_SEARCH, STATE_HAVE_FIRST_SHORT, STATE_HAVE_SECOND_SHORT, 
		STATE_HAVE_FIRST_LONG, STATE_HAVE_SECOND_LONG, STATE_HAVE_FIRST_SIG, STATE_HAVE_SECOND_SIG, STATE_HAVE_THIRD_LONG, STATE_HAVE_DATA};

  static const int MAX_PKT_LEN    = 4096;

  gr_msg_queue_sptr  d_target_queue;		// where to send the packet when received
  state_t            d_state;

  unsigned char      *d_bytes_out;              // hold the current bytes produced by the demapper    

  unsigned int       d_occupied_carriers;
  unsigned int       d_byte_offset;
  unsigned int       d_partial_byte;

  unsigned char      d_packet[MAX_PKT_LEN];	// assembled payload
  int 		     d_packetlen;		// length of packet
  int                d_packet_whitener_offset;  // offset into whitener string to use
  int		     d_packetlen_cnt;		// how many so far
  
  int d_ntx_antennas;
  int d_nrx_antennas; 

  gr_complex * d_derotated_output;  // Pointer to output stream to send deroated symbols out

  std::vector<gr_complex>    d_sym_position;
  std::vector<unsigned char> d_sym_value_out;
  std::vector<gr_complex>    d_dfe;
  unsigned int d_nbits;

  unsigned char d_resid;
  unsigned int d_nresid;
  float d_phase;
  float d_freq;
  float d_phase_gain;
  float d_freq_gain;
  float d_eq_gain;
  int d_pkt_size;
  float d_rate;
  unsigned int d_id_rx;
  
  // file to save snr value per subcarrier
  FILE* filesnr_sc_est_nh; // snr estimated per subcarrier 
  FILE* filesnr_sc_true_nh; // snr true per subcarrier
  
  
  unsigned int d_symbol_count; // the id of the ofdm symbol in a packet, used to take track of the sub-carrier granularity
  
  const char* d_truesnr; // file name containing the symbols sent by the transmitter
  std::vector<gr_complex> distance_true; // vector containing distances between received symbols and sent symbols
  
  std::vector<gr_complex> tx_symbols; // contains the symbols sent at the transmitter
  std::vector<gr_complex> rx_symbols_true; //contains symbols received
  std::vector<gr_complex> rx_symbols_sc_true; //contains symbols received in a OFDM symbol

  std::vector<int> d_subcarrier_map;
  
  std::vector<gr_complex> distance_est; // vector containing distances between received symbols and mapped symbols
  
  // Symbols are divided based on the constellation point where they are mapped on
  // For example in BPSk the map will have size 2, in QPSK size 4 and in 8PSK size 8
  std::map<int, std::vector<gr_complex> > rx_symbols_est;
  std::map<int, std::vector<gr_complex> > rx_symbols_sc_est; //contains symbols in a OFDM symbol
  
  
  // vectors to save values along the all run
  std::vector<float> SNR_est_total;
  std::vector<float> SNR_true_total;
  
  std::map<unsigned int, std::vector<float> > d_snr_result_est;
  std::map<unsigned int, std::vector<float> > d_snr_result_true;
  unsigned int d_count_snr;
  std::map<unsigned int, std::vector<gr_complex> > distance_sc_est;
  std::map<unsigned int, std::vector<gr_complex> > distance_sc_true;
    
  unsigned int d_mod_rate; //number of bits per symbol
  unsigned int d_max_snr_total; // number of OFDM symbols on which SNR per subcarrier is computed
  unsigned int d_snr_num_symbols; // number pf OFDM symbols per packet (NO PREAMBLE)
  unsigned int d_tot_symbols; // total number of symbols in a packet
  unsigned int d_diff_symbols; // number of empty subcarrier in the last OFDM symbol of a packet

 protected:
  digital_ofdm_frame_sink_noheader_80211n(const std::vector<gr_complex> &sym_position, 
			  const std::vector<unsigned char> &sym_value_out,
			  const char* truesnr,
			  gr_msg_queue_sptr target_queue, 
			  int ntx_antennas, int nrx_antennas, 
			  unsigned int occupied_tones, int pkt_size, float rate,
			  float phase_gain, float freq_gain, unsigned int id_rx);

  
  snr_results_nh_80211n_secondshort compute_snr();
  void compute_snr_sc();
  
  void enter_search();
   
  unsigned char slicer(const gr_complex x, unsigned int i);
  unsigned int demapper(const gr_complex *in,
			unsigned char *out);

  bool set_sym_value_out(const std::vector<gr_complex> &sym_position, 
			 const std::vector<unsigned char> &sym_value_out);

 public:
  ~digital_ofdm_frame_sink_noheader_80211n();

  int work(int noutput_items,
	   gr_vector_const_void_star &input_items,
	   gr_vector_void_star &output_items);
};

#endif /* INCLUDED_GR_OFDM_FRAME_SINK_NOHEADER_SECONDSHORT_H */
