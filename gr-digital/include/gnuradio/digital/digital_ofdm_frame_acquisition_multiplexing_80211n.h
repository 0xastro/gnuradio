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

#ifndef INCLUDED_DIGITAL_OFDM_FRAME_ACQUISITION_MULTIPLEXING_80211N_H
#define INCLUDED_DIGITAL_OFDM_FRAME_ACQUISITION_MULTIPLEXING_80211N_H

#include <digital_api.h>
#include <gr_block.h>
#include <vector>
#include <fstream>

class digital_ofdm_frame_acquisition_multiplexing_80211n;
typedef boost::shared_ptr<digital_ofdm_frame_acquisition_multiplexing_80211n> digital_ofdm_frame_acquisition_multiplexing_80211n_sptr;

digital_ofdm_frame_acquisition_multiplexing_80211n_sptr 
DIGITAL_API digital_make_ofdm_frame_acquisition_multiplexing_80211n (int ntxchannels,
									int nrxchannels,
									unsigned occupied_carriers,
									unsigned int fft_length, 
									unsigned int cplen,
									unsigned int pilot_first, unsigned int pilot_spacing,
									float rate,
									unsigned int mod_rate,
									unsigned int pkt_size,
									unsigned int headeronoff,
									const std::vector<gr_complex> &known_symbol0,
									const std::vector<gr_complex> &known_symbol1,
									const std::vector<gr_complex> &known_pilot,
									unsigned int max_fft_shift_len=10);

/*!
 * \brief take a vector of complex constellation points in from an FFT
 * and performs a correlation and equalization. This version of the block also
 * performs Alamouti coding reception off of one or two antennas
 * \ingroup demodulation_blk
 * \ingroup ofdm_blk
 *
 * This block takes the output of an FFT of a received OFDM symbol and finds the 
 * start of a frame based on two known symbols. It also looks at the surrounding
 * bins in the FFT output for the correlation in case there is a large frequency
 * shift in the data. This block assumes that the fine frequency shift has already
 * been corrected and that the samples fall in the middle of one FFT bin.
 *
 * It then uses one of those known
 * symbols to estimate the channel response over all subcarriers and does a simple 
 * 1-tap equalization on all subcarriers. This corrects for the phase and amplitude
 * distortion caused by the channel.
 */

class DIGITAL_API digital_ofdm_frame_acquisition_multiplexing_80211n : public gr_block
{
  /*! 
   * \brief Build an OFDM correlator and equalizer with Spatial multiplexing (2 TX and 2 RX MIMO).
   * \param ntxchannels         The number of transmitter antenna channels (1 or 2)
   * \param occupied_carriers   The number of subcarriers with data in the received symbol
   * \param fft_length          The size of the FFT vector (occupied_carriers + unused carriers)
   * \param cplen		The length of the cycle prefix
   * \param known_symbol        A vector of complex numbers representing a known symbol at the
   *                            start of a frame (usually a BPSK PN sequence)
   * \param max_fft_shift_len   Set's the maximum distance you can look between bins for correlation
   */
  friend DIGITAL_API digital_ofdm_frame_acquisition_multiplexing_80211n_sptr
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
									unsigned int max_fft_shift_len);
  
protected:
  digital_ofdm_frame_acquisition_multiplexing_80211n (int ntx_antennas,
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
									unsigned int max_fft_shift_len);
  
private:
  unsigned char slicer(gr_complex x);
  int correlate(unsigned int channel, const gr_complex *symbol, int zeros_on_left);
  void calculate_eq_pilots(int zeros_on_left, const gr_complex *symbol0, const gr_complex *symbol1, const char *preamble);
  gr_complex coarse_freq_comp(int freq_delta, int count);
  
  
  int d_ntx_antennas;                      // !< \brief number of TX antennas (channels); can be 1 or 2
  int d_nrx_antennas;                      // !< \brief number of RX antennas (channels); can be 1 or 2
  unsigned int d_occupied_carriers;       // !< \brief number of subcarriers with data
  unsigned int d_fft_length;              // !< \brief length of FFT vector
  unsigned int d_cplen;                   // !< \brief length of cyclic prefix in samples
  unsigned int d_freq_shift_len;          // !< \brief number of surrounding bins to look at for correlation
  
  // PREAMBLE stuff
  std::vector<gr_complex> d_known_symbol0; // !< \brief known symbols at start of frame
  std::vector<float> d_known_phase_diff;  // !< \brief factor used in correlation from known symbol
  std::vector<float> d_symbol_phase_diff0; // !< \brief factor used in correlation from received symbol

  // H
  std::vector<gr_complex> d_hestimate00;  // !< channel estimate
  std::vector<gr_complex> d_hestimate01;  // !< channel estimate
  std::vector<gr_complex> d_hestimate10;  // !< channel estimate
  std::vector<gr_complex> d_hestimate11;  // !< channel estimate
  
  // coarse frequency
  std::vector<int> d_coarse_freq;                     // !< \brief search distance in number of bins
  
  // MODULATION parameters
  float d_rate; // 0.5 for FEC=1/2
  unsigned int d_mod_rate; // BPSK=1 QPSK=2 8PSK=3
  unsigned int d_pkt_size; // packet size in bytes (NO HDR NO CRC NO TRAILER)
  
  // PILOTS stuff
  unsigned int d_pilot_length;	     // !< \brief length of the pilot
  unsigned int d_pilot_first;
  unsigned int d_pilot_spacing;
  std::vector<gr_complex> d_known_pilot0; //pilot
  std::vector<gr_complex> d_known_pilot1; //pilot
  FILE* filecorrelation;
  FILE* fileCondNum;
  
  unsigned int d_type_preamble;

 public:


  ~digital_ofdm_frame_acquisition_multiplexing_80211n(void);
  int general_work(int noutput_items,
		   gr_vector_int &ninput_items,
		   gr_vector_const_void_star &input_items,
		   gr_vector_void_star &output_items);
};


#endif
