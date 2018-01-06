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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef INCLUDED_DIGITAL_OFDM_INSERT_PILOT_80211N_H
#define INCLUDED_DIGITAL_OFDM_INSERT_PILOT_80211N_H

#include <digital_api.h>
#include <gr_block.h>
#include <vector>

class digital_ofdm_insert_pilot_80211n;
typedef boost::shared_ptr<digital_ofdm_insert_pilot_80211n> digital_ofdm_insert_pilot_80211n_sptr;

DIGITAL_API digital_ofdm_insert_pilot_80211n_sptr
digital_make_ofdm_insert_pilot_80211n(int fft_length, int zeros_on_left, int zeros_on_right,
				  const std::vector<gr_complex> &pilot,int first_pilot,int space_pilot, int ntx_antennas);

/*!
 * \brief insert "pre-modulated" pilot symbols in each OFDM symbol.
 *
 * \param fft_length length of each symbol in samples.
 * \param zeros_on_left
 * \param pilot   vector of symbols that represent the pre-modulated pilot.
 */

class DIGITAL_API digital_ofdm_insert_pilot_80211n : public gr_block
{
  friend DIGITAL_API digital_ofdm_insert_pilot_80211n_sptr
  digital_make_ofdm_insert_pilot_80211n(int fft_length, int zeros_on_left, int zeros_on_right,
				    const std::vector<gr_complex> &pilot,int first_pilot,int space_pilot, int ntx_antennas);

protected:
  digital_ofdm_insert_pilot_80211n(int fft_length, int zeros_on_left, int zeros_on_right,
			       const std::vector<gr_complex> &pilot,int first_pilot,int space_pilot, int ntx_antennas);

private:

  int					d_fft_length;
  int 					d_zeros_on_left;
  int					d_zeros_on_right;
  const std::vector<gr_complex> 	d_pilot;
  int d_first_pilot;
  int d_space_pilot;
  int d_type_preamble;
  int d_ntx_antennas;



public:
  ~digital_ofdm_insert_pilot_80211n();
  
  int general_work (int noutput_items,
		    gr_vector_int &ninput_items,
		    gr_vector_const_void_star &input_items,
		    gr_vector_void_star &output_items);
  
};

#endif /* INCLUDED_DIGITAL_OFDM_INSERT_PILOT_80211N_H */
