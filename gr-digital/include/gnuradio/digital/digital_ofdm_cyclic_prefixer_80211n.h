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

#ifndef INCLUDED_DIGITAL_OFDM_CYCLIC_PREFIXER_80211N_H
#define INCLUDED_DIGITAL_OFDM_CYCLIC_PREFIXER_80211N_H

#include <digital_api.h>
#include <gr_sync_interpolator.h>
#include <vector>

class digital_ofdm_cyclic_prefixer_80211n;
typedef boost::shared_ptr<digital_ofdm_cyclic_prefixer_80211n> digital_ofdm_cyclic_prefixer_80211n_sptr;

DIGITAL_API digital_ofdm_cyclic_prefixer_80211n_sptr
digital_make_ofdm_cyclic_prefixer_80211n(int fft_length, int cp_length, int symbol_length,unsigned int id_rx,
				  const std::vector<std::vector<gr_complex> > &preamble);


class DIGITAL_API digital_ofdm_cyclic_prefixer_80211n : public gr_sync_interpolator
{
  friend DIGITAL_API digital_ofdm_cyclic_prefixer_80211n_sptr
  digital_make_ofdm_cyclic_prefixer_80211n(int fft_length, int cp_length, int symbol_length,unsigned int id_rx,
				  const std::vector<std::vector<gr_complex> > &preamble);

protected:
  digital_ofdm_cyclic_prefixer_80211n(int fft_length, int cp_length, int symbol_length,unsigned int id_rx,
				  const std::vector<std::vector<gr_complex> > &preamble);

private:

  int						d_fft_length;
  int 						d_cp_length;
  int						d_symbol_length;
  unsigned int					d_id_rx;
  const std::vector<std::vector<gr_complex> > 	d_preamble;
  int 						d_type_preamble;


public:
  ~digital_ofdm_cyclic_prefixer_80211n();
  
  int work (int noutput_items,
	    gr_vector_const_void_star &input_items,
	    gr_vector_void_star &output_items);
  
};

#endif /* INCLUDED_DIGITAL_OFDM_CYCLI_PREFIXER_80211N_H */
