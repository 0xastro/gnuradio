#!/usr/bin/env python
#
# Copyright 2007,2008 Free Software Foundation, Inc.
# 
# This file is part of GNU Radio
# 
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

import math
from numpy import fft
from gnuradio import gr


class ofdm_sync_mimo(gr.hier_block2):
    def __init__(self, fft_length, cp_length, peakthr, logging=False):

        
	gr.hier_block2.__init__(self, "ofdm_sync_mimo",
		gr.io_signature(2, 2, gr.sizeof_gr_complex), # Input signature
                                gr.io_signature3(3, 3, gr.sizeof_float,  gr.sizeof_float, gr.sizeof_char)) # Output signature

        self.input_0 = gr.add_const_cc(0)
        self.input_1 = gr.add_const_cc(0)

        # PN Sync
	print "--> SYNCH MIMO <--"

	self.gr_sub_xx_1 = gr.sub_ff(1)
	self.gr_sub_xx_0 = gr.sub_ff(1)
	self.gr_sample_and_hold_xx_1 = gr.sample_and_hold_ff()
	self.gr_sample_and_hold_xx_0 = gr.sample_and_hold_ff()
	self.gr_peak_detector_xb_1 = gr.peak_detector_fb(0.25, 0.4, 10, 0.001)
	self.gr_peak_detector_xb_0 = gr.peak_detector_fb(0.25, 0.4, 10, 0.001)
	self.gr_multiply_xx_1 = gr.multiply_vcc(1)
	self.gr_multiply_xx_0 = gr.multiply_vcc(1)
	self.gr_multiply_const_vxx_3 = gr.multiply_const_vff((-1, ))
	self.gr_multiply_const_vxx_2 = gr.multiply_const_vff((2, ))
	self.gr_multiply_const_vxx_1 = gr.multiply_const_vff((-1, ))
	self.gr_multiply_const_vxx_0 = gr.multiply_const_vff((2, ))
	moving_sum_taps = [1.0 for i in range(fft_length+cp_length)]
	self.gr_interp_fir_filter_xxx_5 = gr.interp_fir_filter_fff(1,moving_sum_taps)
	self.gr_interp_fir_filter_xxx_4 = gr.interp_fir_filter_fff(1,moving_sum_taps)
	self.gr_interp_fir_filter_xxx_3 = gr.interp_fir_filter_ccc(1,moving_sum_taps)
	self.gr_interp_fir_filter_xxx_2 = gr.interp_fir_filter_fff(1,moving_sum_taps)
	self.gr_interp_fir_filter_xxx_1 = gr.interp_fir_filter_fff(1,moving_sum_taps)
	self.gr_interp_fir_filter_xxx_0 = gr.interp_fir_filter_ccf(1,moving_sum_taps)
	self.gr_delay_1 = gr.delay(gr.sizeof_gr_complex*1, 80)
	self.gr_delay_0 = gr.delay(gr.sizeof_gr_complex*1, 80)
	self.gr_conjugate_cc_1 = gr.conjugate_cc()
	self.gr_conjugate_cc_0 = gr.conjugate_cc()
	self.gr_complex_to_mag_squared_3 = gr.complex_to_mag_squared(1)
	self.gr_complex_to_mag_squared_2 = gr.complex_to_mag_squared(1)
	self.gr_complex_to_mag_squared_1 = gr.complex_to_mag_squared(1)
	self.gr_complex_to_mag_squared_0 = gr.complex_to_mag_squared(1)
	self.gr_complex_to_mag_1 = gr.complex_to_mag(1)
	self.gr_complex_to_mag_0 = gr.complex_to_mag(1)
	self.gr_complex_to_arg_0 = gr.complex_to_arg(1)
	self.gr_complex_to_arg_1 = gr.complex_to_arg(1)
	self.gr_add_xx_2 = gr.add_vcc(1)
	self.gr_add_xx_1 = gr.add_vff(1)
	self.gr_add_xx_0 = gr.add_vff(1)

	##################################################
	# Connections
	##################################################
	self.connect((self.gr_multiply_xx_0, 0), (self.gr_interp_fir_filter_xxx_0, 0))
	self.connect((self.gr_delay_0, 0), (self.gr_complex_to_mag_squared_0, 0))
	self.connect((self.gr_complex_to_mag_squared_0, 0), (self.gr_interp_fir_filter_xxx_1, 0))
	self.connect((self.gr_interp_fir_filter_xxx_1, 0), (self.gr_add_xx_0, 0))
	self.connect((self.gr_complex_to_mag_squared_1, 0), (self.gr_interp_fir_filter_xxx_2, 0))
	self.connect((self.gr_interp_fir_filter_xxx_2, 0), (self.gr_add_xx_0, 1))
	self.connect((self.gr_delay_0, 0), (self.gr_conjugate_cc_0, 0))
	self.connect((self.gr_conjugate_cc_0, 0), (self.gr_multiply_xx_0, 1))
	self.connect((self.gr_interp_fir_filter_xxx_0, 0), (self.gr_complex_to_mag_0, 0))
	self.connect((self.gr_complex_to_mag_0, 0), (self.gr_multiply_const_vxx_0, 0))
	self.connect((self.gr_complex_to_arg_0, 0), (self.gr_sample_and_hold_xx_0, 0))
	self.connect((self.gr_add_xx_0, 0), (self.gr_sub_xx_0, 0))
	self.connect((self.gr_multiply_const_vxx_0, 0), (self.gr_sub_xx_0, 1))
	self.connect((self.gr_sub_xx_0, 0), (self.gr_multiply_const_vxx_1, 0))
	self.connect((self.gr_multiply_const_vxx_1, 0), (self.gr_peak_detector_xb_0, 0))
	self.connect((self.gr_peak_detector_xb_0, 0), (self.gr_sample_and_hold_xx_0, 1))
	
	self.connect((self.gr_multiply_xx_1, 0), (self.gr_interp_fir_filter_xxx_3, 0))
	self.connect((self.gr_delay_1, 0), (self.gr_complex_to_mag_squared_2, 0))
	self.connect((self.gr_complex_to_mag_squared_2, 0), (self.gr_interp_fir_filter_xxx_4, 0))
	self.connect((self.gr_interp_fir_filter_xxx_4, 0), (self.gr_add_xx_1, 0))
	self.connect((self.gr_complex_to_mag_squared_3, 0), (self.gr_interp_fir_filter_xxx_5, 0))
	self.connect((self.gr_interp_fir_filter_xxx_5, 0), (self.gr_add_xx_1, 1))
	self.connect((self.gr_add_xx_1, 0), (self.gr_sub_xx_1, 0))
	self.connect((self.gr_interp_fir_filter_xxx_3, 0), (self.gr_complex_to_mag_1, 0))
	self.connect((self.gr_complex_to_mag_1, 0), (self.gr_multiply_const_vxx_2, 0))
	self.connect((self.gr_multiply_const_vxx_2, 0), (self.gr_sub_xx_1, 1))
	self.connect((self.gr_sub_xx_1, 0), (self.gr_multiply_const_vxx_3, 0))
	self.connect((self.gr_multiply_const_vxx_3, 0), (self.gr_peak_detector_xb_1, 0))
	
	self.connect((self.gr_interp_fir_filter_xxx_0, 0), (self.gr_add_xx_2, 0))
	self.connect((self.gr_interp_fir_filter_xxx_3, 0), (self.gr_add_xx_2, 1))
	
	self.connect((self.gr_add_xx_2, 0), (self.gr_complex_to_arg_0, 0))
	self.connect((self.gr_peak_detector_xb_1, 0), (self.gr_sample_and_hold_xx_1, 1))
	self.connect((self.gr_complex_to_arg_0, 0), (self.gr_sample_and_hold_xx_1, 0))
	self.connect((self.gr_delay_1, 0), (self.gr_conjugate_cc_1, 0))
	self.connect((self.gr_conjugate_cc_1, 0), (self.gr_multiply_xx_1, 1))
	self.connect((self.input_0, 0), (self.gr_complex_to_mag_squared_1, 0))
	self.connect((self.input_0, 0), (self.gr_delay_0, 0))
	self.connect((self.input_0, 0), (self.gr_multiply_xx_0, 0))
	self.connect((self.input_1, 0), (self.gr_complex_to_mag_squared_3, 0))
	self.connect((self.input_1, 0), (self.gr_delay_1, 0))
	self.connect((self.input_1, 0), (self.gr_multiply_xx_1, 0))
        
        self.connect((self,0), self.input_0)
        self.connect((self,1), self.input_1)
        
       
        
	
        # Set output signals
        #    Output 0: fine frequency correction value
        #    Output 1: timing signal
        self.connect(self.gr_sample_and_hold_xx_0, (self,0))
        self.connect(self.gr_sample_and_hold_xx_1, (self,1))
        self.connect(self.gr_peak_detector_xb_0, (self,2))

        
