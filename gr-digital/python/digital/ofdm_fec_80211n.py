#!/usr/bin/env python
#
# Copyright 2010 Free Software Foundation, Inc.
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

import math, sys, time
import fftw3
import psk, qam

from gnuradio import gr
import digital_swig
import gnuradio.gr.gr_threading as _threading
from gnuradio import trellis
from datetime import datetime

# from current directory
import ofdm_fec_packet_utils
import ofdm_packet_utils
from ofdm_receiver_fec import ofdm_receiver_fec
from ofdm_receiver_fec_80211n import ofdm_receiver_fec_80211n
from ofdm_receiver_fec_alamouti import ofdm_receiver_fec_alamouti
from ofdm_receiver_fec_multiplexing_80211n import ofdm_receiver_fec_multiplexing_80211n

ksfreq0_short = [0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j]
ksfreq1_short = [0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j]

ksfreq0_short_nozeros = [0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j]
ksfreq1_short_nozeros = [0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, -1.6330-1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 1.6330+1.6330j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j, 0.0+0.0j]

ksfreq_ant0_long1=[0, 0, 0, 0, 1.0690+0.0000j, 1.0690+0.0000j, 1.0690+0.0000j, 1.0690-0.0000j, -1.0690-0.0000j, -1.0690+0.0000j, 1.0690-0.0000j, 1.0690-0.0000j, -1.0690+0.0000j, 1.0690-0.0000j, -1.0690-0.0000j, 1.0690-0.0000j, 1.0690-0.0000j, 1.0690+0.0000j, 1.0690+0.0000j, 1.0690+0.0000j, 1.0690-0.0000j, -1.0690-0.0000j, -1.0690+0.0000j, 1.0690+0.0000j, 1.0690+0.0000j, -1.0690-0.0000j, 1.0690+0.0000j, -1.0690+0.0000j, 1.0690+0.0000j, 1.0690+0.0000j, 1.0690-0.0000j, 1.0690-0.0000j, 0, 1.0690-0.0000j, -1.0690+0.0000j, -1.0690+0.0000j, 1.0690+0.0000j, 1.0690-0.0000j, -1.0690-0.0000j, 1.0690-0.0000j, -1.0690-0.0000j, 1.0690+0.0000j, -1.0690-0.0000j, -1.0690-0.0000j, -1.0690-0.0000j, -1.0690+0.0000j, -1.0690-0.0000j, 1.0690-0.0000j, 1.0690-0.0000j, -1.0690-0.0000j, -1.0690-0.0000j, 1.0690-0.0000j, -1.0690-0.0000j, 1.0690-0.0000j, -1.0690-0.0000j, 1.0690+0.0000j, 1.0690+0.0000j, 1.0690-0.0000j, 1.0690-0.0000j, -1.0690+0.0000j, -1.0690-0.0000j, 0, 0, 0]

ksfreq_ant0_long2=ksfreq_ant0_long1

ksfreq_ant0_long3=[0, 0, 0, 0, 1.0690+0.0000j, 1.0690+0.0000j, 1.0690+0.0000j, 1.0690-0.0000j, -1.0690-0.0000j, -1.0690+0.0000j, 1.0690-0.0000j, 1.0690-0.0000j, -1.0690+0.0000j, 1.0690-0.0000j, -1.0690-0.0000j, 1.0690-0.0000j, 1.0690-0.0000j, 1.0690+0.0000j, 1.0690+0.0000j, 1.0690+0.0000j, 1.0690-0.0000j, -1.0690-0.0000j, -1.0690+0.0000j, 1.0690+0.0000j, 1.0690+0.0000j, -1.0690-0.0000j, 1.0690+0.0000j, -1.0690+0.0000j, 1.0690+0.0000j, 1.0690+0.0000j, 1.0690-0.0000j, 1.0690-0.0000j, 0, 1.0690-0.0000j, -1.0690+0.0000j, -1.0690+0.0000j, 1.0690+0.0000j, 1.0690-0.0000j, -1.0690-0.0000j, 1.0690-0.0000j, -1.0690-0.0000j, 1.0690+0.0000j, -1.0690-0.0000j, -1.0690-0.0000j, -1.0690-0.0000j, -1.0690+0.0000j, -1.0690-0.0000j, 1.0690-0.0000j, 1.0690-0.0000j, -1.0690-0.0000j, -1.0690-0.0000j, 1.0690-0.0000j, -1.0690-0.0000j, 1.0690-0.0000j, -1.0690-0.0000j, 1.0690+0.0000j, 1.0690+0.0000j, 1.0690-0.0000j, 1.0690-0.0000j, -1.0690+0.0000j, -1.0690-0.0000j, 0, 0, 0]

ksfreq_ant1_long1=[ 0, 0, 0,  0, 1.0690+0.0000j, 0.7559+0.7559j,  -0.0000+1.0690j,  -0.7559+0.7559j,  1.0690-0.0000j , 0.7559+0.7559j,  -0.0000-1.0690j, 0.7559-0.7559j,  -1.0690+0.0000j, 0.7559+0.7559j, 0.0000-1.0690j,  -0.7559+0.7559j,  -1.0690+0.0000j,  -0.7559-0.7559j, 0.0000-1.0690j, 0.7559-0.7559j, 1.0690-0.0000j,  -0.7559-0.7559j,  -0.0000-1.0690j,  -0.7559+0.7559j,  -1.0690-0.0000j, 0.7559+0.7559j, 0.0000-1.0690j,  -0.7559+0.7559j, 1.0690+0.0000j, 0.7559+0.7559j, 0.0000+1.0690j,  -0.7559+0.7559j,  0,  -0.7559-0.7559j, 0.0000+1.0690j,  -0.7559+0.7559j, 1.0690+0.0000j, 0.7559+0.7559j, 0.0000-1.0690j,  -0.7559+0.7559j,  1.0690-0.0000j,  -0.7559-0.7559j,  -0.0000+1.0690j,  -0.7559+0.7559j,  -1.0690-0.0000j,  -0.7559-0.7559j, 0.0000-1.0690j,  -0.7559+0.7559j,  -1.0690+0.0000j, 0.7559+0.7559j,  -0.0000+1.0690j, 0.7559-0.7559j,  -1.0690-0.0000j, 0.7559+0.7559j, 0.0000-1.0690j,  -0.7559+0.7559j,  -1.0690-0.0000j,  -0.7559-0.7559j,  -0.0000-1.0690j,  -0.7559+0.7559j,  -1.0690-0.0000j,  0,  0,  0]

ksfreq_ant1_long2=ksfreq_ant1_long1

ksfreq_ant1_long3=[0, 0, 0, 0, -1.0690-0.0000j, -0.7559-0.7559j, 0.0000-1.0690j, 0.7559-0.7559j, -1.0690-0.0000j, -0.7559-0.7559j, 0.0000+1.0690j, -0.7559+0.7559j, 1.0690-0.0000j, -0.7559-0.7559j, -0.0000+1.0690j, 0.7559-0.7559j, 1.0690-0.0000j, 0.7559+0.7559j, -0.0000+1.0690j, -0.7559+0.7559j, -1.0690+0.0000j, 0.7559+0.7559j, 0.0000+1.0690j, 0.7559-0.7559j, 1.0690+0.0000j, -0.7559-0.7559j, -0.0000+1.0690j, 0.7559-0.7559j, -1.0690-0.0000j, -0.7559-0.7559j, -0.0000-1.0690j, 0.7559-0.7559j, 0, 0.7559+0.7559j, -0.0000-1.0690j, 0.7559-0.7559j, -1.0690-0.0000j, -0.7559-0.7559j, -0.0000+1.0690j, 0.7559-0.7559j, -1.0690-0.0000j, 0.7559+0.7559j, 0.0000-1.0690j, 0.7559-0.7559j, 1.0690+0.0000j, 0.7559+0.7559j, -0.0000+1.0690j, 0.7559-0.7559j, 1.0690-0.0000j, -0.7559-0.7559j, 0.0000-1.0690j, -0.7559+0.7559j, 1.0690+0.0000j, -0.7559-0.7559j, -0.0000+1.0690j, 0.7559-0.7559j, 1.0690+0.0000j, 0.7559+0.7559j, 0.0000+1.0690j, 0.7559-0.7559j, 1.0690+0.0000j, 0, 0, 0]


ksfreq_empty = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
ksfreq_fake = [0,0,0,0,1,1,1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,0,1,-1,-1,1,1,-1,1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,1,-1,1,-1,1,1,1,1,-1,-1,0,0,0]
ksfreq_empty_nozeros = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
ksfreq_fake_nozeros = [1,1,1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,0,1,-1,-1,1,1,-1,1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,1,-1,1,-1,1,1,1,1,-1,-1]


VERBOSE = 0


# /////////////////////////////////////////////////////////////////////////////
#                   mod/demod with packets as i/o
# /////////////////////////////////////////////////////////////////////////////

class ofdm_mod_fec_80211n(gr.hier_block2):
    """
    Modulates an OFDM stream. Based on the options fft_length, occupied_tones, and
    cp_length, this block creates OFDM symbols using a specified modulation option.
    
    Send packets by calling send_pkt
    """
    def __init__(self, options, msgq_limit=2):
        """
	Hierarchical block for sending packets

        Packets to be sent are enqueued by calling send_pkt.
        The output is the complex modulated signal at baseband.

        @param options: pass modulation options from higher layers (fft length, occupied tones, etc.)
        @param msgq_limit: maximum number of messages in message queue
        @type msgq_limit: int
        """

        # A bit of a kluge to get around lack of variable io signatures currently
        if(options.tx_rx_type == "siso" or options.tx_rx_type == "mrc" or options.tx_rx_type == "egc" or options.tx_rx_type == "asc" or options.tx_rx_type == "miso"):
            gr.hier_block2.__init__(self, "ofdm_mod_fec_80211n",
                                    gr.io_signature(0, 0, 0),       # Input signature
                                    gr.io_signature(1, 1, gr.sizeof_gr_complex)) # Output signature
        elif(options.tx_rx_type == "alamouti1" or options.tx_rx_type == "alamouti2" or options.tx_rx_type == "multiplexing"):
            gr.hier_block2.__init__(self, "ofdm_mod_fec_80211n",
                                    gr.io_signature(0, 0, 0),       # Input signature
                                    gr.io_signature(2, 2, gr.sizeof_gr_complex)) # Output signature
	else:
	    sys.stderr.write("ofdm_mod_fec_80211n: tx_rx_type not recognized\n")
	    raise SystemExit  
	  
        self._modulation 	= options.modulation
        self._fft_length 	= options.fft_length
        self._occupied_tones 	= options.occupied_tones
        self._cp_length 	= options.cp_length
        self._pilot_length 	= options.pilot_length
        self._pilot_first 	= options.pilot_first
        self._pilot_spacing 	= options.pilot_spacing
        self._tx_rx_type 	= options.tx_rx_type
        self._ntx_antennas	= options.ntx_antennas
        self._nrx_antennas	= options.nrx_antennas
        
        self._fsm_filein	= options.fsm_filein
	self._fsm_fileout	= options.fsm_fileout
	self._fec_type		= options.fec_type
	self._fsm_iterations	= options.fsm_iterations
	
	self._headeronoff	= options.headeronoff # True add header / False no header
        
        win = [] #[1 for i in range(self._fft_length)]

        zeros_on_left = int(math.ceil((self._fft_length - (self._occupied_tones+self._pilot_length))/2.0))
        zeros_on_right = self._fft_length - self._occupied_tones - self._pilot_length - zeros_on_left

        ksfreq_sig1 = known_symbols_4512_3[0:(self._fft_length)]        
        for i in range(0,zeros_on_left):
	    ksfreq_sig1[i] = 0
	for i in range((self._fft_length-zeros_on_left+1),self._fft_length):
	    ksfreq_sig1[i] = 0

        ksfreq_sig2 = known_symbols_4512_3[(self._fft_length):(self._fft_length*2)]        
        for i in range(0,zeros_on_left):
	    ksfreq_sig2[i] = 0
	for i in range((self._fft_length-zeros_on_left+1),self._fft_length):
	    ksfreq_sig2[i] = 0

	ksfreq_longfake = known_symbols_4512_3[(self._fft_length*2):(self._fft_length*3)]        
        for i in range(0,zeros_on_left):
	    ksfreq_longfake[i] = 0
	for i in range((self._fft_length-zeros_on_left+1),self._fft_length):
	    ksfreq_longfake[i] = 0
        
        if(self._tx_rx_type == "siso" or self._tx_rx_type == "mrc" or self._tx_rx_type == "egc" or self._tx_rx_type == "asc" or self._tx_rx_type == "miso" ):

            # hard-coded known symbols
            preambles_short1 = (ksfreq0_short,)
            preambles_short2 = (ksfreq0_short,)
            
            if VERBOSE:
	      print "ofdm_mod_fec_80211n: len(ksfreq0_short) ", len(ksfreq0_short)
	      print "ofdm_mod_fec_80211n: preambles_short1 ", preambles_short1
	      print "ofdm_mod_fec_80211n: preambles_short2 ", preambles_short2
            
            # hard-coded known symbols
            #preambles_sig1 = (ksfreq_sig,)
            #preambles_sig2 = (ksfreq_sig,)
            preambles_sig1 = (ksfreq_sig1,)
            preambles_sig2 = (ksfreq_sig2,)
            
            if VERBOSE:
	      print "ofdm_mod_fec_80211n: len(ksfreq_sig1) ", len(ksfreq_sig1)
	      print "ofdm_mod_fec_80211n: preambles_sig1 ", preambles_sig1
	      print "ofdm_mod_fec_80211n: preambles_sig2 ", preambles_sig2
            
            # hard-coded known symbols
            preambles_long1 = (ksfreq_ant0_long1,)
            preambles_long2 = (ksfreq_ant0_long2,)
            preambles_long3 = (ksfreq_longfake,)
                        
            if VERBOSE:
	      print "ofdm_mod_fec_80211n: len(preambles_long1) ", len(ksfreq_ant0_long1)
	      print "ofdm_mod_fec_80211n: preambles_long1 ", preambles_long1
	      print "ofdm_mod_fec_80211n: len(preambles_long2) ", len(ksfreq_ant0_long2)
	      print "ofdm_mod_fec_80211n: preambles_long2 ", preambles_long2
	      print "ofdm_mod_fec_80211n: len(preambles_long3) ", len(ksfreq_longfake)
	      print "ofdm_mod_fec_80211n: preambles_long3 (fake)", preambles_long3

	elif( options.tx_rx_type == "multiplexing"):

            # hard-coded known symbols
            preambles_ant0_short1 = (ksfreq0_short,)
            preambles_ant0_short2 = (ksfreq0_short,)    

            if VERBOSE:
	      print "ofdm_mod_fec_80211n: len(ksfreq0_short) ", len(ksfreq0_short)
	      print "ofdm_mod_fec_80211n: preambles_ant0_short1 ", preambles_ant0_short1
	      print "ofdm_mod_fec_80211n: preambles_ant0_short2 ", preambles_ant0_short2
            
            preambles_ant1_short1 = (ksfreq1_short,)
	    preambles_ant1_short2 = (ksfreq1_short,)
	    
            if VERBOSE:
	      print "ofdm_mod_fec_80211n: len(ksfreq1_short) ", len(ksfreq1_short)
	      print "ofdm_mod_fec_80211n: preambles_ant1_short1 ", preambles_ant1_short1
	      print "ofdm_mod_fec_80211n: preambles_ant1_short2 ", preambles_ant1_short2
            
            # hard-coded known symbols
            preambles_ant0_sig1 = (ksfreq_sig1,)
            preambles_ant0_sig2 = (ksfreq_sig2,)
                       
            if VERBOSE:
	      print "ofdm_mod_fec_80211n: len(ksfreq_sig) ", len(ksfreq_sig1)
	      print "ofdm_mod_fec_80211n: preambles_ant0_sig1 ", preambles_ant0_sig1
	      print "ofdm_mod_fec_80211n: preambles_ant0_sig2 ", preambles_ant0_sig2
            
            preambles_ant1_sig1 = (ksfreq_sig1,)
            preambles_ant1_sig2 = (ksfreq_sig2,)
           
            if VERBOSE:
	      print "ofdm_mod_fec_80211n: len(ksfreq_sig) ", len(ksfreq_sig1)
	      print "ofdm_mod_fec_80211n: preambles_ant1_sig1 ", preambles_ant1_sig1
	      print "ofdm_mod_fec_80211n: preambles_ant1_sig2 ", preambles_ant1_sig2
         
            # hard-coded known symbols
            preambles_ant0_long1 = (ksfreq_ant0_long1,)
            preambles_ant0_long2 = (ksfreq_ant0_long2,)
            preambles_ant0_long3 = (ksfreq_ant0_long3,)
            if VERBOSE:
	      print "ofdm_mod_fec_80211n: len(ksfreq_ant0_long1) ", len(ksfreq_ant0_long1)
	      print "ofdm_mod_fec_80211n: len(ksfreq_ant0_long2) ", len(ksfreq_ant0_long2)
	      print "ofdm_mod_fec_80211n: len(ksfreq_ant0_long3) ", len(ksfreq_ant0_long3)
	      print "ofdm_mod_fec_80211n: preambles_ant0_long1 ", preambles_ant0_long1
	      print "ofdm_mod_fec_80211n: preambles_ant0_long2 ", preambles_ant0_long2
	      print "ofdm_mod_fec_80211n: preambles_ant0_long3 ", preambles_ant0_long3
            
            preambles_ant1_long1 = (ksfreq_ant1_long1,)
            preambles_ant1_long2 = (ksfreq_ant1_long2,)
            preambles_ant1_long3 = (ksfreq_ant1_long3,)
            
            if VERBOSE:
	      print "ofdm_mod_fec_80211n: len(ksfreq_ant1_long1) ", len(ksfreq_ant1_long1)
	      print "ofdm_mod_fec_80211n: len(ksfreq_ant1_long2) ", len(ksfreq_ant1_long2)
	      print "ofdm_mod_fec_80211n: len(ksfreq_ant1_long3) ", len(ksfreq_ant1_long3)
	      print "ofdm_mod_fec_80211n: preambles_ant1_long1 ", preambles_ant1_long1
	      print "ofdm_mod_fec_80211n: preambles_ant1_long2 ", preambles_ant1_long2
	      print "ofdm_mod_fec_80211n: preambles_ant1_long3 ", preambles_ant1_long3
            
	elif (self._tx_rx_type == "alamouti1" or self._tx_rx_type == "alamouti2"):
	    sys.stderr.write("ofdm_fec_mod_80211n: alamouti not implemented\n")
	    raise SystemExit  	  
	  
	else:
	    sys.stderr.write("ofdm_fec mod_80211n: tx_rx_type not recognized\n")
	    raise SystemExit  
	  
        symbol_length = options.fft_length + options.cp_length
        
        mods = {"bpsk": 2, "qpsk": 4, "8psk": 8, "qam8": 8, "qam16": 16, "qam64": 64, "qam256": 256}
        arity = mods[self._modulation]
        
        rot = 1
        if self._modulation == "qpsk":
            rot = (0.707+0.707j)
            
        if(self._modulation.find("psk") >= 0):
            constel = psk.psk_constellation(arity)
            rotated_const = map(lambda pt: pt * rot, constel.points())
        elif(self._modulation.find("qam") >= 0):
            constel = qam.qam_constellation(arity)
            rotated_const = map(lambda pt: pt * rot, constel.points())    

	# CHAINS and CONNECTS
        if(options.tx_rx_type == "siso" or options.tx_rx_type == "mrc" or options.tx_rx_type == "egc" or options.tx_rx_type == "asc" or options.tx_rx_type == "miso"):
	  
	    # pilot
	    pilot = known_symbols_4512_3[0:self._pilot_length]
	    for i in range(self._pilot_length):
	      pilot[i] = 1
	      
	    self._pkt_input = digital_swig.ofdm_mapper_bcv(rotated_const, msgq_limit,
                                             options.occupied_tones, options.fft_length)
            self.ant0_preambles_short1 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_short1)
            self.ant0_preambles_short2 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_short2)
            self.ant0_preambles_sig1 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_sig1)
            self.ant0_preambles_sig2 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_sig2)
            self.ant0_preambles_long1 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_long1)
            self.ant0_preambles_long2 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_long2)
            self.ant0_preambles_long3 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_long3)
            self.ifft = gr.fft_vcc(self._fft_length, False, win, True)
            self.scale = gr.multiply_const_cc(1.0 / math.sqrt(self._fft_length))
            
            # add pilot
            self.pilot = digital_swig.ofdm_insert_pilot_80211n(self._fft_length, zeros_on_left, zeros_on_right, \
								pilot,self._pilot_first, self._pilot_spacing, \
								self._ntx_antennas)
	    	    
	    short1_timedomain=[3.266+3.266j, -9.40376+0.166115j, -0.956589-5.57541j, 10.1359-0.898257j, 6.532+0j, 10.1359-0.898257j, -0.956589-5.57541j, -9.40376+0.166115j, 3.266+3.266j, 0.166115-9.40376j, -5.57541-0.956589j, -0.898257+10.1359j, 0+6.532j, -0.898257+10.1359j, -5.57541-0.956589j, 0.166115-9.40376j, 3.266+3.266j, -9.40376+0.166115j, -0.956589-5.57541j, 10.1359-0.898257j, 6.532+0j, 10.1359-0.898257j, -0.956589-5.57541j, -9.40376+0.166115j, 3.266+3.266j, 0.166115-9.40376j, -5.57541-0.956589j, -0.898257+10.1359j, 0+6.532j, -0.898257+10.1359j, -5.57541-0.956589j, 0.166115-9.40376j, 3.266+3.266j, -9.40376+0.166115j, -0.956589-5.57541j, 10.1359-0.898257j, 6.532+0j, 10.1359-0.898257j, -0.956589-5.57541j, -9.40376+0.166115j, 3.266+3.266j, 0.166115-9.40376j, -5.57541-0.956589j, -0.898257+10.1359j, 0+6.532j, -0.898257+10.1359j, -5.57541-0.956589j, 0.166115-9.40376j, 3.266+3.266j, -9.40376+0.166115j, -0.956589-5.57541j, 10.1359-0.898257j, 6.532+0j, 10.1359-0.898257j, -0.956589-5.57541j, -9.40376+0.166115j, 3.266+3.266j, 0.166115-9.40376j, -5.57541-0.956589j, -0.898257+10.1359j, 0+6.532j, -0.898257+10.1359j, -5.57541-0.956589j, 0.166115-9.40376j]
	    preambles_short1_timedomain = (short1_timedomain,)
	    
	    if VERBOSE:
	      print "short1_timedomain ",short1_timedomain
	      print "len(short1_timedomain) ", len(short1_timedomain)
	      
	    # recognize first short to count and then modify the first and second long to put the BIG CP
	    self.cyclic_prefixer = digital_swig.ofdm_cyclic_prefixer_80211n(self._fft_length, self._cp_length, symbol_length, 0, preambles_short1_timedomain)
            
            self.connect((self._pkt_input, 0), (self.ant0_preambles_long3, 0), \
			  (self.ant0_preambles_sig2, 0), (self.ant0_preambles_sig1, 0), \
			  (self.ant0_preambles_long2, 0), (self.ant0_preambles_long1, 0), \
			  (self.ant0_preambles_short2, 0), (self.ant0_preambles_short1, 0))
            self.connect((self._pkt_input, 1), (self.ant0_preambles_long3, 1), \
			  (self.ant0_preambles_sig2, 1), (self.ant0_preambles_sig1, 1), \
			  (self.ant0_preambles_long2, 1), (self.ant0_preambles_long1, 1), \
			  (self.ant0_preambles_short2, 1), (self.ant0_preambles_short1, 1))
            
            self.connect((self.ant0_preambles_short1, 0), (self.pilot, 0))
            self.connect((self.ant0_preambles_short1, 1), (self.pilot, 1))
            
            print "ofdm_mod_fec_80211n: pkt_input --> preambles --> pilot --> ifft --> cyclic_prefixer --> scale --> USRP"
            self.connect((self.pilot,0), self.ifft, self.cyclic_prefixer, self.scale, self)
                        
            if options.log:
                self.connect(self._pkt_input, gr.file_sink(gr.sizeof_gr_complex*options.fft_length,
                                                           "ofdm_mimo-mapper_c.dat"))
                self.connect(self.preambles, gr.file_sink(gr.sizeof_gr_complex*options.fft_length,
                                                          "ofdm_mimo-preambles.dat"))
                self.connect(self.ifft, gr.file_sink(gr.sizeof_gr_complex*options.fft_length,
                                                     "ofdm_mimo-ifft_c.dat"))
                self.connect(self.cp_adder, gr.file_sink(gr.sizeof_gr_complex,
                                                         "ofdm_mimo-cp_adder_c.dat"))
                
        elif (options.tx_rx_type == "alamouti1"  or options.tx_rx_type == "alamouti2" ):

	    sys.stderr.write("ofdm_fec_mod_80211n: alamouti not implemented\n")
	    raise SystemExit  	  

	elif (self._tx_rx_type == "multiplexing"):
	    print "ofdm_mod_fec_80211n: SPATIAL MULTIPLEXING"
	    # pilot0
	    pilot0 = known_symbols_4512_3[0:self._pilot_length]
	    for i in range(self._pilot_length):
	      pilot0[i] = 1
    	    # pilot1
	    pilot1 = known_symbols_4512_3[0:self._pilot_length]
	    for i in range(self._pilot_length):
	      if i%2 == 0:
		pilot1[i] = 1
	      else:
		pilot1[i] = -1
	    
	    self._pkt_input0 = digital_swig.ofdm_mapper_bcv(rotated_const, msgq_limit,
                                             options.occupied_tones, options.fft_length)
	    self._pkt_input1 = digital_swig.ofdm_mapper_bcv(rotated_const, msgq_limit,
                                             options.occupied_tones, options.fft_length)
	    
            self.pilot0 = digital_swig.ofdm_insert_pilot_80211n(self._fft_length, zeros_on_left, zeros_on_right, \
								pilot0,self._pilot_first, self._pilot_spacing, \
								self._nrx_antennas)
            self.pilot1 = digital_swig.ofdm_insert_pilot_80211n(self._fft_length, zeros_on_left, zeros_on_right, \
								pilot1,self._pilot_first, self._pilot_spacing, \
								self._nrx_antennas)
								
	  
            # antenna 0
            self.ant0_preamble_short1 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_ant0_short1)
            self.ant0_preamble_short2 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_ant0_short2)
            self.ant0_preamble_sig1 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_ant0_sig1)
            self.ant0_preamble_sig2 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_ant0_sig2)
            self.ant0_preamble_long1 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_ant0_long1)
            self.ant0_preamble_long2 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_ant0_long2)
            self.ant0_preamble_long3 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_ant0_long3)
            
            # antenna 1
            self.ant1_preamble_short1 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_ant1_short1)
            self.ant1_preamble_short2 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_ant1_short2)
            self.ant1_preamble_sig1 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_ant1_sig1)
            self.ant1_preamble_sig2 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_ant1_sig2)
            self.ant1_preamble_long1 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_ant1_long1)
            self.ant1_preamble_long2 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_ant1_long2)
            self.ant1_preamble_long3 = digital_swig.ofdm_insert_preamble(self._fft_length, preambles_ant1_long3)
            
            self.ifft0 = gr.fft_vcc(self._fft_length, False, win, True)
            self.ifft1 = gr.fft_vcc(self._fft_length, False, win, True)
           
            self.scale0 = gr.multiply_const_cc(1.0 / math.sqrt(self._fft_length))
            self.scale1 = gr.multiply_const_cc(1.0 / math.sqrt(self._fft_length))
            
	    ant0_short1_timedomain=[3.266+3.266j, -9.40376+0.166115j, -0.956589-5.57541j, 10.1359-0.898257j, 6.532+0j, 10.1359-0.898257j, -0.956589-5.57541j, -9.40376+0.166115j, 3.266+3.266j, 0.166115-9.40376j, -5.57541-0.956589j, -0.898257+10.1359j, 0+6.532j, -0.898257+10.1359j, -5.57541-0.956589j, 0.166115-9.40376j, 3.266+3.266j, -9.40376+0.166115j, -0.956589-5.57541j, 10.1359-0.898257j, 6.532+0j, 10.1359-0.898257j, -0.956589-5.57541j, -9.40376+0.166115j, 3.266+3.266j, 0.166115-9.40376j, -5.57541-0.956589j, -0.898257+10.1359j, 0+6.532j, -0.898257+10.1359j, -5.57541-0.956589j, 0.166115-9.40376j, 3.266+3.266j, -9.40376+0.166115j, -0.956589-5.57541j, 10.1359-0.898257j, 6.532+0j, 10.1359-0.898257j, -0.956589-5.57541j, -9.40376+0.166115j, 3.266+3.266j, 0.166115-9.40376j, -5.57541-0.956589j, -0.898257+10.1359j, 0+6.532j, -0.898257+10.1359j, -5.57541-0.956589j, 0.166115-9.40376j, 3.266+3.266j, -9.40376+0.166115j, -0.956589-5.57541j, 10.1359-0.898257j, 6.532+0j, 10.1359-0.898257j, -0.956589-5.57541j, -9.40376+0.166115j, 3.266+3.266j, 0.166115-9.40376j, -5.57541-0.956589j, -0.898257+10.1359j, 0+6.532j, -0.898257+10.1359j, -5.57541-0.956589j, 0.166115-9.40376j]
	    ant0_preambles_short1_timedomain = (ant0_short1_timedomain,)

	    ant1_short1_timedomain=[3.266+3.266j, 0.166115-9.40376j, -5.57541-0.956589j, -0.898257+10.1359j, 0+6.532j, -0.898257+10.1359j, -5.57541-0.956589j, 0.166115-9.40376j, 3.266+3.266j, -9.40376+0.166115j, -0.956589-5.57541j, 10.1359-0.898257j, 6.532+0j, 10.1359-0.898257j, -0.956589-5.57541j, -9.40376+0.166115j, 3.266+3.266j, 0.166115-9.40376j, -5.57541-0.956589j, -0.898257+10.1359j, 0+6.532j, -0.898257+10.1359j, -5.57541-0.956589j, 0.166115-9.40376j, 3.266+3.266j, -9.40376+0.166115j, -0.956589-5.57541j, 10.1359-0.898257j, 6.532+0j, 10.1359-0.898257j, -0.956589-5.57541j, -9.40376+0.166115j, 3.266+3.266j, 0.166115-9.40376j, -5.57541-0.956589j, -0.898257+10.1359j, 0+6.532j, -0.898257+10.1359j, -5.57541-0.956589j, 0.166115-9.40376j, 3.266+3.266j, -9.40376+0.166115j, -0.956589-5.57541j, 10.1359-0.898257j, 6.532+0j, 10.1359-0.898257j, -0.956589-5.57541j, -9.40376+0.166115j, 3.266+3.266j, 0.166115-9.40376j, -5.57541-0.956589j, -0.898257+10.1359j, 0+6.532j, -0.898257+10.1359j, -5.57541-0.956589j, 0.166115-9.40376j, 3.266+3.266j, -9.40376+0.166115j, -0.956589-5.57541j, 10.1359-0.898257j, 6.532+0j, 10.1359-0.898257j, -0.956589-5.57541j, -9.40376+0.166115j]
	    ant1_preambles_short1_timedomain = (ant1_short1_timedomain,)
	    
	    
	    if VERBOSE:
	      print "ant0_short1_timedomain ",ant0_short1_timedomain
	      print "len(ant0_short1_timedomain) ", len(ant0_short1_timedomain)
	      print "ant1_short1_timedomain ",ant1_short1_timedomain
	      print "len(ant1_short1_timedomain) ", len(ant1_short1_timedomain)      
	    
	    self.ant0_cyclic_prefixer = digital_swig.ofdm_cyclic_prefixer_80211n(self._fft_length, self._cp_length, symbol_length, 0, ant0_preambles_short1_timedomain)
	    self.ant1_cyclic_prefixer = digital_swig.ofdm_cyclic_prefixer_80211n(self._fft_length, self._cp_length, symbol_length, 1, ant1_preambles_short1_timedomain)
            
            self.connect((self._pkt_input0, 0), \
			  (self.ant0_preamble_long3, 0), \
			  (self.ant0_preamble_sig2, 0), (self.ant0_preamble_sig1, 0), \
			  (self.ant0_preamble_long2, 0), (self.ant0_preamble_long1, 0), \
			  (self.ant0_preamble_short2, 0), (self.ant0_preamble_short1, 0))
            self.connect((self._pkt_input0, 1), \
			  (self.ant0_preamble_long3, 1), \
			  (self.ant0_preamble_sig2, 1), (self.ant0_preamble_sig1, 1), \
			  (self.ant0_preamble_long2, 1), (self.ant0_preamble_long1, 1), \
			  (self.ant0_preamble_short2, 1), (self.ant0_preamble_short1, 1))
            
            self.connect((self._pkt_input1, 0), \
			  (self.ant1_preamble_long3, 0), \
			  (self.ant1_preamble_sig2, 0), (self.ant1_preamble_sig1, 0), \
			  (self.ant1_preamble_long2, 0), (self.ant1_preamble_long1, 0), \
			  (self.ant1_preamble_short2, 0), (self.ant1_preamble_short1, 0))
            self.connect((self._pkt_input1, 1), \
			  (self.ant1_preamble_long3, 1), \
			  (self.ant1_preamble_sig2, 1), (self.ant1_preamble_sig1, 1), \
			  (self.ant1_preamble_long2, 1), (self.ant1_preamble_long1, 1), \
			  (self.ant1_preamble_short2, 1), (self.ant1_preamble_short1, 1))
            
            self.connect((self.ant0_preamble_short1, 0), (self.pilot0, 0))
            self.connect((self.ant0_preamble_short1, 1), (self.pilot0, 1))
            
            self.connect((self.ant1_preamble_short1, 0), (self.pilot1, 0))
            self.connect((self.ant1_preamble_short1, 1), (self.pilot1, 1))
            
            print "ofdm_mod_fec_80211n: ANT0 pkt_input --> preambles --> pilot --> ifft --> cyclic_prefixer --> scale --> USRP"
            print "ofdm_mod_fec_80211n: ANT1 pkt_input --> preambles --> pilot --> ifft --> cyclic_prefixer --> scale --> USRP"
            self.connect((self.pilot0,0), self.ifft0, self.ant0_cyclic_prefixer, self.scale0, (self,0))
            self.connect((self.pilot1,0), self.ifft1, self.ant1_cyclic_prefixer, self.scale1, (self,1))
            
        else:
	    sys.stderr.write("ofdm fec mod_80211n: tx_rx_type not recognized ")
	    print " %s\n" %(self._tx_rx_type)
	    raise SystemExit  

        if options.verbose:
            self._print_verbage()


    def send_pkt(self, payload='', eof=False):
        """
        Send the payload.

        @param payload: data to send
        @type payload: string
        """
        if eof:
            msg = gr.message(1) # tell self._pkt_input we're not sending any more packets
        else:
	    #ATTENTION: for large packet sizes performance of make_packet function depends on the resources of the machine, it may not work
            if (self._fec_type == "NULL"):
		pkt = ofdm_packet_utils.make_packet(payload, 1, 1, self._headeronoff, whitening=True)
            else:
		pkt = ofdm_fec_packet_utils.make_packet(0,payload, self._fsm_filein, self._fsm_fileout, self._fec_type,1, 1, self._headeronoff, whitening=False)
		
            msg = gr.message_from_string(pkt)
        self._pkt_input.msgq().insert_tail(msg)

    # send separate streams for multiplexing
    def send_pkt_multiplexing(self, payload0='', payload1='', eof=False):
        """
        Send the payload.

        @param payload: data to send
        @type payload: string
        """
        if eof:
            msg0 = gr.message(1) # tell self._pkt_input we're not sending any more packets
            msg1 = gr.message(1)
        else:
	    #ATTENTION: for large packet sizes performance of make_packet function depends on the resources of the machine, it may not work
            if (self._fec_type == "NULL"):
		pkt0 = ofdm_packet_utils.make_packet(payload0, 1, 1, whitening=True)
		pkt1 = ofdm_packet_utils.make_packet(payload1, 1, 1, whitening=True)
            else:
		pkt0 = ofdm_fec_packet_utils.make_packet(0,payload0, self._fsm_filein, self._fsm_fileout, self._fec_type,1, 1, self._headeronoff, whitening=False)
		pkt1 = ofdm_fec_packet_utils.make_packet(1,payload1, self._fsm_filein, self._fsm_fileout, self._fec_type,1, 1, self._headeronoff, whitening=False)
		
            msg0 = gr.message_from_string(pkt0)
            msg1 = gr.message_from_string(pkt1)
            
        self._pkt_input0.msgq().insert_tail(msg0)
        self._pkt_input1.msgq().insert_tail(msg1)
        

    def add_options(normal, expert):
        """
        Adds OFDM-specific options to the Options Parser
        """
        normal.add_option("", "--modulation", type="string", default="bpsk",
                          help="set modulation type (bpsk, qpsk, 8psk, qam{16,64}) [default=%default]")
        expert.add_option("", "--fft-length", type="intx", default=512,
                          help="set the number of FFT bins [default=%default]")
        expert.add_option("", "--occupied-tones", type="intx", default=200,
                          help="set the number of occupied FFT bins [default=%default]")
        expert.add_option("", "--cp-length", type="intx", default=128,
                          help="set the number of bits in the cyclic prefix [default=%default]")
        expert.add_option("", "--pilot-length", type="intx", default=20,
                          help="set pilot length [default=%default]")
        expert.add_option("", "--pilot-spacing", type="intx", default=11,
                          help="set pilot spacing [default=%default]")
        expert.add_option("", "--pilot-first", type="intx", default=5,
                          help="set pilot first [default=%default]")
        expert.add_option("", "--tx-rx-type", default="mrc",
                          help="TX RX type [default=%default]")
        expert.add_option("", "--ntx-antennas", type="intx", default=1,
                          help="Number of TX antennas [default=%default]")
        expert.add_option("", "--nrx-antennas", type="intx", default=1,
                          help="Number of RX antennas [default=%default]")
                          
	# FEC parameters
	expert.add_option("", "--fsm-filein", type="string", default="/home/mimonet/GNURadio/gnuradio-august/fsm_files/awgn1o2_4.fsm",
			help="FEC filein [default=%default]")
	expert.add_option("", "--fsm-fileout", type="string", default="/home/mimonet/GNURadio/gnuradio-august/fsm_files/awgn1o2_4.fsm",
			help="FEC fileout [default=%default]")
	expert.add_option("", "--fec-type", type="string", default="viterbi",
			help="FEC type [default=%default]")
	expert.add_option("", "--fsm-iterations", type="intx", default=10,
			help="FSM iterations [default=%default]")
			
	expert.add_option("", "--headeronoff", type="string", default="True",
			help="Header On/Off [default=%default]")
 
    # Make a static method to call before instantiation
    add_options = staticmethod(add_options)

    def _print_verbage(self):
        """
        Prints information about the OFDM modulator
        """
        print "\nOFDM Modulator:"
        print "Modulation Type: %s"    % (self._modulation)
        print "FFT length:      %3d"   % (self._fft_length)
        print "Occupied Tones:  %3d"   % (self._occupied_tones)
        print "CP length:       %3d"   % (self._cp_length)
        print "FEC PARAMETERS"
        print "FSM filein:	%s" % (self._fsm_filein)
	print "FSM fileout:	%s" % (self._fsm_fileout)
	print "FSM fectype:	%s" % (self._fec_type)
	print "FSM iterations:	%d" % (self._fsm_iterations)
	print "Header on/off: 	%s" % (self._headeronoff)




class ofdm_demod_fec_80211n(gr.hier_block2):
    """
    Demodulates a received OFDM stream. Based on the options fft_length, occupied_tones, and
    cp_length, this block performs synchronization, FFT, and demodulation of incoming OFDM
    symbols and passes packets up the a higher layer.

    The input is complex baseband.  When packets are demodulated, they are passed to the
    app via the callback.
    """

    def __init__(self, options, callback=None):
        """
	Hierarchical block for demodulating and deframing packets.

	The input is the complex modulated signal at baseband.
        Demodulated packets are sent to the handler.

        This is the MIMO version and takes in a stream of interleave channels.
        These channels must first be deinterleaved into separate streams inside ofdm_mimo_receiver.
        This is to the fact that we cannot (yet) have a variable number of input streams to hier_block2s.

        @param options: pass modulation options from higher layers (fft length, occupied tones, etc.)
        @param callback:  function of two args: ok, payload
        @type callback: ok: bool; payload: string
	"""
	gr.hier_block2.__init__(self, "ofdm_demod_fec_80211n",
				gr.io_signature(1, 1, gr.sizeof_gr_complex), # Input signature
				gr.io_signature(1, 1, gr.sizeof_gr_complex)) # Output signature


        self._rcvd_pktq = gr.msg_queue()          # holds packets from the PHY

        self._modulation 	= options.modulation
        self._fft_length 	= options.fft_length
        self._occupied_tones 	= options.occupied_tones
        self._cp_length 	= options.cp_length
        self._pilot_length 	= options.pilot_length
        self._pilot_first 	= options.pilot_first
        self._pilot_spacing 	= options.pilot_spacing
        self._snr 		= options.snr
        self._tx_rx_type 	= options.tx_rx_type
        self._ofdm_sync_type	= options.ofdm_sync_type
	self._fsm_filein	= options.fsm_filein
	self._fsm_fileout	= options.fsm_fileout
	self._fec_type		= options.fec_type
	self._fsm_iterations	= options.fsm_iterations
	self._headeronoff	= options.headeronoff
	self._size		= options.size
	self._snr_type		= int(options.snr_type)
	self._d_samples		= int(options.d_samples)
	self._peakthr		= float(options.peakthr)
	self._file_truesnr	= options.file_truesnr
	self._file_truesnr1	= options.file_truesnr1
	self._downsampling	= int(options.downsampling)
	
	self._headeronoff	= options.headeronoff # True add header / False no header
	self._ntx_antennas	= options.ntx_antennas
        self._nrx_antennas	= options.nrx_antennas
	
	if (self._fec_type == "NULL"):
	    self.bits_after_fec_enc = 1
	    self.bits_after_fec_dec = 1
	    print "fec type= ", self._fec_type  
        elif (self._fec_type == "viterbi"):
	    f2 = open(self._fsm_fileout,'r')
	    firstlineout = f2.readline()
	    strlineout = firstlineout.split(" ")
	    tmp0 = int(strlineout[0],10) - 1
	    tmp2 = int(strlineout[2],10) - 1
	    self.bits_after_fec_enc = len(bin(tmp2)[2:])
	    self.bits_after_fec_dec = len(bin(tmp0)[2:])

	elif (self._fec_type=="sccc_soft" or self._fec_type=="sccc_hard" or self._fec_type == "sccc_turbo" or self._fec_type=="pccc"):
	    # file in
	    f1 = open(self._fsm_filein,'r')
	    firstlinein = f1.readline()
	    strlinein = firstlinein.split(" ")
	    tmp0 = int(strlinein[0],10) - 1
	    tmp2 = int(strlinein[2],10) - 1
	    fec_iner = float(len(bin(tmp2)[2:]))/float(len(bin(tmp0)[2:]))
	    fec_bits_iner = len(bin(tmp0)[2:])
	    self.bits_after_fec_enc = len(bin(tmp2)[2:])

	    # file out
	    f2 = open(self._fsm_fileout,'r')
	    firstlineout = f2.readline()
	    strlineout = firstlineout.split(" ")
	    tmp0 = int(strlineout[0],10) - 1
	    tmp2 = int(strlineout[2],10) - 1
	    fec_outer = float(len(bin(tmp2)[2:]))/float(len(bin(tmp0)[2:]))
	    fec_bits_outer = len(bin(tmp2)[2:])
	    self.bits_after_fec_dec = len(bin(tmp0)[2:])
	    
	    if ( self._fec_type == "pccc"):
		tmp = self.bits_after_fec_enc * fec_bits_outer 
		self.bits_after_fec_enc = tmp
		self.bits_after_fec_dec = 1
        
        
        zeros_on_left = int(math.ceil((self._fft_length - (self._occupied_tones+self._pilot_length))/2.0))
	
        symbol_length = self._fft_length + self._cp_length
        
        # pilot
        pilot = known_symbols_4512_3[0:self._pilot_length]
        for i in range(self._pilot_length):
	      pilot[i] = 1
        
        mods = {"bpsk": 2, "qpsk": 4, "8psk": 8, "qam8": 8, "qam16": 16, "qam64": 64, "qam256": 256}
        ratemods = {"bpsk": 1, "qpsk": 2, "8psk": 3}
        arity = mods[self._modulation]
        rate_mod = ratemods[self._modulation]
        rot = 1
        if self._modulation == "qpsk":
            rot = (0.707+0.707j)

        if(self._modulation.find("psk") >= 0):
            constel = psk.psk_constellation(arity)
            rotated_const = map(lambda pt: pt * rot, constel.points())
        elif(self._modulation.find("qam") >= 0):
            constel = qam.qam_constellation(arity)
            rotated_const = map(lambda pt: pt * rot, constel.points())    
        
        
        rate_encdec = (float(self.bits_after_fec_enc)/float(self.bits_after_fec_dec));
        
        if (self._headeronoff == "True"):
	  headeronoffint = 1
	else:
	  headeronoffint = 0
        
	if (options.tx_rx_type == "siso" or options.tx_rx_type == "mrc" or options.tx_rx_type == "egc" or options.tx_rx_type == "asc" or options.tx_rx_type == "miso"):

            # hard-coded known symbols
            preambles0 = (ksfreq0_short_nozeros,)
            preambles1 = (ksfreq0_short_nozeros,)
            
            if VERBOSE:
	      print "ofdm_demod_fec_80211n: len(ksfreq0_short_nozeros) ", len(ksfreq0_short_nozeros)
	      print "ofdm_demod_fec_80211n: preambles ", preambles0
	    
	    self.ofdm_recv = ofdm_receiver_fec_80211n(self._tx_rx_type, self._ntx_antennas, self._nrx_antennas, 
						self._ofdm_sync_type, self._downsampling, self._snr_type, self._d_samples, self._fft_length,
                                                self._cp_length, self._occupied_tones, self._pilot_first, self._pilot_spacing, 
                                                self._file_truesnr, rate_encdec, 
                                                rate_mod, self._size, headeronoffint, 
                                                self._snr, preambles0, preambles1, 
                                                ksfreq_ant0_long1, ksfreq_ant0_long2, ksfreq_ant0_long3, 
                                                pilot, self._peakthr, options.log)
                                                           
	elif (self._tx_rx_type == "alamouti1" or self._tx_rx_type == "alamouti2"):
	    sys.stderr.write("ofdm_demod_fec_80211n: alamouti not implemented")
	    print " %s\n" %(self._tx_rx_type)
	    raise SystemExit  

	elif (self._tx_rx_type == "multiplexing"):
	    print "ofdm_demod_fec_80211n: SPATIAL MULTIPLEXING"
	    # pilot0
	    pilot0 = known_symbols_4512_3[0:self._pilot_length]
	    for i in range(self._pilot_length):
	      pilot0[i] = 1
    	    # pilot1
	    pilot1 = known_symbols_4512_3[0:self._pilot_length]
	    for i in range(self._pilot_length):
	      if i%2 == 0:
		pilot1[i] = 1
	      else:
		pilot1[i] = -1

           # hard-coded known symbols
            preambles0 = (ksfreq0_short_nozeros,)
	    preambles1 = (ksfreq1_short_nozeros,)
	    
            if VERBOSE:
	      print "ofdm_demod_fec_80211n: len(ksfreq0) ", len(ksfreq0_short_nozeros)
	      print "ofdm_demod_fec_80211n: preambles0 ", preambles0
	      print "ofdm_demod_fec_80211n: len(ksfreq1) ", len(ksfreq1_short_nozeros)
	      print "ofdm_demod_fec_80211n: preambles1 ", preambles1
	    
	    self.ofdm_recv = ofdm_receiver_fec_multiplexing_80211n(self._tx_rx_type, self._ntx_antennas, self._nrx_antennas, 
						self._ofdm_sync_type, self._downsampling, self._snr_type, self._d_samples, self._fft_length,
                                                self._cp_length, self._occupied_tones, self._pilot_first, self._pilot_spacing, 
                                                self._file_truesnr, rate_encdec, 
                                                rate_mod, self._size, headeronoffint, 
                                                self._snr, preambles0, preambles1, 
                                                ksfreq_ant0_long1, ksfreq_ant0_long2, ksfreq_ant0_long3, 
                                                ksfreq_ant1_long1, ksfreq_ant1_long2, ksfreq_ant1_long3, 
                                                pilot0, pilot1, self._peakthr, options.log)

            
        else:
	    sys.stderr.write("ofdm_demod_fec_80211n: tx_rx_type not recognized ")
	    print " %s\n" %(self._tx_rx_type)
	    raise SystemExit  

        phgain = 0.25
        frgain = phgain*phgain / 4.0
        
        if (options.tx_rx_type == "siso" or options.tx_rx_type == "mrc" or options.tx_rx_type == "egc" 
	    or options.tx_rx_type == "asc" or options.tx_rx_type == "miso"):
	    if (self._headeronoff == "True"):
		sys.stderr.write("ofdm_demod_fec_80211n: (self._headeronoff == True) and secondshort is not correct ")
		print " %s\n" %(self._tx_rx_type)
		raise SystemExit  
	    else:
		self.ofdm_demod = digital_swig.ofdm_frame_sink_noheader_80211n(rotated_const, range(arity), self._file_truesnr,
                                             self._rcvd_pktq,
					     self._ntx_antennas, self._nrx_antennas, 
                                             self._occupied_tones, self._size, rate_encdec,
                                             phgain, frgain, 0)
		
        elif (options.tx_rx_type == "alamouti1" or options.tx_rx_type == "alamouti2"):
	    sys.stderr.write("ofdm_demod_fec_80211n: alamouti not implemented ")
	    raise SystemExit
                                             
        elif (options.tx_rx_type == "multiplexing"):
	    print "OFDM FEC: multiplexing creating sinks"
	    if (self._headeronoff == "True"):
		    sys.stderr.write("ofdm_demod_fec_80211n: (self._headeronoff == True) and secondshort is not correct ")
		    print " %s\n" %(self._tx_rx_type)
		    raise SystemExit  
	    else:
		    self.ofdm_sink0 = digital_swig.ofdm_frame_sink_noheader_80211n(rotated_const, range(arity), self._file_truesnr,
                                             self._rcvd_pktq,
                                             self._ntx_antennas, self._nrx_antennas, 
                                             self._occupied_tones, self._size, rate_encdec,
                                             phgain, frgain, 0)
		    self.ofdm_sink1 = digital_swig.ofdm_frame_sink_noheader_80211n(rotated_const, range(arity), self._file_truesnr1,
                                             self._rcvd_pktq,
					     self._ntx_antennas, self._nrx_antennas, 
                                             self._occupied_tones, self._size, rate_encdec,
                                             phgain, frgain, 1)
        else:
	    sys.stderr.write("ofdm_demod_fec_80211n: tx_rx_type not recognized ")
	    print " %s\n" %(self._tx_rx_type)
	    raise SystemExit  

	
	# CONNECTS
        self.connect(self, self.ofdm_recv)
        
        if (options.tx_rx_type == "siso" or options.tx_rx_type == "mrc" or options.tx_rx_type == "egc" or options.tx_rx_type == "asc" or options.tx_rx_type == "mimo2"  or options.tx_rx_type == "miso"):
	  self.connect((self.ofdm_recv, 0), (self.ofdm_demod, 0))
	  self.connect((self.ofdm_recv, 1), (self.ofdm_demod, 1))
	elif (options.tx_rx_type == "alamouti2"):
	  self.connect((self.ofdm_recv, 0), (self.ofdm_sink0, 0))
	  self.connect((self.ofdm_recv, 1), (self.ofdm_sink0, 1))
	elif (options.tx_rx_type == "multiplexing"):
	  self.connect((self.ofdm_recv, 0), (self.ofdm_sink0, 0))
	  self.connect((self.ofdm_recv, 1), (self.ofdm_sink1, 0))
	  self.connect((self.ofdm_recv, 2), (self.ofdm_sink0, 1))
	  self.connect((self.ofdm_recv, 2), (self.ofdm_sink1, 1))
	  
        print "ofdm_demod_fec_80211n: USRP --> ofdm_receiver_fec_80211n --> ofdm_frame_sink"

        if(self._tx_rx_type == "siso"  or self._tx_rx_type == "miso"):
	  self.connect(self.ofdm_recv.chan_filt, self)
        elif(self._tx_rx_type == "mrc" or self._tx_rx_type == "egc" or self._tx_rx_type == "asc" 
	      or self._tx_rx_type == "alamouti2" or self._tx_rx_type == "multiplexing"):
	  self.connect(self.ofdm_recv.chan_filt_0, self)
	elif (self._tx_rx_type == "alamouti1"):
	    sys.stderr.write("DO NOT KNOW WHAT TO DO YET ")
	    print " %s\n" %(self._tx_rx_type)
	    raise SystemExit    
	else:
	    sys.stderr.write("ofdm_fec demod: tx_rx_type not recognized ")
	    print " %s\n" %(self._tx_rx_type)
	    raise SystemExit  
	
	if (options.tx_rx_type == "siso" or options.tx_rx_type == "mrc" or options.tx_rx_type == "egc" or 
	      options.tx_rx_type == "asc" or options.tx_rx_type == "mimo2"  or options.tx_rx_type == "miso"): 
          self.connect(self.ofdm_demod, gr.null_sink(gr.sizeof_gr_complex*self._occupied_tones))   
        elif (options.tx_rx_type == "alamouti2"):
	    self.connect(self.ofdm_sink0, gr.null_sink(gr.sizeof_gr_complex*self._occupied_tones))
	elif (options.tx_rx_type == "multiplexing"):
	    self.connect(self.ofdm_sink0, gr.null_sink(gr.sizeof_gr_complex*self._occupied_tones))
	    self.connect(self.ofdm_sink1, gr.null_sink(gr.sizeof_gr_complex*self._occupied_tones))

        if options.verbose:
            self._print_verbage()
            
        self._watcher = _queue_watcher_thread(self._rcvd_pktq, self._fsm_filein, self._fsm_fileout, self._fec_type, self._fsm_iterations, self._headeronoff, self._size, self._modulation,self.bits_after_fec_enc,self.bits_after_fec_dec,callback)

    def add_options(normal, expert):
        """
        Adds OFDM-specific options to the Options Parser
        """
        normal.add_option("", "--modulation", type="string", default="bpsk",
                          help="set modulation type (bpsk or qpsk) [default=%default]")
        expert.add_option("", "--fft-length", type="intx", default=512,
                          help="set the number of FFT bins [default=%default]")
        expert.add_option("", "--occupied-tones", type="intx", default=200,
                          help="set the number of occupied FFT bins [default=%default]")
        expert.add_option("", "--cp-length", type="intx", default=128,
                          help="set the number of bits in the cyclic prefix [default=%default]")
        expert.add_option("", "--pilot-length", type="intx", default=20,
                          help="set pilot length [default=%default]")
        expert.add_option("", "--pilot-spacing", type="intx", default=11,
                          help="set pilot spacing [default=%default]")
        expert.add_option("", "--pilot-first", type="intx", default=5,
                          help="set pilot first [default=%default]")
        expert.add_option("", "--tx-rx-type", default="mrc",
                          help="TX RX type [default=%default]")
        expert.add_option("", "--ofdm-sync-type", default="ofdm_sync_pn",
                          help="OFDM SYNC type [default=%default]")
        expert.add_option("", "--size", type="intx", default=1024,
                          help="Packet size [default=%default]")
        expert.add_option("", "--ntx-antennas", type="intx", default=1,
                          help="Number of TX antennas [default=%default]")
        expert.add_option("", "--nrx-antennas", type="intx", default=1,
                          help="Number of RX antennas [default=%default]")
                          
        expert.add_option("", "--snr-type", default=3,
                          help="SNR type [default=%default]")
        expert.add_option("", "--d-samples", default=5,
                          help="Number of samples to compute SNR [default=%default]")
        expert.add_option("", "--peakthr", default=0.40,
                          help="peakthr [default=%default]")
        expert.add_option("", "--file-truesnr", type="string", default="/home/mimonet/GNURadio/gnuradio-august/truesnr_files/300bpsk100Falseviterbiawgn1o2_64.fsmawgn1o2_64.fsm_9z7wa0c1e2g3i4",
			help="File true snr [default=%default]")                    
        expert.add_option("", "--file-truesnr1", type="string", default="/home/mimonet/GNURadio/gnuradio-august/truesnr_files/300bpsk100Falseviterbiawgn1o2_64.fsmawgn1o2_64.fsm_y2x45m6o7q8s9u",
			help=" ")
	# FEC parameters
	expert.add_option("", "--fsm-filein", type="string", default="/home/mimonet/GNURadio/gnuradio-august/fsm_files/awgn1o2_4.fsm",
			help="FEC filein [default=%default]")
	expert.add_option("", "--fsm-fileout", type="string", default="/home/mimonet/GNURadio/gnuradio-august/fsm_files/awgn1o2_4.fsm",
			help="FEC fileout [default=%default]")
	expert.add_option("", "--fec-type", type="string", default="viterbi",
			help="FEC type [default=%default]")
	expert.add_option("", "--fsm-iterations", type="intx", default=10,
			help="FSM iterations [default=%default]")
			
	expert.add_option("", "--headeronoff", type="string", default="True",
			help="Header On/Off [default=%default]")
        expert.add_option("", "--downsampling", default=1,
                          help="SNR type [default=%default]")
                          
    # Make a static method to call before instantiation
    add_options = staticmethod(add_options)

    def _print_verbage(self):
        """
        Prints information about the OFDM demodulator
        """
        print "\nOFDM Demodulator:"
        print "Modulation Type: %s"    % (self._modulation)
        print "FFT length:      %3d"   % (self._fft_length)
        print "Occupied Tones:  %3d"   % (self._occupied_tones)
        print "CP length:       %3d"   % (self._cp_length)
        print "FEC PARAMETERS"
        print "FSM filein:	%s" % (self._fsm_filein)
	print "FSM fileout:	%s" % (self._fsm_fileout)
	print "FSM fectype:	%s" % (self._fec_type)
	print "FSM iterations:	%d" % (self._fsm_iterations)



class _queue_watcher_thread(_threading.Thread):
    def __init__(self, rcvd_pktq, fsm_filein, fsm_fileout, fec_type, fsm_iterations, headeronoff, pkt_size, modulation, bits_after_fec_enc, bits_after_fec_dec, callback):
        _threading.Thread.__init__(self)
        self.setDaemon(1)
        self.rcvd_pktq = rcvd_pktq
	self.fsm_filein = fsm_filein
	self.fsm_fileout = fsm_fileout
	self.fec_type = fec_type
	self.fsm_iterations = fsm_iterations
	self.headeronoff = headeronoff
	self.pkt_size = pkt_size
	self.modulation = modulation        
	self.bits_after_fec_enc = bits_after_fec_enc
	self.bits_after_fec_dec = bits_after_fec_dec
	self.callback = callback
        self.keep_running = True
        self.start()

    def run(self):
	countH = 0
        while self.keep_running:
            msg = self.rcvd_pktq.delete_head()
            fsnr0 = open('pkt_snr0','a')
            fsnr1 = open('pkt_snr1','a')

	    if (self.fec_type == "NULL"):
		if (len(msg.to_string())!= self.pkt_size+4):
		    countH +=1
		    
		# File containing decoded packet used to compute BER
		decoded_file = "/home/mimonet/GNURadio/gnuradio-august/ber_files/decoded_" + str(self.pkt_size) + "_9z7wa0c1e2g3i4" 
		fd = open(decoded_file,'r')
		lineoutd = fd.readline()
		sd = lineoutd.split()
		self.senderdecoded = [0] * len(sd)
		for i in range (len(sd)):
		    self.senderdecoded[i] = int(sd[i],16)
		fd.close()
		        
		ok, payload, countBER, wrongHDR = ofdm_packet_utils.unmake_packet(msg.to_string(),self.senderdecoded)
		countBERencoded = 0

		if (msg.arg2() == 0):
		  print >> fsnr0, "%.20f" % time.time(), ok, msg.arg1(), countBER, countBERencoded, self.pkt_size, msg.arg2()
		elif (msg.arg2() == 1):
		  print >> fsnr1, "%.20f" % time.time(), ok, msg.arg1(), countBER, countBERencoded, self.pkt_size, msg.arg2()
		else:
		  sys.stderr.write("ofdm_demod_fec_80211n: no id_rx recognized ")
		  raise SystemExit  
		
		if self.callback:
		    self.callback(ok, payload, countBER, countBERencoded, wrongHDR)
	    
	    else:
		if (len(msg.to_string())!= (self.pkt_size+4)*(float(self.bits_after_fec_dec)/float(self.bits_after_fec_enc))):
		    countH +=1
		    
		if (msg.arg2() == 0):
		    # File containing coded packet used to compute BER
		    coded_file = "/home/mimonet/GNURadio/gnuradio-august/ber_files/coded_" + str(self.pkt_size) + "_" + self.headeronoff + "_" + str(self.fec_type) + "_" + str(self.fsm_filein[53:]) + "_" + str(self.fsm_fileout[53:])  + "_9z7wa0c1"
		    # File containing decoded packet used to compute BER
		    decoded_file = "/home/mimonet/GNURadio/gnuradio-august/ber_files/decoded_" + str(self.pkt_size) + "_9z7wa0c1" 
		elif (msg.arg2() == 1):
		    # File containing coded packet used to compute BER
		    coded_file = "/home/mimonet/GNURadio/gnuradio-august/ber_files/coded_" + str(self.pkt_size) + "_" + self.headeronoff + "_" + str(self.fec_type) + "_" + str(self.fsm_filein[53:]) + "_" + str(self.fsm_fileout[53:])  + "_y2x45m6o"
		    # File containing decoded packet used to compute BER
		    decoded_file = "/home/mimonet/GNURadio/gnuradio-august/ber_files/decoded_" + str(self.pkt_size) + "_y2x45m6o" 
		else:
		  sys.stderr.write("ofdm_demod_fec_80211n: no id_rx recognized ")
		  raise SystemExit     
		
		fc = open(coded_file,'r')
		lineout = fc.readline()
		sc = lineout.split()
		self.sendercoded = [0] * len(sc)
		for i in range (len(sc)):
		    self.sendercoded[i] = int(sc[i],16)
		fc.close()

		fd = open(decoded_file,'r')
		lineoutd = fd.readline()
		sd = lineoutd.split()
		self.senderdecoded = [0] * len(sd)
		for i in range (len(sd)):
		    self.senderdecoded[i] = int(sd[i],16)
		fd.close()
		    
		ok, payload, countBER, countBERencoded, wrongHDR = ofdm_fec_packet_utils.unmake_packet(msg.to_string(), self.fsm_filein, self.fsm_fileout, self.fec_type, self.fsm_iterations, self.sendercoded, self.senderdecoded)
		
		if (msg.arg2() == 0):
		  print >> fsnr0, "%.20f" % time.time(), ok, msg.arg1(), countBER, countBERencoded, self.pkt_size, msg.arg2()
		elif (msg.arg2() == 1):
		  print >> fsnr1, "%.20f" % time.time(), ok, msg.arg1(), countBER, countBERencoded, self.pkt_size, msg.arg2()
		else:
		  sys.stderr.write("ofdm_demod_fec_80211n: no id_rx recognized ")
		  raise SystemExit  
		if self.callback:
		    self.callback(ok, payload, countBER, countBERencoded, wrongHDR)
	    fsnr0.close()
	    fsnr1.close()
   
                
# Generating known symbols with:
# i = [2*random.randint(0,1)-1 for i in range(4512)]

known_symbols_4512_3 = [-1, -1, 1, -1, 1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, 1, 1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, -1, 1, -1, -1, -1, 1, -1, -1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, 1, -1, -1, 1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, 1, -1, -1, -1, -1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, 1, 1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, -1, 1, -1, 1, 1, 1, -1, -1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1, 1, 1, -1, -1, 1, -1, -1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, -1, 1, 1, 1, -1, 1, 1, 1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, 1, -1, 1, -1, -1, -1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, -1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, 1, 1, -1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, 1, 1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, 1, -1, 1, 1, 1, -1, -1, 1, 1, 1, -1, 1, 1, -1, 1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, 1, 1, 1, 1, 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, 1, -1, 1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1, 1, -1, 1, 1, -1, -1, -1, -1, 1, -1, 1, -1, -1, -1, 1, 1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1, 1, -1, -1, 1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, -1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, 1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, 1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1, 1, -1, 1, -1, 1, -1, -1, -1, 1, 1, -1, 1, 1, 1, -1, 1, -1, -1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1, -1, 1, 1, 1, -1, 1, 1, -1, -1, -1, -1, 1, 1, 1, -1, 1, 1, -1, 1, 1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, -1, -1, 1, -1, -1, 1, 1, -1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, -1, -1, 1, 1, 1, 1, -1, -1, 1, -1, -1, -1, 1, -1, -1, -1, 1, 1, -1, -1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, -1, -1, 1, 1, 1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, 1, -1, -1, 1, 1, 1, -1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, 1, 1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, 1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, -1, -1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1, -1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, 1, 1, 1, -1, -1, 1, -1, 1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, -1, -1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1, 1, -1, -1, 1, 1, 1, 1, -1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, 1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, 1, -1, 1, -1, 1, -1, -1, 1, 1, 1, 1, 1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, 1, 1, -1, -1, 1, 1, -1, 1, 1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, 1, 1, -1, 1, 1, 1, -1, -1, 1, -1, 1, -1, -1, -1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, 1, 1, 1, -1, 1, -1, 1, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, -1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, 1, 1, -1, 1, -1, -1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, -1, 1, -1, 1, 1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, 1, -1, -1, -1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1, -1, -1, -1, 1, 1, 1, 1, 1, 1, -1, 1, -1, 1, -1, -1, 1, 1, 1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, 1, -1, 1, -1, 1, 1, 1, -1, -1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 1, -1, 1, 1, -1, -1, -1, -1, 1, 1, 1, 1, 1, -1, -1, 1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, 1, -1, 1, -1, -1, -1, 1, -1, 1, -1, 1, -1, -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, -1, -1, -1, 1, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, -1, 1, -1, 1, 1, 1, -1, 1, -1, -1, 1, 1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, -1, 1, 1, 1, 1, -1, 1, 1, 1, -1, -1, 1, 1, 1, 1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, 1, -1, 1, -1, 1, -1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, 1, 1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, 1, -1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, 1, 1, 1, -1, -1, -1, 1, -1, -1, 1, 1, -1, -1, -1, 1, -1, 1, 1, 1, -1, -1, 1, 1, -1, -1, -1, 1, -1, 1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1, -1, 1, 1, -1, -1, -1, -1, 1, -1, -1, 1, -1, 1, 1, 1, -1, 1, -1, 1, -1, 1, -1, -1, -1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, 1, 1, 1, -1, -1, -1, 1, 1, -1, 1, 1, 1, -1, -1, 1, 1, -1, -1, 1, -1, -1, 1, 1, -1, -1, -1, -1, 1, -1, -1, 1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, -1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, 1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, 1, 1, -1, 1, -1, -1, 1, -1, -1, -1, -1, -1, -1, 1, -1, -1, -1, -1, 1, -1, 1, 1, 1, -1, 1, -1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, -1, -1, -1, 1, -1, -1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, 1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, -1, -1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, 1, 1, -1, -1, 1, -1, 1, 1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, -1, 1, 1, -1, 1, -1, -1, 1, 1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, 1, 1, -1, 1, -1, -1, 1, 1, 1, 1, -1, 1, 1, 1, -1, 1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, 1, -1, 1, -1, -1, -1, 1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1, 1, 1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, -1, 1, -1, -1, -1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, -1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 1, -1, -1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, -1, -1, 1, 1, -1, -1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, 1, -1, -1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1, 1, -1, 1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, 1, -1, 1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, -1, -1, -1, 1, -1, -1, -1, 1, 1, 1, -1, 1, 1, 1, 1, -1, -1, -1, -1, -1, 1, 1, 1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, -1, -1, 1, 1, 1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, -1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, -1, -1, -1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, 1, -1, -1, -1, -1, -1, -1, 1, 1, -1, -1, -1, -1, 1, -1, -1, -1, 1, 1, 1, -1, 1, -1, -1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1, -1, -1, 1, 1, -1, -1, -1, -1, -1, -1, -1, 1, 1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, -1, 1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, 1, 1, -1, -1, -1, -1, -1, 1, 1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, -1, -1, 1, -1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, 1, 1, -1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1, 1, -1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, 1, 1, 1, -1, -1, -1, -1, -1, 1, -1, 1, -1, -1, 1, 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, -1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, -1, -1, -1, -1, 1, -1, 1, 1, 1, -1, -1, -1, -1, 1, 1, 1, -1, -1, 1, -1, 1, -1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, -1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, 1, 1, -1, 1, 1, 1, 1, -1, 1, -1, -1, -1, 1, 1, -1, -1, -1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, 1, -1, 1, 1, 1, -1, -1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 1, -1, 1, 1, 1, -1, 1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, -1, 1, 1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, -1, -1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, 1, 1, -1, 1, 1, 1, 1, -1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, 1, 1, 1, 1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, -1, -1, 1, -1, 1, -1, -1, 1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, -1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, 1, -1, -1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, 1, 1, 1, -1, 1, 1, 1, 1, -1, 1, -1, -1, -1, -1, 1, 1, 1, -1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, 1, -1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, 1, -1, -1, -1, 1, 1, -1, 1, -1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, -1, 1, -1, -1, -1, 1, -1, -1, 1, -1, -1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, 1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, 1, -1, 1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, 1, 1, 1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, 1, 1, -1, -1, 1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, 1, 1, -1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, -1, -1, -1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, -1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, -1, -1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, 1, -1, 1, 1, 1, -1, 1, -1, -1, 1, -1, 1, 1, 1, -1, -1, -1, 1, 1, 1, -1, 1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, 1, -1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, -1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, -1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, -1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, 1, 1, 1, -1, 1, 1, -1, -1, 1, 1, 1, 1, 1, 1, -1, -1, -1, 1, 1, -1, -1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, -1, -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, -1, 1, 1, 1, -1, 1, 1, 1, -1, 1, 1, -1, 1, 1, -1, 1, -1, -1, 1, 1, 1, -1, 1, -1, 1, 1, 1, 1, -1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, -1, 1, 1, -1, -1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1, 1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, -1, -1, -1, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, -1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, -1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, 1, -1, -1, -1, 1, 1, 1, -1, -1, 1, 1, -1, -1, -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1, 1, -1, -1, 1, -1, 1, 1, 1, 1, 1, -1, -1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, -1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, 1, 1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1, -1, 1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, 1, 1, 1, 1, -1, 1, -1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, 1, 1, 1, -1, -1, -1, -1, 1, -1, 1, -1, 1, -1, -1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, -1, -1, -1, 1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, -1, 1, -1, 1, -1, -1, -1, 1, 1, 1, 1, -1, -1, 1, 1, 1, -1, 1, -1, -1, 1, -1, 1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, 1, -1, 1, 1, -1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, -1, 1, -1, 1, -1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, 1, 1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, 1, 1, -1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, 1, 1, 1, 1, -1, 1, -1, -1, -1, -1, -1, -1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, 1, -1]
