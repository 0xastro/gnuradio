#!/usr/bin/env python
#
# Copyright 2006, 2007, 2008 Free Software Foundation, Inc.
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

import math, sys, numpy
import fftw3
from numpy import fft
from gnuradio import gr, blks2

import digital_swig
from ofdm_sync_pn2_80211n import ofdm_sync_pn2_80211n
from ofdm_sync_pn import ofdm_sync_pn
from ofdm_sync_ml import ofdm_sync_ml
from ofdm_sync_pn2 import ofdm_sync_pn2
from ofdm_sync_pn3 import ofdm_sync_pn3

class ofdm_receiver_fec_80211n(gr.hier_block2):
    """
    Performs receiver synchronization on OFDM symbols.

    The receiver performs channel filtering as well as symbol, frequency, and phase synchronization.
    The synchronization routines are available in three flavors: preamble correlator (Schmidl and Cox),
    modifid preamble correlator with autocorrelation (not yet working), and cyclic prefix correlator
    (Van de Beeks).
    """

    
    def __init__(self, tx_rx_type, ntx_antennas, nrx_antennas, 
		      ofdm_sync_type, downsampling, snr_type, d_samples, 
		      fft_length, cp_length, occupied_tones, 
		      pilot_first, pilot_spacing, 
		      file_truesnr, rate, mod_rate, pkt_size, headeronoff, snr, 
		      ks_0, ks_1, 
		      ksfreq_ant0_long1, ksfreq_ant0_long2, ksfreq_ant0_long3, 
		      pilot, peakthr, logging=False):
        """
	Hierarchical block for receiving OFDM symbols.

	The input is the complex modulated signal at baseband.
        Synchronized packets are sent back to the demodulator.

        @param fft_length: total number of subcarriers
        @type  fft_length: int
        @param cp_length: length of cyclic prefix as specified in subcarriers (<= fft_length)
        @type  cp_length: int
        @param occupied_tones: number of subcarriers used for data
        @type  occupied_tones: int
        @param snr: estimated signal to noise ratio used to guide cyclic prefix synchronizer
        @type  snr: float
        @param ks: known symbols used as preambles to each packet
        @type  ks: list of lists
        @param logging: turn file logging on or off
        @type  logging: bool
	"""

	gr.hier_block2.__init__(self, "ofdm_receiver_fec_80211n",
				gr.io_signature(1, 1, gr.sizeof_gr_complex), # Input signature
                                gr.io_signature2(2, 2, gr.sizeof_gr_complex*occupied_tones, gr.sizeof_char)) # Output signature
        
        print "OFDM_RECEIVER_FEC_80211n"
        
        bw = (float(occupied_tones+len(pilot)) / float(fft_length)) / 2.0
        tb = bw*0.12
        chan_coeffs = gr.firdes.low_pass (1.0,                     # gain
                                          1.0,                     # sampling rate
                                          bw+tb,                   # midpoint of trans. band
                                          tb,                      # width of trans. band
                                          gr.firdes.WIN_HAMMING)   # filter type
        self.chan_filt = gr.fft_filter_ccc(1, chan_coeffs)
        
        win = [1 for i in range(fft_length)]

        zeros_on_left = int(math.ceil((fft_length - (occupied_tones+len(pilot)))/2.0))
        zeros_on_right = fft_length - (occupied_tones+len(pilot))-zeros_on_left
        print "ofdm_receiver_fec_80211n: zeros_on_right ", zeros_on_right
        
        if (tx_rx_type == "alamouti1" or tx_rx_type == "alamouti2" ):
	  print "ofdm_receiver_fec_80211n: alamouti1 or alamouti2 call ofdm_receiver_fec_alamouti"
	  raise SystemExit
	  
        elif (tx_rx_type == "multiplexing" ):
	  print "ofdm_receiver_fec_80211n: SPATIAL MULTIPLEXING call ofdm_receiver_fec_multiplexing_80211n"
	
	nco_sensitivity = -2.0/fft_length   # correct for fine frequency
	
        self.ofdm_sync = ofdm_sync_pn3(fft_length*downsampling,
                                          cp_length*downsampling,
                                          peakthr,
                                          logging)
        
        max_coarse=20
	if (max_coarse > zeros_on_right): 
	    max_coarse = zeros_on_right
        # Set up blocks
	if (tx_rx_type == "siso" or tx_rx_type == "miso"):
	    if (tx_rx_type == "siso"):
		print "SINGLE INPUT SINGLE OUTPUT"
	    else: 
		print "MULTIPLE INPUT SINGLE OUTPUT"
		
	    self.nco = gr.frequency_modulator_fc(nco_sensitivity)         # generate a signal proportional to frequency error of sync block
	    self.sigmix = gr.multiply_cc()        
	    
	    ksfreq_ant0_long1_time=[-10.69+0j, 0.840463-6.4877j, 6.27488-7.5092j, -6.28658-7.7242j, -0.191971-3.51627j, 5.13624+4.44663j, -8.71102+2.49731j, -8.33902-0.305411j, -2.39738+11.835j, -3.86243+0.263975j, -4.12618-4.97615j, 4.7588-0.64361j, 5.62505-7.63849j, -8.98046-2.27777j, -3.91383-5.38826j, 2.52578-4.02459j, 4.276+2.138j, 8.15786+1.34757j, -1.53821-10.6675j, 4.01388-0.743627j, 1.67454+6.96069j, -9.35964-0.386414j, 0.067662+11.4769j, 3.64915-3.143j, 6.67338+3.28297j, -2.62142+7.46134j, -7.87682+1.84633j, 4.09291+9.33213j, 1.44438-6.0211j, 6.62485-1.56174j, 2.71952+4.31551j, -0.350375+10.0582j, 10.69+0j, -0.350376-10.0582j, 2.71952-4.31551j, 6.62485+1.56174j, 1.44438+6.0211j, 4.09291-9.33213j, -7.87682-1.84633j, -2.62142-7.46134j, 6.67338-3.28297j, 3.64915+3.143j, 0.0676618-11.4769j, -9.35964+0.386414j, 1.67454-6.96069j, 4.01388+0.743627j, -1.53821+10.6675j, 8.15786-1.34757j, 4.276-2.138j, 2.52578+4.02459j, -3.91383+5.38826j, -8.98046+2.27777j, 5.62505+7.63849j, 4.7588+0.64361j, -4.12618+4.97615j, -3.86243-0.263976j, -2.39738-11.835j, -8.33902+0.305411j, -8.71102-2.49731j, 5.13624-4.44663j, -0.191971+3.51627j, -6.28658+7.7242j, 6.27488+7.5092j, 0.840462+6.4877j]
	    
	    self.sampler = digital_swig.ofdm_sampler_snroncp(fft_length, fft_length+cp_length, downsampling)
	    
	    if (snr_type == -1):
		self.filesinkCP = gr.file_sink(gr.sizeof_float, "snr_timedomainCP.dat")
	    else:
		self.filesinkCP = gr.null_sink(gr.sizeof_float)
	    self.connect((self.sampler,2), self.filesinkCP)
	    
	    self.fft_demod = gr.fft_vcc(fft_length, True, win, True)
	    self.ofdm_frame_acq = digital_swig.ofdm_frame_acquisition_80211n(
								  ntx_antennas, nrx_antennas,
								  occupied_tones,
                                                                  fft_length,
                                                                  cp_length,
                                                                  pilot_first, 
                                                                  pilot_spacing,
                                                                  rate,
                                                                  mod_rate,
                                                                  pkt_size,
                                                                  headeronoff,
                                                                  ksfreq_ant0_long1, pilot, max_coarse)

	    
	    # add SNR EST TIME DOMAIN
	    self.snr_time = digital_swig.ofdm_snr_timedomain(occupied_tones, fft_length, cp_length)
	    if (snr_type == -1):
		self.filesink = gr.file_sink(gr.sizeof_float, "snr_timedomain.dat")
	    else:
		self.filesink = gr.null_sink(gr.sizeof_float)
	    self.nullsink = gr.null_sink(gr.sizeof_char)
	    self.connect((self.sampler,0), (self.snr_time,0), self.filesink)
	    self.connect((self.sampler,1), (self.snr_time,1), self.nullsink)
            
	    self.connect(self, self.chan_filt)                            # filter the input channel
	    self.connect(self.chan_filt, self.ofdm_sync)                  # into the synchronization alg.
	    
	    self.connect((self.ofdm_sync,0), self.nco, (self.sigmix,1))   # use sync freq. offset output to derotate input signal
	    self.connect(self.chan_filt, (self.sigmix,0))                 # signal to be derotated
	    self.connect(self.sigmix, (self.sampler,0))                   # sample off timing signal detected in sync alg
	    self.connect((self.ofdm_sync,1), (self.sampler,1))            # timing signal to sample at

	    self.connect((self.sampler,0), self.fft_demod)                # send derotated sampled signal to FFT
	    self.connect(self.fft_demod, (self.ofdm_frame_acq,0))         # find frame start and equalize signal
	    self.connect((self.sampler,1), (self.ofdm_frame_acq,1))       # send timing signal to signal frame start
	    self.connect((self.ofdm_frame_acq,0), (self,0))               # finished with fine/coarse freq correction,
	    self.connect((self.ofdm_frame_acq,1), (self,1))               # frame and symbol timing, and equalization
	    
	    if logging:
		self.connect(self.chan_filt, gr.file_sink(gr.sizeof_gr_complex, "ofdm_receiver-chan_filt_c.dat"))
		self.connect(self.fft_demod, gr.file_sink(gr.sizeof_gr_complex*fft_length, "ofdm_receiver-fft_out_c.dat"))
		self.connect(self.ofdm_frame_acq, gr.file_sink(gr.sizeof_gr_complex*occupied_tones, "ofdm_receiver-frame_acq_c.dat"))
		self.connect((self.ofdm_frame_acq,1), gr.file_sink(1, "ofdm_receiver-found_corr_b.dat"))
		self.connect(self.sampler, gr.file_sink(gr.sizeof_gr_complex*fft_length, "ofdm_receiver-sampler_c.dat"))
		self.connect(self.sigmix, gr.file_sink(gr.sizeof_gr_complex, "ofdm_receiver-sigmix_c.dat"))
		self.connect(self.nco, gr.file_sink(gr.sizeof_gr_complex, "ofdm_receiver-nco_c.dat"))

        elif (tx_rx_type == "mrc"):
	    print "MAXIMAL RATIO COMBINING"
            nrxchans = 2

	    # Deinterleave the incoming stream into separate channels
	    self.deint = gr.deinterleave(gr.sizeof_gr_complex)
	    # generate a signal proportional to frequency error of sync block
	    self.nco = gr.frequency_modulator_fc(nco_sensitivity)
             
            self.ofdm_frame_acq = digital_swig.ofdm_frame_acquisition_mrc(nrxchans, 
									  occupied_tones, 
									  fft_length, 
									  cp_length,
									  pilot_first, 
									  pilot_spacing,
									  rate,
									  mod_rate,
									  pkt_size,
									  headeronoff,
									  ks_0[0], pilot,max_coarse)
            self.chan_filt_0 = gr.fft_filter_ccc(1, chan_coeffs)
            self.chan_filt_1 = gr.fft_filter_ccc(1, chan_coeffs)
            self.sigmix_0 = gr.multiply_cc()
            self.sigmix_1 = gr.multiply_cc()
            
	    self.sampler_0 = digital_swig.ofdm_sampler_snroncp(fft_length, fft_length+cp_length, downsampling)
            self.sampler_1 = digital_swig.ofdm_sampler_snroncp(fft_length, fft_length+cp_length, downsampling)
            if (snr_type == -1):
		self.filesinkCP_0 = gr.file_sink(gr.sizeof_float, "snr_timedomainCP0.dat")
		self.filesinkCP_1 = gr.file_sink(gr.sizeof_float, "snr_timedomainCP1.dat")
	    else:
		self.filesinkCP_0 = gr.null_sink(gr.sizeof_float) 
		self.filesinkCP_1 = gr.null_sink(gr.sizeof_float)
	    self.connect((self.sampler_0,2), self.filesinkCP_0)
	    self.connect((self.sampler_1,2), self.filesinkCP_1)
            
            self.fft_demod_0 = gr.fft_vcc(fft_length, True, win, True)
            self.fft_demod_1 = gr.fft_vcc(fft_length, True, win, True)
   
	    # add SNR EST TIME DOMAIN
	    self.snr_time_0 = digital_swig.ofdm_snr_timedomain(occupied_tones, fft_length, cp_length)
	    self.snr_time_1 = digital_swig.ofdm_snr_timedomain(occupied_tones, fft_length, cp_length)
	    if (snr_type == -1):
		self.filesink_0 = gr.file_sink(gr.sizeof_float, "snr_timedomain0.dat")
		self.filesink_1 = gr.file_sink(gr.sizeof_float, "snr_timedomain1.dat")
	    else:
		self.filesink_0 = gr.null_sink(gr.sizeof_float)
		self.filesink_1 = gr.null_sink(gr.sizeof_float)
	    self.nullsink_0 = gr.null_sink(gr.sizeof_char)
	    self.nullsink_1 = gr.null_sink(gr.sizeof_char)
	    self.connect((self.sampler_0,0), (self.snr_time_0,0), self.filesink_0)
	    self.connect((self.sampler_0,1), (self.snr_time_0,1), self.nullsink_0)
	    self.connect((self.sampler_1,0), (self.snr_time_1,0), self.filesink_1)
	    self.connect((self.sampler_1,1), (self.snr_time_1,1), self.nullsink_1)

            self.connect(self, self.deint)               # deinterleave channels
	    self.connect((self.ofdm_sync,0), self.nco)   # use sync freq. offset to derotate signal
	    
	    self.connect((self.deint, 0), self.chan_filt_0)              # filter the input channel
	    self.connect((self.deint, 1), self.chan_filt_1)              # filter the input channel
            self.connect(self.nco, (self.sigmix_0,1))                    # use sync freq. offset to derotate signal
            self.connect(self.nco, (self.sigmix_1,1))                    # use sync freq. offset to derotate signal
            self.connect(self.chan_filt_0, (self.sigmix_0,0))           # signal to be derotated
            self.connect(self.chan_filt_1, (self.sigmix_1,0))           # signal to be derotated
            self.connect(self.sigmix_0, (self.sampler_0,0))             # sample off timing signal detected in sync alg
            self.connect(self.sigmix_1, (self.sampler_1,0))             # sample off timing signal detected in sync alg
            self.connect((self.ofdm_sync,1), (self.sampler_0,1))         # timing signal to sample at
            self.connect((self.ofdm_sync,1), (self.sampler_1,1))         # timing signal to sample at
            
            self.connect((self.sampler_0,0), self.fft_demod_0)          # send derotated sampled signal to FFT
            self.connect((self.sampler_1,0), self.fft_demod_1)          # send derotated sampled signal to FFT
            self.connect(self.fft_demod_0, (self.ofdm_frame_acq,1))    # find frame start and equalize signal
            self.connect(self.fft_demod_1, (self.ofdm_frame_acq,2))    # find frame start and equalize signal
            
            self.connect(self.chan_filt_0, (self.ofdm_sync,0))
	    
	    self.connect((self.sampler_0,1), (self.ofdm_frame_acq,0))    # send timing signal to signal frame start
	    self.connect((self.sampler_1,1), gr.null_sink(fft_length*gr.sizeof_char))
	    self.connect((self.ofdm_frame_acq,0), (self,0))               # finished with fine/coarse freq correction,
	    self.connect((self.ofdm_frame_acq,1), (self,1))               # frame and symbol timing, and equalization
	    
        elif (tx_rx_type == "egc"):
	    print "EQUAL GAIN COMBINING"
            nrxchans = 2

	    # Deinterleave the incoming stream into separate channels
	    self.deint = gr.deinterleave(gr.sizeof_gr_complex)
	    # generate a signal proportional to frequency error of sync block
	    self.nco = gr.frequency_modulator_fc(nco_sensitivity)
             
            self.ofdm_frame_acq = digital_swig.ofdm_frame_acquisition_egc(nrxchans, 
									  occupied_tones, 
									  fft_length, 
									  cp_length,
									  pilot_first, 
									  pilot_spacing,
									  rate,
									  mod_rate,
									  pkt_size,
									  headeronoff,
									  ks_0[0], pilot, max_coarse)
            self.chan_filt_0 = gr.fft_filter_ccc(1, chan_coeffs)
            self.chan_filt_1 = gr.fft_filter_ccc(1, chan_coeffs)
            self.sigmix_0 = gr.multiply_cc()
            self.sigmix_1 = gr.multiply_cc()
            
	    self.sampler_0 = digital_swig.ofdm_sampler_snroncp(fft_length, fft_length+cp_length, downsampling)
            self.sampler_1 = digital_swig.ofdm_sampler_snroncp(fft_length, fft_length+cp_length, downsampling)
	    if (snr_type == -1):
		self.filesinkCP_0 = gr.file_sink(gr.sizeof_float, "snr_timedomainCP0.dat")
		self.filesinkCP_1 = gr.file_sink(gr.sizeof_float, "snr_timedomainCP1.dat")
	    else:
		self.filesinkCP_0 = gr.null_sink(gr.sizeof_float)
		self.filesinkCP_1 = gr.null_sink(gr.sizeof_float)
	    self.connect((self.sampler_0,2), self.filesinkCP_0)
	    self.connect((self.sampler_1,2), self.filesinkCP_1)
	    
            self.fft_demod_0 = gr.fft_vcc(fft_length, True, win, True)
            self.fft_demod_1 = gr.fft_vcc(fft_length, True, win, True)
            
	    # add SNR EST TIME DOMAIN
	    self.snr_time_0 = digital_swig.ofdm_snr_timedomain(occupied_tones, fft_length, cp_length)
	    self.snr_time_1 = digital_swig.ofdm_snr_timedomain(occupied_tones, fft_length, cp_length)
	    if (snr_type == -1):
		self.filesink_0 = gr.file_sink(gr.sizeof_float, "snr_timedomain0.dat")
		self.filesink_1 = gr.file_sink(gr.sizeof_float, "snr_timedomain1.dat")
	    else:
		self.filesink_0 = gr.null_sink(gr.sizeof_float)
		self.filesink_1 = gr.null_sink(gr.sizeof_float)
	    self.nullsink_0 = gr.null_sink(gr.sizeof_char)
	    self.nullsink_1 = gr.null_sink(gr.sizeof_char)
	    self.connect((self.sampler_0,0), (self.snr_time_0,0), self.filesink_0)
	    self.connect((self.sampler_0,1), (self.snr_time_0,1), self.nullsink_0)
	    self.connect((self.sampler_1,0), (self.snr_time_1,0), self.filesink_1)
	    self.connect((self.sampler_1,1), (self.snr_time_1,1), self.nullsink_1)

            self.connect(self, self.deint)               # deinterleave channels
	    self.connect((self.ofdm_sync,0), self.nco)   # use sync freq. offset to derotate signal
	    
	    self.connect((self.deint, 0), self.chan_filt_0)              # filter the input channel
	    self.connect((self.deint, 1), self.chan_filt_1)              # filter the input channel
            self.connect(self.nco, (self.sigmix_0,1))                    # use sync freq. offset to derotate signal
            self.connect(self.nco, (self.sigmix_1,1))                    # use sync freq. offset to derotate signal
            self.connect(self.chan_filt_0, (self.sigmix_0,0))           # signal to be derotated
            self.connect(self.chan_filt_1, (self.sigmix_1,0))           # signal to be derotated
            self.connect(self.sigmix_0, (self.sampler_0,0))             # sample off timing signal detected in sync alg
            self.connect(self.sigmix_1, (self.sampler_1,0))             # sample off timing signal detected in sync alg
            self.connect((self.ofdm_sync,1), (self.sampler_0,1))         # timing signal to sample at
            self.connect((self.ofdm_sync,1), (self.sampler_1,1))         # timing signal to sample at
            
            self.connect((self.sampler_0,0), self.fft_demod_0)          # send derotated sampled signal to FFT
            self.connect((self.sampler_1,0), self.fft_demod_1)          # send derotated sampled signal to FFT
            self.connect(self.fft_demod_0, (self.ofdm_frame_acq,1))    # find frame start and equalize signal
            self.connect(self.fft_demod_1, (self.ofdm_frame_acq,2))    # find frame start and equalize signal
            
            self.connect(self.chan_filt_0, (self.ofdm_sync,0))
           
	    self.connect((self.sampler_0,1), (self.ofdm_frame_acq,0))    # send timing signal to signal frame start
	    self.connect((self.sampler_1,1), gr.null_sink(fft_length*gr.sizeof_char))
	    self.connect((self.ofdm_frame_acq,0), (self,0))               # finished with fine/coarse freq correction,
	    self.connect((self.ofdm_frame_acq,1), (self,1))               # frame and symbol timing, and equalization
	
	elif (tx_rx_type == "asc"):
	    print "ANTENNA DIVERSITY COMBINING"
            nrxchans = 2

	    # Deinterleave the incoming stream into separate channels
	    self.deint = gr.deinterleave(gr.sizeof_gr_complex)
	    # generate a signal proportional to frequency error of sync block
	    self.nco = gr.frequency_modulator_fc(nco_sensitivity)
             
            self.ofdm_frame_acq = digital_swig.ofdm_frame_acquisition_asc(nrxchans, 
									  occupied_tones, 
									  fft_length, 
									  cp_length,
									  pilot_first, 
									  pilot_spacing,
									  rate,
									  mod_rate,
									  pkt_size,
									  headeronoff,
									  ks_0[0], pilot, max_coarse)
            
            
            self.chan_filt_0 = gr.fft_filter_ccc(1, chan_coeffs)
            self.chan_filt_1 = gr.fft_filter_ccc(1, chan_coeffs)
            self.sigmix_0 = gr.multiply_cc()
            self.sigmix_1 = gr.multiply_cc()
            
	    self.sampler_0 = digital_swig.ofdm_sampler_snroncp(fft_length, fft_length+cp_length, downsampling)
            self.sampler_1 = digital_swig.ofdm_sampler_snroncp(fft_length, fft_length+cp_length, downsampling)
	    if (snr_type == -1):
		self.filesinkCP_0 = gr.file_sink(gr.sizeof_float, "snr_timedomainCP0.dat")
		self.filesinkCP_1 = gr.file_sink(gr.sizeof_float, "snr_timedomainCP1.dat")
	    else:
		self.filesinkCP_0 = gr.null_sink(gr.sizeof_float)
		self.filesinkCP_1 = gr.null_sink(gr.sizeof_float)
	    self.connect((self.sampler_0,2), self.filesinkCP_0)
	    self.connect((self.sampler_1,2), self.filesinkCP_1)
	    
            self.fft_demod_0 = gr.fft_vcc(fft_length, True, win, True)
            self.fft_demod_1 = gr.fft_vcc(fft_length, True, win, True)
                       
	    # add SNR EST TIME DOMAIN
	    self.snr_time_0 = digital_swig.ofdm_snr_timedomain(occupied_tones, fft_length, cp_length)
	    self.snr_time_1 = digital_swig.ofdm_snr_timedomain(occupied_tones, fft_length, cp_length)
	    if (snr_type == -1):
		self.filesink_0 = gr.file_sink(gr.sizeof_float, "snr_timedomain0.dat")
		self.filesink_1 = gr.file_sink(gr.sizeof_float, "snr_timedomain1.dat")
	    else:
		self.filesink_0 = gr.null_sink(gr.sizeof_float)
		self.filesink_1 = gr.null_sink(gr.sizeof_float)
	    self.nullsink_0 = gr.null_sink(gr.sizeof_char)
	    self.nullsink_1 = gr.null_sink(gr.sizeof_char)
	    self.connect((self.sampler_0,0), (self.snr_time_0,0), self.filesink_0)
	    self.connect((self.sampler_0,1), (self.snr_time_0,1), self.nullsink_0)
	    self.connect((self.sampler_1,0), (self.snr_time_1,0), self.filesink_1)
	    self.connect((self.sampler_1,1), (self.snr_time_1,1), self.nullsink_1)
            
            self.connect(self, self.deint)               # deinterleave channels
	    self.connect((self.ofdm_sync,0), self.nco)   # use sync freq. offset to derotate signal
	    
	    self.connect((self.deint, 0), self.chan_filt_0)              # filter the input channel
	    self.connect((self.deint, 1), self.chan_filt_1)              # filter the input channel
            self.connect(self.nco, (self.sigmix_0,1))                    # use sync freq. offset to derotate signal
            self.connect(self.nco, (self.sigmix_1,1))                    # use sync freq. offset to derotate signal
            self.connect(self.chan_filt_0, (self.sigmix_0,0))           # signal to be derotated
            self.connect(self.chan_filt_1, (self.sigmix_1,0))           # signal to be derotated
            self.connect(self.sigmix_0, (self.sampler_0,0))             # sample off timing signal detected in sync alg
            self.connect(self.sigmix_1, (self.sampler_1,0))             # sample off timing signal detected in sync alg
            self.connect((self.ofdm_sync,1), (self.sampler_0,1))         # timing signal to sample at
            self.connect((self.ofdm_sync,1), (self.sampler_1,1))         # timing signal to sample at
            
            self.connect((self.sampler_0,0), self.fft_demod_0)          # send derotated sampled signal to FFT
            self.connect((self.sampler_1,0), self.fft_demod_1)          # send derotated sampled signal to FFT
            self.connect(self.fft_demod_0, (self.ofdm_frame_acq,1))    # find frame start and equalize signal
            self.connect(self.fft_demod_1, (self.ofdm_frame_acq,2))    # find frame start and equalize signal
            
            self.connect(self.chan_filt_0, (self.ofdm_sync,0))
            
	    self.connect((self.sampler_0,1), (self.ofdm_frame_acq,0))    # send timing signal to signal frame start
	    self.connect((self.sampler_1,1), gr.null_sink(fft_length*gr.sizeof_char))
	    self.connect((self.ofdm_frame_acq,0), (self,0))               # finished with fine/coarse freq correction,
	    self.connect((self.ofdm_frame_acq,1), (self,1))               # frame and symbol timing, and equalization
	    
	elif (tx_rx_type == "alamouti1" or tx_rx_type == "alamouti2"):
	    print "alamouti1 or alamouti2 call ofdm_receiver_fec_alamouti"
	    raise SystemExit
	
	elif (tx_rx_type == "multiplexing"):
	    print "SPATIAL MULTIPLEXING implemented in ofdm_receiver_fec_multiplexing"
	   
	else:
	    sys.stderr.write("ofdm_receiver_fec_80211n: tx_rx_type not recognized\n")
	    print " %s\n" %(tx_rx_type)
	    raise SystemExit