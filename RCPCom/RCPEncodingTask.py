# -*- coding: utf-8 -*-
import io
import os
import socket
import struct
import mmap
import threading
import time


class RCPEncodingTask:
    def __init__(self, context, output_queue_manager):
        self.context = context
        self.output_queue_manager = output_queue_manager
	self.flag = True
        self.encodingThread = threading.Thread(None, self.decodage)
	#self.encodingThread.start()
	
    def stop(self):
	self.flag = False

    def decodage(self):
        while self.flag:
            # send system status to incoming client
            if self.output_queue_manager.get_length() > 0:
                for cpt in range(0, self.output_queue_manager.get_length()):
                    if self.context.get_latest_guidewire_moving_distance_sequence_length()>0:
		        msg = self.context.fetch_latest_guidewire_moving_distance_msg()
		        self.output_queue_manager.add_datagram_by_id(cpt, msg)
            time.sleep(0.05)

