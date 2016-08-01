import time 
from smbus import SMBus 
from threading import Thread

class usonic:
	def __init__(self, start_addr, num_sensors, offset = 1, read_delay = 0.1, disable_mask = [False, False, False, False, False, False], sim_readings = [], threshold=[200,500,500,500,500]):
		self.i2c = SMBus(1)							#initialize i2c bus
		self.start_addr = start_addr				#first sensor address
		self.num_sensors = num_sensors				#number of sensors
		self.readings = [None] * num_sensors		#list to hold latest reading for each sensor
		self.obs_present = [None] * num_sensors
		self.offset = offset						#initialize offset
		self.read_delay = read_delay				#initialize read delay
		self.stop = False
		self.threshold = threshold
		self.disable_mask = disable_mask

		self.obs_present_count = [0] * num_sensors
		self.obs_not_present_count = [0] * num_sensors

		#initialize simulated sensors with set sim readings
		for i in range(0, num_sensors):
			if disable_mask[i] == True:
				self.readings[i] = sim_readings[i]
				if self.readings[i] < self.threshold[i]:
					self.obs_present[i] = True
				else:
					self.obs_present[i] = False

	def stop_sonar(self):
		self.stop = True
		
	def start(self):
		self.stop = False
		self.usonic_thread = Thread(target = self.usonic_scan)
		self.usonic_thread.start()
	
	def usonic_scan(self):
		while(self.stop == False):
			for self.addr in range(self.start_addr, self.start_addr + (self.num_sensors)*self.offset, self.offset):
				self.sen_index = ((self.addr - self.start_addr)/self.offset)

				#only read/write to sensors enabled
				if self.disable_mask[self.sen_index] == False:
					self.i2c.write_byte(self.addr,0x51) #request sensor reading
					time.sleep(self.read_delay)
					datin = self.i2c.read_word_data(self.addr,0xe1)
					self.readings[self.sen_index]=((datin >>8)&0x00ff)|((datin <<8)&0xff00)

					#increment counter based on reading and threshold for each sensor
					if self.readings[self.sen_index] < self.threshold[self.sen_index]:
						self.obs_present_count[self.sen_index] += 1
						self.obs_not_present_count[self.sen_index] = 0
					else:
						self.obs_present_count[self.sen_index] = 0
						self.obs_not_present_count[self.sen_index] += 1

					#set obstacle present based on off/on counts
					if self.obs_present_count[self.sen_index] > 1:
						self.obs_present[self.sen_index] = True
					if self.obs_not_present_count[self.sen_index] > 1:
						self.obs_present[self.sen_index] = False

					if self.obs_present_count[self.sen_index] > 0 and self.sen_index > 0: 
						self.obs_present[self.sen_index] = True
					if self.obs_not_present_count[self.sen_index] > 0 and self.sen_index > 0:
						self.obs_present[self.sen_index] = False



	def set_sim_reading(self, sen_number, sen_sim_reading):
		self.readings[sen_number] = sen_sim_reading
                    
	def change_addr(self, old_addr, new_addr):
		self.i2c.write_block_data(old_addr,224,[170,165,new_addr])
		print "Address change successful"
			

		

