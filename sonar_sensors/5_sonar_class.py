import time 
from smbus import SMBus 
from threading import Thread

class usonic:
	def __init__(self, start_addr, num_sensors, offset = 2, read_delay = 0.1, sim_mask = [False, False, False, False, False, False], sim_readings = []):
		self.i2c = SMBus(1)				#initialize i2c bus
		self.start_addr = start_addr	#first sensor address
		self.num_sensors = num_sensors	#number of sensors
		self.readings = [None] * num_sensors			#list to hold latest reading for each sensor
		self.offset = offset			#initialize offset
		self.read_delay = read_delay	#initialize read delay
		self.stop = False
		self.sim_mask = sim_mask
				
		for i in range(0, num_sensors-1):
			if sim_mask[i] == True:
				self.readings[i] = sim_readings[i]

	def stop(self):
		self.stop = True
		
	def start(self):
		self.stop = False
		self.usonic_thread = Thread(target = self.usonic_scan)
		self.usonic_thread.start()
	
	def usonic_scan(self):
		while(self.stop == False):
			for self.addr in range(self.start_addr, self.start_addr + (self.num_sensors)*self.offset, self.offset):
				self.sen_index = ((self.addr - self.start_addr)/self.offset)
				if self.sim_mask[self.sen_index] == False:
					self.i2c.write_byte(self.addr,0x51) #request sensor reading
					time.sleep(self.read_delay)
 					datin = self.i2c.read_word_data(self.addr,0xe1)
					self.readings[self.sen_index]=((datin >>8)&0x00ff)|((datin <<8)&0xff00)

	def set_sim_reading(self, sen_number, sen_sim_reading):
        	self.readings[sen_number] = sen_sim_reading
                    
	def change_addr(self, old_addr, new_addr):
		self.i2c.write_block_data(old_addr,224,[170,165,new_addr])
#		self.i2c.write_byte(old_addr,0xa5)
#		self.i2c.write_byte(old_addr,new_addr)
		print "Address change successful"
			
usonic = usonic(0x70, 5, offset = 1, read_delay=0.1, sim_mask=[True, False, False, False, False, False], sim_readings = [234,0,0,0,0])	

usonic.start()
time.sleep(1)
print usonic.readings
time.sleep(0.5)
while True:
	print usonic.readings
	time.sleep(1)
	usonic.set_sim_reading(0,123)
		

