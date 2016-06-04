from smbus import SMBus
from time import sleep

i2c = SMBus(1)

while True:
	try:
  		i2c.write_byte(0x70, 0x51)
  		sleep(0.3)
  		datin = i2c.read_word_data(0x70, 0xe1)
  		print ((datin >> 8) & 0x00ff) | ((datin << 8) & 0xff00), 'cm'
	except:
  		print err
