from smbus import SMBus
from time import sleep

i2c = SMBus(1)

fname = raw_input('Enter file name for logging, including extension')
file = open(fname, "w")

i=0

while True:
	try:
  		i = i + 1
		
		i2c.write_byte(0x70, 0x51)
  		sleep(0.1)
  		datin = i2c.read_word_data(0x70, 0xe1)
		dist = ((datin >> 8) & 0x00ff) | ((datin << 8) & 0xff00)
  		print i, dist, 'cm'
		file.write(str(i))
		file.write("	")
		file.write(str(dist))
		file.write("\n")
		
	except KeyboardInterrupt:
  		file.close()
		print "fdonalk"
		break


