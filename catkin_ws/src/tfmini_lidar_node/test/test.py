# -*- coding: utf-8 -*
import serial
import time
ser = serial.Serial("/dev/ttyTHS2", 115200)

def getTFminiData():
  while True:
   
    time.sleep(0.1)
    count = ser.in_waiting
    print(count)
    if count > 8:
      recv = ser.read(9)
      ser.reset_input_buffer()
      
      # type(recv), 'str' in python2(recv[0] = 'Y'), 'bytes' in python3(recv[0] = 89)
      # type(recv[0]), 'str' in python2, 'int' in python3
      if recv[0] == 0x59 and recv[1] == 0x59:
        distance = recv[2] + recv[3] * 256
        strength = recv[4] + recv[5] * 256
        print('(', distance, ',', strength, ')')
        ser.reset_input_buffer()
        #python3
        if recv[0] == 'Y' and recv[1] == 'Y': #python2
          lowD = int(recv[2].encode('hex'), 16)
          highD = int(recv[3].encode('hex'), 16)
          lowS = int(recv[4].encode('hex'), 16)
          highS = int(recv[5].encode('hex'), 16)
          distance = lowD + highD * 256
          strength = lowS + highS * 256
          print(distance, strength)
    else:
      print("Not Enough Data")

if __name__ == '__main__':
  print("entering ")
  try:
    # print("opening")
    # ser.open()
    getTFminiData()
  except KeyboardInterrupt: # Ctrl+C
    if ser != None:
      print("no ser")
      ser.close()
