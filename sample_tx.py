import loralibPi5 as loralib
import time

sf = 7
bw = 125
cr = 1

#############################################
if __name__ == "__main__":

    cnt = 0
    loralib.initialize()
    for i in range(1000):
        loralib.setFrequency(867700000)
        loralib.setBandwidth(bw)
        loralib.setCodingRate(cr)   # coding rate = 4/(4+cr)
        loralib.setHeader(False)    # False=>explicit header, True=>implicit header
        loralib.setSF(sf)
        loralib.setCRC(True)
        
        loralib.setSyncWord(0x12)
        loralib.configPower(2)
        print("Sending msg {}".format(i))
        
        loralib.transmit("Hello world {}".format(cnt))
        cnt += 1
        time.sleep(2.5)    

