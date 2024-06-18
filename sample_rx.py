import loralibPi5 as loralib
import time

sf = 7
bw = 125
cr = 4

#############################################    
if __name__ == "__main__":

    loralib.initialize()
    loralib.setFrequency(867700000)
    loralib.setBandwidth(bw)
    loralib.setCodingRate(cr)
    loralib.setHeader(False) #implicitHeader?
    loralib.setSF(sf)
    loralib.setContMode(False)
    loralib.setCRC(True)
    loralib.optimizeLowRate(sf,bw)
    loralib.setSyncWord(0x12)

    loralib.receive()
    time.sleep(0.1)    
