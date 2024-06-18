#############################################
#                                           #
#    Python library for LoRa SPI chips      #
#          TCL, EPFL, Switzerland           #
#             Joachim Tapparel              #
#                                           #
#############################################


################## Imports ##################
import spidev
import gpiod
import time

################# Constants #################

# SX1276 - Raspberry connections
DIO0                      = 4
RST                       = 17

BUS                       = 0
DEVICE                    = 2

# Registers
REG_FIFO                  = 0x00
REG_OPMODE                = 0x01
REG_FRF_MSB               = 0x06  # FRF
REG_FRF_MID               = 0x07
REG_FRF_LSB               = 0x08
REG_PA_CONFIG             = 0x09
REG_PA_RAMP               = 0x0A
REG_LNA                   = 0x0C  # LOW NOISE AMPLIFIER
REG_FIFO_ADDR_PTR         = 0x0D
REG_FIFO_TX_BASE_AD       = 0x0E
REG_FIFO_RX_BASE_AD       = 0x0F
REG_FIFO_RX_CURRENT_ADDR  = 0x10
REG_IRQ_FLAGS_MASK        = 0x11
REG_IRQ_FLAGS             = 0x12
REG_RX_NB_BYTES           = 0x13
REG_MODEM_STAT            = 0x18
REG_PKT_SNR_VALUE         = 0x19
REG_PKT_RSSI              = 0x1A
REG_RSSI                  = 0x1B
REG_MODEM_CONFIG          = 0x1D
REG_MODEM_CONFIG2         = 0x1E
REG_SYMB_TIMEOUT_LSB  	  = 0x1F
REG_PAYLOAD_LENGTH        = 0x22
REG_MAX_PAYLOAD_LENGTH 	  = 0x23
REG_HOP_PERIOD            = 0x24
REG_MODEM_CONFIG3         = 0x26
REG_SYNC_WORD			  = 0x39
REG_DIO_MAPPING_1         = 0x40
REG_VERSION	  			  = 0x42
REG_PA_DAC                = 0x4D
# Operation mode
OPMODE_MASK               = 0xF8  # 11111000  use & to clear opmode bits
OPMODE_SLEEP              = 0x00  # Sleep
OPMODE_STANDBY            = 0x01  # Standby
OPMODE_TX                 = 0x03  # Transmit
OPMODE_RX                 = 0x05  # Receive continuous
OPMODE_LORA_HF            = 0x80  # LoRa mode at High Frequency + Sleep mode
# Bits masking the corresponding IRQs from the radio
IRQ_LORA_TXDONE_MASK      = 0x08
# DIO function mappings             D0D1D2D3
MAP_DIO0_LORA_TXDONE      = 0x40  # 01------
MAP_DIO1_LORA_NOP         = 0x30  # --11----
MAP_DIO2_LORA_NOP         = 0xC0  # ----11--
# MASKS
# ModemConfig1
BANDWIDTH_MASK            = 0x0F  # 00001111  use & to clear bandwidth bits
CR_MASK                   = 0xF1  # 11110001  use & to clear cr bits
HEADER_MASK               = 0xFE  # 11111110  ...
# ModemConfig2
SF_MASK                   = 0x0F  # 00001111
CONT_MODE_MASK            = 0xF7  # 11110111
CRC_MASK                  = 0xFB  # 11111011
SYMB_TIMEOUT_MASK         = 0xFC  # 11111100
# ModemConfig3
LOW_RATE_MASK             = 0xF7  # 11110111
AGC_MASK                  = 0xFB  # 11111011


SPEED                     = 500000  # Clock speed: between 500 000 and 32 000 000 Hz
OUTPUT                    = 1
INPUT                     = 0
HIGH                      = 1
LOW                       = 0
MAX_FRAME_LEN             = 255


spi = spidev.SpiDev()


######################## Lines ###############
gpiomap = {} # This is a dictionary to avoid repeated chip.get_line(gpio) calls
chip = gpiod.Chip('gpiochip4')

################# Utilities #################

def setup_gpio(gpio, direction, pud):
# def setup_gpio(gpio, direction, pud=0):  # optional pud
  """
  Set gpio as an input or an output
  direction: 0=IN, 1=OUT
  pud: 0=None 1=Up 2=Down
  """
  line = chip.get_line(gpio)
  gpiomap[gpio] = line
  if direction:
    line.request(consumer="p_gpio", type=gpiod.LINE_REQ_DIR_OUT)
  else:
    f = gpiod.LINE_REQ_FLAG_BIAS_DISABLE
    if pud==1:
      f = gpiod.LINE_REQ_FLAG_BIAS_PULL_UP
    if pud==2:
      f = gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN
    line.request("p_gpio", gpiod.LINE_REQ_DIR_IN, f)


def input_gpio(gpio):
  """
  Input from a GPIO channel.
  Returns HIGH=1=True or LOW=0=False
  """
  return gpiomap[gpio].get_value()

def output_gpio(gpio, value):
  """
  Output to a GPIO channel.
  value - 0/1 or False/True or LOW/HIGH
  """
  gpiomap[gpio].set_value(value)


def writeReg(addr, value):
    # Write a value on a register
    # address MSB must be one to write
    spibuf = bytes([addr|0x80, value])
    spi.xfer(spibuf,SPEED,0,8)

def readReg(addr):
    # # Read a value from a register
    # # address MSB must be zero to read

    result = spi.xfer([addr&0x7F,0],SPEED,0,8)
    return result[1]


def writeBuf(addr, value):
    # This function writes strings on the buffer
    # Input: string to send
    # Support for arrays or lists of numbers is not guaranteed
    # address MSB must be one to write
    data = [addr|0x80]
    for x in value:
        data.append(ord(x))
    spibuf = bytes(data)
    spi.xfer(spibuf,SPEED,0,8)

def opmode(mode):
    # Set the operating mode of the chip
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & OPMODE_MASK) | mode)

def setLoRaMode():
    # Set HF Lora Mode
    writeReg(REG_OPMODE, OPMODE_LORA_HF)

############## Setup functions ##############
def initialize():

    bus = BUS
    device = DEVICE
    spi.open(bus, device)
    spi.max_speed_hz = 500000
    spi.mode = 0b00

    # Setup pins
    setup_gpio(DIO0,0,0)
    setup_gpio(RST,1,0)

    output_gpio(RST,0)
    time.sleep(0.1)
    output_gpio(RST,1)
    time.sleep(0.1)
    # Check chipset version
    version = readReg(REG_VERSION)
    if (version != 0x12):
        print(version, "Unrecognized transceiver.")
        exit()
    # Set operating mode to sleep mode in order to modify the registers
    opmode(OPMODE_SLEEP)
    # Enter LoRa mode
    setLoRaMode()

def setFrequency(frequency):
    # Set center frequency
    # Input: carrier frequency in Hz
    frf = (frequency << 19) // 32000000
    writeReg( REG_FRF_MSB, (frf & 0xFF0000) >> 16 )
    writeReg( REG_FRF_MID, (frf & 0x00FF00) >> 8  )
    writeReg( REG_FRF_LSB, (frf & 0x0000FF) )

def getFrequency():
    msb = readReg(REG_FRF_MSB)& 0xFF
    mid = readReg(REG_FRF_MID)& 0xFF
    lsb = readReg(REG_FRF_LSB)& 0xFF
    frf = ((msb<<16)& 0xFF0000) +((mid<<8) & 0x00FF00)+ (lsb & 0x0000FF)

    freq = 32000000*frf/(1<<19)
    return freq


def mapBandwidth(bandwidth):
    # Map the bandwidth to the bits to be written in the register
    # Input: bandwidth in kHz
    bandwidth_Hz = bandwidth*1000
    bandwidth_map = 8
    if   bandwidth_Hz == 7800:
        bandwidth_map = 0
    elif bandwidth_Hz == 10400:
        bandwidth_map = 1
    elif bandwidth_Hz == 15600:
        bandwidth_map = 2
    elif bandwidth_Hz == 20800:
        bandwidth_map = 3
    elif bandwidth_Hz == 31250:
        bandwidth_map = 4
    elif bandwidth_Hz == 41700:
        bandwidth_map = 5
    elif bandwidth_Hz == 62500:
        bandwidth_map = 6
    elif bandwidth_Hz == 125000:
        bandwidth_map = 7
    elif bandwidth_Hz == 250000:
        bandwidth_map = 8
    elif bandwidth_Hz == 500000:
        bandwidth_map = 9
    else:
        print("Wrong bandwidth: Setting 250 kHz.")
        bandwidth_map = 8
    return bandwidth_map

def setBandwidth(bandwidth):
    # Write the bandwidth into the respective register
    # Input: bandwidth in kHz
    bandwidth_map = mapBandwidth(bandwidth)
    modem_config = (readReg(REG_MODEM_CONFIG) & BANDWIDTH_MASK) | (bandwidth_map << 4)
    writeReg(REG_MODEM_CONFIG, modem_config)

def setCodingRate(coding_rate):
    # Write the coding rate into the respective register
    # Input: coding rate mapping (1: 4/5, to 4: 4/8)
    if (coding_rate < 1):
        print("Input coding rate out of range. Setting maximum value: 4/5.")
        coding_rate = 1
    elif (coding_rate > 4):
        print("Input coding rate out of range. Setting minimum value: 4/8.")
        coding_rate = 4
    modem_config = (readReg(REG_MODEM_CONFIG) & CR_MASK) | (coding_rate << 1)
    writeReg(REG_MODEM_CONFIG, modem_config)

def setHeader(implicit_header):
    # Write the header flag into the respective register:
    # implicit (input: 1), explicit (input: 0)
    if (implicit_header != 0) and (implicit_header != 1):
        print("Expected boolean value for implicit header flag. Setting explicit header : 0.")
        implicit_header = 0
    modem_config = (readReg(REG_MODEM_CONFIG) & HEADER_MASK) | implicit_header
    writeReg(REG_MODEM_CONFIG, modem_config)

def setSF(spreading_factor):
    # Write the spreading factor into the respective register
    # Input: spreading factor (7 to 12)
    if (spreading_factor < 7):
        print("Input spreading factor out of range. Setting minimum standard value: 7.")
        spreading_factor = 7
    elif (spreading_factor > 12):
        print("Input spreading factor out of range. Setting maximum value: 12.")
        spreading_factor = 12
    modem_config2 = (readReg(REG_MODEM_CONFIG2) & SF_MASK) | (spreading_factor << 4)
    writeReg(REG_MODEM_CONFIG2, modem_config2)

def setContMode(tx_continuous_mode):
    # Write the continuous tx mode flag into the respective register:
    # on (input: 1), off (input: 0)
    if (tx_continuous_mode != 0) and (tx_continuous_mode != 1):
        print("Expected boolean value for tx mode flag. Disabling continuous mode: 0.")
        tx_continuous_mode = 0
    modem_config2 = (readReg(REG_MODEM_CONFIG2) & CONT_MODE_MASK) | (tx_continuous_mode << 3)
    writeReg(REG_MODEM_CONFIG2, modem_config2)

def setCRC(crc):
    # Write the cyclic redundancy check flag into the respective register:
    # on (input: 1), off (input: 0)
    if (crc != 0) and (crc != 1):
        print("Expected boolean value for CRC flag. Enabling CRC: 1.")
        crc = 1
    modem_config2 = (readReg(REG_MODEM_CONFIG2) & CRC_MASK) | (crc << 2)
    writeReg(REG_MODEM_CONFIG2, modem_config2)

def setSymbolTimeout(symbol_timeout):
    # Get the total symbol timeout and divide the number into LSB and MSB.
    # Write the two parts of the symbol timeout into the respective registers.
    # Maximum input is 1023 symbols since we have 10 bits in total:
    # - 2 LSB from REG_MODEM_CONFIG2 are the 2 MSB of symbol_timeout
    # - the 8 bits of REG_SYMB_TIMEOUT_LSB are the 8 LSB of symbol_timeout
    if (symbol_timeout < 1):
        print("Input symbol timeout out of range. Setting minimum value: 1.")
        symbol_timeout = 1
    elif (symbol_timeout > 1023):
        print("Input symbol timeout out of range. Setting maximum value: 1023.")
        symbol_timeout = 1023
    LSB_mask = 255 # 0011111111
    MSB_mask = 768 # 1100000000
    symbol_timeout_LSB = symbol_timeout & LSB_mask
    symbol_timeout_MSB = (symbol_timeout & MSB_mask) >> 8
    modem_config2 = (readReg(REG_MODEM_CONFIG2) & SYMB_TIMEOUT_MASK) | symbol_timeout_MSB
    writeReg(REG_MODEM_CONFIG2, modem_config2)
    writeReg(REG_SYMB_TIMEOUT_LSB, symbol_timeout_LSB)

def optimizeLowRate(spreading_factor, bandwidth):
    # Write the low rate optimization flag into the respective register.
    # Optimization is mandated for symbol durations higher than 16 ms:
    # Spreading factor and Bandwidth (in kHz) are needed.
    # Remark: you need to pass the bandwidth in kHz, not the mapping
    symbol_time = pow(2.0, spreading_factor) / bandwidth # ms
    low_rate_optimized = 0          # 0  -> Non-optimized Low data rate
    if (symbol_time > 16):
        low_rate_optimized = 1      # 1  -> Optimized Low data rate
    modem_config3 = (readReg(REG_MODEM_CONFIG3) & LOW_RATE_MASK) | (low_rate_optimized << 3)
    writeReg(REG_MODEM_CONFIG3, modem_config3)

def setSyncWord(sync_word):
    # Set sync word. You might use the LoRaWAN public sync word
    if (sync_word < 0):
        print("Input sync word out of range. Setting LoRa value: 0x12.")
        sync_word = 0x12
    elif (sync_word > 255):
        print("Input sync word out of range. Setting LoRa value: 0x12.")
        sync_word = 0x12
    writeReg(REG_SYNC_WORD, sync_word)

def setPayloadLength(payload_length):
    # Set payload length in bytes
    if (payload_length < 1):
        print("Input payload lengths smaller than one not allowed. Setting to 1.")
        payload_length = 1
    elif (payload_length > 255):
        print("Input payload length out of range. Setting maximum value: 255.")
        payload_length = 255
    writeReg(REG_PAYLOAD_LENGTH, payload_length)

def setMaxPayloadLength(max_payload_length):
    # Set maximum payload length in bytes if header payload length exceeds this value,
    # a header CRC error is generated. Allows filtering of packets with a bad size.
    if (max_payload_length < 1):
        print("Max payload lengths smaller than one not allowed. Setting to default: 255.")
        max_payload_length = 255
    elif (max_payload_length > 255):
        print("Input max payload length out of range. Setting maximum value: 255.")
        max_payload_length = 255
    writeReg(REG_MAX_PAYLOAD_LENGTH, max_payload_length)

def setLNA(LNA_gain, LNA_boost_HF, LNA_AGC):
    # LNA configuration: only for receiver
    # Write LNA parameters into the respective registers:
    # - LNA gain from 1 (maximum) to 6 (minimum)
    # - Current booster for high frequency (3) or no boost (0)
    # - AGC flag: gain set by AGC (1) or not (0)
    if (LNA_gain < 1):
        print("Input LNA gain out of range. Setting maximum value: 1.")
        LNA_gain = 1
    elif (LNA_gain > 6):
        print("Input LNA gain out of range. Setting minimum value: 6.")
        LNA_gain = 6
    if (LNA_boost_HF != 0) and (LNA_boost_HF != 3):
        if (LNA_boost_HF != 1):
            print("Expected boolean value for LNA boost (false:0 or true:1/3). Setting default: 0.")
            LNA_boost_HF = 0
        else:
            LNA_boost_HF = 3
    if (LNA_AGC != 0) and (LNA_AGC != 1):
        print("Expected boolean value for LNA AGC flag. Disabling AGC: 0.")
        LNA_AGC = 0
    modem_config3 = (readReg(REG_MODEM_CONFIG3) & AGC_MASK) | (LNA_AGC << 2)
    writeReg(REG_MODEM_CONFIG3, modem_config3)
    # amplifier gain cannot be modified in sleep mode
    opmode(OPMODE_STANDBY)
    LNA_value = (LNA_gain << 5) | LNA_boost_HF
    writeReg(REG_LNA, LNA_value)

def setPaRamp(ramp_up_time):
    # Set PA ramp-up time: only for transmitter
    # Input: mapping from 0 to 15
    if (ramp_up_time < 0) or (ramp_up_time > 15):
        print("Input ramp up time out of range. Setting default value: 40 us.")
        ramp_up_time = 9
    writeReg(REG_PA_RAMP, (readReg(REG_PA_RAMP) & 0xF0) | ramp_up_time)

def configPower(output_power):
    if (output_power > 17):
        output_power = 17
    elif (output_power < 2):
        output_power = 2
    # register accepts a value up to 15 and then the chipset 
    # performs the operation to set the wanted power
    out = output_power - 2
    MaxPow = 7
    writeReg(REG_PA_CONFIG, (0x80|MaxPow<<4|out))
    writeReg(REG_PA_DAC, 0x84)

def transmit(frame):
    # Enter standby mode (required for FIFO loading))
    opmode(OPMODE_STANDBY)
    # Set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
    writeReg(REG_DIO_MAPPING_1, MAP_DIO0_LORA_TXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP)
    # Clear all radio IRQ flags
    writeReg(REG_IRQ_FLAGS, 0xFF)
    # Mask all IRQs but TxDone
    writeReg(REG_IRQ_FLAGS_MASK, (~IRQ_LORA_TXDONE_MASK)&255)
    # Initialize the payload size and address pointers
    writeReg(REG_FIFO_TX_BASE_AD, 0x00)
    writeReg(REG_FIFO_ADDR_PTR, 0x00)
    if (len(frame) > MAX_FRAME_LEN):
        frame = frame[0:MAX_FRAME_LEN]
    setPayloadLength(len(frame))
    # Download buffer to the radio FIFO
    writeBuf(REG_FIFO, frame)
    # Start the transmission
    print("Payload:", frame)
    opmode(OPMODE_TX)
    # End the transmission

def packet_received():
    payload = ""
    receivedbytes = 0
    packet_flag = False
    irqflags = readReg(REG_IRQ_FLAGS)
    # Clear rxDone
    writeReg(REG_IRQ_FLAGS, 0x40)
    # Read irqflags
    if (irqflags and 0x10):
        print("Explicit header")
    else:
        print("Implicit header")
    if (readReg(REG_IRQ_FLAGS) & 0x40):
        print("CRC: on")
    else:
        print("CRC: off")
    print("Coding rate:", (readReg(REG_MODEM_STAT)&0xE0)>>5)
    print("Payload length:", readReg(REG_RX_NB_BYTES))
    #  payload crc: 0x20
    if ((irqflags & 0x20) == 0x20):
        print("CRC error")
        writeReg(REG_IRQ_FLAGS, 0x20)
        return packet_flag, payload, receivedbytes
    else:
        currentAddr = readReg(REG_FIFO_RX_CURRENT_ADDR)
        receivedbytes = readReg(REG_RX_NB_BYTES)
        writeReg(REG_FIFO_ADDR_PTR, currentAddr)
        for x in range(receivedbytes):
            payload = payload + chr(readReg(REG_FIFO))
        packet_flag = True
        return packet_flag, payload, receivedbytes

def receive():
    opmode(OPMODE_RX)
    SNR = 0
    rssicorr = 157
    while(1):
        if(input_gpio(DIO0) == 1):
            packet_flag, rx_message, receivedbytes = packet_received()
            if(packet_flag):
                # Received a message
                value = readReg(REG_PKT_SNR_VALUE)
                if (value & 0x80):
                    # The SNR sign bit is 1
                    # Invert and divide by 4
                    value = ( ( ~value + 1 ) & 0xFF ) >> 2
                    SNR = -value
                else:
                    # Divide by 4
                    SNR = ( value & 0xFF ) >> 2
                print("Packet RSSI:", (readReg(REG_PKT_RSSI)-rssicorr))
                print("RSSI:", readReg(REG_RSSI)-rssicorr)
                print("SNR:", SNR)
                print("Payload length:", receivedbytes)
                print("Message: ")
                print(rx_message)
        time.sleep(1)