__author__ = 'mangokid'

import Slush.Boards.SlushEngine_ModelX as SLX
from Slush.Base import *

class sBoard:

  def __init__(self):
    """ initalize all of the controllers peripheral devices
    """
    self.initSPI()
    self.initGPIOState()
    self.initI2C()    
	       
  def initGPIOState(self):
    """sets the default states for the GPIO on the slush modules. *This
    is currently only targeted at the Raspberry Pi. Other target devices
    will be added in a similar format.
    """  
    gpio.setmode(gpio.BCM) 

    #common motor reset pin
    gpio.setup(SLX.L6470_Reset, gpio.OUT)
    
    #chip select pins, must all be low or SPI will com fail
    gpio.setup(SLX.MTR0_ChipSelect, gpio.OUT)
    gpio.setup(SLX.MTR1_ChipSelect, gpio.OUT)
    gpio.setup(SLX.MTR2_ChipSelect, gpio.OUT)
    gpio.setup(SLX.MTR3_ChipSelect, gpio.OUT)
    gpio.output(SLX.MTR0_ChipSelect, gpio.HIGH)
    gpio.output(SLX.MTR1_ChipSelect, gpio.HIGH)
    gpio.output(SLX.MTR2_ChipSelect, gpio.HIGH)
    gpio.output(SLX.MTR3_ChipSelect, gpio.HIGH)
    
    #IO expander reset pin
    gpio.setup(SLX.MCP23_Reset, gpio.OUT)
    gpio.output(SLX.MCP23_Reset, gpio.HIGH)

    #preforma a hard reset
    gpio.output(SLX.L6470_Reset, gpio.LOW)
    time.sleep(.1)
    gpio.output(SLX.L6470_Reset, gpio.HIGH)    
    time.sleep(.1)  
  
  def initSPI(self):
    """ initalizes the spi for use with the motor driver modules
    """    
    sBoard.spi = spidev.SpiDev()
    sBoard.spi.open(0,0)
    sBoard.spi.max_speed_hz = 100000
    sBoard.spi.bits_per_word = 8
    sBoard.spi.loop = False
    sBoard.spi.mode = 3
    
  def initI2C(self):
    """ initalizes the i2c bus without relation to any of its slaves
    """
    pass

  def deinitBoard(self):
    """ closes the board and deinits the peripherals
    """
    gpio.cleanup() 

  def setIOState(self, port, pinNumber, state):
    """ sets the output state of the industrial outputs on the SlushEngine. This
    currentley does not support the digitial IO
    """
    with closing(i2c.I2CMaster(1)) as bus:
        chip = MCP23017(bus, 0x20)
        chip.reset()
        industrialOutput = chip[port][pinNumber]
        industrialOutput.direction = Out
        industrialOutput.value = state

  def getIOState(self, port, pinNumber):
    """ sets the output state of the industrial outputs on the SlushEngine. This
    currentley does not support the digitial IO
    """
    with closing(i2c.I2CMaster(1)) as bus:
        chip = MCP23017(bus, 0x20)
        chip.reset()
        industrialInput = chip[port][pinNumber]
        industrialInput.direction = In
        industrialInput.pull_up = True
        state = industrialInput.value

    return state


    
