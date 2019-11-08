import Slush.Boards.SlushEngine_ModelX as SLX
from Slush.Base import *
from enum import Enum


class BoardTypes(Enum):
    """
    Defined Slush Board types
    """
    XLT = 0
    D = 1


class sBoard:
    chip = 0
    bus = 0
    xlt_chip_selects = [SLX.MTR0_ChipSelect, SLX.MTR1_ChipSelect, SLX.MTR2_ChipSelect, SLX.MTR3_ChipSelect]

    def __init__(self, board='XLT', debug="LOW"):
        """ initalize all of the controllers peripheral devices
        """
        gpio.setwarnings(False)
        self.board = board
        self.debug = debug
        self.initSPI()
        self.initGPIOState()
        self.initI2C()

    def initGPIOState(self):
        """sets the default states for the GPIO on the slush modules. *This
        is currently only targeted at the Raspberry Pi. Other target devices
        will be added in a similar format.
        """
        gpio.setmode(gpio.BCM)

        # common motor reset pin
        gpio.setup(SLX.L6470_Reset, gpio.OUT)

        # chip select pins, must all be low or SPI will com fail
        if self.board is 'XLT':
            for chip in sBoard.xlt_chip_selects:
                gpio.setup(chip, gpio.OUT)
                gpio.output(chip, gpio.HIGH)

            # IO expander reset pin
            gpio.setup(SLX.MCP23_Reset, gpio.OUT)
            gpio.output(SLX.MCP23_Reset, gpio.HIGH)

        elif self.board is 'D':
            gpio.setup(SLX.MTR0_ChipSelect, gpio.OUT)
            gpio.setup(SLX.MTR1_ChipSelect, gpio.OUT)
            gpio.setup(SLX.MTR2_ChipSelect, gpio.OUT)
            gpio.setup(SLX.MTR3_ChipSelect, gpio.OUT)
            gpio.setup(SLX.MTR4_ChipSelect, gpio.OUT)
            gpio.setup(SLX.MTR5_ChipSelect, gpio.OUT)
            gpio.setup(SLX.MTR6_ChipSelect, gpio.OUT)
            gpio.output(SLX.MTR0_ChipSelect, gpio.HIGH)
            gpio.output(SLX.MTR1_ChipSelect, gpio.HIGH)
            gpio.output(SLX.MTR2_ChipSelect, gpio.HIGH)
            gpio.output(SLX.MTR3_ChipSelect, gpio.HIGH)
            gpio.output(SLX.MTR4_ChipSelect, gpio.HIGH)
            gpio.output(SLX.MTR5_ChipSelect, gpio.HIGH)
            gpio.output(SLX.MTR6_ChipSelect, gpio.HIGH)
        else:
            raise Exception('Board should be ''XLT'' or ''D''')

        # preforma a hard reset
        gpio.output(SLX.L6470_Reset, gpio.LOW)
        time.sleep(.1)
        gpio.output(SLX.L6470_Reset, gpio.HIGH)
        time.sleep(.1)

    def initSPI(self):
        """ initalizes the spi for use with the motor driver modules
        """
        sBoard.spi = spidev.SpiDev()
        """Changed sBoard.spi.open(0,0) to sBoard.spi.open(0,1)
        Tells the RPi to use CS1 for the SlushEngine versus CS0
        The RPiMIB uses CS0, and the Slush UEXT connector also uses CS0  
        """
        sBoard.spi.open(0, 1)
        sBoard.spi.max_speed_hz = 100000
        sBoard.spi.bits_per_word = 8
        sBoard.spi.loop = False
        sBoard.spi.mode = 3

    def initI2C(self):
        """ initalizes the i2c bus without relation to any of its slaves
        """
        self.bus = SMBus.SMBus(1)

        try:
            with closing(i2c.I2CMaster(1)) as bus:
                self.chip = MCP23017(bus, 0x20)
                self.chip.reset()
        except:
            pass

    def deinitBoard(self):
        """ closes the board and deinits the peripherals
        """
        gpio.cleanup()

    def setIOState(self, port, pinNumber, state):

        """ sets the output state of the industrial outputs on the SlushEngine. This
        currentley does not support the digitial IO
        """
        if port == 0:
            self.bus.write_byte_data(0x20, 0x00, 0x00)
            current = self.bus.read_byte_data(0x20, 0x12)
            if state:
                self.bus.write_byte_data(0x20, 0x12, current | (0b1 << pinNumber))
            else:
                self.bus.write_byte_data(0x20, 0x12, current & (0b1 << pinNumber) ^ current)
        elif port == 1:
            self.bus.write_byte_data(0x20, 0x00, 0x00)
            current = self.bus.read_byte_data(0x20, 0x13)
            if state:
                self.bus.write_byte_data(0x20, 0x13, current | (0b1 << pinNumber))
            else:
                self.bus.write_byte_data(0x20, 0x13, current & (0b1 << pinNumber) ^ current)

    def getIOState(self, port, pinNumber):
        """ sets the output state of the industrial outputs on the SlushEngine. This
        currently does not support the digitial IO
        """
        if port == 0:
            self.bus.write_byte_data(0x20, 0x0C, 0xff)
            self.bus.write_byte_data(0x20, 0x00, 0b1 << pinNumber)
            state = self.bus.read_byte_data(0x20, 0x12)
        elif port == 1:
            self.bus.write_byte_data(0x20, 0x0D, 0xff)
            self.bus.write_byte_data(0x20, 0x01, 0b1 << pinNumber)
            state = self.bus.read_byte_data(0x20, 0x13)
        return (state != 0b1 << pinNumber)

    def readInput(self, inputNumber):
        """ sets the input to digital with a pullup and returns a read value
        """
        self.bus.write_byte_data(0x17, inputNumber + 8, 0x00)  # recent version of Slush Engine has inputNumber + 20
        return self.bus.read_byte_data(0x17, inputNumber + 20)

    def setOutput(self, outputNumber, state):
        """ sets the output state of the IO to digital and then sets the state of the
        pin
        """
        self.bus.write_byte_data(0x17, outputNumber, 0x00)
        self.bus.write_byte_data(0x17, outputNumber + 12, state)

    def readAnalog(self, inputNumber):
        """ sets the IO to analog and then returns a read value (10-bit)
        """
        self.bus.write_byte_data(0x17, inputNumber + 8, 0x01)
        result1 = self.bus.read_byte_data(0x17, inputNumber + 20)
        result2 = self.bus.read_byte_data(0x17, inputNumber + 20 + 4)
        return (result1 << 2) + result2

    def setPWMOutput(self, outputNumber, pwmVal):
        """ sets the output to PWM (500Hz) and sets the duty cycle to % PWMVal/255
        """
        self.bus.write_byte_data(0x17, outputNumber, 0x01)
        self.bus.write_byte_data(0x17, outputNumber + 12, pwmVal)
