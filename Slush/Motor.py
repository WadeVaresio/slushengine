"""
@file Motor.py Responsible for all stepper control functionality.
Source Reference: https://github.com/ameyer/Arduino-L6470/blob/master/L6470/L6470.cpp
"""

from Slush.Board import *
from Slush.Devices import L6470Registers as LReg
from Slush.Devices import L6480Registers as LReg6480
from Slush.Boards.BoardUtilities import BoardTypes
import math
import warnings


class Motor(sBoard):
    """
    Class to control a stepper motor
    """

    """ Dictionary holding all of the associated motor chip selects for a given port"""
    chip_assignments = {0: SLX.MTR0_ChipSelect, 1: SLX.MTR1_ChipSelect, 2: SLX.MTR2_ChipSelect, 3: SLX.MTR3_ChipSelect,
                        4: SLX.MTR4_ChipSelect, 5: SLX.MTR5_ChipSelect, 6: SLX.MTR6_ChipSelect}

    boardInUse = 0
    
    def __init__(self, motorNumber: int):
        """
        Setup a motor for
        :param motorNumber: Port the motor has been plugged into on the Slush Engine
        """
        super().__init__()

        # Assign chipSelect from chip_assignments dictionary
        try:
            self.chipSelect = Motor.chip_assignments[motorNumber]
        except KeyError:
            raise ValueError("The given motor number is not acceptable")

        # init the hardware
        self.initPeripherals()
        self.init_chips()
        self.chip_select = None

    def init_chips(self) -> None:
        """
        Initialize all of the stepper motor chips. Set each chips ALARM register to ignore UVLO events.

        Setup the gpio flag pin to be an input and setup event detection.
        :return: None
        """
        original_chip_select = self.chipSelect  # save original chipSelect as it will be changed

        for chip in Motor.chip_assignments.values():
            self.chipSelect = chip  # xfer uses the current chipSelect so we must change it

            gpio.setup(chip, gpio.OUT)
            self.setParam(LReg.ALARM_EN, 0xF7)  # ignore UVLO events
            self.getParam(LReg.STATUS)  # get status to ensure the UVLO status bit is high

        self.chipSelect = original_chip_select  # set the chipSelect back to original

        # Add event detection on the chip monitoring pin
        gpio.remove_event_detect(CHIP_MONITORING_PIN)  # remove any previous event detects (multiple motor setups)
        gpio.setup(CHIP_MONITORING_PIN, gpio.IN)  # setup the flag pin to be an input
        gpio.add_event_detect(CHIP_MONITORING_PIN, gpio.FALLING, callback=lambda channel: self.gpio_callback())  # add event detect on falling edge of pin

    def gpio_callback(self) -> None:
        """
        Function called when the trigger pin has been activated. References self.debug which is inherited from Slush.Board
        If the debug level is "OFF" nothing will happen
        If the debug level is "LOW" there will be a console print notifying of the trigger
        If the debug level is "HIGH" all motors will be freed and the program will exit
        :return: None
        """
        # self.debug is inherited from sBoard
        if self.debug == "OFF":
            return
        if self.debug == 'LOW':
            warnings.warn("THE SLUSH FLAG PIN HAS BEEN ACTIVATED CONSIDER CHANGING MOTOR PARAMETERS", RuntimeWarning)
            return
        if self.debug == "HIGH":
            for chip in Motor.chip_assignments.values():  # Free all motor chips
                self.chip_select = chip
                self.xfer(LReg.HARD_HIZ)
            sys.exit("THE SLUSH FLAG PIN WAS ACTIVATED, CHANGE MOTOR VALUES, ALL MOTORS HAVE BEEN FREED")

    def initPeripherals(self) -> None:
        """
        Initialize the appropriate pins and busses
        :return: None
        """
        # check that the motors SPI is actually working
        if self.getParam(LReg.CONFIG) == 0x2e88:
            print("Motor Drive Connected on GPIO " + str(self.chipSelect))
            self.boardInUse = BoardTypes.XLT
        elif self.getParam(LReg6480.CONFIG) == 0x2c88:
            print("High Power Drive Connected on GPIO " + str(self.chipSelect))
            self.boardInUse = BoardTypes.D
        else:
            print("communication issues; check SPI configuration and cables")

        # based on board type init driver accordingly
        if self.boardInUse == BoardTypes.XLT:
            self.setOverCurrent(2000)
            self.setMicroSteps(16)
            self.setCurrent(70, 90, 100, 100)
            self.setParam(LReg.CONFIG, 0x3688)  # changed 0x3608 to 0x3688 enable OC_SD - shutdown driver if over-current
        if self.boardInUse == BoardTypes.D:
            self.setParam(LReg6480.CONFIG, 0x3688)  # changed 0x3608 to 0x3688 enable OC_SD - shutdown driver if over-current
            self.setCurrent(100, 120, 140, 140)
            self.setMicroSteps(16)
            # New to configure GATECFG1 and OCD_TH
            self.setParam(LReg6480.GATECFG1, 0x5f)  # Igate = 8mA and tcc=3750nS(max)
            self.setParam(LReg6480.OCD_TH, 0x1f)  # OCD_Th 1V (max)

        self.getStatus()
        self.free()
        
    def isBusy(self) -> bool:
        """
        Check to see if the motion engine is busy
        :return: Bool representing whether the motion engine is busy
        """
        status = self.getStatus()
        return (not ((status >> 1) & 0b1))

    def waitMoveFinish(self) -> None:
        """
        Wait for the motor to finish moving.
        NOTE: This method is blocking
        :return: None
        """
        status = 1
        while status:
            status = self.getStatus()
            status = not((status >> 1) & 0b1)

    def setMicroSteps(self, microSteps: int):
        """
        Set the microstepping level
        :param microSteps: The number of microsteps the motor should run at. Should be an int of base 2 from [2,128]
        :return:
        """
        self.free()
        stepVal = 0

        for stepVal in range(0, 8):
            if microSteps == 1:
                break
            microSteps = microSteps >> 1;
            
        self.setParam(LReg.STEP_MODE, (0x00 | stepVal | LReg.SYNC_SEL_1))

    def setThresholdSpeed(self, thresholdSpeed) -> None:
        """
        Set the threshold speed of the motor
        :param thresholdSpeed: Threshold speed the motor is allowed to run at
        :return: None
        """
        if thresholdSpeed == 0:
            self.setParam(LReg.FS_SPD, 0x3ff)
        else:
            self.setParam(LReg.FS_SPD, self.fsCalc(thresholdSpeed))

    def setCurrent(self, hold, run, acc, dec) -> None:
        """
        Set the motors current values
        :param hold: Amount of hold current
        :param run: Amount of run current
        :param acc: Amount of acceleration current
        :param dec: Amount of decceleration current
        :return: None
        """
        self.setParam(LReg.KVAL_RUN, run)
        self.setParam(LReg.KVAL_ACC, acc)
        self.setParam(LReg.KVAL_DEC, dec)
        self.setParam(LReg.KVAL_HOLD, hold)

    def setMaxSpeed(self, speed) -> None:
        """
        Set the maximum speed the motor will run at
        :param speed: Motor's maximum speed
        :return: None
        """
        self.setParam(LReg.MAX_SPEED, self.maxSpdCalc(speed))

    def setMinSpeed(self, speed: float) -> None:
        """
        Set the minimum speed the motor will run at
        :type speed: float
        :param speed: Motor's minimum speed
        :return: None
        """
        self.setParam(LReg.MIN_SPEED, self.minSpdCalc(speed))

    def setAccel(self, acceleration: float) -> None:
        """
        Set the acceleration of the stepper motor
        :type acceleration: float
        :param acceleration: Acceleration value
        :return: None
        """
        accelerationBytes = self.accCalc(acceleration)
        self.setParam(LReg.ACC, accelerationBytes)

    def setDecel(self, deceleration: float) -> None:
        """
        Set the deceleration of the stepper motor
        :type deceleration: float
        :param deceleration: Deceleration value
        :return:
        """
        decelerationBytes = self.decCalc(deceleration)
        self.setParam(LReg.DEC, decelerationBytes)

    def getPosition(self) -> float:
        """
        Get the current position of the stepper motor, based upon it's home value
        :rtype:float
        :return: The current position of the stepper motor, based upon it's home value
        """
        return self.convert(self.getParam(LReg.ABS_POS))

    def getSpeed(self) -> int:
        """
        Get the speed of the motor
        :rtype: int
        :return: The speed of the motor
        """
        return self.getParam(LReg.SPEED)

    ''' set the overcurrent threshold '''
    def setOverCurrent(self, ma_current) -> None:
        """
        Set the over current threshold.
        :param ma_current: The over current threshold amount in milliamps
        :return: None
        """
        OCValue = math.floor(ma_current/375)
        if OCValue > 0x0f: OCValue = 0x0f
        self.setParam((LReg.OCD_TH), OCValue)

    def setStallCurrent(self, ma_current):
        """
        Set the stall current
        :param ma_current: Stall current amount in milliamps
        :return: None
        """
        STHValue = round(math.floor(ma_current/31.25))
        if(STHValue > 0x80): STHValue = 0x80
        if(STHValue < 0): STHValue = 9
        self.setParam((LReg.STALL_TH), STHValue)

    def setLowSpeedOpt(self, enable: bool) -> None:
        """
        Set whether you want the stepper motor to be optimized for low speed
        :param enable: Boolean whether to enable low speed optimization
        :return: None
        """
        self.xfer(LReg.SET_PARAM | LReg.MIN_SPEED[0])
        if enable:
            self.param(0x1000, 13)
        else:
            self.param(0, 13)

    ''' set slope speeds to eliminate BEMF'''

    def setSlope(self, speed, start, acc, dec):
        self.setParam(LReg.INT_SPD, speed)
        self.setParam(LReg.ST_SLP, start)
        self.setParam(LReg.FN_SLP_ACC, acc)
        self.setParam(LReg.FN_SLP_DEC, dec)

    def run(self, dir: int, spd) -> None:
        """
        Run the motor indefinitely in a given direction with a given speed
        :type dir: int
        :param dir: Direction the stepper motor will run in (0 or 1) 0 being clockwise
        :param spd: Speed the stepper motor will run at
        :return: None
        """
        speedVal = self.spdCalc(spd)
        self.xfer(LReg.RUN | dir)
        if speedVal > 0xfffff: speedVal = 0xfffff
        self.xfer(speedVal >> 16)
        self.xfer(speedVal >> 8)
        self.xfer(speedVal)

    def stepClock(self, dir: int) -> None:
        """
        Set the clock source
        :type dir: int
        :param dir: direction to send when setting the clock source
        :return: None
        """
        self.xfer(LReg.STEP_CLOCK | dir)

    def move(self, nStep: int) -> None:
        """
        Move the stepper motor a given number of steps
        :type nStep: int
        :param nStep: Number of steps the stepper motor will move
        :return: None
        """
        dir = 0

        if nStep >= 0:
            dir = LReg.FWD
        else:
            dir = LReg.REV

        n_stepABS = abs(nStep)

        self.xfer(LReg.MOVE | dir)
        if n_stepABS > 0x3fffff: nStep = 0x3fffff
        self.xfer(n_stepABS >> 16)
        self.xfer(n_stepABS >> 8)
        self.xfer(n_stepABS)

    def goTo(self, pos: float) -> None:
        """
        Go to a given position based upon where the stepper motor currently is
        :type pos: float
        :param pos: Position for the stepper motor to go to
        :return: None
        """
        self.xfer(LReg.GOTO)
        if pos > 0x3fffff: pos = 0x3fffff
        self.xfer(pos >> 16)
        self.xfer(pos >> 8)
        self.xfer(pos)

    ''' same as go to but with a forced direction '''
    def goToDir(self, dir: int, pos: float):
        """
        Go to a position based upon where the stepper motor is. Go to this position with a forced direction
        :type dir: int
        :param dir: Direction the stepper motor will run when going to the given position
        :type pos: float
        :param pos: Position for the stepper motor to go
        :return: None
        """
        self.xfer(LReg.GOTO_DIR)
        if pos > 0x3fffff: pos = 0x3fffff
        self.xfer(pos >> 16)
        self.xfer(pos >> 8)
        self.xfer(pos)

    def setLimitHardStop(self, stop: bool) -> None:
        """
        Set whether the stepper motor should stop when it hits a limit switch.
        This is useful when a stepper motor needs to go past it's limit switch during operation, where the limit switch
        serves as a homing mechanism.
        :type stop: bool
        :param stop: Boolean whether the stepper motor should stop when it hits a limit switch
        :return: None
        """
        if self.boardInUse is BoardTypes.XLT:
            if stop == 1: self.setParam([0x18, 16], 0x3688)  # changed 0x3608 and 0x3818 to 0x3688 and 0x3698
            if stop == 0: self.setParam([0x18, 16], 0x3698)  # to enable OC_SD - shutdown driver if over-current
        if self.boardInUse is BoardTypes.D:
            if stop == 1: self.setParam([0x1A, 16], 0x3688)  # changed 0x3608 and 0x3818 to 0x3688 and 0x3698
            if stop == 0: self.setParam([0x1A, 16], 0x3698)  # to enable OC_SD - shutdown driver if over-current

    ''' go until switch press event occurs '''
    def goUntilPress(self, act, dir, spd):
        self.xfer(LReg.GO_UNTIL | act | dir)
        if spd > 0x3fffff: spd = 0x3fffff
        self.xfer(spd >> 16)
        self.xfer(spd >> 8)
        self.xfer(spd)

    def getSwitch(self):
        if self.getStatus() & 0x4: return 1
        else: return 0

    def goUntilRelease(self, act, dir) -> None:
        """
        Move the stepper motor until the attached switch has been released
        :param act:
        :param dir: Direction to move the motor (0 or 1)
        :return: None
        """
        self.xfer(LReg.RELEASE_SW | act | dir)

    def readSwitch(self) -> bool:
        """
        Read the value of the attached switch to the stepper motor
        :rtype: bool
        :return: Boolean whether the switch is depressed
        """
        if self.getStatus() & 0x4: return 1
        else: return 0

    def goHome(self) -> None:
        """
        Move the stepper motor to it's home position
        :return: None
        """
        self.xfer(LReg.GO_HOME)

    def goMark(self) -> None:
        """
        Move the stepper motor to it's mark position
        :return: None
        """
        self.xfer(LReg.GO_MARK)

    def setMark(self, value) -> None:
        """
        Set the stepper motor's mark value
        :param value: Value to set mark at
        :return: None
        """
        if value == 0: value = self.getPosition()
        self.xfer(LReg.MARK)
        if value > 0x3fffff: value = 0x3fffff
        if value < -0x3fffff: value = -0x3fffff

        self.xfer(value >> 16)
        self.xfer(value >> 8)
        self.xfer(value)

    def setAsHome(self) -> None:
        """
        Set the stepper motor's current position to be it's new home position
        :return: None
        """
        self.xfer(LReg.RESET_POS)

    def resetDev(self):
        """
        NOTE: This method has been deprecated as it interferes with GPIO chip monitoring
        Reset the device to initial conditions.
        :return:
        """
        raise DeprecationWarning("This method is no longer in use as it interferes with GPIO Flag Pin monitoring")

    def softStop(self) -> None:
        """
        Stop the stepper motor by decelerating
        :return: None
        """
        self.xfer(LReg.SOFT_STOP)

    def hardStop(self) -> None:
        """
        Immediately stop the stepper motor without deceleration
        :return: None
        """
        self.xfer(LReg.HARD_STOP)

    def softFree(self) -> None:
        """
        Decelerate the stepper motor and disable hold
        :return: None
        """
        self.xfer(LReg.SOFT_HIZ)

    def free(self) -> None:
        """
        Disable the stepper motor, allow it to move freely
        :return: None
        """
        self.xfer(LReg.HARD_HIZ)

    def getStatus(self) -> int:
        """
        Get the status of the stepper motor.
        :rtype: int
        :return: Decimal representation of the stepper motors status register
        """
        temp = 0;
        self.xfer(LReg.GET_STATUS)
        temp = self.xfer(0) << 8
        temp += self.xfer(0)
        return temp

    def accCalc(self, stepsPerSecPerSec: int) -> int:
        """
        Calculates the value of the ACC register
        :type stepsPerSecPerSec: int
        :param stepsPerSecPerSec: stepper motors acceleration units of steps/sec^2
        :rtype: int
        :return: Calculated acceleration register
        """
        temp = float(stepsPerSecPerSec) * 0.137438
        if temp > 4095.0: return 4095
        else: return round(temp)

    def decCalc(self, stepsPerSecPerSec: int):
        """
        Calculated the value of the DEC register
        :type stepsPerSecPerSec: int
        :param stepsPerSecPerSec: stepper motors acceleration in units of steps/sec^2
        :return: Calculated value of the deceleration register
        """
        temp = float(stepsPerSecPerSec) * 0.137438
        if temp > 4095.0: return 4095
        else: return round(temp)

    def maxSpdCalc(self, stepsPerSec: int) -> int:
        """
        Calculates the max speed register
        :type stepsPerSec: int
        :param stepsPerSec: Stepper motors velocity in units of steps/sec
        :rtype: int
        :return: Max speed register calculation
        """
        temp = float(stepsPerSec) * 0.065536
        if temp > 1023.0: return 1023
        else: return round(temp)

    def minSpdCalc(self, stepsPerSec: int) -> int:
        """
        Calculates the min speed register
        :type stepsPerSec: int
        :param stepsPerSec: Stepper motor's velocity in units of steps/sec
        :return: Min speed register calculation
        """
        temp = float(stepsPerSec) * 4.1943
        if temp > 4095.0: return 4095
        else: return round(temp)

    def fsCalc(self, stepsPerSec: int) -> int:
        """
        Calculates the value of the FS speed register
        :type stepsPerSec: int
        :param stepsPerSec: Stepper motor's velocity in units of steps/sec
        :rtype: int
        :return: FS speed register calculation
        """
        temp = (float(stepsPerSec) * 0.065536) - 0.5
        if temp > 1023.0: return 1023
        else: return round(temp)

    def intSpdCalc(self, stepsPerSec: int) -> int:
        """
        Calculates the value of the INT speed register
        :type stepsPerSec: int
        :param stepsPerSec: Stepper motor's velocity in units of steps/sec
        :rtype: int
        :return: INT speed register calculation
        """
        temp = float(stepsPerSec) * 4.1943
        if temp > 16383.0: return 16383
        else: return round(temp)

    def spdCalc(self, stepsPerSec: int) -> int:
        """
        Calculate the stepper motors speed
        :type stepsPerSec: int
        :param stepsPerSec: stepper motors velocity in units of steps/sec
        :rtypeL int
        :return: Stepper motors calculated speed
        """
        temp = float(stepsPerSec) * 67.106
        if temp > float(0x000fffff): return 0x000fffff
        else: return round(temp)

    def param(self, value, bit_len: int):
        """
        Utility function to get a parameter from a register
        :param value: Value to get from the bit
        :param bit_len: Length of the bit
        :return: Value read from the register
        """
        ret_value = 0

        byte_len = bit_len/8
        if (bit_len%8 > 0): byte_len +=1

        mask = 0xffffffff >> (32 - bit_len)
        if value > mask: value = mask

        if byte_len >= 3.0:
            temp = self.xfer(value >> 16)
            ret_value |= temp << 16
        if byte_len >= 2.0:
            temp = self.xfer(value >>8)
            ret_value |= temp << 8
        if byte_len >= 1.0:
            temp = self.xfer(value)
            ret_value |= temp
       
        return (ret_value & mask)

    def xfer(self, data):
        """
        Transfer data to the spi bus
        :param data: data to transfer
        :return: Response from the spi bus
        """
        gpio.setmode(gpio.BCM)
        #mask the value to a byte format for transmission
        data = (int(data) & 0xff)

        #toggle chip select and SPI transfer
        gpio.output(self.chipSelect, gpio.LOW)
        response = sBoard.spi.xfer2([data])
        gpio.output(self.chipSelect, gpio.HIGH)        

        return response[0]

    ''' set a parameter of the motor driver '''
    def setParam(self, param, value):
        self.xfer(LReg.SET_PARAM | param[0])
        return self.paramHandler(param, value)

    def getParam(self, param: tuple) -> int:
        """
        Get a parameter from the motor driver
        :param param: Parameter to get from the motor driver
        :return: Decimal representation of the bit
        """
        self.xfer(LReg.GET_PARAM | param[0])
        return self.paramHandler(param, 0)

    def convert(self, val) -> int:
        """
        Convert twos compliment
        :param val: value to convert
        :return: Values two compliment
        """
        if val > 0x400000/2:
            val = val - 0x400000
        return val

    def paramHandler(self, param: tuple, value: int):
        """
        Switch case to handle parameters
        :param param: Parameter to handle
        :param value: value passed to param() to handle
        :return: decimal representation of the bit
        """
        return self.param(value, param[1])
