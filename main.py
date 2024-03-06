# Coded by Tea S.
# 2022

# @formatter:on
from random import randint

from collections import namedtuple
from machine import Pin, PWM, freq
import utime
import neopixel
import json
import os
import time
import math
# Overlock pico to max cpu freq
# freq(180000000)

# Hardcoded config file for if it doesnt exist on the pico
config = {"buttons":
    {
        "trigger1": 2,
        "mag": 3, },
    "lights": {
        'core': {'length': 5, 'pin': 4, 'flicker': False},
        'mag': {'length': 7, 'pin': 6, 'flicker': False},
        'frame1': {'length': 7, 'pin': 7, 'flicker': False},
        'frame2': {'length': 11, 'pin': 8, 'flicker': False}
    },
    "colors": {
        'heating': [[255, 80, 0], [0, 0, 0]],
        'normal': [[72, 209, 104], [255, 255, 255]],
        'hot': [[225, 0, 0], [1, 1, 1]],
        'flash': [[200, 255, 255], [15, 15, 15]],
        'yellow': [[225, 255, 0], [15, 15, 15]],
        'red': [[255, 0, 0], [15, 15, 15]],
    },
    "servos": {"recoil": {"invert": True, "start": 10, "speed": 50, "curpos": 50, "end": 160, "pin": 10},
               "mag-open": {"invert": False, "start": 10, "speed": 50, "curpos": 50, "end": 75, "pin": 12},
               "mag-rotate": {"invert": True, "start": 5, "speed": 100, "curpos": 1, "end": 180, "pin": 14}},
    "state": "Main",
    "maxAmmo": "10"}

# Load config file from system (it holds all the servo settings and current position so the servos dont judder when it turns back on)
# if "config" in os.listdir():
#     try:
#         file = open("config", "r")
#         readConfig = json.loads(file.readline())
#         if readConfig != config:
#             print("Config file changed, recreating")
#             file = open("config", "wb")
#             file.write(json.dumps(config))
#         else:
#             config = readConfig
#         file.close()
#     except ValueError:
#         # Problem reading config file, recreate it using the above hardcoded config file
#         print("Unable to read config file...Recreating")
#         file = open("config", "wb")
#         file.write(json.dumps(config))
#         file.close()
# else:
#     # Create config file if it doesn't exist
#     file = open("config", "wb")
#     file.write(json.dumps(config))
#     file.close()


def saveSettings(wherefrom=""):
    print("save - " + wherefrom)
    print(config)
    file = open("config", "wb")
    file.write(json.dumps(config))
    file.close()


print(config)

# ===========================START OF CLASSES ==================================


# Class for controlling and setting servo settings, pass in a config file with all the settings of the servo
class Servo:
    def __init__(self, config):

        self.speed = config['speed']
        self.posMin = config['start']
        self.posMax = config['end']
        self.pin = config['pin']
        self.invert = config['invert']
        self.config = config
        self.current_us = 0.0
        self.targetAngle = config['curpos']
        self.angle = config['curpos']
        self.pwm = PWM(Pin(self.pin, Pin.OUT))
        self.pwm.freq(50)
        self.min_us=544.0
        self.max_us=2400.0
        self.min_deg = 0
        self.max_deg = 180
        self._slope = (self.min_us-self.max_us)/(math.radians(self.min_deg)-math.radians(self.max_deg))
        self._offset = self.min_us
        self.setPos(config['start'], True)

    def setPos(self, angle, immediate=False):
        """
        Set the position of the servo motor

        Parameters:
            angle (float): The desired angle for the servo motor in degrees from 0 to 180.
            immediate (bool, optional): If True, the servo motor will immediately move to the specified angle. If False, the servo motor will move to the specified angle gradually. Defaults to False.

        Returns:
            None
        """
        if not self.isMoving():
            if round(angle) != round(self.angle):
                if angle > self.posMax:
                    angle = self.posMax
                elif angle < self.posMin:
                    angle = self.posMin
                if immediate:
                    self.angle = angle
                    self.targetAngle = angle
                    self._moveServo(angle)
                else:
                    self.targetAngle = angle
                # save angle to config file here

    def setMin(self, min):
        """
        Sets the minimum value for the position and updates the position accordingly.

        Parameters:
            min (int): The minimum value for the position.

        Returns:
            None
        """
        self.posMin = min

    def setMax(self, max):
        """
        Set the maximum value for the position.

        Parameters:
            max (int): The maximum value for the position.

        Returns:
            None
        """
        self.posMax = max

    def setSpeed(self, speed):
        """
        Set the speed of the object.

        Parameters:
            speed (int): The desired speed of the object.

        Returns:
            None
        """
        if speed != self.speed:
            self.speed = speed

    def getSpeed(self):
        """
        Get the speed of the object.

        Returns:
            The speed of the object.
        """
        return self.speed

    def setInvert(self, invert):
        self.invert = invert

    def update(self):
        if self.isMoving():
            print(self.config, self.angle)
            if self.angle < self.targetAngle:
                self.angle += (self.speed / 100)
            elif self.angle > self.targetAngle:
                self.angle -= (self.speed / 100)
            self._moveServo(int(self.angle))

    def _moveServo(self, degrees):
        # limit degrees between 0 and 180
        if degrees > 180:
            degrees = 180
        if degrees < 0:
            degrees = 0
        if self.invert:
            degrees = (180 + 0) - degrees
        self.write(degrees)

    def isMoving(self):
        if round(self.angle) == round(self.targetAngle):
            return False
        else:
            return True

    def open(self, immediate=False):
        self.setPos(self.posMax, immediate=immediate)

    def close(self, immediate=False):
        self.setPos(self.posMin, immediate=immediate)
        # Need to improve this, only works if the servo was set using open() or close()

    def toggle(self):
        if round(self.angle) == round(self.posMax):
            self.close()
        elif round(self.angle) == round(self.posMin):
            self.open()

    def write(self,deg):
        self.write_rad(math.radians(deg))

    def read(self):
        return math.degrees(self.read_rad())

    def write_rad(self,rad):
        self.write_us(rad*self._slope+self._offset)

    def read_rad(self):
        return (self.current_us-self._offset)/self._slope

    def write_us(self,us):
        self.current_us=us
        self.pwm.duty_ns(int(self.current_us*1000.0))

    def read_us(self):
        return self.current_us

    def off(self):
        self.pwm.duty_ns(0)


# Pixel class based on RGB tuples, capable of animation from one colour to another, adjusting brightness as a whole and enabling a cool flicker effect (possibly more in the future)
class Pixel:
    def __init__(self, speed, brightness, color, flicker=True):
        # Normal Flicker
        self.targetBrightness = 0
        self.brightness = 0

        self.color = list(color[0])
        self.flashColor = list(color[1])

        self.targetColor = list(color[0])
        self.targetFlashColor = list(color[1])

        self.randomFlicker = randint(int(1 * speed), int(20 * speed))
        self.speed = speed
        self.randomFlickerCount = 0
        self.randomFlickerOn = 0
        self.brightnessNormal = 0
        self.brightnessFlicker = 0
        self.animationSpeed = 20
        self.isFlicker = flicker
        self.animating = False
        self.setBrightness(brightness)
        self.brightnessTimer = Timer(0, 1, 1)
        self.colorTimers = [Timer(0, 1, 1), Timer(0, 1, 1), Timer(0, 1, 1)]

    def getPixelState(self):
        self.randomFlickerCount += 1
        self.randomFlickerOn -= 1
        if self.randomFlickerCount >= self.randomFlicker:
            self.randomFlicker = randint(
                int(1 * self.speed), int(20 * self.speed))
            self.randomFlickerOn = 1
            self.randomFlickerCount = 0
        if self.randomFlickerOn > 0 and self.isFlicker:
            return tuple([int(x * self.brightness / 100) for x in self.flashColor])
        else:
            return tuple([int(x * self.brightness / 100) for x in self.color])

    def update(self):
        # Handle colour animation
        if self.color != self.targetColor:
            for i in range(len(self.color)):
                if self.color[i] > self.targetColor[i]:
                    self.color[i] = max(self.targetColor[i], self.color[i] - self.animationSpeed)
                elif self.color[i] < self.targetColor[i]:
                    self.color[i] = min(255, self.color[i] + self.animationSpeed)
                if self.flashColor[i] > self.targetFlashColor[i]:
                    self.flashColor[i] = max(self.targetFlashColor[i], self.flashColor[i] - self.animationSpeed)
                elif self.flashColor[i] < self.targetFlashColor[i]:
                    self.flashColor[i] = min(255, self.flashColor[i] + self.animationSpeed)
        if self.brightness != self.targetBrightness:
            if self.brightnessTimer.update(True):
                if self.brightness > self.targetBrightness:
                    self.brightness = self.brightness - 1
                if self.brightness < self.targetBrightness:
                    self.brightness = self.brightness + 1

                if self.brightness > 100:
                    self.brightness = 100
                elif self.brightness < 0:
                    self.brightness = 0
                self.brightnessTimer.reset()
        else:
            self.brightnessTimer.reset()

        # Handle random flickering effect
        return self.getPixelState()

    # Get rgb tuple for writing to neopixels

    def getRgb(self):
        return self.rgbCurrent

    def isAnimating(self):
        return self.animatng

    # Set the color to animate to

    def animateColor(self, color, speed=0):
        if speed != 0:
            self.animationSpeed = speed
        if self.targetColor != color:
            self.targetColor = list(color[0])
            self.targetFlashColor = list(color[1])

    def setColor(self, color):
        self.targetColor = list(color[0])
        self.targetFlashColor = list(color[1])
        self.color = list(color[0])
        self.flashColor = list(color[1])

    def setAnimationSpeed(self, speed):
        self.animationSpeed = speed

    def setBrightness(self, brightness):
        if brightness > 100:
            brightness = 100
        elif brightness <= 0:
            brightness = 0
        self.brightness = brightness
        self.targetBrightness = brightness

    def animateBrightness(self, brightness, speed=0):
        if speed != 0:
            self.animationSpeed = speed
        if self.targetBrightness != brightness:
            self.targetBrightness = brightness
            if (brightness - self.brightness) != 0:
                delta = abs(self.animationSpeed / (brightness - self.brightness))
                self.brightnessTimer = Timer(0, delta, 1)
            else:
                self.brightnessTimer = Timer(0, 1, 1)

    def setFlicker(self, flicker):
        self.isFlicker = flicker


# Class for handling neolight lighting (just holds individual pixels and updates them all the logic happens in the pixel class above)
class NeoLight:
    def __init__(self, config):
        self.config = config
        self.brightness = 0
        self.pin = config['pin']
        self.cooldown = False
        self.speed = 1
        self.length = config['length']
        self.np = neopixel.NeoPixel(Pin(config['pin']), config['length'])
        self.pixels = []
        self.color = [[225, 0, 0], [15, 15, 15]]
        self.cooldown = False
        self.flicker = config['flicker']
        for i in range(len(self.np)):
            pixel = Pixel(self.speed, self.brightness,
                          self.color, self.flicker)
            self.pixels.append(pixel)

    def update(self):
        for index, pixel in enumerate(self.pixels):
            self.np[index] = pixel.update()
        self.np.write()

    def animateColor(self, color, speed=0):
        if color != self.color:
            for pixel in self.pixels:
                if speed > 0:
                    pixel.animateColor(color, speed)
                else:
                    pixel.animateColor(color)
            self.color = color

    def animateColorRange(self, color, start, end, speed=0):
        if color != self.color:
            for index, pixel in enumerate(self.pixels):
                if start <= index <= end:
                    if speed > 0:
                        pixel.animateColor(color, speed)
                    else:
                        pixel.animateColor(color)
            self.color = color

    def setColor(self, color):
        for index, pixel in enumerate(self.pixels):
            if index < self.length:
                pixel.setColor(color)
                self.color = color

    def setColorRange(self, color, start, end):
        for index, pixel in enumerate(self.pixels):
            if start <= index <= end:
                pixel.setColor(color)
                self.color = color

    def setPixelColor(self, color, pixel):
        self.pixels[pixel].setColor(color)

    def setBrightness(self, brightness, animate=False, speed=0):
        if brightness > 100:
            brightness = 100
        elif brightness < 0:
            brightness = 0
        for index, pixel in enumerate(self.pixels):
            if animate:
                pixel.animateBrightness(brightness, speed)
            else:
                pixel.setBrightness(brightness)
        self.brightness = brightness

    def setBrightnessRange(self, brightness, start, end, animate=False, speed=0):
        """
        Sets the brightness of the NeoPixel strip between start and end
        :param brightness: Level of brightness from 0-100
        :param start: Start pixel
        :param end: End pixel
        :param animate: Whether to animate the brightness change or not
        """
        if brightness > 100:
            brightness = 100
        elif brightness < 0:
            brightness = 0
        for index, pixel in enumerate(self.pixels):
            if start <= index <= end:
                if pixel.brightness != brightness:
                    if animate:
                        pixel.animateBrightness(brightness, speed)
                    else:
                        pixel.setBrightness(brightness)
            else:
                # pixel.setBrightness(0)
                pass

    def setFlicker(self, flicker):
        self.flicker = flicker
        for index, pixel in enumerate(self.pixels):
            pixel.setFlicker(flicker)

    def setLength(self, length):
        self.length = length


# Class for handling input buttons, you can set if it should fire off once or press and hold to continually output true after a predefined timeout.
class Button:
    """
    Initalizes a new button

    Args:
        pin (int): The pin number.
        single (bool, optional): Indicates whether the button is single-click only. Defaults to False.

    Returns:
        None
    """

    def __init__(self, pin, single=False):
        self.single = single
        self.pin = pin
        self.lock = False
        self.button = Pin(pin, Pin.IN, Pin.PULL_DOWN)
        self.held = False

    def getState(self, single=True):
        """
        Gets the state of the button.

        Parameters:
            single (bool): Whether the button is single-click only (returns true once)

        Returns:
            bool: The state of the button.
        """
        if self.button.value():
            self.held = True
            if not self.lock:
                self.lock = True
                return True
            else:
                if not single:
                    return True
                return False
        else:
            self.lock = False
            self.held = False
            return False

    def getHeld(self):
        return self.held

    def reset(self):
        self.lock = True


# Basic timer class to count down things and call a method when it reaches its goal
class Timer:
    def __init__(self, start, end, interval, functions=[]):
        self.end = end
        self.start = start
        self.current = start
        self.interval = interval
        self.functions = functions

    def update(self, do):
        if do:
            if self.interval > 0:
                if self.current <= self.end:
                    self.current = self.current + self.interval
            else:
                if self.current >= self.start:
                    self.current = self.current + self.interval
        if self.current >= self.end:
            for function in self.functions:
                if callable(function):
                    function()
            return True
        else:
            return False

    def reset(self):
        self.current = self.start

    def getState(self):
        return self.current

    def setStartEnd(self, start, end):
        self.start = start
        self.end = end


buttons = {}
# Initialize all the buttons
for button in config['buttons']:
    buttons[button] = Button(config['buttons'][button], True)


# =============================== METHODS ==================================

# Initialize all the NeoPixels
lights = {}
for light in config['lights']:
    lights[light] = NeoLight(config['lights'][light])
    lights[light].setColor(config['colors']['normal'])
    lights[light].setBrightness(100)
servos ={}
for servo in config['servos']:
    servos[servo] = Servo(config['servos'][servo])
    servos[servo].update()


class State():
    def __init__(self, saveState=False):
        self.saveState = saveState

    def enter(self, sm):
        pass

    def update(self, sm):
        pass

    def exit(self, sm):
        pass


# ========= States ===============
class Main(State):
    def __init__(self):
        super().__init__()
        self.servoOffTimer = Timer(0,100,1)

    def enter(self, sm):
        self.servoOffTimer.reset()
        pass

    def update(self, sm):
        if sm.ammo == 0:
            sm.changeState(NoAmmo())
        if buttons['trigger1'].getState():
            sm.changeState(Firing())
        if buttons['mag'].getState():
            sm.changeState(MagOpen())
    def exit(self, sm):
        self.servoOffTimer.reset()

class Firing(State):

    def __init__(self):
        super().__init__()
        self.firingTimer = Timer(0,20,1)

    def enter(self, sm):
        servos['recoil'].setPos(160,True)
        for light in lights:
            if light != 'mag':
                lights[light].setBrightness(100)
                lights[light].setColor(config['colors']['flash'])

    def update(self, sm):
        if self.firingTimer.update(True):
            servos['recoil'].setPos(5,True)
            for light in lights:
                if light != 'mag':
                    lights[light].setBrightness(100)
                    lights[light].setColor(config['colors']['normal'])
            sm.changeState(Main())
        pass

    def exit(self, sm):
        sm.ammo -= 1
        pass


class MagOpening(State):
    def __init__(self):
        super().__init__()

    def enter(self, sm):
        pass

    def update(self, sm):
        pass

    def exit(self, sm):
        pass


class MagClosing(State):
    def __init__(self):
        super().__init__()

    def enter(self, sm):

        pass

    def update(self, sm):
        pass

    def exit(self, sm):
        pass


class MagOpen(State):
    def __init__(self):
        super().__init__()

    def enter(self, sm):
        servos['mag-open'].open(True)
        pass

    def update(self, sm):
        if buttons['mag'].getState():
            sm.changeState(Main())
        pass

    def exit(self, sm):
        servos['mag-open'].close(True)
        sm.ammo = int(config['maxAmmo'])
        for light in lights:
            lights[light].animateColor(config['colors']['normal'],20)
class NoAmmo(State):
    def __init__(self):
        super().__init__()
        self.servoOffTimer = Timer(0,100,1)


    def enter(self, sm):
        for light in lights:
            if light != "mag":
                lights[light].animateColor(config['colors']['red'],20)
        pass
    def update(self, sm):
        if buttons['mag'].getState():
            sm.changeState(MagOpen())
    def exit(self, sm):
        self.servoOffTimer.reset()

class StateMachine:
    def __init__(self, starting):
        self.currentState = None
        self.changeState(starting)
        # Settings

        self.ammo = int(config['maxAmmo'])
        self.printTimer = Timer(0, 2, 1)

    def changeState(self, state):
        """
        Change the current state of the state machine to the specified state.
        if state has variable called saveState and it is true, saves state to config file

        Parameters:
            state (State): The new state to be set.

        Returns:
            None
        """
        if self.currentState:
            self.currentState.exit(self)
            # Save new state on exiting old state
            if self.currentState.saveState:
                config['state'] = type(state).__name__
                saveSettings()

        self.currentState = state

        if self.currentState:
            self.currentState.enter(self)
            if self.currentState.saveState:
                config['state'] = type(state).__name__
                saveSettings()

    def getCurrentState(self):
        return self.currentState

    def update(self):
        self.currentState.update(self)
        self.printSlowly([self.getCurrentState(),self.ammo])
        self.checkAmmo()
        for light in lights:
            lights[light].update()
        servos['mag-rotate'].update()

    def checkAmmo(self):
        if self.ammo < 0:
            self.ammo = 0
        if self.ammo == 10:
            lights['mag'].setBrightness(100)
            lights['mag'].setColor(config['colors']['normal'])
            servos['mag-rotate'].setPos(5, True)
        elif self.ammo == 9:
            lights['mag'].setBrightnessRange(20,4,4)
        elif self.ammo == 8:
            lights['mag'].setBrightnessRange(0,4,4)
            servos['mag-rotate'].setPos(35, True)
        elif self.ammo == 7:
            lights['mag'].setBrightnessRange(20,3,3)
        elif self.ammo == 6:
            lights['mag'].setBrightnessRange(0,3,3)
            servos['mag-rotate'].setPos(70, True)
        elif self.ammo == 5:
            lights['mag'].setBrightnessRange(20,2,2)
        elif self.ammo == 4:
            lights['mag'].setBrightnessRange(0,2,2)
            servos['mag-rotate'].setPos(105, True)
        elif self.ammo == 3:
            lights['mag'].setBrightnessRange(20,1,1)
        elif self.ammo == 2:
            lights['mag'].setBrightnessRange(0,1,1)
            servos['mag-rotate'].setPos(140, True)
        elif self.ammo == 1:
            lights['mag'].setBrightnessRange(20,0,0)
            lights['mag'].setColorRange(config['colors']['red'],0,0)
        elif self.ammo == 0:
            lights['mag'].setBrightnessRange(0,0,0)
            servos['mag-rotate'].setPos(175, True)
    def printSlowly(self, msgs=[]):
        if self.printTimer.update(True):
            for msg in msgs:
                print(msg)
            self.printTimer.reset()


# Load into previously saved state:
# Only if one of the "safe" states
state = 'Main'
constructor = globals()[state]

mainLogic = StateMachine(constructor())

# MAIN LOOP
loopstart = utime.time()

while True:
    mainLogic.update()
