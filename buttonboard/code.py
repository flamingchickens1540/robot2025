# SPDX-FileCopyrightText: 2018 Kattni Rembor for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""CircuitPython Essentials NeoPixel example"""
import usb_hid # type: ignore
import time
import board
import neopixel
import keypad
from joystick_xl.joystick import Joystick

js = Joystick()

num_pixels = 26

keys = keypad.KeyMatrix(
    row_pins=(board.GP22, board.GP21, board.GP20, board.GP19, board.GP18),
    column_pins=(board.GP0, board.GP1, board.GP2, board.GP3, board.GP4, board.GP5),
    columns_to_anodes=False,
)

pixels = neopixel.NeoPixel(board.GP28, num_pixels, brightness=1, auto_write=False)

KEY_INTAKE_L1 = 23
KEY_INTAKE_L2 = 22
KEY_INTAKE_R1 = 29
KEY_INTAKE_R2 = 28
KEY_L1 = 6
KEY_L2 = 7
KEY_L3 = 8
KEY_L4 = 9
KEY_NP_2 = 4
KEY_NP_1 = 5
KEY_ALGAE_4 = 0
KEY_ALGAE_3 = 1
KEY_ALGAE_2 = 2
KEY_ALGAE_1 = 3
KEY_REEF_H = 21
KEY_REEF_G = 15
KEY_REEF_F = 14
KEY_REEF_E = 20
KEY_REEF_D = 13
KEY_REEF_C = 19
KEY_REEF_B = 12
KEY_REEF_A = 18
KEY_REEF_L = 17
KEY_REEF_K = 11
KEY_REEF_J = 16
KEY_REEF_I = 10

RED = (255, 0, 0)
YELLOW = (255, 150, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
PURPLE = (180, 0, 255)
DIM = (50, 50, 50)

STEP=20
class Selector:
    indexToKey: tuple
    active: int

    def __init__(self, axis: int, default: int, onColor, offColor, keyNums: tuple) -> None:
        self.active = 0
        self.axis = axis
        self.default = default
        self.indexToKey = keyNums
        self.keyToIndex = {}
        self.onColor = onColor
        self.offColor = offColor
        self.refresh()

    def refresh(self):
        self.active = self.default
        for i, key in enumerate(self.indexToKey):
            self.keyToIndex[key] = i
            pixels[ids_to_pixels[self.indexToKey[i]]] = self.offColor
        self.setActive(self.active)

    def update(self, event: keypad.Event):
        if not event.pressed:
            return False
        index = self.keyToIndex.get(event.key_number)
        if index is not None:
            self.setActive(index)
            return True
        return False
    
    def setActive(self, index:int):
        pixels[ids_to_pixels[self.indexToKey[self.active]]] = self.offColor
        self.active = index
        pixels[ids_to_pixels[self.indexToKey[self.active]]] = self.onColor
        js.update_axis((self.axis, int(STEP * index) ))


class Button:
    pressed:bool = False
    def __init__(self, id, key, onColor, offColor) -> None:
        self.key = key
        self.id = id
        self.onColor = onColor
        self.offColor = offColor

    def setActive(self, active:bool):
        self.pressed = active
        pixels[ids_to_pixels[self.key]] = self.onColor if active else self.offColor
        js.update_button((self.id, active))

    def update(self, event:keypad.Event):
        # print("BUTTON", event)
        if event.key_number == self.key:
            self.setActive(event.pressed)
            return True
        return False
    
    def refresh(self):
        self.setActive(self.pressed)

ids_to_pixels = {
    KEY_INTAKE_L1: 19,  # INTAKE L1
    KEY_INTAKE_L2: 18,  # INTAKE L2
    KEY_INTAKE_R1: 24,  # INTAKE R1
    KEY_INTAKE_R2: 25,  # INTAKE R2
    KEY_L1: 11,  # L1
    KEY_L2: 10,  # L2
    KEY_L3: 9,  # L3
    KEY_L4: 8,  # L4

    KEY_NP_2: 1,  # NP 2
    KEY_NP_1: 0,  # NP 1

    KEY_ALGAE_4: 5,  # Algae4
    KEY_ALGAE_3: 4,  # Algae3
    KEY_ALGAE_2: 3,  # Algae2
    KEY_ALGAE_1: 2,  # Algae1

    KEY_REEF_H: 20,  # H
    KEY_REEF_G: 14,  # G
    KEY_REEF_F: 15,  # F
    KEY_REEF_E: 21,  # E
    KEY_REEF_D: 16,  # D
    KEY_REEF_C: 22,  # C
    KEY_REEF_B: 17,  # B
    KEY_REEF_A: 23,  # A
    KEY_REEF_L: 12,  # L
    KEY_REEF_K: 6,  # K
    KEY_REEF_J: 13,  # J
    KEY_REEF_I: 7,  # I
}

povs = [
    Selector(0, 0, (210, 86, 255), (2, 0, 2), (
        KEY_REEF_H,
        KEY_REEF_G,
        KEY_REEF_F,
        KEY_REEF_E,
        KEY_REEF_D,
        KEY_REEF_C,
        KEY_REEF_B,
        KEY_REEF_A,
        KEY_REEF_L,
        KEY_REEF_K,
        KEY_REEF_J,
        KEY_REEF_I,
    )),
    Selector(1, 0, (250, 200, 0), (2, 2, 0), (
        KEY_L1,
        KEY_L2,
        KEY_L3,
        KEY_L4,
    )),
    Selector(2, 0, (250, 0, 0), (3, 0, 0), (
        KEY_INTAKE_L1,
        KEY_INTAKE_L2,
        KEY_INTAKE_R1,
        KEY_INTAKE_R2
    )),
    Button(0,KEY_NP_1,(0,200,100),(0,0,0)),
    Button(1,KEY_NP_2,(0,200,100),(0,0,0)),
    Button(2,KEY_ALGAE_1,(250,50,0),(3,1,0)),
    Button(3,KEY_ALGAE_2,(250,50,0),(3,1,0)),
    Button(4,KEY_ALGAE_3,(250,50,0),(3,1,0)),
    Button(5,KEY_ALGAE_4,(250,50,0),(3,1,0)),
]

def refresh():
    for pov in povs:
        pov.refresh()
    pixels.show()


global activeMode
activeMode = "startup"

class KeyPattern():
    def __init__(self, keys:tuple, mode:str) -> None:
        self.keys = keys
        self.index = 0
        self.mode = mode

    def add(self, key:int):
        global activeMode
        if self.keys[self.index] == key:
            self.index+=1
            if self.index == len(self.keys):
                activeMode = self.mode
                self.index = 0
        else:
            self.index = 0
def campfire():
    pass

patterns = [
    KeyPattern((KEY_ALGAE_1,KEY_ALGAE_1,KEY_ALGAE_4, KEY_ALGAE_4, KEY_ALGAE_1,KEY_ALGAE_1,KEY_ALGAE_4, KEY_ALGAE_4), "startup")
]


stageIndex = 0
def startup_step():
    global stageIndex
    global activeMode
    if stageIndex == 0:
        pixels.fill((0,0,0))
        pixels.show()
    elif stageIndex >= 1 and stageIndex <=24:
        povs[0].setActive((stageIndex-1)%len(povs[0].indexToKey))
        pixels.show()
        time.sleep(0.03)
    elif stageIndex == 25:
        pixels.fill(RED)
        pixels.show()
        time.sleep(0.2)
    elif stageIndex == 26:
        pixels.fill((0, 0, 0))
        pixels.show()
        time.sleep(0.2)
    elif stageIndex == 27:
        activeMode = "normal"
        refresh()
    stageIndex+=1

while True:
    key_event = keys.events.get()
    if key_event:
        if key_event.pressed and activeMode != "normal":
            activeMode = "normal"
            refresh()
            continue
        for pov in povs:
            if pov.update(key_event):
                pixels.show()
                break
    if activeMode == "startup":
        startup_step()
