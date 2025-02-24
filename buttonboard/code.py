# SPDX-FileCopyrightText: 2018 Kattni Rembor for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""CircuitPython Essentials NeoPixel example"""
import usb_hid
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

pixels.fill(RED)
pixels.show()
time.sleep(0.4)
pixels.fill((0, 0, 0))
pixels.show()


class Selector:
    keys: tuple
    active: int

    def __init__(self, axis: int, step: float, onColor, offColor, keys: tuple) -> None:
        self.active = 0;
        self.axis = axis
        self.step = step
        self.keys = keys
        self.keymap = {}
        self.onColor = onColor
        self.offColor = offColor
        for i, key in enumerate(keys):
            self.keymap[key] = i
            pixels[ids_to_pixels[self.keys[i]]] = self.offColor
        pixels[ids_to_pixels[self.keys[self.active]]] = self.onColor

    def update(self, event: keypad.Event):
        index = self.keymap.get(event.key_number)
        if index is not None:
            if not self.active == -1:
                pixels[ids_to_pixels[self.keys[self.active]]] = self.offColor

            if index == self.active:
                self.active = -1
                js.update_axis((self.axis, 0))
            else:
                self.active = index
                pixels[ids_to_pixels[self.keys[self.active]]] = self.onColor
                js.update_axis((self.axis, self.step * (index + 1)))
            return True
        return False


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
    Selector(0, 20, (210, 86, 255), (2, 0, 2), (
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
    Selector(1, 20, (250, 200, 0), (2, 2, 0), (
        KEY_L1,
        KEY_L2,
        KEY_L3,
        KEY_L4,
    )),
    Selector(2, 20, (250, 0, 0), (3, 0, 0), (
        KEY_INTAKE_L1,
        KEY_INTAKE_L2,
        KEY_INTAKE_R1,
        KEY_INTAKE_R2
    )),
    Selector(3, 20, (0, 200, 100), (0, 2, 1), (
        KEY_ALGAE_1,
        KEY_ALGAE_2,
        KEY_ALGAE_3,
        KEY_ALGAE_4
    )),
    Selector(4, 20, (0, 0, 200), (0, 0, 3), (
        KEY_NP_1,
        KEY_NP_2
    ))
]

pixels.show()

while True:
    key_event = keys.events.get()
    if key_event and key_event.pressed:
        for pov in povs:
            if pov.update(key_event):
                pixels.show()
                break
