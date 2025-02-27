"""JoystickXL standard boot.py."""

import usb_hid
from joystick_xl.hid import create_joystick
# This will enable all of the standard CircuitPython USB HID devices along with a
# USB HID joystick.
usb_hid.enable(
    (
        usb_hid.Device.CONSUMER_CONTROL,
        create_joystick(axes=5, buttons=0, hats=0),
    )
)
usb_hid.set_interface_name("Button Board")


