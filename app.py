"""An example of a simple template for Arkitekt Next"""

from arkitekt_next import register
import time
from typing import Generator


@register
def move_robot_slide_holder():
    "A function to move the robot to the slide holder"
    print("Moving robot to slide holder")


@register
def grip_slide_holder():
    "A function to grip the slide holder"
    print("Gripping slide holder")


@register
def release_slide_holder():
    "A function to release the slide holder"
    print("Releasing slide holder")


@register
def pick_up_slide_in_tray(slider: int):
    "A function to pick up a slide in the tray given its index"
    print(f"Picking up slide {slider}")


@register
def drop_slider_in_tray(slider: int):
    "A function to drop a slide in the tray given its index"
    ...
    print(f"Dropping slide {slider}")


@register
def move_robot_to_microscope():
    "A function to move the robot to the microscope"
    ...
    print("Moving robot to microscope")


@register
def move_robot_to_opentrons():
    "A function to move the robot to the OpenTrons robot"
    ...
    print("Moving robot to OpenTrons")


@register
def drop_slide():
    "A function to drop a slide"
    ...
    print("Dropping slide")


@register
def pickup_slide():
    "A function to pick up a slide"
    ...
