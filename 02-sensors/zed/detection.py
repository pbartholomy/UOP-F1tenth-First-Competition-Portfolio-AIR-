
import pygame
import time

pygame.init()
pygame.joystick.init()

count = pygame.joystick.get_count()
print("Controller count:", count)

if count > 0:
    js = pygame.joystick.Joystick(0)
    js.init()
    print("Controller name:", js.get_name())
    print("Axes:", js.get_numaxes())
    print("Buttons:", js.get_numbuttons())
    print("Hats:", js.get_numhats())

pygame.quit()
