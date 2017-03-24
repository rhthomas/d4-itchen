##
# \file xboxcontroller.py
# \date 2017-03-06
# \author Felix de Neve
# \author Rhys Thomas
# \brief Decodes Xbox controller data and sends to Arduino via serial for
# transmission.
# \details Prints button commands, elevation/yaw/pitch/roll and whether the remote is
# armed or disarmed. Uses ncurses interface as to not print hundreds of lines
# to the terminal. 
# \image html ./images/tui.png

import pygame
import math
import serial
import curses

# curses screen
myscreen = curses.initscr()
# hide the cursor
curses.curs_set(0)

# serial connection to arduino
ser=serial.Serial('/dev/cu.usbmodem1411',115200)

# Inititialising joystick with pygame
pygame.init()
pygame.display.init()
pygame.joystick.init()

# Sets clock
clock = pygame.time.Clock()

# Instantiates controller
Joystick = pygame.joystick.Joystick(0)
Joystick.init()

# sending remote data to drone at 20Hz
dataRate = 20

myscreen.clear()
myscreen.addstr(0, 0, "Xbox to Drone control program")
myscreen.addstr(1, 0, "-----------------------------")
myscreen.addstr(2, 0, "Press <start> to arm")
myscreen.addstr(3, 0, "Press <back> to disarm")
myscreen.addstr(4, 0, "Press <xbox> to abort")
myscreen.addstr(5, 0, "-----------------------------")
myscreen.refresh()

try:
	# main while loop
	while True:
		# start button to arm
		while not Joystick.get_button(4):
			pygame.event.pump()
			myscreen.addstr(7, 0, "disarmed", curses.A_BLINK)
			myscreen.refresh()
			pass

		myscreen.move(7,0)
		myscreen.clrtoeol()
		myscreen.addstr(7, 0, "ARMED", curses.A_REVERSE)
		stop = 0xAA

		# armed loop
		while stop != 0x00:
			clock.tick(dataRate)
			pygame.event.pump()

			# select button to dissarm
			if Joystick.get_button(5):
				break

			# stops the drone if center button is pressed
			if Joystick.get_button(10):
				stop = 0x00

			# Stores axis positions of Joystick
			yaw=int(math.ceil((Joystick.get_axis(0)+1)*127) if (Joystick.get_axis(0)>0.3 or Joystick.get_axis(0)<-0.3) else 127)
			elev=int((math.ceil((math.sqrt(math.sqrt(abs(-Joystick.get_axis(1)))))*255)) if (Joystick.get_axis(1)<0) else 0)
			pitch=int(math.ceil((Joystick.get_axis(3)+1)*127))
			roll=int(math.ceil((Joystick.get_axis(2)+1)*127))

			# send to arduino over usb
			array = [stop,elev,yaw,pitch,roll]
			ser.write(bytearray(array))

			# print the data to the terminal
			myscreen.addstr(9, 0,  "elev  = " + str(elev)  + "     ")
			myscreen.addstr(10, 0, "yaw   = " + str(yaw)   + "     ")
			myscreen.addstr(11, 0, "pitch = " + str(pitch) + "     ")
			myscreen.addstr(12, 0, "roll  = " + str(roll)  + "     ")
			myscreen.refresh()

# if there is a keyboard interrupt
except KeyboardInterrupt:
	# clear the screen
	curses.endwin()
