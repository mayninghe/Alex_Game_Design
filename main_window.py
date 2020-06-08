"""
For users:
Welcome to your new favorite game: 
    - Most important rule, have fun
    - Second most important rule, D is for dev mode and displays alot of usefull information on the screen, but it does lower the framerate
    - A for new averages, do this alot as the sensors drift
    - S is for new maximums, make sure they give it their all, as low maximums result in signal noise taking a large role in the behavior
    - The game is by default very hard, use 9 and 0 to adjust the required effort by the user (Effort modifier is shown in dev mode, at 0.5 
      they need roughly half of their max to reach the objective zone)
    - Friction increases on a loss and decreases on a win
    - Torques are always being recorded in the data files
    - Escape to exit
    - P drops a new cup or box, but avoid dropping multiple objects at once
    - Cups are harder than boxes as you need to watch the angle, use c to switch between the two modes
    - Also, have fun, my high score is 117 with no cheating

For Coders:
Hello whoever is picking up the coding portion of this project, my Name is Alex Webb, I wrote this in the fall of 2019, and it was my first project in python.
As such, it is probably an absolute mess, for which I am sorry. If you are here to improve what I have, there are a few places I can recomend 
starting (although you'll probably know better than me anyway):
    1. My csv writes arre a mess, they are all over the place, disorganized, and sometimes repetitive, the normal data file is fine, however, the info 
    file and especically the object info file need help. (obj file creates 1 line for each object each loop, so if there is more than one object its not very 
    usefull in a line graph, point graphs can be fine but confusing)
    2. I never incorporated lings code for increased sampling rate because sampling rate is tied into the physics update speed (computation time controls 
    frame rate which affects the simulation timestep)
    3. Honestly way too many things are tied into computation time and frame rate. Success/Fail states use variables as counters when they should use timers
    instead. If these are fixed then switching to the faster sampling speed should be easy
    4 This code is not really optimized for full screen:
        a. the game only takes up part of the screen, and it is not centered, some work on margins and positions should fix this
        b. the file name has to be typed in before the game can run, so if it is in full screen, the game opens, but it does not run until you switch back to 
        the terminal and type in a file name. Since the game isn't running yet, the pc thinks the window is unresponsive, and it becomes hard to switch back
        to the terminal and aname the file. If you use multiple monitors/desktops it is easier to run in fullscreen, or just hardcode a file name
    5. If you have question I can be contacted at 7813151550, but I'm not going to answer unless you text me first
Remember that you can drag objects with the mouse for debugging, good luck
"""

import timeit
import time
import os
import arcade
import pymunk
import pyglet
import csv
from datetime import datetime
import nidaqmx as daq
from physics_utility import (
    PymunkSprite,
    check_grounding,  # this allows to check for contact to another object however it is not used here
    resync_physics_sprites,
)

from constants import *

## MyGame inherits arcade.Window
class MyGame(arcade.Window):
    """ Main application class. """

    def __init__(self, width, height, title):
        super().__init__(width, height, title, fullscreen=False) #### Make fullscreen true if you want full screen ####
        # setup stings for file names
        self.str = input('name file: ') ## Until someone smarter than me fixes it, if in fullscreen, hardcode a file name ########
        self.trial = 1
        self.str2 = self.str + 'Trial' + str(self.trial) + 'info.csv'  # extra info file
        self.str3 = self.str + 'Trial' + str(self.trial) + '.csv'  # Torque data file
        self.str4 = self.str + 'Trial' + str(self.trial) + 'obj.csv'  # Object position and angle file
        # create files (may be unnecessary)
        with open(self.str4, 'w', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=' ')
        with open(self.str3, 'w', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=' ')
        with open(self.str2, 'w', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter='|')
            spamwriter.writerow(['current date and time', datetime.now()])  # add date and time to info file
            spamwriter.writerow(['--------'])

        # Set the working directory (where we expect to find files) to the same
        # directory this .py file is in. You can leave this out of your own
        # code, but it is needed to easily run the examples using "python -m"
        # as mentioned at the top of this program.
        file_path = os.path.dirname(os.path.abspath(__file__))
        os.chdir(file_path)

        arcade.set_background_color(arcade.color.DARK_SLATE_GRAY)
        '''
        # Pymunk is used to allocate the space, which includes gravity and all the fancy stuff. We may be able to just use arcade animation sprite
        # Arcade.shape class
        '''

        # -- Pymunk
        self.space = pymunk.Space()
        self.space = pymunk.Space()
        self.space.gravity = GRAVITY

        # Physics joint used for grabbing items
        self.grab_joint = None

        # Lists of sprites
        self.dynamic_sprite_list = arcade.SpriteList()
        self.static_sprite_list = arcade.SpriteList()
        self.kinematic_sprite_list = arcade.SpriteList()
        # Used for dragging shapes around with the mouse
        self.shape_being_dragged = None
        self.last_mouse_position = 0, 0

        '''
        Need to understand which of these variables are unnecessary at this moment
        As well as how are the ones necessary used.
        self.maxR, self.maxL, self.effortScale

        maxL and maxR should read input from the maximum measurement
        '''
        
        # whole big pile of variables used later in code
        self.draw_time = 0  # used to calculate drawing time
        self.processing_time = 0  # used to calculate processing time
        self.score = 0  # score is adjusted based on win or loss (serves no practical purpose) 
        self.maxL = 1  # dummy values for maximum voltage from left joint   
        self.cupmode = 1  # 1 for cups and angle requirement, 0 for boxes and no angle       
        self.maxR = 1  # dummy values for maximum voltage from right joint   
        self.ytarg = 0  # target Y value, used in calculation of altitude control     
        self.qtarg = 0  # target angle value, used in calculation of angle control
        self.qveltarg = 0  # target angular velocity
        self.obj = 0  # counter used to determine if objective is completed
        self.dev = 0  # variable used to toggle dev mode (extra info on screen)
        self.succsess = 0  # true (1) if system is in success state
        self.failure = 0  # true (1) if system is in failure state
        self.friction = 0.5  # controls friction between box and platform
        # (kinematic and static friction are the same in this engine)       
        self.losscounter = 100  # counter used to control time in loss state
        # Change the default effortScale to be 0.2   -  eventually will need to change that so that it can be any percentage   
        self.effortScale = 0.2  # determines what % of max is needed to achieve target height     
        self.effort = 0  # true (1) if system is in effort mode   
        self.block = 0  # true (1) if screen blocker is enabled
        self.target = [460, 540, 975, 1025]  # borders of objective zone

        # start Therese's edit: weight variable
        self.torqueThreshold = min(abs(self.maxL), abs(self.maxR)) # force threshold at which angular velocity now depends on difference between R and L torques
        # end Therese's edit

        # start Therese's new edit (to replace above declaration of self.target)
        # Initalize self.target depending on max left and right torques
        #self.target[2] = (abs(self.maxL) + abs(self.maxR)) * 400 # may have to change scaling factor
        #if self.target[2] > 975:
        #    self.target[2] = 975
        #self.target[3] = self.target[2] + 50
        # end Therese's new edit
        
        # Connect to DAQ
        try:
            self.daqtask0 = daq.Task()  # create task
            self.daqtask0.ai_channels.add_ai_voltage_chan("Dev1/ai0")  # add channel 1 to task
            self.daqtask0.ai_channels.add_ai_voltage_chan("Dev1/ai1")  # add chanel 2 to same task
        except:
            # exit code on failed daq connection
            print("Failed to connect to DAQ. Check connections and DAQ status.")
            print("Exiting safely")
            exit(0)

        """Take a an average to establish offset levels for sensors, note, the offset will drift significantly after 
        this, and offsets will need to be collected again"""
        self.averagecount = 0  # indexing variable used to control length of average
        self.ch0avg = []  # empty array for channel 0
        self.ch1avg = []  # empty array for channel 1
        while self.averagecount < 100:  # loop 100x for 100 point average
            self.averagecount += 1  # increment indexing variable
            self.temp = self.daqtask0.read()  # read both channels as 1 measurement
            self.ch0avg.append(self.temp[0])  # Retrieve channel 0 from reading
            self.ch1avg.append(self.temp[1])  # Retrieve channel 0 from reading
        self.avg0 = sum(self.ch0avg) / len(self.ch0avg)  # take average for channel 0
        self.avg1 = sum(self.ch1avg) / len(self.ch1avg)  # take average for channel 0

        # add initial averages to info file
        with open(self.str2, 'a', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter='|')
            spamwriter.writerow([f'Initial Average Left {self.avg1:.3f}'])
            spamwriter.writerow([f'Initial Average Right {self.avg0:.3f}'])
            spamwriter.writerow(['--------'])

        # Set the viewport boundaries
        # These numbers set where we have 'scrolled' to (functionality not used as scrolling is removed)
        self.view_left = 0
        self.view_bottom = 0

        # Loads in audio files used later in code
        opening = pyglet.media.load('audio/phonePay.wav')
        self.hit = pyglet.media.load('audio/hitmarker2.wav')
        self.win = pyglet.media.load('audio/levelup.wav')
        opening.play()  # play opening jingle

        self.start = time.time()  # record start time
        self.reset = time.time()  # record time since last reset

        # Create the player (the platform)
        x = 500
        y = (SPRITE_SIZE + SPRITE_SIZE / 2) + 450
        """this is the layout for creating a new physical object in the simulation space, as well as 
        attaching a sprite to it. It is repeated many times throughout the code"""
        self.player = PymunkSprite("images/metalBar_thin.png", x, y, scale=1, moment=70, mass=1, width=445, height=29,
                                   body_type=pymunk.Body.KINEMATIC)
        self.kinematic_sprite_list.append(self.player)  # append the player to the KINEMATIC sprite list
        self.space.add(self.player.body, self.player.shape)  # add to the space

    def on_draw(self):
        """ Render the screen. """

        # This command has to happen before we start drawing
        arcade.start_render()

        # Start timing how long this takes
        draw_start_time = timeit.default_timer()

        # draw target rectangle
        arcade.draw_rectangle_outline(500, 1000, 80, 50, arcade.color.BLUSH)

        # Draw all the sprites
        self.static_sprite_list.draw()
        self.dynamic_sprite_list.draw()
        self.kinematic_sprite_list.draw()

        if self.block == 1:  # if the screen blocker is enabled, block the screen
            arcade.draw_rectangle_filled(SCREEN_WIDTH / 2, 0, SCREEN_WIDTH, SCREEN_HEIGHT+0, arcade.color.DARK_SLATE_GRAY)
            # to increase how much of the screen is blocked, increase the number added to SCREEN_HEIGHT, and vice versa

        # Display dev info
        if self.dev == 1:
            output = f"Processing time: {self.processing_time:.3f} s"
            arcade.draw_text(output, 20 + self.view_left, SCREEN_HEIGHT - 20 + self.view_bottom, arcade.color.WHITE, 12)
            output = f"Drawing time: {self.draw_time:.3f} s"
            arcade.draw_text(output, 20 + self.view_left, SCREEN_HEIGHT - 40 + self.view_bottom, arcade.color.WHITE, 12)
            output = f"Left Voltage: {self.data0[1]:.3f} V"
            arcade.draw_text(output, 20 + self.view_left, 60 + self.view_bottom, arcade.color.WHITE, 12)
            output = f"Right Voltage: {self.data0[0]:.3f} V"
            arcade.draw_text(output, 220 + self.view_left, 60 + self.view_bottom, arcade.color.WHITE, 12)
            output = f"Processed Left: {self.left:.3f} V"
            arcade.draw_text(output, 20 + self.view_left, 40 + self.view_bottom, arcade.color.WHITE, 12)
            output = f"Processed Right: {self.right:.3f} V"
            arcade.draw_text(output, 220 + self.view_left, 40 + self.view_bottom, arcade.color.WHITE, 12)
            output = f"Torque Left: {self.torqueLeft:.3f} N/m"
            arcade.draw_text(output, 20 + self.view_left, 20 + self.view_bottom, arcade.color.WHITE, 12)
            output = f"Torque Right: {self.torqueRight:.3f} N/m"
            arcade.draw_text(output, 220 + self.view_left, 20 + self.view_bottom, arcade.color.WHITE, 12)
            output = "Control Bar with Torques, P to drop initial Box, A to re-zero, S for new MVT "
            arcade.draw_text(output, 20 + self.view_left, SCREEN_HEIGHT - 60 + self.view_bottom, arcade.color.WHITE, 12)
            output = "9 & 0 to alter required effort, B to block screen, E to switch between effort and torque"
            arcade.draw_text(output, 20 + self.view_left, SCREEN_HEIGHT - 80 + self.view_bottom, arcade.color.WHITE, 12)
            output = f"Effort Multiplier: {self.effortScale:.1f}"
            arcade.draw_text(output, 800 + self.view_left, SCREEN_HEIGHT - 85 + self.view_bottom, arcade.color.AMBER,
                             12)
            if self.effort == 1:
                output = "Balancing Based on Effort"
                arcade.draw_text(output, 20 + self.view_left, SCREEN_HEIGHT - 100 + self.view_bottom,
                                 arcade.color.WHITE, 12)
            else:
                output = "Balancing Based on Torque"
                arcade.draw_text(output, 20 + self.view_left, SCREEN_HEIGHT - 100 + self.view_bottom,
                                 arcade.color.WHITE, 12)
            output = f"Max Left: {self.maxL:.3f}"
            arcade.draw_text(output, 20 + self.view_left, SCREEN_HEIGHT - 120 + self.view_bottom, arcade.color.WHITE,
                             12)
            output = f"Max Right: {self.maxR:.3f}"
            arcade.draw_text(output, 220 + self.view_left, SCREEN_HEIGHT - 120 + self.view_bottom, arcade.color.WHITE,
                             12)
            output = f"Cup?: {self.cupmode:.0f}"
            arcade.draw_text(output, 220 + self.view_left, SCREEN_HEIGHT - 140 + self.view_bottom, arcade.color.WHITE,
                             12)

        # Display other info
        if self.succsess == 1:
            output = f"Relax Arms"
            arcade.draw_text(output, 370 + self.view_left, SCREEN_HEIGHT - 500 + self.view_bottom, arcade.color.WHITE,
                             50)

        output = f"Score: {self.score:.0f}"
        arcade.draw_text(output, 800 + self.view_left, SCREEN_HEIGHT - 45 + self.view_bottom, arcade.color.AMBER, 24)
        output = f"{time.time() - self.reset:.1f}"
        arcade.draw_text(output, 800 + self.view_left, SCREEN_HEIGHT - 70 + self.view_bottom, arcade.color.AMBER, 20)

        # record drawing time for display if needed
        self.draw_time = timeit.default_timer() - draw_start_time

    def on_mouse_press(self, x, y, button, modifiers):
        """ Allows user to move objects with mouse, very useful for troubleshooting code, not meant to be used in
        actual trials """

        if button == arcade.MOUSE_BUTTON_LEFT:
            # Store where the mouse is clicked. Adjust accordingly if we've
            # scrolled the viewport.
            self.last_mouse_position = (x + self.view_left, y + self.view_bottom)

            # See if we clicked on any physics object
            shape_list = self.space.point_query(self.last_mouse_position, 1, pymunk.ShapeFilter())

            # If we did, remember what we clicked on
            if len(shape_list) > 0:
                self.shape_being_dragged = shape_list[0]

    def on_mouse_release(self, x, y, button, modifiers):
        """ Handle mouse up events """
        if button == arcade.MOUSE_BUTTON_LEFT:
            # Release the item we are holding (if any)
            self.shape_being_dragged = None

    def on_mouse_motion(self, x, y, dx, dy):
        """ Handle mouse motion events """
        if self.shape_being_dragged is not None:
            # If we are holding an object, move it with the mouse
            self.last_mouse_position = (x + self.view_left, y + self.view_bottom)
            self.shape_being_dragged.shape.body.position = self.last_mouse_position
            self.shape_being_dragged.shape.body.velocity = dx * 20, dy * 20

    def update(self, delta_time):
        """ Update the sprites """

        # Keep track of how long this function takes.
        start_time = timeit.default_timer()

        # Check for sprites that fall off the screen.
        # If so, get rid of them.
        for sprite in self.dynamic_sprite_list:
            if sprite.shape.body.position.y < 0:
                # Remove sprites from physics space
                self.space.remove(sprite.shape, sprite.shape.body)
                # Remove sprites from physics list
                sprite.kill()

                try:  # play noise for loss
                    self.hit.play()
                except:  # exception in place to prevent crash
                    print("multi-fail detected, stop messing around")
                self.score -= 10  # adjust score
                self.failure = 1  # switch to failure state
                self.friction = self.friction * 1.25  # increase friction to lower difficulty

                # Record failure in info file
                with open(self.str2, 'a', newline='') as csvfile:
                    spamwriter = csv.writer(csvfile, delimiter=' ')
                    spamwriter.writerow(['Fail'])
                # Increase trial number
                self.trial += 1  # Increment file

                self.str4 = self.str + 'Trial' + str(self.trial) + 'obj.csv'  # Increase trial number on object file
                with open(self.str4, 'w', newline='') as csvfile:  # Initialize new object file
                    spamwriter = csv.writer(csvfile, delimiter=' ')

                self.str3 = self.str + 'Trial' + str(self.trial) + '.csv'  # Increase trial number on data file
                with open(self.str3, 'w', newline='') as csvfile:  # Initialize data file
                    spamwriter = csv.writer(csvfile, delimiter=' ')

                with open(self.str2, 'a', newline='') as csvfile:  # Initialize info file
                    spamwriter = csv.writer(csvfile, delimiter='|')  # Record start data for new trial
                    spamwriter.writerow([f'Failed lift at {time.time() - self.start:.5f}'])
                    spamwriter.writerow([f'Score now {self.score:.2f}'])
                    spamwriter.writerow([f'Friction Increased to {self.friction:.3f}'])
                    spamwriter.writerow(['--------'])
                self.str2 = self.str + 'Trial' + str(self.trial) + 'info.csv'  # Same with info file
                with open(self.str2, 'w', newline='') as csvfile:
                    spamwriter = csv.writer(csvfile, delimiter=' ')

            if self.cupmode == 1:  # Additional failure conditions for cup mode
                if abs(sprite.angle) > 25:
                    # Remove sprites from physics space
                    self.space.remove(sprite.shape, sprite.shape.body)
                    # Remove sprites from physics list
                    sprite.kill()
                    try:  # play noise for loss
                        self.hit.play()
                    except:  # exception in place to prevent crash
                        print("multi-fail detected, stop messing around")
                    self.score -= 10  # Adjust score
                    self.failure = 1  # Switch to failure state
                    self.friction = self.friction * 1.25  # increase friction

                    with open(self.str2, 'a', newline='') as csvfile:
                        spamwriter = csv.writer(csvfile, delimiter=' ')
                        spamwriter.writerow(['Fail'])   #record Fail in info file
                    self.trial += 1 # Increment trial number for data files

                    self.str4 = self.str + 'Trial' + str(self.trial) + 'obj.csv'  # Increase trial number on object file
                    with open(self.str4, 'w', newline='') as csvfile:
                        spamwriter = csv.writer(csvfile, delimiter=' ')  # Initialize object file

                    self.str3 = self.str + 'Trial' + str(self.trial) + '.csv'
                    with open(self.str3, 'w', newline='') as csvfile:
                        spamwriter = csv.writer(csvfile, delimiter=' ')  # Initialize data file

                    with open(self.str2, 'a', newline='') as csvfile:
                        spamwriter = csv.writer(csvfile, delimiter='|')  # Add final data points to info file
                        spamwriter.writerow([f'Failed lift due to angle at {time.time() - self.start:.5f}'])
                        spamwriter.writerow([f'Score now {self.score:.2f}'])
                        spamwriter.writerow([f'Friction Increased to {self.friction:.3f}'])
                        spamwriter.writerow(['--------'])
                    self.str2 = self.str + 'Trial' + str(self.trial) + 'info.csv'
                    with open(self.str2, 'w', newline='') as csvfile:
                        spamwriter = csv.writer(csvfile, delimiter=' ')  # Initialize new data file

        # Check for succsess condtion
        for sprite in self.dynamic_sprite_list:
            # Large inefficient if statement to determine whether or not the object is in the target zone
            if sprite.shape.body.position.x < self.target[1] and sprite.shape.body.position.x > self.target[
                0] and sprite.shape.body.position.y > self.target[2] and sprite.shape.body.position.y < self.target[3]:
                self.obj += 2  # Increase objective tracker
                if self.obj > 50:   # If objective tracker ##### probably should just be a timer #####
                    self.space.remove(sprite.shape, sprite.shape.body)  # Remove the object on a win
                    sprite.kill()   # Kill the sprite in the physics space
                    try:
                        self.win.play()     # Play the win sound effect
                    except:
                        print("multi-win detected, that shouldn't even be possible")  # Crash protection
                    scoretime = 20 + self.reset - time.time()   # used to adjust score based on speed
                    if scoretime > 0:                           # Score adjustment
                        timeMult = 1 + 0.05 * scoretime         # More scoring adjustment
                    else:
                        timeMult = 1 + 0.02 * scoretime
                    if timeMult < 0.2:
                        timeMult = 0.2
                    self.score += 10 * timeMult * self.effortScale / self.friction      # all scoring adjustment
                    self.succsess = 1   # Switch to success state
            if self.obj > 0:    # Objective counter is always moving down for each object
                self.obj -= 1
                # Note this means if you have two objects the have to both be in the objective zone to win

        if self.succsess == 1:  # If in success state
            self.obj -= 0.5     # Decrease objective tracker
            if self.obj == 0:   # Once objective tracker is back to 0
                self.friction = self.friction * 0.8  # Decrease friction on a win to increase difficulty

                with open(self.str2, 'a', newline='') as csvfile:       # Record all info and create new data files
                    spamwriter = csv.writer(csvfile, delimiter=' ')
                    spamwriter.writerow(['Success'])
                self.trial += 1
                self.str3 = self.str + 'Trial' + str(self.trial) + '.csv'
                self.str2 = self.str + 'Trial' + str(self.trial) + 'info.csv'
                self.str4 = self.str + 'Trial' + str(self.trial) + 'obj.csv'
                with open(self.str4, 'w', newline='') as csvfile:
                    spamwriter = csv.writer(csvfile, delimiter=' ')
                with open(self.str3, 'w', newline='') as csvfile:
                    spamwriter = csv.writer(csvfile, delimiter=' ')
                with open(self.str2, 'a', newline='') as csvfile:
                    spamwriter = csv.writer(csvfile, delimiter='|')
                    spamwriter.writerow([f'Successful lift at {time.time() - self.start:.5f}'])
                    spamwriter.writerow([f'Score now {self.score:.2f}'])
                    spamwriter.writerow([f'Friction Decreased to {self.friction:.3f}'])
                    spamwriter.writerow(['--------'])                   # Finish creating new files

                if self.cupmode == 1:   # Drop new cup if in cupmode
                    sprite = PymunkSprite("images/papercuptrim.png", 500, self.player.body.position[1] + 100,
                                          scale=0.25, width=85, height=170, friction=self.friction)
                else:                   # Drop new box if in box mode
                    sprite = PymunkSprite("images/boxCrate_double.png", 500, self.player.body.position[1] + 100,
                                          scale=0.5, friction=self.friction)
                self.dynamic_sprite_list.append(sprite)  # Add new sprite to lists and physics space
                self.space.add(sprite.body, sprite.shape)
                self.succsess = 0   # Leave succsess state
                self.reset = time.time()

        if self.failure == 1:   # If in failure state
            self.losscounter -= 1
            if self.losscounter == 0:   # once loss counter variable is zero reset process
                self.losscounter = 100  # reset loss counter ##### probably should just be a timer #####
                if self.cupmode == 1:   # Drop new cup if in cup mode
                    sprite = PymunkSprite("images/papercuptrim.png", 500, self.player.body.position[1] + 100,
                                          scale=0.25, width=85, height=170, friction=self.friction)
                else:                   # Drop new box if in box mode
                    sprite = PymunkSprite("images/boxCrate_double.png", 500, self.player.body.position[1] + 100,
                                          scale=0.5, friction=self.friction)
                self.dynamic_sprite_list.append(sprite)     # add new sprites to lists and physics space
                self.space.add(sprite.body, sprite.shape)
                self.failure = 0    # Exit failure space
                self.reset = time.time()

        # Update physics
        # Use a constant time step, don't use delta_time
        self.space.step(1 / 40.0) ####### Change This to adjust for shorter computation time #########

        self.data0 = self.daqtask0.read()   # read the daq   #####NEEDs TO BE CHANGED TO REFLECT ARM ARRANGMENT####
        self.right = -self.data0[0] + self.avg0  # split into left and right readings and add offset
        self.left = -self.data0[1] + self.avg1

        self.torqueRight = 53.0 * self.right  # convert from voltage to torque
        self.torqueLeft = 51.9 * self.left

        with open(self.str3, 'a', newline='') as csvfile:  # Record torques in data file
            spamwriter = csv.writer(csvfile, delimiter=' ')  # Include time
            spamwriter.writerow([self.torqueLeft, self.torqueRight, time.time() - self.start])

        # start Therese's edit
        # Turn screen block on if sum of left and right torques is too low
        """
        screen_block_threshold = 0.5 * (abs(self.maxL) + abs(self.maxR))
        if abs(self.torqueLeft) + abs(53 * self.torqueRight) >= screen_block_threshold: 
            self.block = 0
        else:
            self.block = 1
        """
        # end Therese's edit

        # Angle Control
        # Set target angle based on the difference in torques
        self.qtarg = ((2 * self.left / self.maxL) - (2 * self.right / self.maxR)) * 1.25 # this equation makes no sense
        if self.qtarg < 0.07 and self.qtarg > -0.07: # if the angle target is small enough, set it to 0
            self.qtarg = 0
        self.qdiff = abs(self.qtarg - self.player.body.angle)   # Calculate difference between target and actual
        if self.player.body.angle < self.qtarg:                 # Control the angular velocity of the platform
            self.qveltarg = 7 * self.qdiff                      # this gives smoother motion than controlling
        if self.player.body.angle > self.qtarg:                 # the angle directly
            self.qveltarg = -7 * self.qdiff                     # Velocity is lower for small differences
        if self.qdiff < 0.05:
            self.qveltarg = self.qveltarg / 3
            if self.qdiff < 0.01:
                self.qveltarg = 0
        # self.player.body._set_angular_velocity(self.qveltarg)   # set the angular velocity to the calculated one
        # start Therese's edit to replace above line
        # Set angular velocity to 0 if target torque sum (75% of MVT) hasn't been reached yet
        # Otherwise angular velocity should reflect actual torque difference
        if abs(self.torqueLeft) + abs(self.torqueRight) < self.torqueThreshold:
            self.player.body._set_angular_velocity(0)
        else:
            self.player.body._set_angular_velocity(self.qveltarg)
        # end Therese's edit

        


        # Position Control
        # Set target height based on average of tourques (weighted with effort scale)
        self.ptarg = 200 + ((self.left / (self.maxL * self.effortScale)) + (
                    self.right / (self.maxR * self.effortScale))) * 600 / 2
        self.pdiff = abs(self.ptarg - self.player.body.position[1]) # Get difference between target and actual
        if self.player.body.position[1] < self.ptarg:               # Again control velocity for smoother motion
            self.pveltarg = 7 * self.pdiff                          # Scale speed based on difference between actual
        if self.player.body.position[1] > self.ptarg:               # and target
            self.pveltarg = -7 * self.pdiff
        if self.pdiff < 0.05:
            self.pveltarg = self.pveltarg / 3
            if self.pdiff < 0.01:
                self.pveltarg = 0
        self.player.body._set_velocity([0, self.pveltarg])          # Set the velocity to equal the target

        # start Therese's edit for position control (to replace above block of code)
        # y position depends on sum of left and right torques
        # self.player.body.position[1] += 5 * (abs(self.torqueLeft) + abs(self.torqueRight)) # test to see if 5 is the right scale
        # end Therese's edit 

        if self.shape_being_dragged is not None:    # used to drag objects with mouse
            self.shape_being_dragged.shape.body.position = self.last_mouse_position
            self.shape_being_dragged.shape.body.velocity = 0, 0

        # Record Object Position in obj file
        for sprite in self.dynamic_sprite_list: # This records data for each object each time through the loop, so if there is more than one object
                                                # then you get multiple entries at roughly the same time
            with open(self.str4, 'a', newline='') as csvfile:
                spamwriter = csv.writer(csvfile, delimiter=' ')
                spamwriter.writerow([sprite.shape.body.position.x, sprite.shape.body.position.y, sprite.angle,
                                     time.time() - self.start])
        # Re-sync the sprites to the physics objects that shadow them
        resync_physics_sprites(self.dynamic_sprite_list)
        resync_physics_sprites(self.kinematic_sprite_list)
        # Save the time it took to do this.
        self.processing_time = timeit.default_timer() - start_time

    def on_key_press(self, symbol: int, modifiers: int):
        """ Handle keyboard presses. """

        if symbol == arcade.key.A:  # Take new averages to calibrate offset
            arcade.start_render()   # Start drawing
            output = f"Completely Relax Arms"
            arcade.draw_text(output, 120 + self.view_left, SCREEN_HEIGHT - 500 + self.view_bottom,
                             arcade.color.BEAU_BLUE, 50)  # Send instructions
            arcade.finish_render()  # Finish drawing
            self.averagecount = 0   # Indexing variable
            self.ch0avg = []    # Blank array for averages
            self.ch1avg = []    # Blank array for averages
            while self.averagecount < 100:
                self.averagecount += 1  # Increment indexing variable
                self.temp = self.daqtask0.read()  # Read DAQ
                self.ch0avg.append(self.temp[0])  # Spilt reading into arrays
                self.ch1avg.append(self.temp[1])
            self.avg0 = sum(self.ch0avg) / len(self.ch0avg)  # Take averages and adjust offsets
            self.avg1 = sum(self.ch1avg) / len(self.ch1avg)

            with open(self.str2, 'a', newline='') as csvfile:
                spamwriter = csv.writer(csvfile, delimiter='|')
                spamwriter.writerow([f'New Averages Taken at {time.time() - self.start:.5f}'])
                spamwriter.writerow([f'New Average Left {self.avg1:.3f}'])
                spamwriter.writerow([f'New Average Right {self.avg0:.3f}'])
                spamwriter.writerow(['--------'])

        elif symbol == arcade.key.S:    # Recalibrate the maximum torques
            arcade.start_render()   # Start Drawing
            output = f"Apply Maximum Torque Now"    # Some instructions
            arcade.draw_text(output, 120 + self.view_left, SCREEN_HEIGHT - 500 + self.view_bottom,
                             arcade.color.ATOMIC_TANGERINE, 50)  # Display the instructions
            arcade.finish_render()  # Finish Drawing
            self.maxCount = 0   # Indexing variable for maximum count
            self.ch0Max = []    # Blank array for maximums
            self.ch1Max = []    # Blank array for maximums
            while self.maxCount < 100:  # Loop to fill arrays
                self.maxCount += 1      # Increase index
                self.temp = self.daqtask0.read()    # temp variable from reading daq  #####NEEDs TO BE CHANGED TO REFLECT ARM ARRANGMENT####
                '''
                Math is not right line 603
                '''
                self.ch0Max.append(-self.temp[0] + self.avg0)    # Store temp into array     
                self.ch1Max.append(-self.temp[1] + self.avg0)    # Store temp into array

            self.max0 = max(self.ch0Max)  # get max from array
            self.max1 = max(self.ch1Max)
            self.mvt = min(self.max0, self.max1)    # MVT is the smaller of the two maxim

            ## Note the distinction between effort mode or not here, we do not want to use the maximum torque as the target.
            # There needs to be a variable that defines the target level

            if self.effort == 1:    # If in effort mode, use individual maximums
                self.maxL = self.max0
                self.maxR = self.max1
                
            else:   # If not in effort mode use MVT for both arms
                self.maxR = self.mvt
                self.maxL = self.mvt

            with open(self.str2, 'a', newline='') as csvfile:
                spamwriter = csv.writer(csvfile, delimiter='|')
                if self.effort == 1:
                    spamwriter.writerow(['In effort mode, Maximums set to individual limits'])
                else:
                    spamwriter.writerow(['In torque mode, Maximums set to equal values (lower of two arms)'])
                spamwriter.writerow([f'New Maximums Taken at {time.time() - self.start:.5f}'])
                spamwriter.writerow([f'New Max Left {self.maxL:.3f}'])
                spamwriter.writerow([f'New Max Right {self.maxR:.3f}'])
                spamwriter.writerow(['--------'])

        elif symbol == arcade.key.E:    # Switch to effort mode (don't know if you need this, but its there)
            # Switches between individual maximums based on each arms individual max or just same value based on
            # the smaller of the two values
            with open(self.str2, 'a', newline='') as csvfile:
                spamwriter = csv.writer(csvfile, delimiter='|')
                if self.effort == 0:    # If not in effort mode
                    self.effort = 1     # Enter effort mode
                    self.maxL = self.max0   # Left max is now the max from the left
                    self.maxR = self.max1   # Right max isnow the max from the right
                    spamwriter.writerow([
                                            f'Switched to effort mode, Maximums set to individual limits at {time.time() - self.start:.5f}'])
                    spamwriter.writerow(['--------'])  # Mark the time
                else:
                    self.effort = 0
                    self.maxR = self.mvt   # Max right is now the lower of the two maximums
                    self.maxL = self.mvt   # Max left is now the lower of the two maximums
                    spamwriter.writerow([
                                            f'Switched to Torque mode, Maximums set to equal values (lower of two arms) at {time.time() - self.start:.5f}'])
                    spamwriter.writerow(['--------'])  # Mark the time

        elif symbol == arcade.key.D:  # Switch to dev mode
            # Displays additional information on screen
            if self.dev == 0:
                self.dev = 1
            else:
                self.dev = 0

        elif symbol == arcade.key.B:  # Enable screen blocker
            with open(self.str2, 'a', newline='') as csvfile:
                spamwriter = csv.writer(csvfile, delimiter='|')
                if self.block == 0: # If blocker is off
                    self.block = 1  # Turn it on
                    # Mark time of change
                    spamwriter.writerow([f'Screen Blocker Enabled at {time.time() - self.start:.5f}'])
                else:  # If Blocker is on
                    self.block = 0  # Turn it off
                    # Mark time of change
                    spamwriter.writerow([f'Screen Blocker Disabled at {time.time() - self.start:.5f}'])
                spamwriter.writerow(['--------'])

        elif symbol == arcade.key.ESCAPE:  # You can use escape to shut down
            print('Shutting Down')  # Just hitting X works fine too
            exit(0)

        elif symbol == arcade.key.KEY_0:  # Increase required effort
            self.effortScale += 0.1  # increase
            with open(self.str2, 'a', newline='') as csvfile:
                spamwriter = csv.writer(csvfile, delimiter='|')
                spamwriter.writerow([f'Effort Scale increased to {self.effortScale:.1f}'])  # Mark time of change
                spamwriter.writerow([f'Time is {time.time() - self.start:.5f}'])
                spamwriter.writerow(['--------'])

        elif symbol == arcade.key.KEY_9:  # Decrease Required effort
            if self.effortScale > 0.2:  # Check if effort scale is not to low
                self.effortScale -= 0.1  # Decrease required effort
            with open(self.str2, 'a', newline='') as csvfile:
                spamwriter = csv.writer(csvfile, delimiter='|')
                spamwriter.writerow([f'Effort Scale decreased to {self.effortScale:.1f}'])  # Mark time of change
                spamwriter.writerow([f'Time is {time.time() - self.start:.5f}'])
                spamwriter.writerow(['--------'])

        elif symbol == arcade.key.C:  # Switch to and from using cups
            if self.cupmode == 0:  # If not in cup mode
                self.cupmode = 1  # Switch to cupmode
                for sprite in self.dynamic_sprite_list:
                    # Remove sprites from physics space
                    self.space.remove(sprite.shape, sprite.shape.body)
                    # Remove sprites from physics list
                    sprite.kill()
                with open(self.str2, 'a', newline='') as csvfile:
                    spamwriter = csv.writer(csvfile, delimiter='|')
                    spamwriter.writerow(
                        [f'Switched to Cups at: {time.time() - self.start:.5f}'])  # Mark time for switch
                    spamwriter.writerow(['--------'])
            elif self.cupmode == 1:  # If in cup mode
                self.cupmode = 0  # Switch out of cupmode
                for sprite in self.dynamic_sprite_list:
                    # Remove sprites from physics space
                    self.space.remove(sprite.shape, sprite.shape.body)
                    # Remove sprites from physics list
                    sprite.kill()
                with open(self.str2, 'a', newline='') as csvfile:
                    spamwriter = csv.writer(csvfile, delimiter='|')
                    spamwriter.writerow(
                        [f'Switched to Boxes at: {time.time() - self.start:.5f}'])  # mark time for switch
                    spamwriter.writerow(['--------'])

        elif symbol == arcade.key.K:  # Kill sprites, hit it a lot if there are a lot of sprites
            for sprite in self.dynamic_sprite_list:
                # Remove sprites from physics space
                self.space.remove(sprite.shape, sprite.shape.body)
                # Remove sprites from physics list
                sprite.kill()

    def on_key_release(self, symbol: int, modifiers: int):
        """ Handle keyboard releases. """
        # Drop a box or cup on release of P key
        if symbol == arcade.key.P:
            if self.cupmode == 1:
                # drop in new cup
                sprite = PymunkSprite("images/papercuptrim.png", 500, self.player.body.position[1] + 100, scale=0.25,
                                      width=85, height=170, friction=self.friction)
                self.dynamic_sprite_list.append(sprite)
                self.space.add(sprite.body, sprite.shape)
            if self.cupmode == 0:
                # drop in new box
                sprite = PymunkSprite("images/boxCrate_Double.png", 500, self.player.body.position[1] + 100, scale=0.5,
                                      friction=self.friction)
                self.dynamic_sprite_list.append(sprite)
                self.space.add(sprite.body, sprite.shape)


def main():
    MyGame(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_TITLE)  # Create the game window
    arcade.run()  # Run the game (loops update and draw


if __name__ == "__main__":
    main()  # Run main
