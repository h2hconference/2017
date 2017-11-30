# H2HC I'm your Founder CozmoPatrol SDK Script

import asyncio
import sys,time
import numpy as np
import cv2
from random import randint


import cozmo
from cozmo.util import degrees
from PIL import Image

class HALIGN:
    LEFT = 0
    RIGHT = 1
    CENTER = 2
    JUSTIFY = 3

class VALIGN:
    TOP = 0
    BOTTOM = 1
    MIDDLE = 2
    JUSTIFY = 3


class CozmoScreen:
    def __init__(self):
        self.img = np.zeros((32,128))
        self.bigfont = { 'fontFace': cv2.FONT_HERSHEY_PLAIN, 'fontScale': 1.5, 'thickness': 2 }
        self.font = { 'fontFace': cv2.FONT_HERSHEY_PLAIN, 'fontScale': 1, 'thickness': 1 }
        self.smallfont = { 'fontFace': cv2.FONT_HERSHEY_PLAIN, 'fontScale': 0.25, 'thickness': 1 }

    def putText(self, text, offset = (0,2), halign = HALIGN.LEFT, valign = VALIGN.TOP, font = None, color = 255):
        font = self.font if font is None else font
        lines0 = text.split('\n')
    
        lines = []
        height = 0
        width = 0
        for line in lines0:
            words = line.split(" ")
            size,_ = cv2.getTextSize(text=line,**font)
            height += size[1]
            width = max(width,size[0])

            # get width by indivual words
            wwidth = 0
            for word in words:
                s,_ = cv2.getTextSize(text=word,**font)
                wwidth += s[0]

            lines.append( (size,wwidth,words))

        # compute y0
        ys = 2
        height0 = lines[0][0][1]
        if valign == VALIGN.JUSTIFY:
            ys = (self.img.shape[0] - height - offset[1]) / float(len(lines))
            y0 = height0 + offset[1]
        elif valign == VALIGN.TOP:
            y0 = height0 + offset[1]
        elif valign == VALIGN.MIDDLE:
            y0 = (self.img.shape[0] + height)//2 + offset[1]
        elif valign == VALIGN.BOTTOM:
            y0 = self.img.shape[0] - height - offset[1] + height0
            
        y = float(y0)
        for size, wwidth, words in lines:

            # compute x0
            xs = 0
            if len(words) > 1:
                spacew = (size[0]-wwidth) / float(len(words)-1)
            else:
                spacew = 0

            if halign == HALIGN.JUSTIFY:
                xs = (self.img.shape[1] - size[0] - offset[0]) / float(len(words))
                spacew = 0
                x0 = offset[0]
            elif halign == HALIGN.LEFT:
                x0 = offset[0]
            elif halign == HALIGN.CENTER:
                x0 = (self.img.shape[1] - size[0])//2 + offset[0]
            elif halign == HALIGN.RIGHT:
                x0 = self.img.shape[1] - size[0] - offset[0]

            x = x0
            for word in words:
                org = (int(x),int(y))
                cv2.putText(self.img, text=word, **font, org=org, color=color)
                s,_ = cv2.getTextSize(text=word,**font)
                x += spacew + xs + s[0]
                
            y += size[1] + ys
            

    def export(self):
        return self.img
            


def cozmo_program(robot: cozmo.robot.Robot):
    robot.camera.image_stream_enabled = True


    initial_pose_angle = robot.pose_angle

    patrol_offset = 0  # middle
    max_pose_angle = 45  # offset from initial pose_angle (up to +45 or -45 from this)

    # Time to wait between each turn and patrol, in seconds
    time_between_turns = 2.5
    time_between_patrols = 7

    time_for_next_turn = time.time() + time_between_turns
    time_for_next_patrol = time.time() + time_between_patrols


    start_h2hc = False
    while True:

        if (time.time() > time_for_next_turn):
            print("TURN START")
            # pick a random amount to turn
            angle_to_turn = randint(10,40)

            # 50% chance of turning in either direction
            if randint(0,1) > 0:
                angle_to_turn = -angle_to_turn

            # Clamp the amount to turn

            face_angle = (robot.pose_angle - initial_pose_angle).degrees

            face_angle += angle_to_turn
            if face_angle > max_pose_angle:
                angle_to_turn -= (face_angle - max_pose_angle)
            elif face_angle < -max_pose_angle:
                angle_to_turn -= (face_angle + max_pose_angle)

            # Turn left/right
            robot.turn_in_place(degrees(angle_to_turn)).wait_for_completed()

            # Tilt head up/down slightly
            robot.set_head_angle(degrees(randint(30,44))).wait_for_completed()

            # Queue up the next time to look around
            time_for_next_turn = time.time() + time_between_turns

        if (time.time() > time_for_next_patrol):

            # Check which way robot is facing vs initial pose, pick a new patrol point

            face_angle = (robot.pose_angle - initial_pose_angle).degrees
            drive_right = (patrol_offset < 0) or ((patrol_offset == 0) and (face_angle > 0))

            # Turn to face the new patrol point

            if drive_right:
                robot.turn_in_place(degrees(90 - face_angle)).wait_for_completed()
                patrol_offset += 1
            else:
                robot.turn_in_place(degrees(-90 - face_angle)).wait_for_completed()
                patrol_offset -= 1

            # Drive to the patrol point, playing animations along the way

            robot.drive_wheels(15, 15)
            for i in range(1,4):
                robot.play_anim("anim_hiking_driving_loop_0" + str(i)).wait_for_completed()

            # Stop driving

            robot.stop_all_motors()

            # Turn to face forwards again

            face_angle = (robot.pose_angle - initial_pose_angle).degrees
            if face_angle > 0:
                robot.turn_in_place(degrees(-90)).wait_for_completed()
            else:
                robot.turn_in_place(degrees(90)).wait_for_completed()

            # Queue up the next time to patrol
            time_for_next_patrol = time.time() + time_between_patrols

        if start_h2hc:
            robot.move_head(-5) # start moving head down so it mostly happens in parallel with lift
            robot.set_lift_height(0.0).wait_for_completed()
            robot.set_head_angle(degrees(-25.0)).wait_for_completed()

            # Start slowly raising lift and head
            robot.move_lift(0.05)

            scr = CozmoScreen()
            scr.putText("H2HC", halign=HALIGN.CENTER, valign = VALIGN.MIDDLE, font = scr.bigfont)

            robot.move_head(0.15)
    
            image = Image.fromarray(scr.export())
            face_image = cozmo.oled_face.convert_image_to_screen_data(image, invert_image=False)
            robot.display_oled_face_image(face_image, 500)
            say = robot.say_text("I am your founder!!", in_parallel=True)
            say.wait_for_completed()

            robot.play_anim(name="id_poked_giggle").wait_for_completed()
            robot.move_lift(0)
            robot.play_anim(name="anim_fistbump_success_03").wait_for_completed()

            # Tilt head up to look for people
            robot.set_head_angle(cozmo.robot.MAX_HEAD_ANGLE).wait_for_completed()
            robot.set_lift_height(0.0).wait_for_completed()
            start_h2hc = False
        else:
            owner_face = None
            intruder_face = None
            for visible_face in robot.world.visible_faces:
                print('VISIBLE FACE (%s)'% visible_face.name)
                start_h2hc = True

        # Update times first/last seen owner or an intruder

        time.sleep(0.5)




cozmo.run_program(cozmo_program)
