import numpy as np

quadrant_near_object = np.zeros((3,3), dtype=bool)

test = (640,250)

for i in range(0,3):
    col = test[0] / 214
    row = test[1] / 160
    if row == 3:
        row = 2

    quadrant_near_object[row][col] = True

if (quadrant_near_object[0][0] == True or quadrant_near_object[0][1] == True or quadrant_near_object[0][2] == True):
    object_left_column = True
if (quadrant_near_object[1][0] == True or quadrant_near_object[1][1] == True or quadrant_near_object[1][2] == True):
    object_mid_column = True
if (quadrant_near_object[2][0] == True or quadrant_near_object[2][1] == True or quadrant_near_object[2][2] == True):
    object_right_column = True
if (quadrant_near_object[0][0] == True or quadrant_near_object[1][0] == True or quadrant_near_object[2][0] == True):
    object_top_row = True
if (quadrant_near_object[0][1] == True or quadrant_near_object[1][1] == True or quadrant_near_object[2][1] == True):
    object_mid_row = True
if (quadrant_near_object[0][2] == True or quadrant_near_object[1][2] == True or quadrant_near_object[2][2] == True):
    object_bottom_row = True

print quadrant_near_object