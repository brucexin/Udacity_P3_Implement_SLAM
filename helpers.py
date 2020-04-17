from robot_class import robot
from math import *
import random
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns


# --------
# this helper function displays the world that a robot is in
# it assumes the world is a square grid of some given size
# and that landmarks is a list of landmark positions(an optional argument)
def display_world(world_size, position, landmarks=None):
    
    # using seaborn, set background grid to gray
    sns.set_style("dark")

    # Plot grid of values
    world_grid = np.zeros((world_size+1, world_size+1))

    # Set minor axes in between the labels
    ax=plt.gca()
    cols = world_size+1
    rows = world_size+1

    ax.set_xticks([x for x in range(1,cols)],minor=True )
    ax.set_yticks([y for y in range(1,rows)],minor=True)
    
    # Plot grid on minor axes in gray (width = 1)
    plt.grid(which='minor',ls='-',lw=1, color='white')
    
    # Plot grid on major axes in larger width
    plt.grid(which='major',ls='-',lw=2, color='white')
    
    # Create an 'o' character that represents the robot
    # ha = horizontal alignment, va = vertical
    ax.text(position[0], position[1], 'o', ha='center', va='center', color='r', fontsize=30)
    
    # Draw landmarks if they exists
    if(landmarks is not None):
        # loop through all path indices and draw a dot (unless it's at the car's location)
        for idx, pos in enumerate(landmarks):
            if(pos != position):
                ax.text(pos[0], pos[1], 'x-%d'%idx, ha='center', va='center', color='purple', fontsize=20)
    
    # Display final result
    plt.show()
    

# --------
# this helper function displays the world that a robot is in
# it assumes the world is a square grid of some given size
# and that landmarks is a list of landmark positions(an optional argument)
# objects is a list contained all trajectory objects. Every trajectory object
# has (poses, landmarks, color) properties. Poses is robot position list of x 
# and y axis on every sense poiont. landmarks is landmarks position, and color 
# is plot color used for the trajectory object.
# measurement_range is sense circle radius.
def display_trajectory(world_size, objects, landmarks=None, measurement_range=None):
    
    # using seaborn, set background grid to gray
    sns.set_style("dark")

    # Plot grid of values
    world_grid = np.zeros((world_size+1, world_size+1))

    # Set minor axes in between the labels
    ax=plt.gca()
    cols = world_size+1
    rows = world_size+1

    ax.set_xticks([x for x in range(1,cols)],minor=True )
    ax.set_yticks([y for y in range(1,rows)],minor=True)
    
    # Plot grid on minor axes in gray (width = 1)
    plt.grid(which='minor',ls='-',lw=1, color='white')
    
    # Plot grid on major axes in larger width
    plt.grid(which='major',ls='-',lw=2, color='white')
    
    # Create an 'o' character that represents the robot
    # ha = horizontal alignment, va = vertical
    for (trajectory, landmarks, clr) in objects:
        p_x = None
        p_y = None
        for idx, position in enumerate(trajectory):
            ax.text(position[0], position[1], 'o-%d'%idx, ha='center', va='center', color=clr, fontsize=15)
            if p_x != None:
                dx = position[0] - p_x
                dy = position[1] - p_y
                arrow = plt.arrow(p_x, p_y, dx, dy, head_width=1, length_includes_head=True, color=clr)
                ax.add_patch(arrow)
            p_x = position[0]
            p_y = position[1]
        #
        #if measurement_range:
        #    circ=plt.Circle((position[0],position[1]), radius=measurement_range, color='g', fill=False)
        #    ax.add_patch(circ)
    
        # Draw landmarks if they exists
        if(landmarks is not None):
            # loop through all path indices and draw a dot (unless it's at the car's location)
            for idx, pos in enumerate(landmarks):
                if(pos != position):
                    ax.text(pos[0], pos[1], 'x-%d'%idx, ha='center', va='center', color=clr, fontsize=20)
    
    # Display final result

    
# --------
# this routine makes the robot data
# the data is a list of measurements and movements: [measurements, [dx, dy]]
# collected over a specified number of time steps, N
#
def make_data(N, num_landmarks, world_size, measurement_range, motion_noise, 
              measurement_noise, distance):

    # check that data has been made
    try:
        check_for_data(num_landmarks, world_size, measurement_range, motion_noise, measurement_noise)
    except ValueError:
        print('Error: You must implement the sense function in robot_class.py.')
        return []
    
    complete = False
    
    while not complete:
        r = robot(world_size, measurement_range, motion_noise, measurement_noise)
        r.make_landmarks(num_landmarks)

        data = []

        seen = [False for row in range(num_landmarks)]
    
        # guess an initial motion
        orientation = random.random() * 2.0 * pi
        dx = cos(orientation) * distance
        dy = sin(orientation) * distance
            
        for k in range(N-1):
    
            # collect sensor measurements in a list, Z
            Z = r.sense()

            # check off all landmarks that were observed 
            for i in range(len(Z)):
                seen[Z[i][0]] = True
    
            # move
            while not r.move(dx, dy):
                # if we'd be leaving the robot world, pick instead a new direction
                orientation = random.random() * 2.0 * pi
                dx = cos(orientation) * distance
                dy = sin(orientation) * distance

            # collect/memorize all sensor and motion data
            #print('motion', dx, dy, 'position', r.x, r.y)
            data.append([Z, [dx, dy, r.x, r.y]])

        # we are done when all landmarks were observed; otherwise re-run
        complete = (sum(seen) == num_landmarks)
        #if not complete:
        #    print('cannot seen all, continue!')

    print(' ')
    print('Landmarks: ', r.landmarks)
    print(r)
    
    return data, r


def check_for_data(num_landmarks, world_size, measurement_range, motion_noise, measurement_noise):
    # make robot and landmarks
    r = robot(world_size, measurement_range, motion_noise, measurement_noise)
    r.make_landmarks(num_landmarks)
    
    
    # check that sense has been implemented/data has been made
    test_Z = r.sense()
    if(test_Z is None):
        raise ValueError
