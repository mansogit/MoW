# DESCRIPTION: 
# OBJECTIVE: Read all grid locations and identify all barriers within 10 mins
# INPUTS: 
#   NxN size
#   start position
#   end position

# OUTPUTS:
#   store maze and plot for visuals
#   identify barriers
#   find the best path for race phase

# Import packages
#import pyhula # to use api commands
#import time # not sure if we need this
from time import time
import numpy as np # call package from venv\Lib\site-packages
import pandas as pd # call package from venv\Lib\site-packages
import matplotlib.pyplot as plt 
import matplotlib.patches as patches
#from matplotlib.patches import Rectangle
#import matplotlib.patches as mpatches


# User inputs

# mazeSize = input(("Enter number of columns and rows respectively of maze (M x N)): "))
# numCols,numRows = mazeSize.split('X')
# startPos = input(("Enter start X and Y respectively (X,Y)): "))
# xi,yi = startPos.split(',')
# endPos = input(("Enter end X and Y respectively (X,Y)): "))
# xf,yf = endPos.split(',')
mazeSize = "4X3"
numCols,numRows = mazeSize.split('X')
startPos = "0,0"
xi,yi = startPos.split(',')
endPos = "3,2"
xf,yf = endPos.split(',')

print("Maze :" + mazeSize + "type is " + str(type(mazeSize)))
print(" columns=" + numCols + "type is " + str(type(numCols)))
print("rows=" + numRows + " type is " + str(type(numRows)))


# construct grid
x = np.linspace(0,float(numCols))
y = np.linspace(0,float(numRows))
cols,rows = np.meshgrid(x,y)

# echo user inputs
print("\nMaze size:" + str(numCols) + "x" + str(numRows))
print(f"\nStart: (X={xi}, Y={yi})\nEnd: (X={xf}, Y={xf})")
blockSize = 0.30 # cm per block
borderDistance = blockSize/2


# Plot grid as a sanity check
plt.isinteractive() # make the plot interactive if off then plt.ioff()
fig, ax = plt.subplots(figsize=(8,8)) 
ax.set_xlim(-0.5,int(numCols)-0.5)
ax.set_ylim(-0.5,int(numRows)-0.5) 
ax.set_title('Maze Grid')
ax.set_xlabel('X (columns)')
ax.set_ylabel('Y (rows)')  
ax.grid(color='r', linestyle='-', linewidth=2)


# Draw maze grid
for block in range(int(numCols) + 1):
    ax.axvline( block - 0.5 , color='grey', linewidth=2)

for block in range(int(numRows) + 1):
    ax.axhline(block - 0.5 , color='grey', linewidth=2)

plt.show()
# Draw start and end
ax.add_patch(patches.Rectangle((xi-0.4 , yi-0.4), borderDistance, borderDistance, 
                        facecolor='green', label='Start'))
ax.add_patch(patches.Rectangle((xf - 0.4, yf -0.4), 0.8, 0.8, 
                        facecolor='red', alpha=0.15, label='End'))


# # Initialize parameters:

# speed = 50 # speed of drone
height = 80 # height from ground

# # # Connect to drone
# # api = pyhula.UserApi()
# # if not api.connect():
# #     print("Connection failed")
# #     exit()
# # else: 
# #     print("Connected successfully")
# #     # Add blocks that will work here

# # Command the drone to take off to a predefined safe altitude
# api.single_fly_takeoff(height)
# time.sleep(3)   # Allow motors and sensors to stabilize after takeoff

#     # QR Code Positioning Switch
#     # 0-Enable QR code positioning
#     # 1-Disable QR code positioning.
# api.Plane_cmd_switch_QR(0)

#     # Align and hover over QR card:
#     # mode = 0  → Optical flow alignment (default / hover stability)
#     # mode = 1  → Camera alignment (based on camera frame detection)
#     # qr_id = 0 → QR card index
#     # while True:
# api.single_fly_Optical_flow_alignment(3, 20, 90)
#     # print(f" QR {api.single_fly_Optical_flow_recognition(3,20)}")
