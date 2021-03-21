# AKIG 2020
# Matthias Rosa - 01607526
# Victoria Kostjak - 11771176

# workflow for getting current pose of the huskey
# with automatic measures by a tachymeter

# 1 connect with the huskey

# 2 the huseky has to be on the starting point faceing the 
# closer wall with the SIC laserscanner

# 3 set transformation between map and tachy_aim_data
# command: $ rosrun tf static_transform_publisher 0 0 0 0 0 0 /map /tachy_aim_Data 125

# 4 set the tachymeter orientation onto the centerpoint of the
# SIC laserscanner mounted at the front of the huskey

# 5 start huseky_tf2pose.py with python3
# programm publish current huseky pose as a posestamped

# 6 start akig_xyz_hzvd_provider.py with python3
# programm publish needed data for measure the fixed points
# the data is published as a pointcloud2
# the list of fixed points (fixpoints.txt) has to be imported here
# required layout: point number Y X Z
# separated by tabulator

# 7 start akig_pose_provider.py with python3
# programm waits for four measures from tachymeter
# then calculates the current pose with a "Rueckwaertsschnitt"
# in a Grauss-Markov-Modell
# it prints the iteration of the calculation, the adjusted parameters
# and the estimated parameters [y,x,z,ou]
# the current pose is published as a posestamped

# 8 start akig_measure.py with python3
# programm start the tachymeter which measure the fixed points
# the data is published as a pointstamped 
