# AKIG 2020 Victoria Kostjak - 11771176
# Programm zur Berechnung der aktuellen Pose mittels ausgeglichenem Rueckwaertsschnitt
# Grundstruktur des Codes von Finn Linzer
# Program which publish the current pose after calculate it with a "Rueckwaertsschnitt"
# in a Grauss-Markov-Modell

import rospy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import math
import numpy as np
import tf
import sensor_msgs.point_cloud2 as pc2
from scipy.linalg import block_diag

def pose_provider():

    # publisher of current pose
    pub = rospy.Publisher('pose',geometry_msgs.msg.PoseStamped, queue_size=10)
    rospy.init_node('akig_RS_provider')
    cmd = geometry_msgs.msg.PoseStamped()

    # get measured Hz and V value, programm waits for four measures
    Hz_rad = list()
    V_rad = list()
    for i in range(0,4):
    	temp = rospy.wait_for_message("/tachymeasure", geometry_msgs.msg.PointStamped)
    	Hz_rad.append(temp.point.x)
    	V_rad.append(temp.point.y)

    # get coordinates of the fixed points
    fest = rospy.wait_for_message("/fixed_points", sensor_msgs.msg.PointCloud2)
    fest = list(pc2.read_points(fest))
    X_Fp = list()
    Y_Fp = list()
    Z_Fp = list()
    for i in range(len(fest)):
    	X_Fp.append(fest[i][0])
    	Y_Fp.append(fest[i][1])
    	Z_Fp.append(fest[i][2])

    # calculate Qll matrix
    gon_o_r = 0.0005
    rad_o_r = (gon_o_r/200)*math.pi
    o_r = rad_o_r**2
    ell_Hz = np.diag(o_r*np.ones(len(Hz_rad)))
    ell_V = np.diag(o_r*np.ones(len(V_rad)))
    ell = block_diag(ell_Hz,ell_V)
    Qll = (1/o_r)*ell

    # value of the termination condition
    Epsilon = 0.00000001


    # vector of the unknown [y[m],x[m],z[m],ou[]] geodetic system
    npose = rospy.wait_for_message("/husky/current_pose", geometry_msgs.msg.PoseStamped)
    X0 = [npose.pose.position.x,npose.pose.position.y,npose.pose.position.z, 0]

    iteration = 1

    while True:
        print(iteration)

        # calculate horizontal distances
        s = list()

        for i in range(0,len(Hz_rad)):
 
            s.append(math.sqrt((X0[0]-Y_Fp[i])**2 + (X0[1]-X_Fp[i])**2))

        # calculate "Schraegstrecke"
        d = list()

        for i in range(0,len(V_rad)):

            d.append(math.sqrt((X0[0]-Y_Fp[i])**2 + (X0[1]-X_Fp[i])**2 + (X0[2]-Z_Fp[i])**2))


        # L0_Hz
        deltax = [x-X0[1] for x in X_Fp]

        deltay = [y-X0[0] for y in Y_Fp]
        
        pre_gon = list()


        for i in range(0,len(deltax)):

            pre_gon.append(math.atan2(deltax[i], deltay[i]))

            pre_gon[i] = ((pre_gon[i]*(-1))/math.pi)*200

            pre_gon[i] = pre_gon[i] + (X0[3]/math.pi)*200

        
        L0_Hz = list()

        for k in range(0,len(pre_gon)):

            if pre_gon[k] < 0:

                pre_gon[k] = pre_gon[k] +400;

            L0_Hz.append((pre_gon[k]/200)*math.pi )

            L0_Hz[k] = 2*math.pi - L0_Hz[k]


        l_Hz = list()

        for i in range(0,len(Hz_rad)):

            l_Hz.append(Hz_rad[i] - L0_Hz[i])

            while l_Hz[i] > math.pi:

                l_Hz[i] = l_Hz[i] -2*math.pi

            while l_Hz[i] < -math.pi:

                l_Hz[i] = l_Hz[i] +2*math.pi

        # L0_V
        L0_V = list()

        l_V = list()

        for i in range(0,len(V_rad)):

            L0_V.append(math.atan2(s[i], Z_Fp[i]-X0[2]))


            l_V.append(V_rad[i] - L0_V[i])


        # Hz + V
        L0 = [L0_Hz, L0_V]

        L = [Hz_rad, V_rad]

        l = np.concatenate([l_Hz, l_V])


        # Designmatrix A
        # Hz
        A_Hz_x = list()

        A_Hz_y = list()

        A_Hz_z = list()

        A_Hz_ou = list()

        for i in range(0,len(Hz_rad)):

            # derivation X
            A_Hz_x.append((X_Fp[i]-X0[1])/(s[i]**2))

            # derivation Y
            A_Hz_y.append(-1*(Y_Fp[i]-X0[0])/(s[i]**2))

            # derivation Z
            A_Hz_z.append(0)

            # derivation ou
            A_Hz_ou.append(-1)


        # V
        A_V_x = list()

        A_V_y = list()

        A_V_z = list()

        A_V_ou = list()

        for i in range(0,len(V_rad)):

            # derivation X
            A_V_x.append(-((Y_Fp[i]-X0[0])*(Z_Fp[i]-X0[2]))/(s[i]*d[i]**2))

            # derivation Y
            A_V_y.append(-((X_Fp[i]-X0[1])*(Z_Fp[i]-X0[2]))/(s[i]*d[i]**2))

            # derivation Z
            A_V_z.append(s[i]/d[i]**2)

            # derivation ou
            A_V_ou.append(0)

        # HZ + V
        A_Hz = np.transpose(np.array([A_Hz_x,A_Hz_y,A_Hz_z,A_Hz_ou]))

        A_V = np.transpose(np.array([A_V_x,A_V_y,A_V_z,A_V_ou]))

        A = np.concatenate((A_Hz, A_V))

        # Gauss-Markov-Modell
        P = np.linalg.inv(Qll)

        N = np.transpose(A)@P@A

        n = np.transpose(A)@P@l

        Qxx = np.linalg.inv(N)

        dx = Qxx@n
        print('dx')
        print(dx)
        X0 = X0 + dx

        iteration = iteration +1

        betragdx = np.absolute(dx)

        dxCheck = np.max(betragdx)

        # publish current pose when (dxCheck < Epsilon) == true
        if dxCheck < Epsilon:
            print(X0)
            cmd.header = std_msgs.msg.Header()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "/huskypose"

            quat = tf.transformations.quaternion_from_euler(0,0,X0[3])
            cmd.pose.position.x = X0[1]
            cmd.pose.position.y = X0[0]
            cmd.pose.position.z = X0[2]
            cmd.pose.orientation.x = quat[0]
            cmd.pose.orientation.y = quat[1]
            cmd.pose.orientation.z = quat[2]
            cmd.pose.orientation.w = quat[3]
            
            pub.publish(cmd)
            break


if __name__ == '__main__':
	try:
		pose_provider()
	except rospy.ROSInterruptException:
		pass
