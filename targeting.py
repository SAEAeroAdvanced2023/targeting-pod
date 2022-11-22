import numpy as np
from Transformation_matrices import Transformation_translation, camera_calibration_matrix, Transformation_rotation, isect_line_plane_v3
import matplotlib.pyplot as plt


def coor_camera_to_inertial_frame (v_dist,roll,yaw,pitch,g_roll, g_yaw, g_pitch,CCM,CCM_inv,pix_x,pix_y,g_dist,c_dist,f,gnd,plots):

# matrix used to find camera location coordinates
    C = np.array([[0], [0], [0], [1]])

# camera calibration
    if CCM_inv is None:
        #generate Camera calibration matrix from set of pictures
        CCM = camera_calibration_matrix()
        CCM_inv = np.linalg.inv(CCM)

# adding column and row to be able to multiply CCM_inv with other matrices
        column_to_be_added = np.array([[0], [0], [0]])
        newrow = np.array([0, 0, 0, 1])
        CCM_inv = np.append(CCM_inv, column_to_be_added, axis=1)
        CCM_inv = np.vstack([CCM_inv, newrow])
    else:
        CCM_inv = CCM_inv

# target coord in camera
    pix = np.array([[pix_x], [pix_y], [1], [1]])


# the following matrices are used to find the location of the camera and the object in the sensor frame all wrt the inertial frame (extrinsic matrices)

# roll matrix (90 deg rotation around z axis) used to rotate the sensor frame pixel axis
    cc = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
# pitch matrix (90 deg rotation around y axis) used to go from cam
    cam = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])
# depth matrix for depth of object in the sensor frame wrt to camera frame
    im = np.array([[f, 0, 0, 0], [0, f, 0, 0], [0, 0, f, 0], [0, 0, 0, 1]])
# rotation & translation from inertial frame to camera frame
    trans_c = Transformation_translation(c_dist[0], c_dist[1], c_dist[2])
    rot_g = Transformation_rotation(g_yaw, g_pitch, g_roll)
    trans_g = Transformation_translation(g_dist[0], g_dist[1], g_dist[2])
    rot_v = Transformation_rotation(yaw,pitch,roll)
    trans_i = Transformation_translation(v_dist[0], v_dist[1], v_dist[2])

# camera coordinates wrt inertial frame
    P_cc = np.linalg.multi_dot([np.linalg.inv(trans_i[0]),rot_v, np.linalg.inv(trans_g[0]),rot_g, np.linalg.inv(trans_c[0]),C])
# coordinates of object seen in the camera sensor frame wrt inertial frame
    q_obj = np.linalg.multi_dot([np.linalg.inv(trans_i[0]),rot_v, np.linalg.inv(trans_g[0]),rot_g, np.linalg.inv(trans_c[0]), cam, im, cc,CCM_inv,pix])
# intersection point between line formed with the P_cc & q_obj and plane(ground), this also represents the location of the target wrt to inertial frame
    t = isect_line_plane_v3(P_cc[:3],q_obj[:3],gnd[0],gnd[1])

# validation method

# distance between camera and target (shortest distance)
    t_norm = np.linalg.norm((t-P_cc[:3]),2)
    r = np.sqrt(np.square(pix[0][0]-CCM[0][2])+np.square(pix[1][0]-CCM[1][2]))
# angle between depth and line to target from center of optical lens (camera)
    Beta = np.arctan((r/((CCM[0][0]+CCM[1][1])/2)))
# depth distance
    l = t_norm*np.cos(Beta)
# depth matrix
    Q =np.array([[l,0,0,0],[0,l,0,0],[0,0,l,0],[0,0,0,1]])
# combination of all transfomation matrices
    T = np.linalg.multi_dot([np.linalg.inv(trans_i[0]),rot_v, np.linalg.inv(trans_g[0]),rot_g, np.linalg.inv(trans_c[0]), cam, Q, cc,CCM_inv,])
# location of the target wrt to inertial frame
    inertial_frame_coord =np.linalg.multi_dot([T,pix])

# results

# 1 line plane intersection method
    latitude = t[0]
    longitude = t[1]
    height = t[2]

# 2 Validation
    latitude1 = inertial_frame_coord[0]
    longitude1 = inertial_frame_coord[1]
    height1 = inertial_frame_coord[2]

# error
    error_lat =  np.abs((latitude1-latitude)/latitude1)*100
    error_long =  np.abs((longitude1-longitude)/longitude1)*100
    height_diff =  np.abs((height1-height))
    total_error = np.average([error_lat,error_long])

    if total_error<= 1:
        print('very accurate result')
    elif total_error<= 5 and total_error> 5:
        print('very accurate result')
    else:
        print('very accurate result')

    #add estimate angle error
    #loop
    #add weighted average based on camera angle and position of object in sensor frame(pix_x and pix_y)

# plots

    if (plots):
        in2body= np.dot(np.linalg.inv(np.linalg.multi_dot([rot_v,trans_i[0]])),C)
        body2gim = np.dot(np.linalg.inv(np.linalg.multi_dot([rot_g,trans_g[0],rot_v,trans_i[0]])),C)
        gim2cam = np.dot(np.linalg.inv(np.linalg.multi_dot([trans_c[0],rot_g,trans_g[0],rot_v,trans_i[0]])),C)
        fig = plt.figure()
        ax = plt.axes(projection = '3d')
        ax.set_xlim([-10,30])
        ax.set_ylim([-20,20])
        ax.set_zlim([-30,0])
        b_g = [body2gim[0][0]-in2body[0][0] ,
                  body2gim[1][0]-in2body[1][0], body2gim[2][0]-in2body[2][0]]

        g_c = [-(b_g[0]+in2body[0][0])+gim2cam[0][0], -(b_g[1]+in2body[1][0])+gim2cam[1][0], -(b_g[2]+in2body[2][0])+gim2cam[2][0]]
        c_t = [t[0]-(in2body[0][0]+b_g[0]+g_c[0]), t[1]-(in2body[1][0]+b_g[1]+g_c[1]), t[2]-(in2body[2][0]+b_g[2]+g_c[2])]
        start = [0,0,0]
        ax.quiver(start[0],start[1],start[2],in2body[0][0],in2body[1][0],in2body[2][0]) #inertial to body
        ax.quiver(in2body[0][0], in2body[1][0], in2body[2][0], b_g[0], b_g[1], b_g[2], color='y') #body to gimbal
        ax.quiver(b_g[0]+in2body[0][0], b_g[1]+in2body[1][0], b_g[2]+in2body[2][0],g_c[0],g_c[1],g_c[2], color = 'g') #gimbal to cam
        ax.quiver(b_g[0]+in2body[0][0]+g_c[0], b_g[1]+in2body[1][0]+g_c[1], b_g[2]+in2body[2][0]+g_c[2],c_t[0],c_t[1],c_t[2], color = 'r') #cam to target

    return latitude1,longitude1,height1,total_error
if __name__ == '__main__':
# camera calibration matrices
    CCM_inv = np.array([[0.00154274,0.,-0.51571205,0.], [0.,0.0015459,-0.40726403,0.], [ 0.,0.,1.,0.], [ 0.,0.,0.,1.]])
    CCM = np.array([[648.19832304,0.,334.28368528], [0.,646.87336044,263.44824863], [0.,0.,1.]])
# pixels seen in sensor frame
    pix_x = 0
    pix_y = 0
# vehicule distance from inertial point (GPS coordinates)
    v_dist = np.array([0, 0, -210])
# coordinates of gimbal wtr to centroid of the aircraft
    g_dist = np.array([0, 0, 0])
# distance of camera center vision from center of rotation of gimbal
    c_dist = np.array([0,0,0])
# yaw, pitch, roll = aircraft rotation
    yaw=0
    pitch=-np.pi/5
    roll=np.pi/2

#g_yaw, g_pitch, g_roll = gimbal's rotation
    g_yaw=0
    g_pitch=0
    g_roll=0

# focal length of camera in meters
    f = 0.035
# ground definition ( point & normal vector)
    gnd = np.array([[1,1,0],[0,0,1]])


#follows right hand rule convention
#follow convention NED (North East Down)
#angles in radiants

    coor_camera_to_inertial_frame (v_dist=v_dist, yaw=yaw, pitch=pitch, roll=roll, g_yaw=g_yaw, g_pitch=g_pitch, g_roll=g_roll, CCM=CCM, CCM_inv=CCM_inv, pix_x=pix_x, pix_y=pix_y, g_dist=g_dist, c_dist=c_dist, f=f, gnd=gnd, plots=False)




