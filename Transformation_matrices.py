import numpy as np
#import pandas as pd
import copy
import os
import math



def digit_formatting(number,num_of_digits):
    '''
    :param number:
    :param num_of_digits:
    :return: number with correct number of digits
    '''

    a = str(number)
    deci = a[::-1].find('.')
    if deci<0:
        deci = 0
    num = a.replace('.', '')
    length = len(num)

    if (length<num_of_digits):
        decimal = '.' + str(deci +(num_of_digits-length)) + 'f'
        first = format(number, decimal)
    else:
        first = str(round(number, abs((deci+num_of_digits)-length)))

    return first

# Function to convert list to string
def listToString(s):
    # initialize an empty string
    str1 = ""

    # traverse in the string
    for ele in s:
        str1 += ele

        # return string
    return str1

#function allows to chnage all extension files of a directory
def change_ext(directory,init,out):
    #directory : working directory (string)
    #init : initial extension (string)
    #out : output extension (string)




    files = copy.deepcopy(os.listdir(directory))
    os.chdir(directory)  # changes current directory to working directory
    for file in files:
        if file.endswith(init):
            if out == None:
                output = file[:-len(init)]
            else:
                output = out

            os.system('move' + ' ' + file + ' ' + output)


            print(os.path.join("/mydir", file))

#this file contains all transformation matricies for the geo-location of the target

def Transformation_translation(x, y, z):
    T_translation = np.array([[1,0,0,-x],[0,1,0,-y],[0,0,1,-z],[0,0,0,1]])
    T_translation_inverse = np.linalg.inv(T_translation)
    return T_translation, T_translation_inverse

def Transformation_rotation(a,b,c):


    T_rotation = np.array([[math.cos(a)*math.cos(b), math.cos(a)*math.sin(b)*math.sin(c)-math.sin(a)*math.cos(c), math.cos(a)*math.sin(b)*math.cos(c)+math.sin(a)*math.sin(c), 0],[math.sin(a)*math.cos(b), math.sin(a)*math.sin(b)*math.sin(c)+math.cos(a)*math.cos(c), math.sin(a)*math.sin(b)*math.cos(c)-math.cos(a)*math.sin(c), 0],[-math.sin(b), math.cos(b)*math.sin(c), math.cos(b)*math.cos(c), 0],[0, 0, 0, 1]])
    return T_rotation

def Transformation_rotation_2(t,phi,psi):
    T_rotation2 = np.array([[math.cos(t)*math.cos(psi),math.cos(t)*math.sin(psi),-math.sin(t),0],[math.sin(phi)*math.sin(t)*math.cos(psi)-math.cos(phi)*math.sin(psi),math.sin(phi)*math.sin(t)*math.sin(psi)+math.cos(phi)*math.cos(psi),math.sin(phi)*math.cos(t),0],[math.cos(phi)*math.sin(t)*math.cos(psi)+math.sin(phi)*math.sin(psi),math.cos(phi)*math.sin(t)*math.sin(psi)-math.sin(phi)*math.cos(psi), math.cos(phi)*math.cos(t),0],[0,0,0,1]])
    return T_rotation2
def Pix2feet(n_pix):
    n_feet = 0.00086805544619423*n_pix

    return n_feet

def gimbal_rot(el,az):
    rot = np.array([[math.cos(el)*math.cos(az), math.cos(el)*math.sin(az), math.sin(el),0],[-math.sin(el),math.cos(az),0,0],[-math.sin(el)*math.cos(az),-math.sin(el)*math.sin(az),math.cos(el),0],[0,0,0,1]])
    return rot

def camera_calibration_matrix(foc_x,foc_y,f_theta,c_x,c_y):
    #foc_x = width pixel
    #foc_y = height pixel
    #f_theta = skiew (shear)
    #cx and cy principal points
    #if analog camera shear angle not equal 0, digitial shear angle close to
    CCM = np.array([[foc_x,f_theta,c_x],[0,foc_y,c_y],[0,0,1]])
    CCM_inv = np.linalg.inv(CCM)
    # Array to be added as column
    column_to_be_added = np.array([[0], [0], [0]])

    # Adding column to array using append() method
    A= np.append(CCM_inv, column_to_be_added, axis=1)
    newrow = [0, 0, 0, 1]
    A = np.vstack([A, newrow])
    return A
   # https: // www.mathworks.com / help / vision / ref / cameracalibrator - app.html
   # https: // www.youtube.com / watch?v = nRVuLFQ_Bng & ab_channel = CyrillStachniss
def camera_calibration_matrix_2():
    #https: // www.analyticsvidhya.com / blog / 2021 / 10 / a - comprehensive - guide -for -camera - calibration - in -computer - vision /  #:~:text=The%20camera%20matrix%20is%20a,algorithm%20computes%20the%20camera%20matrix.

    import cv2
    import numpy as np
    import os
    import glob
    # Defining the dimensions of checkerboard
    CHECKERBOARD = (7, 10)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # Creating vector to store vectors of 3D points for each checkerboard image
    objpoints = []
    # Creating vector to store vectors of 2D points for each checkerboard image
    imgpoints = []
    # Defining the world coordinates for 3D points
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    prev_img_shape = None
    # Extracting path of individual image stored in a given directory
    source_path =  r'C:\Users\melzo\OneDrive\Documents\GitHub\targeting-math-sim\images'
    images = [f for f in glob.glob('images2/*.jpg')]
    #images = glob.glob('./images/*.jpg')


    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        # If desired number of corners are found in the image then ret = true
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        """
        If desired number of corner are detected,
        we refine the pixel coordinates and display
        them on the images of checker board
        """
        if ret == True:
            objpoints.append(objp)
        # refining pixel coordinates for given 2d points.
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        #img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        #cv2.imshow('img', img)
        #cv2.waitKey(0)
    cv2.destroyAllWindows()
    h, w = img.shape[:2]


    """
    Performing camera calibration by
    passing the value of known 3D points (objpoints)
    and corresponding pixel coordinates of the
    detected corners (imgpoints)
    """

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print("Camera matrix : n")
    print(mtx)
    print("dist : n")
    print(dist)
    print("rvecs : n")
    print(rvecs)
    print("tvecs : n")
    print(tvecs)

    return mtx


# intersection function
def isect_line_plane_v3(p0, p1, p_co, p_no, epsilon=1e-6):
    """
    p0, p1: Define the line.
    p_co, p_no: define the plane:
        p_co Is a point on the plane (plane coordinate).
        p_no Is a normal vector defining the plane direction;
             (does not need to be normalized).

    Return a Vector or None (when the intersection can't be found).
    """

    u = sub_v3v3(p1, p0)
    dot = dot_v3v3(p_no, u)

    if abs(dot) > epsilon:
        # The factor of the point between p0 -> p1 (0 - 1)
        # if 'fac' is between (0 - 1) the point intersects with the segment.
        # Otherwise:
        #  < 0.0: behind p0.
        #  > 1.0: infront of p1.
        w = sub_v3v3(p0, p_co)
        fac = -dot_v3v3(p_no, w) / dot
        u = mul_v3_fl(u, fac)
        return add_v3v3(p0, u)

    # The segment is parallel to plane.
    return None

# generic math functions

def add_v3v3(v0, v1):
    return (
        v0[0] + v1[0],
        v0[1] + v1[1],
        v0[2] + v1[2],
    )


def sub_v3v3(v0, v1):
    return (
        v0[0] - v1[0],
        v0[1] - v1[1],
        v0[2] - v1[2],
    )


def dot_v3v3(v0, v1):
    return (
        (v0[0] * v1[0]) +
        (v0[1] * v1[1]) +
        (v0[2] * v1[2])
    )


def len_squared_v3(v0):
    return dot_v3v3(v0, v0)


def mul_v3_fl(v0, f):
    return (
        v0[0] * f,
        v0[1] * f,
        v0[2] * f,
    )