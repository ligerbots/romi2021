import cv2
import json
import numpy as np
import glob
from timeit import default_timer as timer
import math
with open("vision/calib.json") as f:
    data = json.load(f)
    camera_matrix = np.array(data["camera_matrix"])
    distortion_matrix = np.array(data["distortion"])

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters_create()

side_length = 6.4 #in
grid_length = 15
world_position = np.array([
    [side_length/2, 0, -side_length/2],
    [side_length/2, 0, side_length/2],
    [-side_length/2, 0, side_length/2],
    [-side_length/2, 0, -side_length/2],
])

ceil_cam_height = 103

scale_x = 1/ceil_cam_height* camera_matrix[0,0]
scale_y = 1/ceil_cam_height* camera_matrix[1,1]
print(scale_x)
real_positions = []
for x in reversed(range(6)):
    for y in reversed(range(3)):
        real_positions.append(world_position+[x*grid_length,0,y*grid_length])

def process_image(frame):
    frame_undistort = cv2.undistort(frame, camera_matrix, distortion_matrix, None, camera_matrix)
    corners, ids, rejected = cv2.aruco.detectMarkers(frame_undistort, arucoDict, parameters=arucoParams)
    if(ids is None or len(ids)<=4):
        return(None)
    img_pos = []
    world_pos = []
    for (markerCorner, [markerID]) in zip(corners, ids):
        corners = markerCorner.reshape((4, 2))
        img_pos.extend(corners)
        world_pos.extend(real_positions[markerID])
    world_pos = np.array(world_pos)
    img_pos = np.array(img_pos)

    ret, rvec, tvec = cv2.solvePnP(world_pos, img_pos, camera_matrix, None)
    rvec = np.squeeze(rvec)
    tvec = np.squeeze(tvec)
    rot, _ = cv2.Rodrigues(rvec)
    rot_inv = rot.transpose()
    dirvec = np.array([1,0,0]) @ rot
    dir = math.degrees(math.atan2(dirvec[2],dirvec[0]))
    pzero_world = np.matmul(rot_inv, -tvec)

    imagePoints, _ = cv2.projectPoints(world_pos, rvec, tvec, camera_matrix, None)
    for [pos] in imagePoints:
        cv2.circle(frame_undistort, tuple(pos.astype(np.int)), radius=2, color=(0, 0, 255), thickness=2)
    for pos in img_pos:
        cv2.circle(frame_undistort, tuple(pos.astype(np.int)), radius=2, color=(255, 0, 0), thickness=2)
    return((frame_undistort, np.concatenate([pzero_world,[dir]])))

center = np.array([660,355])

def process_image_2d(frame):
    def find_rigid_transform(x, y):
        # https://scicomp.stackexchange.com/a/6901
        assert len(x) == len(y)
        x_center = np.mean(x, axis=0)
        y_center = np.mean(y, axis=0)
        A = x - x_center
        B = y - y_center
        C = B.T @ A
        U, S_diag, V_T = np.linalg.svd(C)
        R = U @ V_T
        if(np.linalg.det(R) < 0):
            R = U @ np.diag([1, -1]) @ V_T
        d = y_center - R @ x_center
        return R, d

    corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
    if(ids is None or len(ids)<=4):
        return((None, None))
    img_pos = []
    world_pos = []
    for (markerCorner, [markerID]) in zip(corners, ids):
        corners = markerCorner.reshape((4,1, 2))
        corners_undistort = cv2.undistortPoints(corners, camera_matrix, distortion_matrix, P=camera_matrix).reshape((-1, 2))
        for i in range(len(corners_undistort)):
            if(np.linalg.norm(corners_undistort[i]-center)>250):continue
            img_pos.append(corners_undistort[i])

            world_pos.append(real_positions[markerID][i]*[scale_x,0,scale_y])

    world_pos = np.array(world_pos)[:,[True,False,True]]
    img_pos = np.array(img_pos)
    R, d = find_rigid_transform(world_pos, img_pos)
    transformed_world_pos = world_pos @ R.T + d
    for pos in img_pos:
        cv2.circle(frame, tuple(pos.astype(np.int)), radius=2, color=(255, 0, 0), thickness=2)
    for pos in transformed_world_pos:
        cv2.circle(frame, tuple(pos.astype(np.int)), radius=2, color=(0, 0, 255), thickness=2)
    dir = [0,1]@R.T
    cv2.circle(frame, tuple(center.astype(np.int)), radius=2, color=(0, 0, 255), thickness=2)

    pos = (center-d)@R/scale_x
    return((frame, [pos[0],0,pos[1],math.degrees(math.atan2(dir[1],dir[0]))]))
