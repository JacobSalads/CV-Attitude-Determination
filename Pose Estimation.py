#Live Camera Attititude Determination Script

import cv2
import numpy as np
from cv2 import aruco
from scipy.spatial.transform import Rotation as Rot


#Camera matrix and distortion matrix are values produced from camera calibration script.
camera_matrix = np.array([[663.5762908334771, 0.0, 339.2802880890685], [0.0, 659.8421269970602, 231.61401863896123],
                          [0.0, 0.0, 1.0]])
distortion_matrix = np.array([[0.03671346889193375, -0.6360889979245963, -0.004661529664190291,
                               0.01555645833474018, 1.447028199952007]])

#Below defines the marker being referenced
aruco_dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
aruco_parameters = aruco.DetectorParameters()

cap = cv2.VideoCapture(0)

aruco_ids = []
aruco_corners = []

while True:
    ret, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    _, thresholded = cv2.threshold(gray, 50, 255, cv2. THRESH_BINARY + cv2.THRESH_OTSU)

    corners, ids, rejected = aruco.detectMarkers(thresholded, aruco_dictionary, parameters=aruco_parameters)

    if ids is not None and len(ids) > 0:
        for i in range(len(ids)):
            aruco_id = ids[i][0]
            aruco_ids.append(aruco_id)
            aruco_corners.append(corners[i][0])

            aruco.drawDetectedMarkers(frame, corners)

            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 1.0, camera_matrix, distortion_matrix)

            #print(f"Marker ID: {aruco_id}")
            print(f"Translation Vector (tvec): {tvec}")
            #print(f"Rotation Vector (rvec): {rvec}")
            #print("-" * 20)

            #rvec_array = np.array(rvec)
            #rvec_array_1x3 = np.array(rvec).reshape(1, 3)
            #rvec_yz = [rvec_array_1x3[0, 1], rvec_array_1x3[0, 2]]
            #rvec_xz = [rvec_array_1x3[0, 0], rvec_array_1x3[0, 2]]
            #rvec_xy = [rvec_array_1x3[0, 0], rvec_array_1x3[0, 1]]

            #global_xy = [0,0,1]
            #global_xz = [0,1,0]
            #global_yz = [1,0,0]

            try:
                # Inserting rotation vector into the Rodrigues rotation formula
                R = cv2.Rodrigues(rvec)[0]

                try:
                    # Convert rotation vector to rotation matrix
                    R_matrix, _ = cv2.Rodrigues(rvec)

                    # Convert rotation matrix to quaternion
                    quaternion = Rot.from_matrix(R_matrix).as_quat()

                    ## Uncomment below to view quaternion values
                    #print(f"Quaternion: {quaternion}")

                    # You can convert quaternion to Euler angles for inspection if necessary
                    euler = Rot.from_quat(quaternion).as_euler('xyz', degrees=True)
                    #print(f"Euler Angles (degrees): Roll={euler[0]:.2f}, Pitch={euler[1]:.2f}, Yaw={euler[2]:.2f}")

                except cv2.error:
                    continue


            except cv2.error:
                continue
            #Edit values below to change the position of axes displayed on the aruco marker
            axis_points = np.float32([[0,0,0], [1,0,0], [0,1,0], [0,0,-1]]).reshape(-1,3)
            image_points, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, distortion_matrix)

            frame = cv2.line(frame, tuple(image_points[0].ravel().astype(int)),
                             tuple(image_points[1].ravel().astype(int)), (0,0,255), 5)
            frame = cv2.line(frame, tuple(image_points[0].ravel().astype(int)),
                             tuple(image_points[2].ravel().astype(int)), (0,255,0), 5)
            frame = cv2.line(frame, tuple(image_points[0].ravel().astype(int)),
                             tuple(image_points[3].ravel().astype(int)), (255,0,0), 5)

    combined_frame = np.hstack((frame, cv2.cvtColor(thresholded, cv2.COLOR_GRAY2BGR)))
    cv2.imshow('Aruco Marker Pose Esitamation', combined_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()