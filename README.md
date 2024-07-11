# CV-Attitude-Determination
A Python CV Script used to determine the attitude of an object through the use of ArUco markers. Produces a quaternion and converts into an Euler angle sequence, displaying orientation in terms of yaw, pitch and roll. 

The following versions is what I used when creating this script:

Python 3.11

numpy 1.26.6

opencv-contrib-python 4.9.0.80

opencv-python 4.9.0.80

PyYAML 6.0.1


Steps taken to use main script:

1. Use the "pi_calibration" script to take images with your chosen camera (Named pi_calibration due to original use being for a raspberry pi camera).
2. Use script "Camera_Calibration_Matrix" to call images taken in step 1 for the creation of a yaml file containing the camera distortion matrix/coefficients.
3. Import camera distortion matrix/coefficients into the script "Pose Estimation" and click run.
