import cv2

# Raspberry Pi Camera Setup
camera = cv2.VideoCapture(0)
camera.set(3, 640)  # Set width
camera.set(4, 480)  # Set height

num = 0

while camera.isOpened():
    success, img = camera.read()

    if not success:
        break

    cv2.imshow('Img', img)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # Press 'Esc' to exit
        break
    elif key == ord('s'):  # Press 's' to save image
        filename = 'calibration_image' + str(num) + '.png'
        cv2.imwrite(filename, img)
        print(f"Image saved: {filename}")
        num += 1

# Release resources
camera.release()
cv2.destroyAllWindows()
