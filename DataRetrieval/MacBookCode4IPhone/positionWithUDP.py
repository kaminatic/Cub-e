import cv2
import numpy as np
import socket
import time

#python positionwithUDP.py

# === UDP CONFIG ===
UDP_IP = "192.168.1.138"   # Replace with receiving device IP
UDP_PORT = 9000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# === Camera calibration parameters ===
camera_matrix = np.array([
    [1.47523046e+03, 0.00000000e+00, 9.37894668e+02],
    [0.00000000e+00, 1.47986611e+03, 5.65464546e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
], dtype=np.float32)

dist_coeffs = np.array([
    -0.04744469, 0.05537846, 0.00451427, -0.00684932, -0.13298897
], dtype=np.float32)

# === ChArUco board configuration ===
squaresX, squaresY = 15, 11
square_length = 0.02
marker_length = 0.018
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
board = cv2.aruco.CharucoBoard((squaresX, squaresY), square_length, marker_length, dictionary)
detector = cv2.aruco.CharucoDetector(board)
board.setLegacyPattern(True)

# === Board center in 3D ===
center_x = (squaresX - 1) * square_length / 2
center_y = (squaresY - 1) * square_length / 2
board_center_3D = np.array([[center_x, center_y, 0.0]], dtype=np.float32)

# === Camera ===
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Press 'q' to quit.")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)

        if marker_corners is not None:
            cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)

        if charuco_corners is not None and charuco_ids is not None:
            cv2.aruco.drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids)

            retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                charuco_corners, charuco_ids, board, camera_matrix, dist_coeffs, None, None
            )

            if retval:
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

                center_2D, _ = cv2.projectPoints(board_center_3D, rvec, tvec, camera_matrix, dist_coeffs)
                center_coords = center_2D[0][0]
                center_point = (int(center_coords[0]), int(center_coords[1]))
                cv2.circle(frame, center_point, 10, (0, 255, 255), -1)

                R, _ = cv2.Rodrigues(rvec)
                adjusted_tvec = tvec + R @ board_center_3D.T
                camera_pos = -R.T @ adjusted_tvec
                cam_cm = camera_pos.ravel() * 100  # in cm

                text = f"Cam Pos (cm): X={cam_cm[0]:.1f}, Y={cam_cm[1]:.1f}, Z={cam_cm[2]:.1f}"
                text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                cv2.rectangle(frame, (15, 15), (15 + text_size[0] + 10, 50), (0, 0, 0), -1)
                cv2.putText(frame, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                # === SEND OVER UDP ===
                timestamp = time.time()  # UNIX timestamp (e.g. 1718723456.123)
                udp_msg = f"{timestamp:.3f},{cam_cm[0]:.2f},{cam_cm[1]:.2f},{cam_cm[2]:.2f}"
                sock.sendto(udp_msg.encode(), (UDP_IP, UDP_PORT))

        cv2.imshow('ChArUco Detection + Pose (center origin)', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Exiting...")

finally:
    cap.release()
    cv2.destroyAllWindows()
    sock.close()
