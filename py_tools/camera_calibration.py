import cv2
import numpy as np

# 参考元
# http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html


if __name__ == "__main__":
    count = 0
    cap = cv2.VideoCapture(1)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    objp = np.zeros((6*9,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
    
    objpoints = []
    imgpoints = []

    while 1:
        status, frame = cap.read()

        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (9,6), None)

        if ret == True:
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            img = cv2.drawChessboardCorners(frame, (9,6), corners2,ret)
            cv2.imshow('camera',img)
            k = cv2.waitKey(10)
            if k == ord('a'):
                imgpoints.append(corners2)
                objpoints.append(objp)
                print("saved!")
            elif k == 27:
                print("calibration starts!")
                break

        else:
            cv2.imshow('camera',frame)
            k = cv2.waitKey(10)
            if k == 27:
                print("calibration starts!")
                break

    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    
    cv2.destroyAllWindows()

    print("image size = ", gray.shape)
    print("K = ")
    print(mtx)
    print("dist = ")
    print(dist)

    while 1:
        status, frame = cap.read()
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),0,(w,h))
        calibrated_image = cv2.undistort(gray, mtx, dist, None, newcameramtx)

        cv2.imshow("calibrated image", calibrated_image)
        k = cv2.waitKey(10)

        if k == 27:
            break

    np.savetxt('calibration/k_param.txt', newcameramtx)
    np.savetxt('calibration/d_param.txt', dist)

    cv2.destroyAllWindows()
    print("Calibration Params are saved!")