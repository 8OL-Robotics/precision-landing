from IO.camera.gazebo_video_stream import Video

import cv2
import math
import numpy as np
import pickle
from cv2 import aruco, drawFrameAxes

CALIBRATION_PICKLE_PATH = "/home/bassam/aruco-board-detection-main/rpi_bassam_f"


def rvectvec_to_euler(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    y = math.atan2(R[1][0], R[0][0])
    p = math.atan2(-R[2][0], math.sqrt(R[2][1] ** 2 + R[2][2] ** 2))
    r = math.atan2(R[2, 1], R[2, 2])
    return [r * (180 / math.pi), p * (180 / math.pi), y * (180 / math.pi)]
    # rmat,_ = cv2.Rodrigues(rvec)
    # P = np.hstack((rmat,tvec.T))
    # return cv2.decomposeProjectionMatrix(P)[6]


class ArucoReader:
    def __init__(self, q, mode="simulation") -> None:

        if mode == "simulation":
            self._video_stream = Video()

        self._outqueue = q
        calib_params = pickle.load(open(CALIBRATION_PICKLE_PATH, "rb"))
        self.camera_matrix = calib_params["mtx"]
        self.distortion_coeficients = calib_params["dist"]
        calib_params["dist"] *= 0
        camera_matrix = np.array([[1.0, 0.0, 1.05], [0.0, 1.0, 4.0], [0.0, 0.0, 1.0]])

        ## board init

        self.aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        markerlength = 12

        array_of_ids = np.asarray([1, 9, 2, 15])
        """
        coordinate_array = [
            [
                [-34.9, 10.2, 0.0],
                [-14.9, 10.2, 0.0],
                [-14.9, -10.2, 0.0],
                [-34.9, -10.2, 0.0],
            ],
            [
                [16.25, 19.8, 0.0],
                [34.25, 19.8, 0.0],
                [34.25, 1.8, 0.0],
                [16.25, 1.8, 0.0],
            ],
            [[-3.3, 20.6, 0.0], [7.7, 20.7, 0.0], [8, 9.7, 0.0], [-3.1, 9.4, 0.0]],
            [[0.6, 6.55, 0.0], [3.2, 6.55, 0.0], [3.2, 3.85, 0.0], [0.6, 3.85, 0.0]],
        ]
        """
        coordinate_array = [[[9.6, -74.6, 0.0], [9.6, -34.0, 0.0], [-31.0, -34.0, 0.0], [-31.0, -74.6, 0.0]], [[29.9, -9.9, 0.0], [29.9, 11.1, 0.0], [8.9, 11.1, 0.0], [8.9, -9.9, 0.0]], [[30.0, 29.4, 0.0], [30.0, 66.0, 0.0], [-6.6, 66.0, 0.0], [-6.6, 29.4, 0.0]], [[2.4, -2.4, 0.0], [2.4, 2.6, 0.0], [-2.6, 2.6, 0.0], [-2.6, -2.4, 0.0]]]
        
        print(coordinate_array)
        coordinate_array = np.asarray(coordinate_array).astype(np.float32)
        self.board = cv2.aruco.Board_create(
            coordinate_array, self.aruco_dictionary, array_of_ids
        )

    def start_reading(self):

        ## generation ofa aruco board
        # fourcc = cv2.VideoWriter_fourcc(*'XVID')
        # result = cv2.VideoWriter('output.avi',fourcc, 20.0, (2160,3840))
        # result =  cv2.VideoWriter('02_proc.MOV',cv2.VideoWriter_fourcc(*'MOV'),30,(2160,3840))

        ## later, from camera scanning acquire rvec and tvec in each iteration

        print("AR: Started Aruco Reader")

        while True:

            if not self._video_stream.frame_available():
                continue

            frame = self._video_stream.frame()

            corners, ids, _ = aruco.detectMarkers(frame, self.aruco_dictionary)
            frame = aruco.drawDetectedMarkers(frame, corners, ids, (0, 255, 0))
            retval, rvec, tvec = cv2.aruco.estimatePoseBoard(
                corners,
                ids,
                self.board,
                self.camera_matrix,
                self.distortion_coeficients,
                np.asarray([]),
                np.asarray([]),
            )

            # print(retval)
            if rvec is None:
                rvec, tvec = [], []
            # print(tvec)
            if retval:
                
                #print(f"AR: Received pose estimate {rvec,tvec}")

                self._outqueue.put([rvec, tvec])
                frame = drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.distortion_coeficients,
                    rvec,
                    tvec,
                    25,
                )
            # result.write(frame)
            cv2.imshow("hello", frame)
            if cv2.waitKey(1) == ord("q"):
                break
        cv2.destoyAllWindows()


if __name__ == "__main__":
    aruco_reader = ArucoReader(None)
    aruco_reader.start_reading()
