from IO.camera.gazebo_video_stream import Video

import cv2
import math
import numpy as np
import pickle
from cv2 import aruco, drawFrameAxes

CALIBRATION_PICKLE_PATH = "/home/harsh/Desktop/precisionLanding/precision-landing/solo_calib10_best_1.01"
#CALIBRATION_PICKLE_PATH = "/home/bassam/projects/supreme-octo-parakeet/solo_calib"


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
        #calib_params["dist"] *= 0
        #camera_matrix = np.array([[1.0, 0.0, 1.05], [0.0, 1.0, 4.0], [0.0, 0.0, 1.0]])
        self.camera_matrix = calib_params["mtx"]
        self.distortion_coeficients = calib_params["dist"] 
        self.aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        markerlength = 12
        
        array_of_ids = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])

        coordinate_array = [[[-52.681736188014526, 13.48521061411248, 0.0], [19.871981540958718, 13.48521061411248, 0.0], [19.871981540958718, 86.03892834308573, 0.0], [-52.681736188014526, 86.03892834308573, 0.0]], [[-8.266306371032414, 11.733410588463228, 0.0], [-52.57224868641145, 11.733410588463228, 0.0], [-52.57224868641145, -32.572531726915805, 0.0], [-8.266306371032414, -32.572531726915805, 0.0]], [[-6.806473016324703, -32.572531726915805, 0.0], [20.528906550577187, -32.572531726915805, 0.0], [20.528906550577187, -5.237152160013913, 0.0], [-6.806473016324703, -5.237152160013913, 0.0]], [[3.8138146391738954, -4.361252147189287, 0.0], [20.30993154737103, -4.361252147189287, 0.0], [20.30993154737103, 12.134864761007849, 0.0], [3.8138146391738954, 12.134864761007849, 0.0]], [[-6.915960517927782, 2.390477118333877, 0.0], [2.864922958613883, 2.4269729522015697, 0.0], [2.864922958613883, 12.17136059487554, 0.0], [-6.915960517927782, 12.17136059487554, 0.0]], [[-7.0254480195308595, -4.2152688117185155, 0.0], [-1.2591062684354009, -4.2152688117185155, 0.0], [-1.2591062684354009, 1.5510729393769431, 0.0], [-7.0254480195308595, 1.5510729393769431, 0.0]], [[2.901418792481576, -1.1131229329646297, 0.0], [-0.34671042174308137, -1.1131229329646297, 0.0], [-0.34671042174308137, -4.361252147189287, 0.0], [2.901418792481576, -4.361252147189287, 0.0]], [[0.8211562620230874, -0.4926937572138525, 0.0], [3.193385463423118, -0.4926937572138525, 0.0], [3.193385463423118, 1.879535444186178, 0.0], [0.8211562620230874, 1.879535444186178, 0.0]], [[-0.565685424949238, 0.7846604281553947, 0.0], [0.5291895910815453, 0.7846604281553947, 0.0], [0.5291895910815453, 1.879535444186178, 0.0], [-0.565685424949238, 1.879535444186178, 0.0]], [[-0.565685424949238, -0.565685424949238, 0.0], [0.565685424949238, -0.565685424949238, 0.0], [0.565685424949238, 0.565685424949238, 0.0], [-0.565685424949238, 0.565685424949238, 0.0]]]


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

                self._outqueue.put([rvectvec_to_euler(rvec,tvec), tvec])
                frame = drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.distortion_coeficients,
                    rvec,
                    tvec,
                    25,
                )
            # result.write(frame)
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) == ord("q"):
                break
        cv2.destoyAllWindows()


if __name__ == "__main__":
    aruco_reader = ArucoReader(None)
    aruco_reader.start_reading()
