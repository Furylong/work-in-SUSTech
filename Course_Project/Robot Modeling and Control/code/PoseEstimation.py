import os

import cv2
import mediapipe as mp
import time

import numpy as np


# functions which calculate angles
def calculate_angle(a, b, c):
    a = np.array(a)
    b = np.array(c)
    c = np.array(c)

    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])

    angle = np.abs(radians * 180.0 / np.pi)

    if angle > 180.0:
        angle = 360 - angle
    return angle


def print_coor2csv(lmList, count):
    cur_dir = os.getcwd()
    os.chdir("output")
    temp = "output_" + str(count) + "_of_joint_" + str(lmList[0]) + ".csv"
    with open(temp, "w") as f:
        f.write(str(lmList[0]) + "," + str(lmList[1]) + "," + str(lmList[2]) + "," + str(lmList[3]))
    os.chdir(cur_dir)


def print_angle2tcsv(angle_elbow, angle_shoulder, angle_wrist, count):
    cur_dir = os.getcwd()
    os.chdir("output")
    temp = "output_" + str(count) + ".csv"
    with open(temp, "w") as f:
        f.write(str(angle_elbow) + "," + str(angle_shoulder) + "," + str(angle_wrist))
    os.chdir(cur_dir)


class poseDetector():
    def __init__(self, mode=False, upBody=False, smooth=True,
                 detectionCon=0.5, trackCon=0.5):

        self.mode = mode
        self.upBody = upBody
        self.smooth = smooth
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpDraw = mp.solutions.drawing_utils
        # self.mpPose = mp.solutions.pose
        # self.pose = self.mpPose.Pose(self.mode, self.upBody, self.smooth,
        self.pose = mp.solutions.pose.Pose(static_image_mode=self.mode, smooth_landmarks=self.smooth,
                                           min_detection_confidence=self.detectionCon,
                                           min_tracking_confidence=self.trackCon)

    def findPose(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.pose.process(imgRGB)
        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(img, self.results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS)
        return img

    def findPosition(self, img, draw=True):
        lmList = []
        if self.results.pose_landmarks:
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                h, w, c = img.shape
                # print(id, lm)
                cx, cy = int(lm.x * w), int(lm.y * h)
                if id is 16:
                    print(lm.x, lm.y, lm.z)
                lmList.append([id, lm.x, lm.y, lm.z])  # 输出的数据是ID，cx，cy, cz的list
                # if draw:
                #     cv2.circle(img, (cx, cy), 5, (255, 0, 0), cv2.FILLED)
        return lmList


def main():
    # 视频识别
    cap = cv2.VideoCapture("sources/test7.mp4")  # 需要输入一个视频的绝对路径或相对路径
    pTime = 0

    mp_drawing = mp.solutions.drawing_utils
    mp_pose = mp.solutions.pose

    detector = poseDetector()
    count = 0
    with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        while True:

            success, img = cap.read()
            if img is None:
                print("The end of the Video!")
                break
            img = detector.findPose(img)  # 寻找POSE 返回到img
            results = pose.process(img)
            # Extract landmarks
            if results.pose_landmarks is None:
                continue
                cv2.waitKey(1)
            landmarks = results.pose_landmarks.landmark  # 保存不同的landmarks
            lmList = detector.findPosition(img)  # 打印需要的节点，img后面可加draw=False来标定目标节点
            print_coor2csv(lmList[16], count)  # 在lmListp[]中指定关节
            print_coor2csv(lmList[14], count)
            print_coor2csv(lmList[12], count)
            print_coor2csv(lmList[18], count)
            print_coor2csv(lmList[24], count)
            print_coor2csv(lmList[22], count)
            # cv2.circle(img, (lmList[14][1], lmList[14][2]), 15, (0, 0, 255), cv2.FILLED)  # 指定节点

            # cTime = time.time()
            # fps = 1 / (cTime - pTime)
            # pTime = cTime
            #
            # cv2.putText(img, str(int(fps)), (70, 50), cv2.FONT_HERSHEY_PLAIN, 3,
            #             (225, 0, 9), 3)

            # print(landmarks)
            # Get coorfinates
            shoulder = [landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,
                        landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
            elbow = [landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x,
                     landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y]
            wrist = [landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].x,
                     landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].y]
            hip = [landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x,
                   landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y]
            index = [landmarks[mp_pose.PoseLandmark.RIGHT_INDEX.value].x,
                     landmarks[mp_pose.PoseLandmark.RIGHT_INDEX.value].y]

            # calculate angle
            angle_elbow = calculate_angle(shoulder, elbow, wrist)
            angle_shoulder = calculate_angle(hip, shoulder, elbow)
            angle_wrist = calculate_angle(elbow, wrist, index)
            print_angle2tcsv(angle_elbow, angle_shoulder, angle_wrist, count)
            count += 1
            # Set the angle to frame
            h, w, c = img.shape
            cv2.putText(img, str(angle_elbow),
                        tuple(np.multiply(elbow, [h, w]).astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)

            cv2.putText(img, str(angle_shoulder),
                        tuple(np.multiply(shoulder, [h, w]).astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)

            cv2.putText(img, str(angle_wrist),
                        tuple(np.multiply(wrist, [h, w]).astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)

            cv2.imshow("Image", img)
            cv2.waitKey(1)


if __name__ == "__main__":
    main()
