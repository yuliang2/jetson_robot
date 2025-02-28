import cv2
import math
from ultralytics import YOLO

class PoseEstimator:
    def __init__(self, model_path="yolo11n-pose.pt"):
        self.model = YOLO(model_path)
        self.angles = {}

    def calculate_angle(self, p1, p2, p3):
        v1 = (p1[0] - p2[0], p1[1] - p2[1])
        v2 = (p3[0] - p2[0], p3[1] - p2[1])

        mod1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
        mod2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)

        if mod1 == 0 or mod2 == 0:
            return -1

        cos_theta = (v1[0] * v2[0] + v1[1] * v2[1]) / (mod1 * mod2)
        angle = math.degrees(math.acos(max(min(cos_theta, 1), -1)))
        return angle

    def get_joint_angles(self, keypoints):
        angles = {}

        if keypoints[5][1] > 0.1 and keypoints[7][1] > 0.1 and keypoints[9][1] > 0.1:
            angles['left_elbow'] = self.calculate_angle(keypoints[5], keypoints[7], keypoints[9])

        if keypoints[6][1] > 0.1 and keypoints[8][1] > 0.1 and keypoints[10][1] > 0.1:
            angles['right_elbow'] = self.calculate_angle(keypoints[6], keypoints[8], keypoints[10])

        if keypoints[7][1] > 0.1 and keypoints[5][1] > 0.1 and keypoints[11][1] > 0.1:
            angles['left_shoulder'] = self.calculate_angle(keypoints[7], keypoints[5], keypoints[11])

        if keypoints[8][1] > 0.1 and keypoints[6][1] > 0.1 and keypoints[12][1] > 0.1:
            angles['right_shoulder'] = self.calculate_angle(keypoints[8], keypoints[6], keypoints[12])

        if keypoints[11][1] > 0.1 and keypoints[13][1] > 0.1 and keypoints[15][1] > 0.1:
            angles['left_knee'] = self.calculate_angle(keypoints[11], keypoints[13], keypoints[15])

        if keypoints[12][1] > 0.1 and keypoints[14][1] > 0.1 and keypoints[16][1] > 0.1:
            angles['right_knee'] = self.calculate_angle(keypoints[12], keypoints[14], keypoints[16])

        return angles

    def inference(self, frame):
        results = self.model(frame, stream=False)
        result = results[0]
        annotated_frame = result.plot()

        keypoints = result.keypoints.xy.cpu().numpy()
        angles = {}
        kpts = keypoints[0]
        if len(kpts) > 15:
            angles = self.get_joint_angles(kpts)


        for j, (x, y) in enumerate(kpts):
            if x > 0 and y > 0:
                cv2.putText(annotated_frame, str(j), (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 0, 0), 1)

        return annotated_frame, angles


if __name__ == "__main__":
    pose_estimator = PoseEstimator()
    camera_index = 4
    cap = cv2.VideoCapture(camera_index)
    cap.set(5, 30)  # 帧率
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        annotated_frame, angles = pose_estimator.inference(frame)

        cv2.imshow("YOLO-Pose", annotated_frame)
        print(angles)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

