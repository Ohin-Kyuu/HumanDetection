from ultralytics import YOLO
from yolo.importClass import convert_to_pose_array
import cv2 

class YoloDetection:
    def __init__(self, model_path="person480.pt"):
        self.model = YOLO(model_path)
        self.results_img_array = []

    @convert_to_pose_array
    def detect(self, color_image, depth_image):
        if color_image is None or depth_image is None:
            return []

        results = self.model(color_image)
        detected_objects = []

        for object in results:
            for box in object.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = box.conf[0].item()

                if confidence >= 0.7 and object.names[0] == "person":
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    depth = float(depth_image[int(cy), int(cx)]) / 1000.0
                    
                    detected_objects.append({"x": cx, "y": cy, "z": depth})

        return detected_objects
    
    @convert_to_pose_array
    def color_detect(self, color_image):
        if color_image is None:
            return []

        results = self.model(color_image)
        detected_objects = []

        for object in results:
            results_img = object.plot()
            self.results_img_array.append(results_img)

            for box in object.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = box.conf[0].item()

                if confidence >= 0.7 and object.names[0] == "pedestrian":
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    depth = 0.0
                    
                    detected_objects.append({"x": cx, "y": cy, "z": depth})

        return detected_objects
    