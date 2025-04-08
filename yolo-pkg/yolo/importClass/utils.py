from geometry_msgs.msg import Pose, PoseArray
from functools import wraps

def convert_to_pose_array(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        detections = func(*args, **kwargs) 
        
        pose_array = PoseArray()
        pose_array.poses = [
            Pose(position=Pose().position.__class__(x=d["x"], y=d["y"], z=d["z"]))
            for d in detections
        ]
        
        return pose_array
    return wrapper
