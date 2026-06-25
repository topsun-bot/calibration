from .detector import Detection3D, YOLO3DDetector

def __getattr__(name):
    if name == "RealSenseCamera":
        from .camera import RealSenseCamera
        return RealSenseCamera
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = ["Detection3D", "YOLO3DDetector", "RealSenseCamera"]
