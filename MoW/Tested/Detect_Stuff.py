# video streaming , cv , Use drone onnx model (no flying)
import pyhula
from hula_video import hula_video
from onnxdetector import onnxdetector
import cv2
import time

api = pyhula.UserApi()

if not api.connect():
    print("connect error")
else:
    print("connection to station by wifi")
    vid = hula_video(hula_api = api, display= False )
    huladetector = onnxdetector(model = "detect_3_object_12_11.onnx", label="object.txt", confidence_thres =0.3)
    vid.video_mode_on()
    vid.startrecording()
    f = 0
    while True:
        frame = vid.get_video()
        obj_found , frame = huladetector.detect(frame)
        cv2.imshow("HULAA",frame)
        cv2.waitKey(1)
        time.sleep(0.1)
        if not obj_found is None:
            print(f"Found {obj_found}")
            f+=1
            if f == 10:
                break
        else:
            print("Looking for the object")
    cv2.destroyAllWindows()
    vid.stoprecording()
    vid.close()