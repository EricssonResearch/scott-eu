import cv2
import numpy as np
from visualize_cv import model, display_instances, class_names
import time

capture = cv2.VideoCapture('rosbag_video.mpg')
size = (
    int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
    int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
)
codec = cv2.VideoWriter_fourcc(*'DIVX')
output = cv2.VideoWriter('rosbag_masked2.avi', codec, 25.0, size)

while(capture.isOpened()):
    ret, frame = capture.read()
    if ret:
        # add mask to frame
        # print("start detecting ....")
        a = time.time()
        results = model.detect([frame], verbose=1)
        # print("end detecting ...")
        b = time.time()
        real_FPS = 1.0/(b-a)
        print("------------> detection FPS %.2f" % real_FPS)
        r = results[0]
        frame = display_instances(
            frame, r['rois'], r['masks'], r['class_ids'], class_names, r['scores']
        )
        output.write(frame)
        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

capture.release()
output.release()
cv2.destroyAllWindows()
