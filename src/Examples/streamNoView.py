from tpa64 import USBTPA64

with USBTPA64("COM8") as cam:
    for i, frame in enumerate(cam.iter_frames(interval=0.2)):
        print(i, frame.ambient_c, frame.min_c, frame.max_c)
        if i >= 20:
            break
