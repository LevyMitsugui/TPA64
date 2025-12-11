from tpa64 import USBTPA64, Frame


with USBTPA64("COM8") as cam:
    module_id, fw = cam.get_info()
    print("Module:", module_id, "FW:", fw)
    print("Serial:", cam.get_serial())

    frame = cam.get_frame()
    print(f"Ambient: {frame.ambient_c:.2f} °C")
    print(f"Min: {frame.min_c:.2f} °C, Max: {frame.max_c:.2f} °C")

    img = frame.to_image(size=(256, 256), dynamic_range=True)
    img.save("snap.png")
