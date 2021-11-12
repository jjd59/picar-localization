import math
from imutils.video import VideoStream
from pyzbar import pyzbar
import imutils
import cv2
import time
import numpy as np
import os



class Car:
    # Camera specific measurement
    # If the QR code is 9.5 cm, 512 x 512 px, and the camera views it at a distance of 30 cm,
    # and the image is played in a frame that is 400 px,
    # then the QR code appears to be 130 px.

    # actual physical size of the QR
    QR_SIZE_CM = 9.5
    # size of the frame of the playback window
    FRAME_SIZE_PX = 1200
    # actual physical distance of the camera to the QR code when the QR image appears to be QR_SIZE_PX in a frame that is FRAME_SIZE_PX
    QR_DISTANCE_CM = 30.0
    # ratio of apparent size of QR code to size of playback frame at a distance of 30 cm
    CAMERA_SCALER_PX = 130 / 400

    # size of the QR code viewed from the camera at a distance of QR_DISTANCE_CM when the frame is FRAME_SIZE_PX
    QR_SIZE_PX = CAMERA_SCALER_PX * FRAME_SIZE_PX

    QR_DIST_SCALER = QR_SIZE_PX * QR_DISTANCE_CM

    def __init__(self, name, room):
        self.name = name
        self.position = (None, None, None)
        self.vs = VideoStream(src=0).start()
        self.sensor_data = None
        self.room = room

    def display(self):
        print(self.name)
        print(self.position)
        print(self.sensor_data)

    def sense(self):
        frame = self.vs.read()
        frame = imutils.resize(frame, width=self.FRAME_SIZE_PX)
        # find the barcodes in the frame and decode each of the barcodes
        barcodes = pyzbar.decode(frame)

        # loop over the detected barcodes
        self.sensor_data = []
        for barcode in barcodes:
            # extract the bounding box location of the barcode and draw
            # the bounding box surrounding the barcode on the image
            (x, y, w, h) = barcode.rect
            #cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            
            pts = []
            for pt in barcode.polygon:
                pts.append([pt.x, pt.y]) 
            polypts = np.array(pts)
            polypts = polypts.reshape((-1, 1, 2))
            cv2.polylines(frame, [polypts], True, (255, 0, 0), 1)
            # the barcode data is a bytes object so if we want to draw it
            # on our output image we need to convert it to a string first
            barcode_data = barcode.data.decode("utf-8")
            barcode_type = barcode.type
            # draw the barcode data and barcode type on the image
            qr_size_px = (h + w) / 2
            if qr_size_px > 1:
                qr_center_px = x + w / 2
                qr_shift_px = qr_center_px - self.FRAME_SIZE_PX / 2

                qr_distance_cm = self.QR_DIST_SCALER / qr_size_px
                qr_shift_cm = qr_shift_px / qr_size_px * self.QR_SIZE_CM
                #qr_angle_rad = math.asin(qr_shift_cm / qr_distance_cm)
                sense_result = {
                    "marker": barcode_data,
                    "distance": qr_distance_cm,
                    #"angle": qr_angle_rad,
                    "dx": qr_shift_cm,
                    "dy": math.sqrt(qr_distance_cm ** 2 - qr_shift_cm ** 2)
                }
                self.sensor_data.append(sense_result)
                text1 = "{} {:.1f} {:.1f}".format(barcode_data, sense_result["dx"], sense_result["dy"])
                text2 = "{:.1f} {:.1f} ".format(qr_shift_cm, qr_distance_cm)
                cv2.putText(frame, text1, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv2.putText(frame, text2, (x, y + h + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                time.sleep(0.1)

        # show the output frame
        cv2.imshow(self.name, frame)

    def run(self):
        while True:
            self.sense()
            for sensor_result in self.sensor_data:
                self.position = self.room.calculate_position(sensor_result)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
        cv2.destroyAllWindows()
        self.vs.stop()

    def test(self):
        self.display()
        self.run()
        self.display()


class Room:

    EMPTY = 0
    OBSTACLE = 1
    MARKER = 2

    def __init__(self, name):
        self.name = name
        self._grid = {}

    def display(self):
        print(self.name)
        print(self._grid)

    def add_object(self, position, object_type, object_id=None):
        self._grid.update({(object_type, object_id): position})

    def add_marker(self, position):
        self.add_object(position, self.MARKER, "1001")

    def locate_marker(self, marker_id):
        if (self.MARKER, marker_id) in self._grid.keys():
            return self._grid[(self.MARKER, marker_id)]
        return None

    def calculate_position(self, sensor_result):
        m = sensor_result["marker"]
        d = sensor_result["distance"]
        #a = sensor_result["angle"]
        dx = sensor_result["dx"]
        dy = sensor_result["dy"]
        (mx, my) = self.locate_marker(m)
        rx = mx - dx
        ry = my - dy
        return (rx, ry)

    def test(self):
        self.display()
        self.add_object((0, 0), self.EMPTY)
        self.display()
        self.add_object((10, 5), self.OBSTACLE)
        self.display()
        self.add_marker((0,100))
        self.display()
        print(self.locate_marker(1001))
        print(self.locate_marker(1002))
        print(self.calculate_position({"marker": "1001", "distance": 1.0, "angle": 0, "dx": 0, "dy": 1}))
        print(self.calculate_position({"marker": "1001", "distance": 1.0, "angle": 0, "dx": 0.2, "dy": 0.98}))

office = Room("office")
office.test()

print(os.name)

my_car = Car("Misy", office)
my_car.test()





