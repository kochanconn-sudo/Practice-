import config
import zmq
import json
import time
import base64
import numpy as np
import cv2


class BaseCameraWrapper:
    def read(self):
        raise NotImplementedError("Subclasses must implement 'read'.")

    def release(self):
        raise NotImplementedError("Subclasses must implement 'release'.")

    def get_data(self):
        return self.read()[1] # [1]only return image data
    
    def cleanup(self):
        self.release()
        print(f"Camera cleanup complete.")


class CameraSim(BaseCameraWrapper):
    def __init__(self, device_id=0):
        self.device_id = device_id
        self.topic_name = f"camera_{device_id}"
        self.latest_frame = None

        # ZeroMQ subscriber
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, self.topic_name)
        self.subscriber.setsockopt(zmq.RCVTIMEO, 100)
        self.subscriber.connect("tcp://localhost:5555")
    
    def __del__(self):
        self.release()

    def read(self):
        try:
            [topic, msg] = self.subscriber.recv_multipart()
            topic_name = topic.decode('utf-8')
            if topic_name == self.topic_name:
                data = json.loads(msg.decode('utf-8'))
                jpeg_data = base64.b64decode(data["data"])
                image_data = np.frombuffer(jpeg_data, dtype=np.uint8)
                frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

                # Determine flip settings based on camera ID
                if self.device_id == 0:
                    vflip = config.CAMERA_0_VFLIP
                    hflip = config.CAMERA_0_HFLIP
                else:
                    vflip = config.CAMERA_1_VFLIP
                    hflip = config.CAMERA_1_HFLIP

                if vflip:
                    frame = cv2.flip(frame, 0)
                if hflip:
                    frame = cv2.flip(frame, 1)
                
                self.latest_frame = cv2.resize(frame, (config.IMAGE_W, config.IMAGE_H))
                return True, self.latest_frame

        except json.JSONDecodeError as e:
            print("json decoding error")
        except zmq.Again as e:
            print("No message received within the timeout period.")
        
        if self.latest_frame is not None:
            return True, self.latest_frame
        else:
            return False, None

    def release(self):
        self.subscriber.close()
        self.context.term()


def create_camera(device_id, use_multiprocess=False):
    return CameraSim(device_id=device_id)


if __name__ == "__main__":
    
    camera = None

    if "camera_0" in config.ACTIVE_SENSORS:
        camera = create_camera(device_id=0)

    try:
        while True:
            ret, frame = camera.read()
            if ret:
                # RGB→BGR変換（cv2.imshowはBGR形式を期待）
                cv2.imshow("Camera", frame[:, :, ::-1])
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break

        camera.release()
    except KeyboardInterrupt:
        pass
