import config
import zmq
import json
import time

class Ultrasonic:

    def __init__(self, sensor_name):
        self.sensor_name = sensor_name
        self.distance = 0.0
        self.sim_time = 0.0

        # ZeroMQ subscriber
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, self.sensor_name)
        self.subscriber.setsockopt(zmq.RCVTIMEO, 100)
        self.subscriber.connect("tcp://localhost:5555")

    def __del__(self):
        self.subscriber.close()
        self.context.term()

    def measure(self):
        try:
            [topic, msg] = self.subscriber.recv_multipart()
            topic_name = topic.decode('utf-8')
            if topic_name == self.sensor_name:
                data = json.loads(msg.decode('utf-8'))
                self.distance = data["distance"] * 1000
                self.sim_time = data["sim_time"]
        except json.JSONDecodeError as e:
            print("json decoding error")
        except zmq.Again as e:
            print("No message received within the timeout period.")
        
        return self.distance
    
                

if __name__ == '__main__':
    sensor_names = config.ULTRASONIC_SENSOR_LIST
    sensors = []
    for sensor_name in sensor_names:
        sensors.append(Ultrasonic(sensor_name))

    try:
        while True:
            datalist = {}
            for sensor in sensors:
                distance = sensor.measure()
                datalist[sensor.sensor_name] = distance

            print(datalist)
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
