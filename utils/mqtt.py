import paho.mqtt.client as mqtt
import threading


class MqttClient(threading.Thread):
    def __init__(self, client_name):
        super(MqttClient, self).__init__()

        self._client_name = client_name
        self._client = mqtt.Client(client_id='fleet_{0}'.format(self._client_name), clean_session=True)
        self._client.on_connect = self.on_connect

    def on_connect(self, client, userdata, flags, rc):
        pass

    def publish(self, topic, payload, qos=0):
        pass

    def subscribe(self, topic, qos=0):
        pass

    def disconnect(self):
        pass

    def run(self) -> None:
        pass

