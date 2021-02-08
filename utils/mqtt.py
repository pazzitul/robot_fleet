import logging
import threading

import paho.mqtt.client as mqtt
from django.conf import settings

logger = logging.getLogger('django')


class MqttClient(threading.Thread):
    """

    """
    def __init__(self, client_name):
        super(MqttClient, self).__init__()

        self._client_name = client_name
        self._client = mqtt.Client(client_id='fleet2_{0}'.format(self._client_name), clean_session=True)
        self._client.on_connect = self.on_connect
        self._client.reconnect_delay_set(1, 10)

    def on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            logger.error('MQTT server connect fail, error code with {0}'.format(rc))
            return
        else:
            logger.info(f'MQTT server connect successful, Client name: {self._client_name!r}')
            client.subscribe('/robot/#')

    def publish(self, topic, payload, qos=0):
        return self._client.publish(topic=topic, payload=payload, qos=qos)

    def subscribe(self, topic, qos=0):
        return self._client.subscribe(topic=topic, qos=qos)

    def disconnect(self):
        pass

    def callback_add(self, topic, func):
        self._client.message_callback_add(topic, func)

    def run(self) -> None:
        self._client.connect(settings.MQTT_SERVER_CONFIG['host'], settings.MQTT_SERVER_CONFIG['port'])
        self._client.loop_forever(timeout=5, max_packets=1, retry_first_connection=True)

