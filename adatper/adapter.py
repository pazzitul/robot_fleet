import json
import logging
import threading
from abc import abstractmethod, ABCMeta
from datetime import datetime
from threading import Timer
from channels.layers import get_channel_layer
from mission.models import MissionModel
from robot.models import RobotModel
from utils.mqtt import MqttClient
from asgiref.sync import async_to_sync
from utils.coordinate import map_to_canvas, euler_from_quaternion
import math
LOG = logging.getLogger('django')


class Lifecycle(metaclass=ABCMeta):
    @abstractmethod
    def initialize(self):
        pass

    @abstractmethod
    def is_initialized(self) -> bool:
        pass

    @abstractmethod
    def terminate(self):
        pass


class RobotCommAdapter(Lifecycle, metaclass=ABCMeta):
    @abstractmethod
    def enable(self):
        """Enables this comm adapter, i.e. turns it on

        """
        pass

    @abstractmethod
    def disable(self):
        """Disables this comm adapter, i.e. turns it off.

        """
        pass

    @abstractmethod
    def is_enabled(self) -> bool:
        pass

    @abstractmethod
    def get_robot_model(self) -> RobotModel:
        """Returns an observable model of the robot's and its comm adapter's attributes.

        Returns: An observable model of the robot's and its comm adapter's attributes.

        """
        pass

    @abstractmethod
    def get_mission_model(self) -> MissionModel:
        """Returns a mission model of the robot's and its comm adapter's attributes.

        Returns: A mission model of the robot's and its comm adapter's attributes.

        """
        pass

    @abstractmethod
    def en_queue(self, command):
        pass

    @abstractmethod
    def de_queue(self):
        pass


class RobotAdapter(RobotCommAdapter):

    def __init__(self, sn):
        self._name = sn
        self._enabled = False
        self._client = MqttClient(sn)
        self._initialized = False
        self._x = 0
        self._y = 0

    def initialize(self):
        if self.is_initialized():
            LOG.debug(f'{self._name}: Already initialized')
            return

        # 注册mqtt 回调函数
        # 初始化位置 Notify
        self._client.callback_add('/robot/+/command/init_position/ntf', self.on_init_position_ntf)
        # 任务 Ack
        self._client.callback_add('/robot/+/command/movement/ack', self.on_movement_ack)
        # 任务 Notify
        self._client.callback_add('/robot/+/command/movement/ntf', self.on_movement_ntf)
        self._client.callback_add(f'/robot/{self._name}/report/real_info', self.on_real_info)

    def is_initialized(self) -> bool:
        return self._initialized

    def terminate(self):
        if not self.is_initialized():
            LOG.debug(f'{self._name}: Not initialized')
            return

        self.disable()
        self._initialized = False

    def enable(self):
        if self.is_enabled():
            return

        if self.connect():
            self._enabled = True

    def disable(self):
        if not self.is_enabled():
            return

        if self.disconnect():
            self._enabled = False

    def is_enabled(self) -> bool:
        return self._enabled

    def connect(self):
        """连接mqtt broker

        Returns:

        """
        try:
            self._client.setDaemon(True)
            self._client.start()
        except Exception as e:
            LOG.error(e.__context__)
            return False

        return True

    def disconnect(self):
        """ 断开mqtt broker

        Returns:

        """
        try:
            self._client.join(2)

        except Exception as e:
            LOG.error(e.__context__)
            return False

        return True

    def get_robot_model(self) -> RobotModel:
        pass

    def get_mission_model(self) -> MissionModel:
        pass

    def en_queue(self, command):
        pass

    def de_queue(self):
        pass

    def map_switch(self, msg):
        self._client.publish('/robot/{0}/command/map/set'.format(self._name), json.dumps(msg))

    def set_movements(self, msg):
        self._client.publish('/robot/{0}/command/movement/set'.format(self._name), json.dumps(msg))

    # ------------------------ Callback -------------------
    def on_init_position_ntf(self, client, obj, msg):
        pass

    def on_movement_ack(self, client, obj, msg):
        pass

    def on_movement_ntf(self, client, obj, msg):
        pass

    def on_real_info(self, client, obj, msg):
        message_type = msg.topic.split('/')[-1]

        data = json.loads(msg.payload)
        self._x = data['x']
        self._y = data['y']

        canvas_point = map_to_canvas({
            'x': data['x'],
            'y': data['y']
        })

        data['canvas_x'] = canvas_point['x']
        data['canvas_y'] = canvas_point['y']
        roll_x, pitch_y, yaw_z = euler_from_quaternion(data['direction'][0],
                                      data['direction'][1],
                                      data['direction'][2],
                                      data['direction'][3]
                                      )
        angle = (180 / math.pi * yaw_z)
        # if angle < 0:
        #     angle += 720

        data['angle'] = -angle
        channel_layer = get_channel_layer()
        # print(data)
        async_to_sync(channel_layer.group_send)(message_type + '_' + self._name, {
            'type': 'real_info',
            'message': json.dumps(data)
        })

    def get_position(self):
        return {
            'x': self._x,
            'y': self._y,
            'z': 0
        }


class MasterAdapter(RobotCommAdapter):
    def __init__(self):
        super(MasterAdapter, self).__init__()
        self._name = 'master_adapter'
        # 用于与机器人通信客户端
        self._client = MqttClient(self._name)
        # enabled 标志
        self._enabled = False
        self._initialized = False
        # 机器人适配器列表
        self._robot_adapter_list = {}
        # 机器人注册列表, 存放第一次接入的机器人sn, 在界面点击接入按钮后从列表中移除
        self._robot_registration_list = []
        # 定时器, 定时根据心跳包检测机器人是否在线
        self._timers = {}
        self._run = threading.Thread(target=self.proc)

    def initialize(self):
        if self.is_initialized():
            LOG.debug(f'{self._name}: Already initialized')
            return

        # 订阅mqtt 消息, 注册回调函数
        self._client.callback_add('/robot/+/regist/request', self.on_register)
        self._client.callback_add('/robot/+/report/heartbeat/client', self.on_heartbeat)

        self.enable()
        self._initialized = True

    def terminate(self):
        if not self.is_initialized():
            LOG.debug(f'{self._name}: Not initialized')
            return

        self.disable()
        self._initialized = False

    def is_initialized(self) -> bool:
        return self._initialized

    def enable(self):
        if self.is_enabled():
            return

        if self.connect():
            self._enabled = True

    def disable(self):
        if not self.is_enabled():
            return

        if self.disconnect():
            self._enabled = False

    def is_enabled(self) -> bool:
        return self._enabled

    def connect(self):
        """连接mqtt broker

        Returns:

        """
        try:
            self._client.setDaemon(True)
            self._client.start()
        except Exception as e:
            LOG.error(e.__context__)
            return False

        return True

    def disconnect(self):
        """ 断开mqtt broker

        Returns:

        """
        try:
            self._client.join(2)

        except Exception as e:
            LOG.error(e.__context__)
            return False

        return True

    def get_registration_list(self):
        return self._robot_registration_list

    def remove_registration_list(self, sn):
        return self._robot_registration_list.remove(sn)

    def get_robot_model(self) -> RobotModel:
        pass

    def get_mission_model(self) -> MissionModel:
        pass

    def get_robot_adapter(self, sn):
        return self._robot_adapter_list.get(sn, None)

    def set_offline(self, arg, **kwargs):
        robot_adapter = self.get_robot_adapter(arg[0])
        if robot_adapter:
            robot_adapter.terminate()

    def en_queue(self, command):
        pass

    def de_queue(self):
        pass

    def proc(self):
        pass

    def register(self, sn):
        res = {
            'timestamp': datetime.now().strftime('%Y-%m-%D %H:%M:%S'),
            'message_type': 'regist'
        }
        if sn in self.get_registration_list():
            self._robot_registration_list.remove(sn)

        robot = RobotModel.objects.create(sn=sn, name=sn)
        if robot:
            adapter = RobotAdapter(sn)
            if adapter:
                self._robot_adapter_list[sn] = adapter
                adapter.enable()
                self._client.publish(f'/robot/{sn}/regist/response', json.dumps(res))

    # ---------------------- Callback --------------------
    def on_register(self, client, obj, msg):
        """ 机器人注册回调函数

        Args:
            client:
            obj:
            msg:

        Returns:

        """
        robot_sn = msg.topic.split('/')[2]
        # 如果已经注册过了, 则直接回复response, 否则将sn 加入到_robot_registration_list 列表中
        if RobotModel.objects.filter(sn=robot_sn).exists():
            res = {
                'timestamp': datetime.now().strftime('%Y-%m-%D %H:%M:%S'),
                'message_type': 'regist'
            }
            if robot_sn in self.get_registration_list():
                self._robot_registration_list.remove(robot_sn)

            self._client.publish(f'/robot/{robot_sn}/regist/response', json.dumps(res))

        elif robot_sn not in self.get_registration_list():
            self._robot_registration_list.append(robot_sn)

    def on_heartbeat(self, client, obj, msg):
        """ 机器人心跳回调函数, 用于初始化/开启/关闭 机器人适配器

        Args:
            client:
            obj:
            msg:

        Returns:

        """
        robot_sn = msg.topic.split('/')[2]
        robot_adapter = self.get_robot_adapter(robot_sn)
        if robot_adapter:
            if not robot_adapter.is_enabled():
                robot_adapter.enable()
        else:
            self._robot_adapter_list[robot_sn] = RobotAdapter(robot_sn)
            self._robot_adapter_list[robot_sn].initialize()
            self._robot_adapter_list[robot_sn].enable()

        if self._timers.get(robot_sn, None):
            self._timers[robot_sn].cancel()

        self._timers[robot_sn] = Timer(10, self.set_offline, [robot_sn], {})
        self._timers[robot_sn].setDaemon(True)
        self._timers[robot_sn].start()


MASTER_ADAPTER = MasterAdapter()
