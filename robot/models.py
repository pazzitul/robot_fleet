from django.db import models
from django_mysql.models import JSONField


def position_default_value():
    return {
        "x": 0,
        "y": 0,
        "z": 0
    }

def orienation_default_value():
    return {
        "x": 0,
        "y": 0,
        "z": 0,
        "w": 1
    }


class RobotModel(models.Model):

    ROBOT_STATUS = (
        (0, 'UNKNOWN'),
        (1, 'UNAVAILABLE'),
        (2, 'ERROR'),
        (3, 'IDLE'),
        (4, 'PAUSE'),
        (5, 'EXECUTING'),
        (6, 'CHARGING')
    )

    sn = models.CharField(max_length=128, unique=True, verbose_name='编号')
    name = models.CharField(max_length=128, verbose_name='名称')
    # map = models.ForeignKey('map.MapModel', on_delete=models.CASCADE)
    # type = models.ForeignKey('robot.RobotTypeModel', on_delete=models.CASCADE)
    status = models.IntegerField(choices=ROBOT_STATUS, default=0, verbose_name='状态')
    position = JSONField(default=position_default_value, verbose_name='位置')
    orientation = JSONField(default=orienation_default_value, verbose_name='方向')
    power = models.IntegerField(default=0, verbose_name='电量')

    class Meta:
        db_table = 'fleet_robot'
        verbose_name_plural = '机器人模型'
        verbose_name = '机器人模型'


class RobotTypeModel(models.Model):
    """Model class of Robot Type
    """

    name = models.CharField(max_length=64)

    class Meta:
        db_table = 'fleet_robot_type'
