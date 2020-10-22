from django.db import models


class RobotModel(models.Model):
    """Model class of Robot

    Attributes:
        sn: Robot factory number, fixed.
        name: Robot name
        map: Robot bound map
        type: Robot type
    """

    ROBOT_STATUS = (
        (0, 'UNKNOWN'),
        (1, 'UNAVAILABLE'),
        (2, 'ERROR'),
        (3, 'IDLE'),
        (4, 'PAUSE'),
        (5, 'EXECUTING'),
        (6, 'CHARGING')
    )

    sn = models.CharField(max_length=128, unique=True)
    name = models.CharField(max_length=128)
    map = models.ForeignKey('map.MapModel', on_delete=models.CASCADE)
    type = models.ForeignKey('robot.RobotTypeModel', on_delete=models.CASCADE)
    status = models.IntegerField(choices=ROBOT_STATUS, default=0)
    position = models.JSONField(default={'x': 0, 'y': 0, 'z': 0})
    orientation = models.JSONField(default={'x': 0, 'y': 0, 'z': 0, 'w': 1})
    power = models.IntegerField(default=0)

    class Meta:
        db_table = 'fleet_robot'


class RobotTypeModel(models.Model):
    """Model class of Robot Type
    """

    name = models.CharField(max_length=64)

    class Meta:
        db_table = 'fleet_robot_type'