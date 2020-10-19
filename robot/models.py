from django.db import models


class RobotModel(models.Model):
    """@Model class of Robot

    Attributes:
        sn: Robot factory number, fixed.
        name: Robot name
        map: Robot bound map
        type: Robot type
    """
    sn = models.CharField(max_length=128, unique=True)
    name = models.CharField(max_length=128)
    map = models.ForeignKey('map.MapModel', on_delete=models.CASCADE)
    type = models.ForeignKey('robot.RobotTypeModel', on_delete=models.CASCADE)

    class Meta:
        db_table = 'fleet_robot'


class RobotTypeModel(models.Model):
    """@Model class of Robot Type
    """

    class Meta:
        db_table = 'fleet_robot_type'