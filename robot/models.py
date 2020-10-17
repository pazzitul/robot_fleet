from django.db import models


class RobotModel(models.Model):
    sn = models.CharField(max_length=128, unique=True)
    name = models.CharField(max_length=128)
    map = models.ForeignKey('map.MapModel', on_delete=models.CASCADE)
    type = models.ForeignKey('robot.RobotTypeModel', on_delete=models.CASCADE)

    class Meta:
        db_table = 'fleet_robot'


class RobotTypeModel(models.Model):

    class Meta:
        db_table = 'fleet_robot_type'