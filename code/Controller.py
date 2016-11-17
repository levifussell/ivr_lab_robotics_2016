from Robot import Robot

from abc import ABCMeta, abstractmethod

class Controller:

    __metaclass__ = ABCMeta

    def __init__(self, robot):
        self.robot = robot

    def update(self):
        raise NotImplementedError( "Controller method update not implemented" )