from abc import ABC, abstractmethod

from motor_registry import MotorController


class Task(ABC):

    @property
    @abstractmethod
    def name() -> str:
        pass

    @abstractmethod
    def update(motors: MotorController) -> None:
        pass


class Path:
    def __init__(self, *args: Task):
        self.path: Task = args