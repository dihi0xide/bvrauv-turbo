from abc import ABC, abstractmethod
from typing import Tuple

from pi.library.motor_controller import MotorController


class Task(ABC):

    @property
    @abstractmethod
    def name(self) -> str:
        pass

    @abstractmethod
    def update(self, motors: MotorController) -> None:
        pass


class Path:
    def __init__(self, *args: Task):
        self.path: Tuple[Task, ...] = args