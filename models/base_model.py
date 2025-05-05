from abc import ABC, abstractmethod


class BaseDecisionModel(ABC):
    @abstractmethod
    def decide_lane_change(self, **kwargs):
        """
        Determines whether the vehicle should change lanes.
        Args:
            **kwargs: Variable number of keyword arguments.
        Returns:
            bool: True if the vehicle decides to change lanes, False otherwise.
        """
        pass
