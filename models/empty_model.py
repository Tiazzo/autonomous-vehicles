import traci

from models.base_model import BaseDecisionModel


class EmptyLaneChangeModel(BaseDecisionModel):
    def __init__(self):
        pass

    def decide_lane_change(
        self, traci_vehicle: traci._vehicle.VehicleDomain, **kwargs: dict[str, any]
    ):
        assert "USING EMPTY LANE CHANGE DECISION-MAKING MODEL!"
