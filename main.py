from models.base_model import BaseDecisionModel
from models.improved_liu_model import LiuImproved
from models.liu_model import Liu
from models.ml_model import ML
from models.sl2015_model import SL2015
from simulation_manager import SimulationManager

# SCENARIO = "A"
# SCENARIO = "B"
SCENARIO = "C"


def main():
    scenario = f"Scenario{SCENARIO}"

    models = {
        "Liu": Liu(),
        "ML": ML(),
        "LiuImproved": LiuImproved(),
        "SL2015": SL2015(),
    }

    sim_manager = SimulationManager(scenario=scenario, models=models, max_steps=500)
    sim_manager.run_all_simulations()


if __name__ == "__main__":
    main()
