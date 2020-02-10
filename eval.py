import argparse
import pprint
from typing import Any, Dict

import yaml

from baselines.agents import *  # specific agent is given in agent-config
from tesse_gym.eval.agent import Agent
from tesse_gym.eval.treasure_hunt_benchmark import TreasureHuntBenchmark
from tesse_gym.eval.utils import get_agent_cls


def main(env_args: Dict[str, Any], agent_args: Dict[str, Any]) -> Dict[str, Dict[str, float]]:
    """ Run GOSEEK evaluation over the specified environment and
    agent configurations.

    Args:
        env_args (Dict[str, Any]): Environment configurations.
        agent_args (Dict[str, Any]): Agent configurations.

    Returns:
        Dict[str, Dict[str, float]]: Dictionary containing overall evaluation performance as
            well as a summary for each episode.
    """
    benchmark = TreasureHuntBenchmark(**env_args)
    agent = get_agent_cls(agent_args["name"], Agent)(agent_args)
    return benchmark.evaluate(agent)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--env-config", type=str)
    parser.add_argument("--agent-config", type=str)
    args = parser.parse_args()

    with open(args.env_config) as f:
        env_args = yaml.load(f, Loader=yaml.FullLoader)

    with open(args.agent_config) as f:
        agent_args = yaml.load(f, Loader=yaml.FullLoader)

    results = main(env_args, agent_args)

    print("------ Environment Configuration -----")
    pprint.pprint(env_args, depth=2)

    print("\n----- Agent Configuration -----")
    pprint.pprint(agent_args, depth=2)

    print("\n----- Per Episode Score -----")
    pprint.pprint(results, depth=2)
