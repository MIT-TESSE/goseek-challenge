###################################################################################################
# DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.
#
# This material is based upon work supported by the Under Secretary of Defense for Research and
# Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions
# or recommendations expressed in this material are those of the author(s) and do not necessarily
# reflect the views of the Under Secretary of Defense for Research and Engineering.
#
# (c) 2020 Massachusetts Institute of Technology.
#
# MIT Proprietary, Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)
#
# The software/firmware is provided to you on an As-Is basis
#
# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013
# or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work
# are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other
# than as specifically authorized by the U.S. Government may violate any copyrights that exist in
# this work.
###################################################################################################

from typing import Any, Dict

import numpy as np

from tesse_gym.eval.agent import Agent


class StableBaselinesPPO(Agent):
    """ Stable Baselines PPO agent for GOSEEK submission. """

    def __init__(self, config: Dict[str, Any]) -> None:
        """ Initialize agent.

        Args:
            config (Dict[str, Any]): Agent configuration.
        """
        from stable_baselines import PPO2
        self.model = PPO2.load(config["weights"])
        self.state = None

        # Number of environments used to train model
        # to which stable-baselines input tensor size is fixed
        self.n_train_envs = self.model.initial_state.shape[0]

    def act(self, observation: np.ndarray) -> int:
        """ Act on an observation.

        args:
            observation (np.ndarray): observation.

        returns:
            int: an action in [0, 4) defined as follows
                - 0: forward 0.5m
                - 1: right 8 degrees
                - 2: left 8 degrees
                - 3: declare target
        """
        observation = np.repeat(observation[np.newaxis], self.n_train_envs, 0)
        actions, state = self.model.predict(
            observation, state=self.state, deterministic=False
        )
        self.state = state  # update model state
        return actions[0]

    def reset(self) -> None:
        """ Reset model state. """
        self.state = None


class RandomAgent(Agent):
    """ Agent that takes random actions. """

    def __init__(self, config: Dict[str, Any]) -> None:
        """ Initialize agent.

        Args:
            config (Dict[str, Any]): Agent configuration
        """

        self.action_space = np.arange(0, 4)

        # give probability for actions in `self.action_space`
        self.action_probability = np.array(config["action_probability"])
        self.action_probability /= self.action_probability.sum()

    def act(self, observation: np.ndarray) -> int:
        """ Take a uniformly random action.

        args:
            observation (np.ndarray): observation.

        returns:
            int: an action in [0, 4) defined as follows
                - 0: forward 0.5m
                - 1: right 8 degrees
                - 2: left 8 degrees
                - 3: declare target
        """
        return np.random.choice(self.action_space, p=self.action_probability)

    def reset(self) -> None:
        """ Nothing required on episode reset. """
        pass
