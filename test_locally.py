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

import argparse
import subprocess
import time
import os

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-s", "--simulator", help="Path to the simulator binary", required=True
    )
    parser.add_argument(
        "-i",
        "--image",
        help="Name of the docker image to use for local evaluation",
        required=True,
    )
    perception_mode_flag = parser.add_mutually_exclusive_group(required=True)
    perception_mode_flag.add_argument(
        "-g", "--groundtruth", help="Use groundtruth observations", action="store_true"
    )
    perception_mode_flag.add_argument(
        "-p",
        "--perception",
        help="Use realistic perception for observations",
        action="store_true",
    )

    args = parser.parse_args()

    # ---- perception mode

    if args.perception:
        raise NotImplementedError(
            "This perception mode will be released in an update to the goseek-challenge soon. Please stay tuned!"
        )

    # ---- groundtruth mode

    # run simulator
    sim = subprocess.Popen([args.simulator, "--set_resolution", "320", "240"])
    time.sleep(10)  # wait for the simulator to open

    output = subprocess.check_output(
        [
            "docker",
            "run",
            "--rm",
            "--network=host",
            "--gpus",
            "all",
            "-v"
            # note to goseek-challenge participants: we will use an internal config for our challenge evaluations
            "{}/config:/config".format(os.getcwd()),
            "-t",
            "{}".format(args.image),
            "/bin/bash",
            "-c",
            "python eval.py --episode-config /config/ground-truth.yaml --agent-config agent.yaml",
        ]
    )
    print(output.decode('utf-8'))

    # cleanup
    sim.terminate()

