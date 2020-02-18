import argparse
import subprocess
import time
import os


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

