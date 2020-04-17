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
import ast
import hashlib
import os
import pprint
import subprocess
import time
from datetime import datetime

RECALL_WEIGHT = 1
PRECISION_WEIGHT = 0.1
COLLISIONS_WEIGHT = 0.1
ACTIONS_WEIGHT = 0.1
EPISODE_LENGTH = 400

DOCKER_VIO_LOGDIR = "/catkin_ws/src/Kimera-VIO-ROS/output_logs"
HOST_VIO_LOGDIR = "kimera_vio_logs"


def _time_hash(prefix):
    """ Get has based on current time. """
    hash = hashlib.sha1()
    hash.update(str(time.time()).encode())
    return f"{prefix}_{hash.hexdigest()}"


def summarize(results):
    n_episodes = len(results) - 1

    metrics = {}
    metrics["Recall"] = results["total"]["recall"]
    metrics["Precision"] = results["total"]["precision"]
    metrics["Collisions"] = (
        results["total"]["collisions"] / n_episodes
    )  # results has total collisions
    metrics["Actions"] = (
        results["total"]["steps"] / n_episodes
    )  # results has total steps

    metrics["Weighted Total"] = (
        RECALL_WEIGHT * metrics["Recall"]
        + PRECISION_WEIGHT * metrics["Precision"]
        - COLLISIONS_WEIGHT * metrics["Collisions"] / EPISODE_LENGTH
        - ACTIONS_WEIGHT * metrics["Actions"] / EPISODE_LENGTH
    )
    return metrics


def start_perception_subprocess(image_name):
    """ Start docker container with perception server as subprocess. """
    container_name = _time_hash("goseek_perception")
    return (
        subprocess.Popen(
            [
                "docker",
                "run",
                "--rm",
                "--name",
                f"{container_name}",
                "--network=host",
                "--gpus",
                "all",
                "-t",
                f"{image_name}",
                "/bin/bash",
                "-c",
                "source /catkin_ws/devel/setup.bash && "
                "roslaunch tesse_gym_bridge run_goseek_perception.launch",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ),
        container_name,
    )


def start_submission_subprocess(config_dir, submission_image, episode_config_file):
    submission_name = _time_hash("submission")
    return (
        subprocess.Popen(
            [
                "docker",
                "run",
                "--name",
                f"{submission_name}",
                "--network=host",
                "--gpus",
                "all",
                "-v",
                # note to goseek-challenge participants:
                # we will use an internal config for our challenge evaluations
                "{}:/config".format(config_dir),
                "-t",
                "{}".format(submission_image),
                "/bin/bash",
                "-c",
                "python eval.py --episode-config /config/{} --agent-config agent.yaml".format(
                    episode_config_file
                ),
            ],
        ),
        submission_name,
    )


def kill_docker_container(container_name):
    """ Kill docker container by name. """
    subprocess.call(
        ["docker", "rm", "--force", f"{container_name}"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def get_docker_logs(container_name):
    """ Get logs of docker container. """
    p = subprocess.run(
        ["docker", "logs", container_name],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return p.stdout.decode(), ""  # TODO parse output to stdout and stderr


def docker_cp(container_name, container_dir, host_dir):
    subprocess.run(["docker", "cp", f"{container_name}:{container_dir}", host_dir])


def write_logs(fname, data):
    with open(fname, "w") as f:
        f.write(data)


def extract_metrics(process, submission_stdout):
    if process.returncode == 0:
        results = submission_stdout.split("----- Per Episode Score -----")[-1]
        metrics = summarize(ast.literal_eval(results))
    else:  # Agent failure returns -1 weighted score
        metrics = {
            "Recall": 0,
            "Precision": 0,
            "Collisions": 0,
            "Actions": 0,
            "Weighted Total": -1,
        }
    return metrics


def log_perception_data(container_name, logdir):
    perception_stdout, perception_stderr = get_docker_logs(container_name)
    docker_cp(
        container_name, DOCKER_VIO_LOGDIR, f"{logdir}/{HOST_VIO_LOGDIR}",
    )

    write_logs(f"{logdir}/perception-stdout.txt", perception_stdout)
    write_logs(f"{logdir}/perception-stderr.txt", perception_stderr)


def main(
    simulator: str,  # path to simulator
    kimera: bool,  # specify whether to run kimera docker
    image: str,  # path to docker image
    config: str,  # path to configuration,
    logdir: str = None,  # optional log dir
):

    try:
        if logdir is None:
            logdir = f"logs/{datetime.now().strftime('%m-%d-%Y_%H-%M-%s')}"
        if not os.path.exists(logdir):
            os.makedirs(logdir)

        # run simulator
        sim = subprocess.Popen([simulator, "--set_resolution", "320", "240"])

        # run perception server
        if kimera:
            perception, perception_container_name = start_perception_subprocess(
                "goseek-kimera"
            )
            time.sleep(20)

        time.sleep(10)  # wait for the simulator to open

        print("Running agent...")

        # Split up configuration
        dirname, filename = os.path.split(os.path.abspath(config))
        p, submission_name = start_submission_subprocess(dirname, image, filename)
        p.communicate()  # wait for process to stop

    except Exception as ex:
        print(f"CAUGHT EXCEPTION: {ex}")

    except KeyboardInterrupt:
        print(f"Caught keyboard interrupt, exiting...")

    finally:
        # cleanup
        sim.terminate()
        print("stopped simulator...")

        stdoutdata, stderrdata = get_docker_logs(submission_name)
        metrics = extract_metrics(p, stdoutdata)
        write_logs(f"{logdir}/submission-stdout.txt", stdoutdata)

        kill_docker_container(submission_name)
        print("stopped submission...")

        if kimera:
            log_perception_data(perception_container_name, logdir)
            kill_docker_container(perception_container_name)
            perception.terminate()
            print("stopped perception pipeline...")

    return stdoutdata, stderrdata, metrics, p.returncode


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

    if args.perception:
        episode_config = "{}/config/perception-pipeline.yaml".format(os.getcwd())
        kimera = True
    else:  # ground truth mode
        episode_config = "{}/config/ground-truth.yaml".format(os.getcwd())
        kimera = False

    stdoutdata, stderrdata, metrics, returncode = main(
        args.simulator, kimera, args.image, episode_config
    )

    print("\n***** Summary Metrics *****")
    pprint.pprint(metrics)
