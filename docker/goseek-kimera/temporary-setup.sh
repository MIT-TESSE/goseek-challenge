# this is a temporary workaround to add repositories into the docker container.
# when repositories are on github.com this file will be removed

rm -rf tmp

git clone git@github.mit.edu:TESS/tesse-gym-bridge.git -b feature/kimera-integration tmp/tesse-gym-bridge
git clone git@github.mit.edu:TESS/tesse-segmentation-ros.git tmp/tesse-segmentation-ros
git clone git@github.mit.edu:TESS/tesse-ros-bridge.git -b feature/data-config tmp/tesse-ros-bridge