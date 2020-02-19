# this is a temporary workaround to add repositories into the docker container.
# when repositories are on github.com this file will be removed
# TODO(MMAZ) when doing so, also edit goseek-challenge/docker/.gitignore
git clone git@github.mit.edu:TESS/goseek-challenge.git
git clone git@github.mit.edu:TESS/tesse-gym.git -b 0.1.1-SNAPSHOT
git clone git@github.mit.edu:TESS/tesse-interface.git
