#!/bin/bash

# Parse Arguments
# -- simulation path
# -- docker name
# -- flag

# Launch Simulator
./simulator/goseek-v0.1.0.x86_64  --set_resolution 320 240 &
pid=$!
sleep 10  # Wait for simulator to open up

# (Optionally, launch Kimera)


# Run Docker
sudo -E docker run --network="host" --gpus all  \
  -v $(pwd)/config:/config \
  --rm -it submission /bin/bash -c      \
  "python eval.py --episode-config /config/ground-truth.yaml --agent-config agent.yaml"

# Cleanup
kill -9 $pid
