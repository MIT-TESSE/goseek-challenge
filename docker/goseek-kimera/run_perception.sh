docker run  --rm --network=host --gpus all -it goseek-kimera /bin/bash \
    -c "source /catkin_ws/devel/setup.bash && \
    roslaunch tesse_gym_bridge run_kimera_tesse.launch \
    kimera_vio:=true \
    kimera_semantics:=false \
    use_ground_truth:=false  \
    publish_depth:=false \
    publish_segmentation:=false"
