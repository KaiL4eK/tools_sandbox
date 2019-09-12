#!/bin/bash

MO_PATH=/opt/intel/openvino/deployment_tools/model_optimizer/mo_tf.py
GRAPH_PATH=graph.pb
INPUT_SHAPE="[2,416,416,3]"
OUTPUT_DIR="openvino_models"
OUTPUT_MODEL="sample"

docker run --rm -it --privileged \
    -v /dev:/dev -v `pwd`:/home/developer \
    docker-openvino \
    bash -c \
    "python3 $MO_PATH --input_model $GRAPH_PATH --model_name $OUTPUT_MODEL --input_shape $INPUT_SHAPE --output_dir $OUTPUT_DIR --data_type FP32"

#  --log_level=DEBUG
