#!/bin/sh

echo "Setting up environment..."

DIR=$(pwd)
echo "Current directory: "
echo $DIR

export GAZEBO_PLUGIN_PATH=$DIR/plugins/build
export GAZEBO_MODEL_PATH=$DIR/models

echo "Done!"
