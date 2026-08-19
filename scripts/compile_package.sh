#!/bin/bash

usage() {
  echo "Usage: run this script from the root of the project."
  echo "Example: "
  echo "./scripts/compile_package.sh"
}

if [ $# -ne 0 ]; then
  usage
  exit 1
fi

ROOT_DIR="$(pwd)"
echo "ROOT_DIR: $ROOT_DIR"

echo "Compiling the package..."
if cd ~/workspace/catkin_ws_fwl/; then
  rm -rf ~/workspace/catkin_ws_fwl/*
fi

mkdir -p ~/workspace/catkin_ws_fwl/src/
cd ~/workspace/catkin_ws_fwl/src/
cp -r $ROOT_DIR/franka_weight_lift/ .   # ~/workspace/catkin_ws_fwl/src/
catkin_init_workspace   # ~/workspace/catkin_ws_fwl/src/

cd ~/workspace/catkin_ws_fwl/
catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/workspace/libfranka/build

if grep -Fq "source ~/workspace/catkin_ws_fwl/devel/setup.bash" ~/.bashrc; then
  echo "The source command line already exists in ~/.bashrc."
else
  echo "source ~/workspace/catkin_ws_fwl/devel/setup.bash" >> ~/.bashrc
  echo "The source command line has been appended to ~/.bashrc."
fi

if [ -f ~/.zshrc ]; then
  if grep -Fq "source ~/workspace/catkin_ws_fwl/devel/setup.zsh" ~/.zshrc; then
    echo "The source command line already exists in ~/.zshrc."
  else
    echo "source ~/workspace/catkin_ws_fwl/devel/setup.zsh" >> ~/.zshrc
    echo "The source command line has been appended to ~/.zshrc."
  fi
fi
