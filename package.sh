#!/bin/bash

set -e

root=$(pwd)

# Jenkins sets both of these but for non-jenkins builds try to do
# the right thing...
if [ -z "$WORKSPACE" ]; then
    WORKSPACE=$root
fi

build_num="$BUILD_NUMBER"
if [ -z "$build_num" ]; then
    build_num="0"
fi

rm -rf packages
mkdir -p packages

rosdep update

function build_and_install_package {
  cd $WORKSPACE
  to_remove_copy=false
  if [ ! -d $1 ]; then
    echo "making directory $1 ..."
    to_remove_copy=true
    mkdir -p $1
    cp -r "include/" "launch/" "src/" "tests/" "tools/" "CMakeLists.txt" "package.xml" $1/
  fi
  rosdep install --from-paths $1 --ignore-src -y
  cd $1/
  sed -i s/$2/$3/ package.xml
  rm -rf obj-* debian build
  echo "-----------------------"
  echo "Building $1"
  echo "-----------------------"
  bloom-generate rosdebian .
  fakeroot make -d -f debian/rules binary
  rm -rf obj-* debian build
  cd $WORKSPACE
  mv *.deb packages
  echo "-----------------------"
  echo "Installing $1 (and other packages)"
  echo "-----------------------"
  sudo dpkg -i packages/*.deb
  if [ "$to_remove_copy" = true ]; then
    rm -rf $1
    echo "removed directory $1"
  fi
}

build_and_install_package "hmi_can_publisher" "0.0.0" "0.1.$build_num"
