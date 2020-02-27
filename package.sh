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

cat > ${WORKSPACE}/plusai_hmi_can_publisher.yaml <<EOF
can_common:
  ubuntu: [ros-kinetic-can-common]
plusai_msgs:
  ubuntu: [ros-kinetic-plusai-msgs]
EOF

cat > ${WORKSPACE}/plusai_hmi_can_publisher.list <<EOF
yaml file:////${WORKSPACE}/plusai_hmi_can_publisher.yaml
EOF

# Put this first so our packages get found first.
sudo cp plusai_hmi_can_publisher.list /etc/ros/rosdep/sources.list.d/00-plusai_hmi_can_publisher.list

rosdep update

cd code

old_ver="0.1.0"
new_ver="0.1.$build_num"
sed -i s/$old_ver/$new_ver/ package.xml

bloom-generate rosdebian --os-name ubuntu \
                         --os-version xenial \
                         --ros-distro kinetic .
 
fakeroot make -d -f debian/rules binary
