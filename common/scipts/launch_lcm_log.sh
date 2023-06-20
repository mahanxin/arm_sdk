#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
# cd ${DIR}/../lcm_msgs/java
export CLASSPATH=${DIR}/../lcm_msgs/java/my_types.jar
pwd

if [ $# == 0 ];then
  com=7666
else
  com=$1
fi

lcm-logger --lcm-url=udpm://239.255.76.67:$com?ttl=255
