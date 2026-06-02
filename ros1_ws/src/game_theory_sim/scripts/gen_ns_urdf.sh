#!/usr/bin/env bash
# Genera l'URDF del turtlebot3 iniettando <robotNamespace> nei plugin gazebo
# (diff_drive + laser) cosi' i topic diventano /<ns>/cmd_vel e /<ns>/scan.
# Uso: gen_ns_urdf.sh <model> <ns>     (stampa l'URDF su stdout)
set -e
MODEL="$1"
NS="$2"
URDF="$(rospack find turtlebot3_description)/urdf/turtlebot3_${MODEL}.urdf.xacro"

xacro --inorder "$URDF" | sed -E \
  -e "s#(<plugin[^>]*libgazebo_ros_diff_drive\.so[^>]*>)#\1<robotNamespace>/${NS}</robotNamespace>#" \
  -e "s#(<plugin[^>]*libgazebo_ros_laser\.so[^>]*>)#\1<robotNamespace>/${NS}</robotNamespace>#"
