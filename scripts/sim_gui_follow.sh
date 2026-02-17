#!/bin/zsh

# Set gui camera to follow target in gz-sim

# Define the tracking parameters
track_mode=2
follow_pgain=0.1

# The name of the entity to follow
follow_target_name="scorpio"

# Offset from the followed entity (in meters)
follow_offset_x=0.0
follow_offset_y=0.0
follow_offset_z=0.6

# Publish the CameraTrack message to control camera behavior
gz topic -t /gui/track \
  -m gz.msgs.CameraTrack \
  -p "track_mode: $track_mode, \
      follow_target: {name: \"$follow_target_name\"}, \
      follow_offset: {x: $follow_offset_x, y: $follow_offset_y, z: $follow_offset_z}, \
      follow_pgain: $follow_pgain"
