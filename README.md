Camera driver
```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:="/dev/video0" \
  -p pixel_format:="mjpeg2rgb" \
  -p image_width:=640 \
  -p image_height:=480
```

Aruco detectore
```bash
ros2 run aruco_opencv aruco_tracker_autostart --ros-args \
  -p cam_base_topic:=/image_raw \
  -p marker_size:=0.1
```