gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=2000 speed-preset=superfast ! rtph264pay config-interval=1 pt=96 ! udpsink host=100.95.74.67 port=5000
