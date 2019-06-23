#!/bin/bash

PORT=5000
DEVICE=/dev/video0
export GST_DEBUG_DUMP_DOT_DIR=dot
rm -rf $GST_DEBUG_DUMP_DOT_DIR; mkdir -p $GST_DEBUG_DUMP_DOT_DIR

# videotestsrc
# gst-launch-1.0 v4l2src device=$DEVICE ! videoconvert ! \
				# videoscale ! video/x-raw,width=320,height=240 ! \
				# clockoverlay shaded-background=true font-desc="Sans 38" ! \
				# theoraenc ! oggmux ! tcpserversink host=0.0.0.0 port=8080

gst-launch-1.0 -v v4l2src device=$DEVICE ! "video/x-raw,width=640,framerate=30/1" ! videoconvert n-threads=8 ! \
        x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay pt=96 ! udpsink host=0.0.0.0 port=$PORT

# MESON_BUILD_ROOT="${MESON_BUILD_ROOT:-build}"
INPUT_DIR="${INPUT_DIR:-dot}"

if [ -d "$INPUT_DIR" ]; then
    DOT_FILES=`find $INPUT_DIR -name "*.dot"`
    for file in $DOT_FILES
    do
        dest=`sed s/.dot/.pdf/ <<< "$file"`
        dot -Tpdf $file > $dest
    done
else
    echo "Input directory $INPUT_DIR does not exist"
fi

exit

gst-launch-1.0 -v v4l2src device=$DEVICE ! videoconvert ! autovideosink
gst-launch-1.0 -v v4l2src device=$DEVICE ! videoconvert ! fpsdisplaysink
gst-launch-1.0 -v v4l2src device=$DEVICE ! jpegdec ! fpsdisplaysink
gst-launch-1.0 -v v4l2src device=$DEVICE do-timestamp=true num-buffers=300 ! jpegdec ! fpsdisplaysink

gst-launch-1.0 -v v4l2src device=$DEVICE ! jpegdec ! fpsdisplaysink

gst-launch-1.0 -v v4l2src device=$DEVICE ! "video/x-raw,width=960,framerate=15/1" ! videoconvert n-threads=8 ! fpsdisplaysink sync=false
gst-launch-1.0 -v v4l2src device=$DEVICE ! "image/jpeg, width=1280, framerate=30/1" ! jpegparse ! jpegdec ! fpsdisplaysink sync=false
# gst-launch-1.0 -v v4l2src device=$DEVICE ! "image/jpeg, width=640, framerate=30/1" ! jpegparse ! multipartmux ! tcpserversink host=0.0.0.0 port=5000
gst-launch-1.0 -v v4l2src device=$DEVICE ! "image/jpeg, width=640, framerate=30/1" ! jpegparse ! tcpserversink host=0.0.0.0 port=5000
gst-launch-1.0 -v v4l2src device=$DEVICE ! "image/jpeg, width=640, framerate=30/1" ! jpegparse ! rtpjpegpay ! udpsink host=0.0.0.0 port=$PORT

gst-launch-1.0 -v v4l2src device=$DEVICE ! "video/x-raw,width=640,framerate=30/1" ! videoconvert n-threads=8 ! \
        x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay pt=96 ! udpsink host=0.0.0.0 port=$PORT

gst-launch-1.0 -v v4l2src device=$DEVICE ! "video/x-raw,width=640,framerate=30/1" ! videoconvert n-threads=8 ! \
        x264enc tune=zerolatency bitrate=500 speed-preset=superfast threads=8 ! \
        tcpserversink host=0.0.0.0 port=$PORT recover-policy=keyframe sync-method=latest-keyframe

# Clients
gst-launch-1.0 tcpclientsrc port=$PORT ! h264parse ! avdec_h264 ! fpsdisplaysink sync=false

gst-launch-1.0 -v udpsrc port=$PORT ! \
    "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! \
    rtph264depay ! avdec_h264 ! videoconvert ! fpsdisplaysink

gst-launch-1.0 udpsrc port=$PORT ! application/x-rtp,encoding-name=JPEG ! rtpjpegdepay ! jpegdec ! fpsdisplaysink
gst-launch-1.0 tcpclientsrc port=$PORT ! application/x-rtp,encoding-name=JPEG ! rtpjpegdepay ! jpegdec ! fpsdisplaysink
cvlc tcp://127.0.0.1:5000