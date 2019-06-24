#!/bin/bash

export GST_ADDR=192.168.31.90
export GST_PORT=5000
export GST_DEVICE=/dev/video0
export GST_DEBUG_DUMP_DOT_DIR=dot
rm -rf $GST_DEBUG_DUMP_DOT_DIR; mkdir -p $GST_DEBUG_DUMP_DOT_DIR

# videotestsrc
# gst-launch-1.0 v4l2src device=$DEVICE ! videoconvert ! \
				# videoscale ! video/x-raw,width=320,height=240 ! \
				# clockoverlay shaded-background=true font-desc="Sans 38" ! \
				# theoraenc ! oggmux ! tcpserversink host=0.0.0.0 port=8080

CAMERA_SOURCE_MJPEG="v4l2src device=$GST_DEVICE ! \"image/jpeg, width=640, height=480, framerate=30/1\""
DISPLAY_SINK="fpsdisplaysink sync=false"
TCP_SINK="tcpserversink port=$GST_PORT"
TCP_SINK="udpsink port=$GST_PORT"

# gst-launch-1.0 -v v4l2src device=$DEVICE ! "video/x-raw,width=640,framerate=30/1" ! videoconvert n-threads=8 ! \
        # x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay pt=96 ! udpsink host=0.0.0.0 port=$PORT

# gst-launch-1.0 -v $CAMERA_SOURCE_MJPEG ! jpegparse ! $DISPLAY_SINK


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

# gst-launch-1.0 -v v4l2src device=$GST_DEVICE ! videoconvert ! autovideosink
# gst-launch-1.0 -v v4l2src device=$GST_DEVICE ! videoconvert ! fpsdisplaysink
# gst-launch-1.0 -v v4l2src device=$GST_DEVICE ! jpegdec ! fpsdisplaysink
# gst-launch-1.0 -v v4l2src device=$GST_DEVICE do-timestamp=true num-buffers=300 ! jpegdec ! fpsdisplaysink

# gst-launch-1.0 -v v4l2src device=$GST_DEVICE ! jpegdec ! fpsdisplaysink

# gst-launch-1.0 -v v4l2src device=$GST_DEVICE ! "image/jpeg, width=1280, framerate=30/1" ! jpegparse ! jpegdec ! fpsdisplaysink sync=false
# gst-launch-1.0 -v v4l2src device=$DEVICE ! "image/jpeg, width=640, framerate=30/1" ! jpegparse ! multipartmux ! tcpserversink host=0.0.0.0 port=5000
# gst-launch-1.0 -v v4l2src device=$GST_DEVICE ! "image/jpeg, width=640, height=480, framerate=30/1" ! jpegparse ! rtpjpegpay ! udpsink host=0.0.0.0 port=$GST_PORT

# gst-launch-1.0 -v v4l2src device=$GST_DEVICE ! "video/x-raw,width=640,framerate=30/1" ! videoconvert ! \
#         x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay pt=96 ! udpsink host=0.0.0.0 port=$GST_PORT

# gst-launch-1.0 -v v4l2src device=$GST_DEVICE ! "video/x-raw,width=640,framerate=30/1" ! videoconvert ! \
#         x264enc tune=zerolatency bitrate=500 speed-preset=superfast threads=8 ! \
#         tcpserversink host=0.0.0.0 port=$GST_PORT recover-policy=keyframe sync-method=latest-keyframe



# Test
# gst-launch-1.0 -v videotestsrc ! "video/x-raw,width=640, height=480, framerate=30/1" ! videoconvert ! fpsdisplaysink sync=false
# gst-launch-1.0 -v $CAMERA_SOURCE_MJPEG ! jpegparse ! $TCP_SINK
# gst-launch-1.0 -v $CAMERA_SOURCE_MJPEG ! jpegparse ! $DISPLAY_SINK


# UDP
gst-launch-1.0 -v v4l2src device=$GST_DEVICE ! "video/x-raw,width=640,framerate=30/1" ! videoconvert ! \
    x264enc tune=zerolatency bitrate=5000 speed-preset=superfast ! \
    rtph264pay pt=96 ! udpsink host=192.168.31.76 port=$GST_PORT

gst-launch-1.0 -v v4l2src device=$GST_DEVICE ! "video/x-raw,width=640,framerate=30/1" ! videoconvert ! \
    jpegenc ! \
    rtpjpegpay pt=96 ! udpsink host=192.168.31.76 port=$GST_PORT

# TCP
gst-launch-1.0 -v v4l2src device=$GST_DEVICE ! "video/x-raw,width=640, height=480, framerate=30/1" ! videoconvert ! \
    jpegenc ! \
    tcpserversink host=0.0.0.0 port=$GST_PORT recover-policy=keyframe sync-method=latest-keyframe

gst-launch-1.0 -v v4l2src device=$GST_DEVICE ! "video/x-raw,width=640, height=480, framerate=30/1" ! videoconvert ! \
    x264enc tune=zerolatency bitrate=5000 speed-preset=superfast ! \
    tcpserversink host=0.0.0.0 port=$GST_PORT recover-policy=keyframe sync-method=latest-keyframe

gst-launch-1.0 -v videotestsrc ! "video/x-raw,width=640, height=480, framerate=30/1" ! videoconvert ! \
    jpegenc ! \
    tcpserversink host=0.0.0.0 port=$GST_PORT recover-policy=keyframe sync-method=latest-keyframe

gst-launch-1.0 -v videotestsrc ! "video/x-raw,width=640, height=480, framerate=30/1" ! videoconvert ! \
    x264enc tune=zerolatency bitrate=2000 speed-preset=superfast ! \
    tcpserversink host=0.0.0.0 port=$GST_PORT recover-policy=keyframe sync-method=latest-keyframe


# Clients
# TCP: MJPEG faster than H264
gst-launch-1.0 -v tcpclientsrc host=$GST_ADDR port=$GST_PORT ! h264parse ! avdec_h264 ! fpsdisplaysink sync=false
gst-launch-1.0 -v tcpclientsrc host=$GST_ADDR port=$GST_PORT ! jpegparse ! jpegdec ! fpsdisplaysink sync=false

# UDP
gst-launch-1.0 -v udpsrc port=$GST_PORT ! \
    "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! \
    rtph264depay ! avdec_h264 ! fpsdisplaysink sync=false

gst-launch-1.0 -v udpsrc port=$GST_PORT ! \
    "application/x-rtp,encoding-name=JPEG" ! \
    rtpjpegdepay ! jpegdec ! fpsdisplaysink sync=false

# cvlc tcp://127.0.0.1:5000