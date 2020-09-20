# Processing image data with OpenCV using C++
go to opendlv-perception-dev-cpp folder in two separate terminal:

First terminal

docker-compose -f h264-replay-viewer.yml up

Second terminal

docker build -t opendlv-perception-dev .

# docker run --rm -ti --init --net=host --ipc=host -v /tmp:/tmp -e DISPLAY=$DISPLAY myapp --cid=111 --name=img.argb --width=1280 --height=720 --verbose

