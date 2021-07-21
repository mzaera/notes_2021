#!/usr/bin/env bash
RUNTIME="runc"
GPU_OPTS=
POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"
case $key in
    -n|--nvidia)
    RUNTIME="nvidia"
    GPU_OPTS="-e NVIDIA_DRIVER_CAPABILITIES=all --gpus all "
    shift
    ;;
    *)    # unknown option
    POSITIONAL+=("$1")
    shift
    ;;
esac
done
set -- "${POSITIONAL[@]}"
# if [ $# -lt 0]
# then
#     echo "Usage: $0 [-n --nvidia] <docker image> [<dir with workspace> ...]"
#     exit 1
# fi
USERID=$(id -u)
GROUPID=$(id -g)
docker run -it \
 --name=ingeniarius-simulators \
 -e USER=developer \
 -e PASSWORD=developer \
 -e QT_X11_NO_MITSHM=1 \
 -e XAUTHORITY \
 -e DISPLAY \
 -v "/etc/localtime:/etc/localtime:ro" \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 --privileged --net=host --pid=host --ipc=host \
 --runtime=$RUNTIME \
 $GPU_OPTS \
 -u $USERID:$GROUPID \
 ingeniarius/ingeniarius-simulators