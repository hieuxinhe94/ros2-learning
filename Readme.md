<code>
# docker image build -t my_ros .

# docker run -it --user ros --env="DISPLAY=$DISPLAY"  --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --privileged  --network=host --ipc=host -v $PWD/source:/source my_ros:latest bash

 </code>
