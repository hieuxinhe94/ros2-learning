<code>
 docker image build -t my_ros .
 docker run -it --user ros --network=host --ipc=host -v $PWD/source:/source my_ros:latest bash
 </code>
