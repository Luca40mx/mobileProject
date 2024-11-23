FROM ubuntu:20.04
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update 

# ROS2 domain_id 30 ports interval
EXPOSE 12400-12431
EXPOSE 11811/udp

# Workspace setup
RUN mkdir /workspace
COPY ./scripts /workspace/scripts

WORKDIR /workspace
RUN chmod u+x ./scripts/*.sh

# Run installers
RUN ./scripts/install_mono.sh
RUN ./scripts/install_ros2_foxy.sh
RUN ./scripts/install_unityHub.sh

RUN ln -s /usr/lib/x86_64-linux-gnu/libpython3.8.so.1.0 /usr/lib/x86_64-linux-gnu/libpython3.6m.so.1.0

ADD entrypoint.sh /entrypoint.sh
RUN chmod u+x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

EXPOSE 7000-60000