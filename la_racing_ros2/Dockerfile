FROM osrf/ros:galactic-desktop

# Essential packages for remote debugging and login in
RUN apt-get update && apt-get install -y \
    apt-utils gcc g++ openssh-server cmake build-essential gdb gdbserver rsync vim

# Taken from - https://docs.docker.com/engine/examples/running_ssh_service/#environment-variables

RUN mkdir /var/run/sshd
RUN echo 'root:root' | chpasswd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# SSH login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile


WORKDIR /la_racing_ws

# MANIPULATE BASHRC
RUN sed -i '/^# If not running interactively/i source /opt/ros/galactic/setup.bash' ~/.bashrc
RUN sed -i '/^# If not running interactively/i [[ -f /la_racing_ws/install/local_setup.bash ]] && source /la_racing_ws/install/local_setup.bash' ~/.bashrc

# Start SSH Server
CMD ["/usr/sbin/sshd", "-D", "-p", "2222"]