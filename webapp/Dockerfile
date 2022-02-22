FROM ubuntu:20.04

# install Husarnet client
RUN apt update -y && \
    apt install -y curl && \
    apt install -y gnupg2 && \
    apt install -y systemd && \
    curl https://install.husarnet.com/install.sh | bash

RUN update-alternatives --set ip6tables /usr/sbin/ip6tables-nft

# legacy xtables support for ipv4 and ipv6
RUN update-alternatives --set iptables /usr/sbin/iptables-legacy
RUN update-alternatives --set ip6tables /usr/sbin/ip6tables-legacy

# install webserver service
RUN apt install -y nginx

# some optional modules
RUN apt install -y vim
RUN apt install -y iputils-ping

# Find your JOINCODE at https://app.husarnet.com
ENV JOINCODE=""
ENV HOSTNAME=my-container-1

# HTTP PORT
EXPOSE 80

# copy project files into the image
COPY init-container.sh /opt
RUN chmod +x /opt/init-container.sh

# initialize a container
CMD /opt/init-container.sh

