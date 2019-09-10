FROM ubuntu:18.04

RUN apt-get update -y
RUN apt-get install -y 		\
	python3-pip git npm cmake sudo

# RUN npm install -g configurable-http-proxy
# RUN pip3 install --upgrade pip

# RUN pip3 install 		\
# 	jupyterhub==0.8.1	\
# 	tornado==4.5.3		\
# 	notebook

RUN useradd --create-home --shell /bin/bash -G sudo user
RUN echo "user:1" | chpasswd

RUN apt install \
    wget unzip

USER user
WORKDIR /home/user

RUN git clone --recursive https://github.com/yjxiong/temporal-segment-networks