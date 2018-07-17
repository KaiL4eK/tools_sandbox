FROM ubuntu:16.04

RUN apt-get update -y
RUN apt-get install -y 		\
	python3-pip

RUN pip3 install --upgrade pip

RUN pip3 install 		\
	jupyterhub==0.8.1	\
	tornado==4.5.3		\
	notebook


