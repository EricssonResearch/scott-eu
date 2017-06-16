FROM ubuntu:16.10
LABEL maintainer "konstantinos.vandikas@ericsson.com"

# update/upgrade base system
RUN apt-get update
RUN apt-get -yq upgrade

# install python, pip
RUN apt-get install -yq python python-pip

# upgrade pip
RUN pip install --upgrade pip

# COPY mission2plan.py file to docker image
RUN mkdir /opt/warehousecontroller
WORKDIR /opt/warehousecontroller
COPY mission2plan.py .
COPY requirements.txt .
COPY kb.json .
COPY mission.json . 

# INSTALL requirements.txt
RUN pip install -r requirements.txt

# expose port
EXPOSE 5000

# start flask application
CMD python mission2plan.py
