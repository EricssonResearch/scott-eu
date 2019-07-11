FROM ubuntu:18.04

# update/upgrade base system
RUN apt-get update && apt-get -yq upgrade

# install python, pip
RUN apt-get install -yq python python-pip && pip install --upgrade pip

# COPY mission2plan.py file to docker image
RUN mkdir /opt/warehousecontroller
WORKDIR /opt/warehousecontroller
COPY mission2plan.py .
COPY requirements.txt .
COPY kb.json .
COPY mission.json .
COPY whdomain-2.pddl .

# INSTALL requirements.txt
RUN pip install -r requirements.txt

# expose port
EXPOSE 5000

# start flask application
CMD python mission2plan.py
