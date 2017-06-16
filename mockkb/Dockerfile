FROM ubuntu:16.10
LABEL maintainer "konstantinos.vandikas@ericsson.com"

# update/upgrade base system
RUN apt-get update
RUN apt-get -yq upgrade

# install python, pip
RUN apt-get install -yq python python-pip

# upgrade pip
RUN pip install --upgrade pip

# COPY app.py file to docker image
RUN mkdir /opt/mockkb
WORKDIR /opt/mockkb
COPY app.py .
COPY requirements.txt .

# INSTALL requirements.txt
RUN pip install -r requirements.txt

# expose port
EXPOSE 5001

# ENTRYPOINT
ENTRYPOINT ["python", "app.py"]
