FROM ubuntu:16.04

# update/upgrade base system
RUN apt-get update && apt-get -yq upgrade

# install gcc, make, flex and bison
RUN apt-get install -yq make gcc wget flex bison python python-pip && pip install --upgrade pip

# download ff-metric from source
WORKDIR /opt
RUN wget --quiet https://fai.cs.uni-saarland.de/hoffmann/ff/Metric-FF-v2.0.tgz

# uncompress Metric-FF
RUN tar xfz Metric-FF-v2.0.tgz

# compile Metric-FF
WORKDIR /opt/Metric-FF-v2.0
RUN make

# copy flask-app
RUN mkdir /opt/flask-wrapper
COPY app.py /opt/flask-wrapper
COPY requirements.txt /opt/flask-wrapper
WORKDIR /opt/flask-wrapper

# INSTALL requirements.txt
RUN pip install -r requirements.txt

# expose port
EXPOSE 5000

# set path for ff_metric
ENV ff_metric_path=/opt/Metric-FF-v2.0

# set entry point
ENTRYPOINT ["python", "app.py"]
