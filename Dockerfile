FROM ubuntu:22.04 AS build

RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        python3 \
        python3-dev \
        python3-venv \
        python3-pip

RUN python3 -m venv /opt/pipenv
RUN /opt/pipenv/bin/pip3 install pipenv

WORKDIR /tmp
COPY ./Pipfile ./Pipfile.lock .

ENV PIPENV_VENV_IN_PROJECT=1 
RUN /opt/pipenv/bin/pipenv sync --python /usr/bin/python3 --site-packages

FROM ubuntu:22.04

RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y lsb-release gnupg curl

ENV OSRF_GPG_PATH=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output $OSRF_GPG_PATH
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=$OSRF_GPG_PATH] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list
ENV OSRF_GPG_PATH=

RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        python3 \
        python3-gz-transport13 \
        python3-gz-msgs10 \
        && \
    DEBIAN_FRONTEND=noninteractive apt-get clean

RUN mkdir /app
ADD --keep-git-dir=false https://github.com/px4/px4-gazebo-models.git /app/resources
COPY --from=build /tmp/.venv /app/.venv
COPY ./src /app/src

WORKDIR /app
CMD ["/app/.venv/bin/python3", "/app/src/main.py"]
