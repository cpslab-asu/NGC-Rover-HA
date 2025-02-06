FROM ubuntu:22.04 AS build

RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y python3 python3-pip

RUN pip install uv

RUN mkdir /app
COPY ./pyproject.toml ./uv.lock /app
RUN uv venv \
    --project /app \
    --system-site-packages \
    --python 3.10 \
    --python-preference only-system \
    --relocatable
RUN uv sync --project /app --locked --group docker --no-install-project

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
        python3-gz-math7 \
        python3-gz-msgs10 \
        python3-gz-transport13 \
        && \
    DEBIAN_FRONTEND=noninteractive apt-get clean

COPY --from=build /app /app
COPY ./src /app/src

COPY <<EOF /usr/local/bin/controller
#!/usr/bin/bash

/app/.venv/bin/python3 /app/src/main.py \$@
EOF

RUN chmod +x /usr/local/bin/controller


CMD ["/usr/local/bin/controller"]
