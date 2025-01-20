FROM ghcr.io/astral-sh/uv:debian AS build

RUN mkdir /app
COPY ./pyproject.toml ./uv.lock /app
RUN uv venv --project /app --system-site-packages --python /usr/bin/python3 --relocatable
RUN uv sync --project /app --locked 

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

COPY --from=build /app /app
COPY ./src /app/src
ADD --keep-git-dir=false https://github.com/px4/px4-gazebo-models.git /app/resources

CMD ["/app/.venv/bin/controller"]
