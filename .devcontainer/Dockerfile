ARG VARIANT
FROM --platform=linux/amd64 mcr.microsoft.com/vscode/devcontainers/cpp:${VARIANT}

ARG ARDUINO_IDE_VERSION
ARG ARDUINO_CLI_VERSION
ARG USER_NAME

RUN apt update \
    && apt install -y \
    git \
    bash-completion

# Adds the user to the group of which the group id corresponds to the host 'dialout' group id.
# If the group id doesn't exists locally, create a new group with the right group id and add the user to it.
# Add also the 'll' alias for the user and bash completion for git.
COPY ./envfile /root/
RUN . /root/envfile \
    && : "${DIALOUT_GID:=20}" \
    && rm -f /root/envfile \
    && echo "alias ll='ls -ltr'" >> /home/$USER_NAME/.bashrc \
    && echo ". /usr/share/bash-completion/completions/git" >> /home/$USER_NAME/.bashrc \
    && if grep -q ":${DIALOUT_GID}:" /etc/group; then \
        usermod -a -G "$(grep ":${DIALOUT_GID}:" /etc/group | cut -d ':' -f 1)" $USER_NAME; \
    else \
        groupadd -g "${DIALOUT_GID}" vscodedialout && usermod -a -G vscodedialout $USER_NAME; \
    fi

# Install Arduino CLI - https://github.com/arduino/arduino-cli/releases/download/v1.4.1/arduino-cli_1.4.1_Linux_64bit.tar.gz
RUN mkdir -p /opt \
    && cd /tmp \
    && wget https://github.com/arduino/arduino-cli/releases/download/v${ARDUINO_CLI_VERSION}/arduino-cli_${ARDUINO_CLI_VERSION}_Linux_64bit.tar.gz \
    && tar -xf arduino-cli_${ARDUINO_CLI_VERSION}_Linux_64bit.tar.gz \
    && rm -f arduino-cli_${ARDUINO_CLI_VERSION}_Linux_64bit.tar.gz \
    && mv arduino-cli /usr/local/bin/arduino-cli \
    && arduino-cli core install arduino:avr


# Set the default user
USER $USER_NAME

LABEL arduino-cli-version="${ARDUINO_CLI_VERSION}"
LABEL description="Image based on Debian providing Arduino build and debug environment"

# Project specific section
# ========================

# Install any extern libraries here. Example:
# Install LiquidCrystal I2C library
# RUN arduino-cli lib install "LiquidCrystal I2C"