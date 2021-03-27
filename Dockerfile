# どのイメージを基にするか
FROM ubuntu

ENV PYTHON_VER="3.8"
ENV PYTHON_FULL_VER="3.8.5"
ENV PYTHON_ENV_NAME="ompl"

ENV DEV="git curl wget build-essential gcc g++ make"
ENV DEBIAN_FRONTEND=noninteractive

LABEL maintainer="Ryo Kabutan"

RUN echo "ompl building..." &&\
    apt-get update && \
    apt-get -y install $DEV &&\
    mkdir /tmp/ompl_sources && \
    cd /tmp/ompl_sources && \
    wget https://raw.githubusercontent.com/ompl/ompl/main/install-ompl-ubuntu.sh.in -O install-ompl-ubuntu.sh && \
    chmod u+x install-ompl-ubuntu.sh && \
    ./install-ompl-ubuntu.sh --python -g
RUN echo "pyenv setupping ..." &&\
    git clone https://github.com/pyenv/pyenv.git ~/.pyenv &&\
    echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc &&\
    echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc && \
    echo 'if command -v pyenv 1>/dev/null 2>&1; then\n  eval "$(pyenv init -)"\nfi' >> ~/.bashrc && \
    git clone https://github.com/pyenv/pyenv-virtualenv.git ~/.pyenv/plugins/pyenv-virtualenv && \
    echo 'eval "$(pyenv virtualenv-init -)"' >> ~/.bashrc
RUN echo "python installing ..." &&\
    apt install -y libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev llvm libncurses5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev && \
    ~/.pyenv/bin/pyenv install $PYTHON_FULL_VER && \
    ~/.pyenv/bin/pyenv virtualenv $PYTHON_FULL_VER $PYTHON_ENV_NAME
RUN ln -s /usr/lib/python3/dist-packages/ompl ~/.pyenv/versions/$PYTHON_ENV_NAME/lib/python$PYTHON_VER/site-packages/ompl && \
    ~/.pyenv/versions/$PYTHON_ENV_NAME/bin/python -m pip install --upgrade pip && \
    ~/.pyenv/versions/$PYTHON_ENV_NAME/bin/python -m pip install numpy pyplusplus pygccxml autopep8 flake8 pytest