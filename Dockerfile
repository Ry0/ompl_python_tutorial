FROM ry0ka/ompl_python:latest

LABEL maintainer="Ryo Kabutan"

# RUN apt install -y libxkbcommon-x11-0
RUN mkdir -p ~/.config/matplotlib
RUN echo "backend : tkAgg" > ~/.config/matplotlib/matplotlibrc
# RUN echo "export DISPLAY=:1" > ~/.bashrc
