FROM ubuntu:18.04
ARG URL=https://software-dl.ti.com/ccs/esd/CCSv9/CCS_9_2_0/exports/CCS9.2.0.00013_linux-x64.tar.gz

RUN apt-get update

RUN dpkg --add-architecture i386 && apt-get update && apt-get install -y \
  libc6:i386                    \
  libx11-6:i386                 \
  libasound2:i386               \
  libatk1.0-0:i386              \
  libcairo2:i386                \
  libcups2:i386                 \
  libdbus-glib-1-2:i386         \
  libgconf-2-4:i386             \
  libgcrypt20:i386              \
  libgdk-pixbuf2.0-0:i386       \
  libgtk-3-0:i386               \
  libice6:i386                  \
  libncurses5:i386              \
  libsm6:i386                   \
  liborbit2:i386                \
  libudev1:i386                 \
  libusb-0.1-4:i386             \
  libstdc++6:i386               \
  libstdc++6					\
  libxt6						\
  libxt6:i386                   \
  libxtst6:i386                 \
  libgnomeui-0:i386             \
  libusb-1.0-0-dev:i386         \
  libcanberra-gtk-module:i386   \
  gtk2-engines-murrine:i386     \
  libpython2.7				    \
  unzip         				\
  curl
  
RUN mkdir /var/tmp/ccs 
RUN curl -L $URL \
  | tar xvz -C /var/tmp/ccs 
  
RUN /var/tmp/ccs/CCS9.2.0.00013_linux-x64/ccs_setup_9.2.0.00013.bin --enable-components PF_MSP432 --mode unattended

RUN rm -fr /var/tmp/ccs

# workspace folder for CCS
RUN mkdir /workspace

# directory for the ccs project
VOLUME /wd
WORKDIR /wd