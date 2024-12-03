#!/bin/bash
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-12-01
# modified: 2024-12-03
#

echo "installing…"

sudo apt update
sudo apt -y upgrade
sudo apt -y autoremove
sudo apt -y install git tcsh vim i2c-tools
sudo apt -y install python3-pip
sudo apt -y install python3-numpy
sudo pip3 install icm20948 --break-system-packages
sudo pip3 install -r requirements.txt --break-system-packages
git config --global user.email "ichiro.furusato@gmail.com"
git config --global user.name "Ichiro Furusato"

echo "complete."
