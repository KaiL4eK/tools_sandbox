#!/bin/bash

mkdir -p pkgs; cd pkgs
wget -N https://dl.influxdata.com/telegraf/releases/telegraf_1.12.2-1_amd64.deb
wget -N https://dl.influxdata.com/influxdb/releases/influxdb_1.7.8_amd64.deb
wget -N https://dl.grafana.com/oss/release/grafana_6.3.6_amd64.deb

sudo dpkg -i *.deb

cd ../

sudo cp --backup=numbered telegraf.conf /etc/telegraf/

sudo systemctl enable influxdb telegraf grafana-server
sudo systemctl restart influxdb telegraf grafana-server
