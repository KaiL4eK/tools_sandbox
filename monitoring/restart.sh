#!/bin/bash

sudo systemctl restart influxdb telegraf grafana-server
sudo systemctl status influxdb telegraf grafana-server
