#!/bin/bash
var="$(xxd -ps /sys/class/i2c-adapter/i2c-0/of_node/clock-frequency)"
var=${var//[[:blank:].\}]/}
printf "%d\n" 0x$var
