make a file named setup-jetson-share.sh

paste this inside

#!/bin/bash

IFACE=$(nmcli -t -f DEVICE,TYPE device status | awk -F: '$2=="ethernet"{print $1; exit}')

sudo nmcli connection delete Jetson-Internet-Share 2>/dev/null

sudo nmcli connection add \
  type ethernet \
  ifname "$IFACE" \
  con-name Jetson-Internet-Share \
  ipv4.method shared \
  ipv6.method disabled

sudo nmcli connection modify Jetson-Internet-Share connection.autoconnect yes

sudo nmcli connection up Jetson-Internet-Share


Connect the ethernet cable, and run that file using ./setup-jetson-share.sh\
then run 

ssh crabbot@10.42.0.50
pass: toor