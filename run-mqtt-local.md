on mac-os terminal run

>mosquitto -c /opt/homebrew/etc/mosquitto/mosquitto.conf -v

make sure that the conf file at `/opt/homebrew/etc/mosquitto/mosquitto.conf` already set to the 1883 port and set anonymous connection to true
use 
> sudo nano /opt/homebrew/etc/mosquitto/mosquitto.conf

to make changes in the configuration file.

set to

> listener 1883 \
> allow_anonymous true