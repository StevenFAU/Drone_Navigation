`ardupilot-install-prereqs-ubuntu.sh` is an unmodified copy of
`Tools/environment_install/install-prereqs-ubuntu.sh` from ArduPilot master
(https://github.com/ArduPilot/ardupilot, GPLv3), taken 2026-11-06. ArduPilot 4.6.3's
own copy doesn't support Ubuntu 26.04. `setup/01_install_system.sh` copies it into
`~/ardupilot/Tools/environment_install/` before running it.
