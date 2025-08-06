# Create entry script that sources all environments
# !/bin/bash

set -e

source /opt/ros/$ROS_DISTRO/setup.bash

echo "Provided arguments: $@"

exec $@

# echo '#!/bin/bash\n\
# source /opt/ros/$ROS_DISTRO/setup.bash\n\
# source /opt/microros_ws/install/local_setup.bash\n\
# export PATH=$PATH:~/.platformio/penv/bin:~/.local/bin\n\
# exec "$@"' > /home/$USERNAME/entrypoint.sh && \
#     chmod +x /home/$USERNAME/entrypoint.sh
# At start up of container we always sourcing ros components