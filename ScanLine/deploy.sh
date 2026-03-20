#!/bin/bash

# Configuration
# Change 'navq' to the actual username (e.g., navq) if different
# Change 'navqplus.local' to the actual IP address or hostname of your NavQPlus
REMOTE_USER="user"
REMOTE_HOST="stdracing.local"
REMOTE_DIR="/home/$REMOTE_USER/ScanLine"

# Ensure we are in the project directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

echo "Deploying ScanLine to $REMOTE_USER@$REMOTE_HOST..."

# Use rsync with an INCLUDE-only approach.
# Only the files needed for runtime are sent — the 1.6 GB pixy2/ SDK,
# tests, .git, docs, etc. are all excluded.
rsync -avz \
    --include='main.py' \
    --include='config.py' \
    --include='visualization.py' \
    --include='synapse_msgs.py' \
    --include='synapse_tinyframe.py' \
    --include='foxglove_server.py' \
    --include='pixy2.py' \
    --include='requirements.txt' \
    --include='camera/' \
    --include='camera/__init__.py' \
    --include='camera/base.py' \
    --include='camera/pixy2_cam.py' \
    --include='camera/pixy2fast_cam.py' \
    --include='camera/webcam.py' \
    --include='camera/mipi_cam.py' \
    --include='detection/' \
    --include='detection/__init__.py' \
    --include='detection/scanline.py' \
    --include='detection/lidar.py' \
    --include='processing/' \
    --include='processing/__init__.py' \
    --include='processing/pipeline.py' \
    --include='steering/' \
    --include='steering/__init__.py' \
    --include='steering/controller.py' \
    --include='get_raw_frame/' \
    --include='get_raw_frame/get_raw_frame' \
    --exclude='*' \
    ./ "$REMOTE_USER@$REMOTE_HOST:$REMOTE_DIR"

if [ $? -eq 0 ]; then
    echo "================================================="
    echo "Deployment successful!"
    echo "To run the project on the NavQPlus, SSH into it:"
    echo "    ssh $REMOTE_USER@$REMOTE_HOST"
    echo "Then run:"
    echo "    cd ScanLine"
    echo "    python3 main.py"
    echo "================================================="
else
    echo "Deployment failed."
    exit 1
fi
