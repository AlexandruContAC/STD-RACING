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

# Use rsync to copy the project files to the NavQPlus.
# It excludes __pycache__, .git, and .pytest_cache to save time.
rsync -avz --exclude='__pycache__' \
           --exclude='.git' \
           --exclude='.pytest_cache' \
           --exclude='.agents' \
           --exclude='deploy.sh' \
           --exclude='get_raw_frame/get_raw_frame' \
           --exclude='get_raw_frame/*.o' \
           --exclude='pixy2/build' \
           --exclude='pixy2/src/host/libpixyusb2/src/obj/*.o' \
           --exclude='pixy2/src/host/libpixyusb2/src/lib/*.a' \
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
