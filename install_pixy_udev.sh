# Ensure script is run with sudo
if [ "$EUID" -ne 0 ]; then
  echo "Please run this script with sudo:"
  echo "sudo ./install_pixy_udev.sh"
  exit
fi

echo "Installing udev rule for Pixy cameras..."

# Create the udev rule file granting read/write access to everyone for Charmed Labs Pixy
cat << 'EOF' > /etc/udev/rules.d/99-pixy.rules
# Charmed Labs Pixy
SUBSYSTEM=="usb", ATTR{idVendor}=="b1ac", ATTR{idProduct}=="f000", MODE="0666"
EOF

# Reload udev rules to apply immediately
udevadm control --reload-rules
udevadm trigger

echo "udev rule installed successfully."
echo "You can now run get_raw_frame without sudo."
