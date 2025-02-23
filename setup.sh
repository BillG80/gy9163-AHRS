#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Starting GY91 & GY63 Sensor System Setup...${NC}"

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo -e "${RED}Please run as root (sudo)${NC}"
    exit 1
fi

# Update system
echo -e "${YELLOW}Updating system packages...${NC}"
apt-get update
apt-get upgrade -y

# Install required packages
echo -e "${YELLOW}Installing required packages...${NC}"
apt-get install -y python3-pip python3-smbus i2c-tools git

# Enable I2C if not already enabled
echo -e "${YELLOW}Enabling I2C interfaces...${NC}"
if ! grep -q "i2c-dev" /etc/modules; then
    echo "i2c-dev" >> /etc/modules
fi
if ! grep -q "i2c-bcm2708" /etc/modules; then
    echo "i2c-bcm2708" >> /etc/modules
fi

# Enable both I2C buses in config.txt
if ! grep -q "dtparam=i2c_arm=on" /boot/config.txt; then
    echo "dtparam=i2c_arm=on" >> /boot/config.txt
fi
if ! grep -q "dtparam=i2c1=on" /boot/config.txt; then
    echo "dtparam=i2c1=on" >> /boot/config.txt
fi

# Create installation directory
echo -e "${YELLOW}Creating installation directory...${NC}"
INSTALL_DIR="/opt/gy9163-sensor-system"
mkdir -p $INSTALL_DIR

# Copy files to installation directory
echo -e "${YELLOW}Copying files...${NC}"
cp -r src/* $INSTALL_DIR/
cp requirements.txt $INSTALL_DIR/
cp service/gy9163.service /etc/systemd/system/

# Set permissions
echo -e "${YELLOW}Setting permissions...${NC}"
chown -R pi:pi $INSTALL_DIR
chmod -R 755 $INSTALL_DIR

# Install Python requirements
echo -e "${YELLOW}Installing Python requirements...${NC}"
pip3 install -r requirements.txt

# Enable and start service
echo -e "${YELLOW}Enabling and starting service...${NC}"
systemctl enable gy9163.service
systemctl start gy9163.service

# Check I2C buses
echo -e "${YELLOW}Checking I2C buses...${NC}"
echo "I2C Bus 0:"
i2cdetect -y 0
echo "I2C Bus 1:"
i2cdetect -y 1

echo -e "${GREEN}Installation complete!${NC}"
echo -e "${YELLOW}Please reboot your Raspberry Pi to ensure all changes take effect.${NC}"
echo -e "${YELLOW}After reboot, check service status with: sudo systemctl status gy9163.service${NC}"

# Ask for reboot
read -p "Would you like to reboot now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]
then
    reboot
fi
