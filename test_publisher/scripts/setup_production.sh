#!/bin/bash

# Exit on any error
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Build web client
echo "Building web client..."
cd "$PROJECT_ROOT/web-client"
npm install
npm run build

# Create web root directory and copy built files
echo "Setting up web files..."
sudo mkdir -p /var/www/ros2-web-client
sudo cp -r "$PROJECT_ROOT/web-client/dist/"* /var/www/ros2-web-client/

# Setup Nginx configuration
echo "Setting up Nginx configuration..."
sudo cp "$PROJECT_ROOT/config/nginx/ros2-web-client.conf" /etc/nginx/sites-available/
sudo ln -sf /etc/nginx/sites-available/ros2-web-client.conf /etc/nginx/sites-enabled/
sudo nginx -t
sudo systemctl restart nginx

# Setup systemd services
echo "Setting up systemd services..."
mkdir -p ~/.config/systemd/user/
cp "$PROJECT_ROOT/config/systemd/"*.service ~/.config/systemd/user/

# Reload systemd
systemctl --user daemon-reload

# Enable services
systemctl --user enable ros2-random-publisher
systemctl --user enable ros2-web-bridge

echo "Setup complete! You can now use 'ros2 launch test_publisher test_system_prod.launch.py' to start the system"