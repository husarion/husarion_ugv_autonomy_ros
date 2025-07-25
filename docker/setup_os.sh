#!/bin/bash

set -e

# 1. Create sysctl configuration for increased buffers
SYSCTL_CONF="/etc/sysctl.d/10-cyclone-max.conf"

echo "Creating $SYSCTL_CONF..."
cat <<EOF | sudo tee "$SYSCTL_CONF"
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
net.core.rmem_max=2147483647
EOF

echo "Applying sysctl settings..."
sudo sysctl --system

# 2. Create CAN interface setup script
SETUP_CAN_SH="/usr/local/sbin/setup_can0.sh"

echo "Creating $SETUP_CAN_SH..."
cat <<'EOF' | sudo tee "$SETUP_CAN_SH"
#!/bin/bash
slcand -o -s6 -t hw -S 3000000 /dev/ttyACM0 can0
ip link set up can0
EOF

sudo chmod +x "$SETUP_CAN_SH"

# 3. Create systemd service
SERVICE_FILE="/etc/systemd/system/setup_can0.service"

echo "Creating $SERVICE_FILE..."
cat <<EOF | sudo tee "$SERVICE_FILE"
[Unit]
Description=Enable CAN port for Husarion UGV
After=syslog.target network.target multi-user.target user@1000.service

[Service]
Type=simple
ExecStart=$SETUP_CAN_SH
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

# 4. Enable and optionally start the service
echo "Reloading systemd and enabling setup_can0.service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable setup_can0.service

# Optional: start service now
sudo systemctl start setup_can0.service

echo "✅ Configuration completed."
