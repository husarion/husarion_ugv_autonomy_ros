# 🚀 Simulation Demo

This is a basic demo that's a good place to start when preparing your robot for navigation, so you can familiarize yourself with the basic concepts in this repository. When launched in this example, The Panther will be equipped with the following devices, which were added using the [configuration.yaml](./docker/config/components.yaml) file.

- Velodyne Puck
- RPLIDAR S3
- Astra Camera
- WiBotic receiver

## 📋 Requirements

- **Just**: to simplify the execution of this project, we are utilizing [just](https://github.com/casey/just). Install it with:

```bash
sudo snap install just
```

## 🧭 Run Autonomy - Navigation + Docking

### Step 1: Run navigation

Run navigation on your laptop in `husarion_ugv_autonomy_ros` directory:

```bash
just start-simulation
```

### Step 2: Control the robot from a Web Browser

1. Install and run husarion-webui

    ```bash
    just start-visualization
    ```

2. Open the your browser on your laptop and navigate to:

    - <http://localhost:8080/ui>
    - http://{ip_address}:8080/ui (devices in the same LAN)

### Step 3: Docking

To dock the robot execute:

```bash
just dock main
```

To undock the robot execute:

```bash
just undock
```

## ✅ Next Steps

After reviewing this demonstration, we encourage you to experiment and familiarize yourself with the autonomous functionality and use this knowledge to configure the physical robot accordingly. Check:

- [compose.simulation.yaml](./docker/compose.simulation.yaml)
- [Hardware Demo](demo-hw.md)
