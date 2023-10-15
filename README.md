Pull the PX4 Ubuntu 22.04 dev image

```
docker pull px4io/px4-dev-base-jammy
```

This workspace creates a container overlays the PX4 firmware for sil.
Clone the firmware with
```

cd firmware
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
```

Build the Docker images.

```
cd ..
docker compose build
```

To use the dev container launch it using
```
# Start the dev container
docker compose up dev

# Open as many interactive shells as you want to the container
docker compose exec -it dev bash
```

To run PX4 in a new container shell
```
cd firmware/PX4-Autopilot
make px4_sitl gz_x500
```

To run uXRCE-DDS middleware (expose uORD to ros2), run in a new terminal
```
docker compose exec -it dev bash
MicroXRCEAgent udp4 -p 8888
```
