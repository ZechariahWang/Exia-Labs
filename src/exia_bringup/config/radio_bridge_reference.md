# Radio Bridge Reference (Archived)

The radio bridge (`radio_bridge_node`) was the original transport layer between the
base station laptop and the ATV robot, using RFD900x radios over FTDI USB serial.

## Hardware
- Radio: RFD900x (900MHz band)
- Interface: FTDI USB serial (VID 0403:6001) -> `/dev/exia_radio`
- Baud: 57600
- Udev rule: `scripts/99-exia-radio.rules`

## Protocol
- RSA-2048 handshake + AES-256-GCM session encryption
- Base generates random AES key, sends via RSA-encrypted fragments ($K1/$K2)
- Robot decrypts, ACKs with SHA-256 hash ($A)
- All subsequent frames: `$` + base64(AES-GCM encrypted payload) + `\n`
- Nonce counters: even=base, odd=robot (monotonic, anti-replay)

## Message Types
| Type | Dir | Purpose |
|------|-----|---------|
| H | base->robot | Heartbeat (10Hz) with seq number |
| A | robot->base | Heartbeat ACK |
| N | base->robot | Navigation goal (all coord_types) |
| C | base->robot | Cancel navigation |
| E | base->robot | E-stop activate |
| X | base->robot | E-stop clear |
| V | base->robot | cmd_vel relay (linear.x, angular.z) |
| D | base->robot | Teleop relay (throttle, brake, steer) |
| G | base->robot | Gear command |
| S | robot->base | Telemetry at 5Hz (state, pos, heading, speed, gps, imu) |
| R | robot->base | Navigation status relay |
| I | robot->base | Image chunks (JPEG, base64, reassembled on base) |
| EA | robot->base | E-stop acknowledgment |

## Safety
- Robot triggers e-stop if no heartbeat for 1.5s
- Base resets handshake after 30 missed ACKs (3s at 10Hz)
- E-stop: zero cmd_vel at 20Hz, cancel active navigation
- Heartbeat-loss e-stop auto-clears when heartbeats resume

## Config
- `src/exia_bringup/config/radio_params.yaml`
- Key dir: `~/.exia/` (radio_private.pem, radio_public.pem)
- Key gen: `scripts/generate_radio_keys.sh`

## Launch
```bash
ros2 launch exia_bringup radio.launch.py role:=robot
ros2 launch exia_bringup radio.launch.py role:=base
```

## Source
- `src/exia_driver/exia_driver/radio_bridge_node.py` (preserved, not deleted)
- `src/exia_bringup/launch/radio.launch.py` (preserved)

## Why Replaced
The radio frequency (900MHz) was blocked in the operating area. Replaced with
Starlink/WiFi IP network using native ROS 2 DDS (CycloneDDS unicast).
