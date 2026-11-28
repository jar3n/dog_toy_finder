# micro-ROS UDP transport

`picow_udp_transport.c` connects the micro-ROS client to the agent over Wi-Fi
using lwIP's raw UDP API. It implements the four callbacks that
`rmw_uros_set_custom_transport()` expects, so micro-ROS can use the Pico W radio
instead of a serial link.

| File | Purpose |
|---|---|
| `picow_udp_transport.h` | transport context and callback declarations |
| `picow_udp_transport.c` | open/close/write/read callbacks over a connected lwIP UDP PCB |

## Wiring it up

`ros/ros_node.c` installs the callbacks before the first ping:

```c
rmw_uros_set_custom_transport(
  false,                    // no stream framing (see below)
  &transport_context,
  picow_udp_transport_open,
  picow_udp_transport_close,
  picow_udp_transport_write,
  picow_udp_transport_read);
```

The agent address comes from `config/ros_config.h` (`ROS_AGENT_IP`,
`ROS_AGENT_PORT`) and is stored in `transport_context` at compile time.

## Framing must stay off

The micro-ROS agent's `udp4` transport has no stream-framing layer; framing is
only implemented for serial, CAN, and custom transports. The first argument to
`rmw_uros_set_custom_transport()` must therefore be **`false`**.

With framing enabled, the client wraps every XRCE message in a 7-byte frame
(`0x7E`, source, destination, length, payload, CRC). The agent still receives
and decodes the inner ping, but its reply is sent unframed, so the client's
framing reader never finds a `0x7E` start flag and the ping fails forever.
`docker compose logs micro_ros_agent` then shows a 23-byte request roughly every
1.5 s and no session is ever created:

```
recv_message | [==>> UDP <<==] | client_key: 0x00000000, len: 23, data:
0000: 7E 00 00 10 00 80 00 00 00 02 01 08 00 00 0A FF FD 02 00 00 00 CF 12
```

If that line reappears, check that the flashed firmware really passes `false`.
The staged binary can be verified without serial output:

```bash
arm-none-eabi-objdump -d --disassemble=ros_node_init \
    build/artifacts/dtf_pico_fw.elf | grep -B2 rmw_uros_set_custom_transport
```

The instruction immediately before the call should load zero into `r0`
(`movs r0, #0`).

## Buffering and timeouts

- Incoming datagrams are copied out of the lwIP callback into a 4-entry ring of
  `UXR_CONFIG_CUSTOM_TRANSPORT_MTU` (1024 byte) buffers, so the network callback
  never blocks on the micro-ROS session.
- Datagrams larger than the MTU or arriving while all four slots are full are
  dropped. XRCE retries them at the session layer.
- `picow_udp_transport_read()` polls the ring in 1 ms steps until its millisecond
  timeout expires. `rmw_uros_ping_agent()` and the rcl read loop both use this
  path.
- The PCB is connected to the agent address, so the kernel discards datagrams
  from any other host and port.

## Related files

- `../config/ros_config.h` - agent address, port, and ping/time-sync timeouts.
- `../ros/README.md` - session startup, connection troubleshooting, and log
  publishing.
- `../wifi/README.md` - the UDP log mirror on port 5005, useful when USB serial
  is not available.
