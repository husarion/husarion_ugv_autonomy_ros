# husarion_ugv_video_relay

`ffmpeg_to_compressed_video` copies each H.264 `ffmpeg_image_transport_msgs/msg/FFMPEGPacket` it receives into a `foxglove_msgs/msg/CompressedVideo` (format `h264`), with no decode and no re-encode. The bytes, the stamp and the `frame_id` cross unchanged.

It exists for one reason: the airlock of Husarion UGV OS carries `CompressedVideo` but not `FFMPEGPacket`, so the OAK's on-chip H.264 from stock depthai-ros cannot reach the robot's Cockpit as it is published. When the airlock carries `FFMPEGPacket` (its 1.30.0 image), this package and its compose service go.

## Usage

```bash
ros2 run husarion_ugv_video_relay ffmpeg_to_compressed_video --ros-args \
  -r __ns:=/panther_0957 \
  -r input:=oak/rgb/image_raw/compressed \
  -r output:=oak/rgb/image_raw/h264
```

It is also a component, `husarion_ugv_video_relay::FfmpegToCompressedVideo`, to load into the depthai container.

| Name | Kind | Default | What |
| -- | -- | -- | -- |
| `input` | topic, subscribed | | `FFMPEGPacket`, best effort |
| `output` | topic, published | | `CompressedVideo`, reliable, depth 10 (as depthai publishes) |
| `frame_id` | parameter | `""` | Replaces the packet's `frame_id` when set |

A packet whose `encoding` is not H.264 (its first token is not `h264`, `libx264` or `h264_*`) is dropped with a warning every 10 s, never relabelled. depthai labels its on-chip stream with `i_low_bandwidth_ffmpeg_encoder`, `libx264` by default, so keep that parameter at its default.
