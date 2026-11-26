# husarion_ugv_map_gateway ROS API

Names are relative to the robot's namespace. `~/` is the node's own name (`map_gateway`).

## Services

| Name | Type | What it does |
| -- | -- | -- |
| `~/serialize_map` | `slam_toolbox/srv/SerializePoseGraph` | Saves the current map as `<maps_directory>/<filename>/`: `map.yaml` + image (through `map_saver/save_map`), `map.georef.yaml` (epoch, image digest, GNSS alignment, the robot's pose) and slam_toolbox's pose graph. `result` 0 when the pair is complete, 255 when refused. |
| `~/save_map` | `nav2_msgs/srv/SaveMap` | The same save. `map_url` is the name; the other request fields are ignored. `result` true when the pair is complete. For clients without slam_toolbox's message types (the Cockpit's rosbridge). |
| `~/map_preview` | `nav2_msgs/srv/LoadMap` | A saved map's grid for a picker: `map_url` is the saved name, `map` its grid halved (walls kept) until no side is over `preview_max_side` cells, with the pair's origin and the resulting resolution. Nothing is loaded into Nav2. `result` 0, or 1 when the name is not a complete saved pair. |
| `cockpit/nav/pause` | `std_srvs/srv/Trigger` | Pauses Nav2's lifecycle nodes. |
| `cockpit/nav/resume` | `std_srvs/srv/Trigger` | Resumes them. |

A save is refused, with the reason in the node's log, when the name is not 1 to 64 characters of `A-Z a-z 0-9 _ -`, when that name is already saved (a pair is never overwritten), while another save runs, before the map epoch is known, without a robot pose in the map frame, or without `map_saver` (localization mode).
