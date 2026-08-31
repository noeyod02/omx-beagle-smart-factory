# Demo cell: the settings a restart must not lose

These files describe the cell as it physically stands, so that bringing the
demo back up after a reboot needs no typing. They lived in `/tmp` until
2026-08-31, which meant every reboot silently took the aiming, the device
paths and the station wiring with it - and the loss only showed up as a
camera that saw nothing or an arm that refused every job.

| file | runs on | what it pins |
|---|---|---|
| `arrival.yaml` | PC1 | the warehouse/bay camera: which stream, where the bay is in it, where the parts wait |
| `stock_monitor.yaml` | PC2 | the stock camera: which physical USB port, the YOLO weights, the four cell rectangles |
| `station_c_tm.yaml` | PC2 | station C's task manager (the arm that owns cells c and d) |

Station A and B need no file here: `omx_stock_relay.launch.py` starts them
with their own arguments.

Two things in these files are measurements, not preferences, and are wrong
the moment anything is moved:

- **camera rectangles** (`roi`, `part_roi`, and the cell rectangles in
  `stock_bins_cam.yaml`). A wrong rectangle does not error - it reads as
  absence, so a carrier parked in plain sight becomes an empty bay.
- **the stock camera's device path**, given by physical USB port
  (`/dev/v4l/by-path/...`) rather than `/dev/videoN`, which changes whenever
  the cameras are re-plugged.

Bring the whole cell up with `scripts/demo_up.sh` (see its header for the
order and why it is that order).
