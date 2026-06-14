# Mapping API

`mapping_api.py` is a standalone HTTP server that controls the rtabmap mapping
pipeline running on the Rove robot. It runs as its own process, independent
of `run.launch.py`, and can be started before or after the main launch.

---

## Starting the server

```bash
source install/setup.bash
python3 mapping_api.py
```

It will start the server on `localhost` on this machine at the port `8888`, if you are trying to access it from a different machine,
then open it at **`http://<robot-ip>:8888/`** in a browser for the interactive Swagger UI.

The server prints a request log to stdout:
```
[mapping_api] listening on :8888
[mapping_api] swagger → http://<robot-ip>:8888/
[http] 192.168.2.145 "GET /mapping/status HTTP/1.1" 200 -
```

> **Note:** The Swagger UI loads its assets from `unpkg.com` — the swagger UI
> needs internet access, but the API doesn't.

---

## Startup order

The API can start **before** `run.launch.py`. While rtabmap is not yet running:

- `GET` routes respond immediately
- `POST` routes wait 5 seconds then return a clean error:
  ```json
  {"ok": false, "message": "/rtabmap/pause not available — is rtabmap running?"}
  ```

Once `run.launch.py` starts and rtabmap comes up, all routes start working
automatically — no restart of the API needed.

---

## Routes

### `GET /minimap`

Returns a robot-centred crop of the current 2-D occupancy grid, rotated so
the robot always faces up.

```bash
curl http://<robot-ip>:8888/minimap
```

```json
{
  "timestamp":   1718000000.0,
  "local_size":  200,
  "resolution":  0.05,
  "robot_x":     3.2,
  "robot_y":    -1.1,
  "robot_yaw":   1.57,
  "robot_found": true,
  "data":        [[...], ...]
}
```

`data` is a `200 × 200` 2-D array of integers:

| Value | Meaning  |
|-------|----------|
| `-1`  | Unknown  |
| `0`   | Free     |
| `100` | Occupied |

Returns `{"error": "No map yet"}` if rtabmap hasn't published a map yet.

---

### `GET /mapping/status`

Returns the current state of the mapping pipeline.

```bash
curl http://<robot-ip>:8888/mapping/status
```

```json
{
  "state":         "running",
  "node_count":    42,
  "loop_closures": 3,
  "db_size_mb":    18.4,
  "timestamp":     1718000000.0
}
```

| `state`    | Meaning                                                  |
|------------|----------------------------------------------------------|
| `unknown`  | API started before rtabmap — waiting for first info msg  |
| `running`  | Actively integrating scans into the map                  |
| `paused`   | Node alive but not integrating — call `/mapping/go`      |

---

### `POST /mapping/go`

Resumes mapping after a pause. **No-op if already running** — safe to call
at any time.

```bash
curl -X POST http://<robot-ip>:8888/mapping/go
```

```json
{"ok": true, "message": "ok"}
```

> rtabmap starts in `running` state when `run.launch.py` launches.
> You only need this after calling `/mapping/pause`.

---

### `POST /mapping/pause`

Pauses scan integration. The pose graph stays in memory and the robot can
still move freely — it just won't add new nodes to the map.

```bash
curl -X POST http://<robot-ip>:8888/mapping/pause
```

```json
{"ok": true, "message": "ok"}
```

---

### `POST /mapping/restart`

Clears the working memory **and the DB on disk**, then immediately starts
building a fresh map. The rtabmap node itself is not restarted.

```bash
curl -X POST http://<robot-ip>:8888/mapping/restart
```

```json
{"ok": true, "message": "ok"}
```

> ⚠️ **This is irreversible.** Export the map first if you want to keep it.

---

### `POST /mapping/new_map`

Starts a new sub-map while keeping the existing pose graph. The previously
mapped area remains in memory and loop closure can still connect the new
area back to it.

```bash
curl -X POST http://<robot-ip>:8888/mapping/new_map
```

Use this for **multi-floor or multi-session mapping** — for example, mapping
floor 1, then triggering a new map before moving to floor 2. For
single-location mapping you will never need this route.

---

### `POST /mapping/export`

Exports a camera-coloured point cloud to `/mnt/ssd/maps/`.

**This call blocks** until export completes. Depending on map size this
typically takes 10–120 seconds — do not set a short HTTP timeout on the
client.

**Sequence (automatic):**
1. Pause rtabmap so the database is not written to mid-export
2. Run `rtabmap-export` on `~/.ros/rtabmap.db`
3. Resume rtabmap
4. Return result

```bash
# Default filename (scan_YYYYMMDD_HHMMSS.ply)
curl -X POST http://<robot-ip>:8888/mapping/export

# Custom filename
curl -X POST http://<robot-ip>:8888/mapping/export \
  -H "Content-Type: application/json" \
  -d '{"filename": "lab_run1.ply"}'
```

```json
{
  "ok":         true,
  "message":    "Export complete",
  "path":       "/mnt/ssd/maps/lab_run1.ply",
  "size_mb":    142.3,
  "duration_s": 38.2
}
```

**Request body (optional):**

| Field      | Type   | Default                        | Description                    |
|------------|--------|--------------------------------|--------------------------------|
| `filename` | string | `scan_YYYYMMDD_HHMMSS.ply`     | Basename only — no path needed |

> If `rtabmap-export` is not found, run
> `find /opt/ros/humble -name "rtabmap-export"` on the robot and update
> `RTABMAP_EXPORT_BIN` at the top of `mapping_api.py`.

---

### `POST /mapping/config`

Hot-reloads `rtabmap.yaml` parameters into the live rtabmap node without
restarting anything.

```bash
# Use the default config/rtabmap.yaml
curl -X POST http://<robot-ip>:8888/mapping/config

# Use a different file
curl -X POST http://<robot-ip>:8888/mapping/config \
  -H "Content-Type: application/json" \
  -d '{"path": "/home/nathan/my_tuned_params.yaml"}'
```

```json
{
  "ok":      true,
  "message": "Applied 24 parameters from /path/to/rtabmap.yaml",
  "path":    "/path/to/rtabmap.yaml"
}
```

**Request body (optional):**

| Field  | Type   | Default              | Description                             |
|--------|--------|----------------------|-----------------------------------------|
| `path` | string | `config/rtabmap.yaml` | Absolute path to any valid YAML file   |

The YAML can be flat or ROS 2 node format:

```yaml
# Flat (simpler)
Grid/CellSize: "0.05"
RGBD/LinearUpdate: "0.1"

# ROS 2 node format
rtabmap:
  ros__parameters:
    Grid/CellSize: "0.05"
```

> RTABMap internal parameters are always **strings** (quoted).
> ROS node parameters like `cloud_output_voxel_size` use native types (float/bool/int).

---

## Error responses

All `POST` routes return the same shape on failure:

```json
{"ok": false, "message": "explanation of what went wrong"}
```

HTTP status codes:

| Code  | Meaning                                              |
|-------|------------------------------------------------------|
| `200` | Success                                              |
| `503` | ROS service unavailable or call failed               |
| `404` | Route not found                                      |

---

## Troubleshooting

**Routes return `not available — is rtabmap running?`**
rtabmap is not up yet or crashed. Check `ros2 launch rove_color_mapping run.launch.py`.

**`/minimap` returns `"No map yet"`**
rtabmap is running but hasn't built a map yet, or `/grid_prob_map` is not
being published. Check: `ros2 topic hz /grid_prob_map`

**Export returns `rtabmap-export not found`**
Find it on the robot: `find /opt/ros/humble -name "rtabmap-export"`
Then update `RTABMAP_EXPORT_BIN` at the top of `mapping_api.py`.

**Swagger UI is blank / assets don't load**
The browser needs internet access to load Swagger UI from `unpkg.com`.
The robot itself does not need internet. Test the API directly with `curl`
if the browser has no internet.

**Port already in use**
```bash
# Find what's using port 8888
sudo ss -tlnp | grep 8888
# Kill a stale process
pkill -f mapping_api.py
```