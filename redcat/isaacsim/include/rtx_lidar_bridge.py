# SPDX-License-Identifier: BSD-3-Clause

"""Generic RTX lidar setup for Isaac Sim: sensor creation, elevation/channel-count control, and
ROS 2 writer attachment (LaserScan for 2D configs, PointCloud2 for 3D). Not tied to any specific
robot -- callers supply the full prim path and frame_id to use.

Must only be imported after a SimulationApp has been constructed -- its own imports below need
Isaac Sim's runtime to already exist.
"""

import carb
import isaacsim.core.experimental.utils.prim as prim_utils
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

_OVERRIDDEN_EMITTER_SUFFIXES = (":elevationDeg", ":azimuthDeg", ":channelId", ":fireTimeNs")


def _read_laser_scan_metadata(prim: object) -> dict[str, float | list[float]]:
    """Read scan geometry from the lidar prim, needed to initialize the LaserScan writer."""
    rotation_rate = float(prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get() or 0)
    near_range = float(prim.GetAttribute("omni:sensor:Core:nearRangeM").Get() or 0)
    far_range = float(prim.GetAttribute("omni:sensor:Core:farRangeM").Get() or 0)
    firing_rate = int(prim.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get() or 0)
    if rotation_rate <= 0 or firing_rate <= 0:
        raise RuntimeError("LaserScan: scanRateBaseHz or patternFiringRateHz is 0 on the lidar prim")
    return {
        "horizontalFov": 360.0,
        "horizontalResolution": 360.0 * rotation_rate / firing_rate,
        "depthRange": [near_range, far_range],
        "rotationRate": rotation_rate,
        "azimuthRange": [-180.0, 180.0],
    }


def _remap_lidar_elevation(prim: object, min_deg: float, max_deg: float) -> None:
    """Linearly rescale a rotary lidar's per-channel elevation angles into [min_deg, max_deg]
    (degrees above horizontal). Passing min_deg == max_deg pins every channel to that one
    elevation (used to force a 2D config flat to the horizontal plane).

    Each channel's elevation is baked into the profile as a fixed
    'omni:sensor:Core:emitterState:<channel>:elevationDeg' attribute (scalar or array,
    depending on the profile). A stock profile like Example_Rotary is symmetric about the
    horizon; this rescales whatever range it ships with into the requested one.
    """
    elevation_attrs = [
        attr for attr in prim.GetAttributes()
        if attr.GetName().startswith("omni:sensor:Core:emitterState:") and attr.GetName().endswith(":elevationDeg")
    ]
    if not elevation_attrs:
        carb.log_warn("No emitterState elevationDeg attributes found; elevation_range_deg ignored.")
        return

    def _as_list(value: object) -> list[float]:
        return list(value) if hasattr(value, "__iter__") else [value]

    all_values = [v for attr in elevation_attrs for v in _as_list(attr.Get())]
    src_min, src_max = min(all_values), max(all_values)
    src_span = (src_max - src_min) or 1.0
    dst_span = max_deg - min_deg

    for attr in elevation_attrs:
        raw = attr.Get()
        remapped = [min_deg + (v - src_min) / src_span * dst_span for v in _as_list(raw)]
        attr.Set(remapped if hasattr(raw, "__iter__") else remapped[0])

    print(f"[INFO] Remapped lidar elevation from [{src_min:.1f}, {src_max:.1f}] deg to"
          f" [{min_deg}, {max_deg}] deg across {len(elevation_attrs)} channel(s).")


def _resample(values: list, new_len: int) -> list:
    """Nearest-neighbor resample a list to new_len entries, preserving its existing values."""
    if len(values) == new_len:
        return list(values)
    if new_len == 1:
        return [values[0]]
    return [values[round(i * (len(values) - 1) / (new_len - 1))] for i in range(new_len)]


def _set_lidar_elevation_step(prim: object, step_deg: float, min_deg: float, max_deg: float) -> None:
    """Regenerate a rotary lidar's channels at a fixed angular spacing across [min_deg, max_deg]
    (degrees above horizontal) -- controls how densely it sweeps in elevation, expressed as the
    delta angle between adjacent planes rather than a raw channel count.

    Channel count is derived as round(range / step) + 1, then baked into
    'omni:sensor:Core:numberOfChannels' / 'numberOfEmitters'. Every 'emitterState:<channel>:*'
    array must share that same length: elevationDeg/azimuthDeg/channelId/fireTimeNs are
    regenerated fresh (every channel fires at azimuth offset 0 -- the ROTARY scan sweeps azimuth
    via the lidar's own rotation, so that alone covers 360 deg); everything else the profile
    ships (calibration/optics fields like distanceCorrectionM, focalDistM, ...) is nearest-
    neighbor resampled from its existing values instead, since we have no principled new values
    for those.
    """
    elevation_attr = next(
        (attr for attr in prim.GetAttributes()
         if attr.GetName().startswith("omni:sensor:Core:emitterState:") and attr.GetName().endswith(":elevationDeg")),
        None,
    )
    if elevation_attr is None:
        carb.log_warn("No emitterState elevationDeg attribute found; elevation_step_deg ignored.")
        return
    prefix = elevation_attr.GetName().removesuffix(":elevationDeg")

    orig_elevations = elevation_attr.Get()
    orig_count = len(orig_elevations) if hasattr(orig_elevations, "__iter__") else 1

    num_channels = max(1, round(abs(max_deg - min_deg) / step_deg) + 1)
    elevations = (
        [min_deg] if num_channels == 1
        else [min_deg + i * (max_deg - min_deg) / (num_channels - 1) for i in range(num_channels)]
    )
    prim.GetAttribute("omni:sensor:Core:numberOfChannels").Set(num_channels)
    prim.GetAttribute("omni:sensor:Core:numberOfEmitters").Set(num_channels)
    prim.GetAttribute(f"{prefix}:elevationDeg").Set(elevations)
    prim.GetAttribute(f"{prefix}:azimuthDeg").Set([0.0] * num_channels)
    prim.GetAttribute(f"{prefix}:channelId").Set(list(range(1, num_channels + 1)))
    fire_time_attr = prim.GetAttribute(f"{prefix}:fireTimeNs")
    if fire_time_attr.IsValid():
        fire_time_attr.Set([0] * num_channels)

    # Resample every other per-channel array on this emitter state to the new length, so nothing
    # is left stale at the profile's original channel count.
    for attr in prim.GetAttributes():
        name = attr.GetName()
        if not name.startswith(f"{prefix}:") or name.endswith(_OVERRIDDEN_EMITTER_SUFFIXES):
            continue
        raw = attr.Get()
        if not hasattr(raw, "__iter__") or len(raw) != orig_count or len(raw) == 0:
            continue
        attr.Set(_resample(list(raw), num_channels))

    print(f"[INFO] Set lidar to {num_channels} channel(s) (~{step_deg} deg spacing),"
          f" elevation [{min_deg}, {max_deg}] deg above horizontal.")


class RtxLidarBridge:
    """Creates an RTX lidar at a given prim path and publishes it to ROS 2, auto-selecting
    LaserScan vs PointCloud2 from the config name ("2D" suffix -> LaserScan).

    Not robot-specific: prim_path is the full path to create the sensor at (caller decides where
    to mount it), and elevation defaults are Isaac Sim's own stock behavior, not tuned to any
    particular sensor -- pass elevation_range_deg/elevation_step_deg to match a real device.

    Usage, matching Isaac Sim's own bring-up ordering constraints:
        lidar = RtxLidarBridge(prim_path, config="Example_Rotary", frame_id="lidar_link")
        lidar.create()                          # before SimulationManager.setup_simulation()
        SimulationManager.setup_simulation(...)
        lidar.attach_writers()                  # after setup_simulation()
        lidar.describe()                        # anytime after attach_writers()
    """

    def __init__(
        self,
        prim_path: str,
        config: str,
        frame_id: str,
        *,
        tick_rate: float = 10.0,
        translation: tuple[float, float, float] = (0.0, 0.0, 0.0),
        elevation_range_deg: tuple[float, float] | None = None,
        elevation_step_deg: float | None = None,
        debug_vis: bool = True,
    ):
        self.path = prim_path
        self.config = config
        self.frame_id = frame_id
        self.tick_rate = tick_rate
        self.translation = translation
        self.debug_vis = debug_vis
        self.is_2d = config.strip().lower().endswith("2d")

        # Default matches Isaac Sim's own stock profile behavior: 3D sweeps horizon-to-zenith
        # (0-90 deg above horizontal); 2D is pinned flat to the horizontal plane (0 deg range).
        # Override with elevation_range_deg for a specific real sensor's actual FOV.
        if elevation_range_deg is not None:
            self.elevation_range_deg = tuple(elevation_range_deg)
        elif self.is_2d:
            self.elevation_range_deg = (0.0, 0.0)
        else:
            self.elevation_range_deg = (90.0, 0.0)
        self.elevation_step_deg = elevation_step_deg

        self._lidar = None
        self._sensor = None

    def create(self) -> None:
        """Author the OmniLidar prim and apply elevation/channel-count settings."""
        self._lidar = Lidar.create(
            path=self.path,
            config=self.config,
            tick_rate=self.tick_rate,
            translations=[list(self.translation)],
        )
        prim = prim_utils.get_prim_at_path(self._lidar.paths[0])
        if self.elevation_step_deg is not None and not self.is_2d:
            _set_lidar_elevation_step(prim, self.elevation_step_deg, *self.elevation_range_deg)
        else:
            if self.elevation_step_deg is not None:
                carb.log_warn("elevation_step_deg is ignored for 2D lidar configs (always 1 plane).")
            _remap_lidar_elevation(prim, *self.elevation_range_deg)

    def attach_writers(self) -> None:
        """Wrap the lidar prim, create its render product, and attach the ROS 2 writer plus the
        optional viewport debug draw.
        """
        self._sensor = LidarSensor(self._lidar, annotators=[])

        if self.is_2d:
            # 2D lidar -> LaserScan on /scan.
            laser_scan_meta = _read_laser_scan_metadata(prim_utils.get_prim_at_path(self._lidar.paths[0]))
            self._sensor.attach_writer(
                "RtxLidarROS2PublishLaserScan",
                topicName="scan",
                frameId=self.frame_id,
                **laser_scan_meta,
            )
        else:
            # 3D lidar -> PointCloud2 on /point_cloud.
            self._sensor.attach_writer(
                "RtxLidarROS2PublishPointCloud",
                topicName="point_cloud",
                frameId=self.frame_id,
            )

        # Visualize the scan in the viewport (RGBA in [0, 1]).
        if self.debug_vis:
            self._sensor.attach_writer(
                "draw-point-cloud",
                color=[0.0, 1.0, 0.5, 1.0],  # bright green
                size=0.05,
            )

    def describe(self) -> None:
        """Print the active topic/message type and matching rviz2 setup instructions."""
        if self.is_2d:
            print(f"[INFO] RTX lidar ({self.config}) publishing LaserScan on /scan")
            print(f"[INFO] In rviz2: set Fixed Frame to '{self.frame_id}' and add a LaserScan display on /scan.")
        else:
            print(f"[INFO] RTX lidar ({self.config}) publishing PointCloud2 on /point_cloud")
            print(f"[INFO] In rviz2: set Fixed Frame to '{self.frame_id}' and add a PointCloud2"
                  " display on /point_cloud.")
