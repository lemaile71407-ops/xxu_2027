# Maps

Store Nav2 map files here.

`rmuc_2025.yaml` / `rmuc_2025.pgm` belong to the pinned rm_sim_26 field.
They are rasterized from collision geometry, including low barriers; they are
not a measured SLAM map. The upstream version is pinned in
[simulation.repos](../../../simulation.repos); regenerate with
[generate_rmuc_map.py](../../../tools/navigation/generate_rmuc_map.py).
The old Small Point-LIO simulation entries have been retired. Current RMUL
navigation uses `rmul_navigation.launch.py` with explicitly matched ZLIO/TDT
configs, prior map, Nav2 map and parameters; see the
[navigation guide](../../../tools/navigation/README.md#当前主线与收敛范围).

Save a map with the existing tool after the map topic is ready:

```bash
ros2 run nav2_map_server map_saver_cli -f src/xxu_bringup/maps/your_map
```
