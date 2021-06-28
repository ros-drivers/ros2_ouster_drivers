Changelog
=========

[unreleased] Added a new SensorTins implementation (2021-06-23)

**General Changes**

* Added an example metadata file.
* Added/exposed the functions for loading and saving metadata to a json file.
* Contibutors: Matthew Young (Trimble Inc)

**SensorTins**

* Added a new SensorImplementation called SensorTins. This implementation uses the Tins library to find LiDAR and IMU packets, so it works with real data as well as data replayed from a pcap file. 
* Added a parameter allowing the user to switch between SensorTins and the original Sensor driver implementation
* Updated readme and parameter file to explain usage of the SensorTins implementation.
* Contibutors: Matthew Young (Trimble Inc)

[unreleased] Ported client changes for FW 2.0/2.1 (2021-06-23)
-------------

**ouster_client**

* From [ouster_example #239](https://github.com/ouster-lidar/ouster_example/commit/e1176f427f68eb0807bb15ccabe34aea47a1c5d3). 
  * Ported `Optional` code: 
    * Optional header and license file are now present in their own folder: `include/ros2_ouster/client/optional-lite/`
    * Ported some mode-getting functions that use Optional to types.cpp
  * Ported the `sensor_config` struct 
    * Ported sensor_config struct to types.hpp
    * Ported the get/set_config functions for sensor_config to client.cpp/.hpp
  * Ported some documentation updates, and a != operator for LidarScan objects to lidar_scan.h.
  * Ported/expanded data objects added by ouster_example #239  
    * Ported several additional enumerated types (e.g. OperatingMode and MultipurposeIOMode) to types.h
    * Ported parsing functions for additional enumerated types to types.cpp
    * Ported ColumnWindow definition and added it as a parameter to data_format struct, which is the only existing struct changed.
  * In lidar_scan.cpp, ported some changes to `make_xyz_lut` and `cartesian` functions  
* From [ouster_example/#246](https://github.com/ouster-lidar/ouster_example/commit/2b49e6a2f3dbd0462c557974a3f428915067fd2f)
  * Removed a json include incorrectly added to types.h  
* From [ouster_example #259](https://github.com/ouster-lidar/ouster_example/commit/b8b23c35d7b719d69341a438d386a801688aa6a4#diff-db793e44a91d87bc6e0d94870833d5d9eeb4420e2fbbec4e2468717a359a651f)
  * Increased default timeout time from 30 to 60 seconds for `init_client` and `read_imu_packet`
  * In netcompat.cpp, changed non_blocking_mode from off to on (0 to 1) if using Win32.
  * Ported signal_multiplier parameter 
    * Added parameter to `sensor_config` in types.h
    * Added signal_multiplier parameter setting to some functions in client.cpp and types.cpp
* Updated changelog

**General Changes**

* Changed indenting in client/types/lidar_scan source header files to be more consistent with existing indenting style.

[0.3.0] (YYYY-MM-DD)
------------------
* Increased the rate at which the timer callback fires to process incoming data
  from 1280 Hz to 2560 Hz.
  (`#55 <https://github.com/SteveMacenski/ros2_ouster_drivers/issues/55>`_)
* Reordered in-memory data of point cloud data processor to row-major order
  (`#52 <https://github.com/SteveMacenski/ros2_ouster_drivers/issues/52>`_)
* Start tracking changes in CHANGELOG at 0.3.0
* Contributors: Steve Macenski, Tom Panzarella
