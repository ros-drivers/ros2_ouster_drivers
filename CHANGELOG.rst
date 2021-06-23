Changelog
=========

[unreleased] Ported client changes for FW 2.0/2.1 (2021-06-23)
-------------

**ouster_client**

* add support for new signal multiplier config parameter
* add early version of a C++ API covering the full sensor configuration interface
* increase default initialization timeout to 60 seconds to account for the worst case: waking up from STANDBY mode
* Contibutors: Matthew Young (Trimble Inc)

[0.3.0] (YYYY-MM-DD)
------------------
* Increased the rate at which the timer callback fires to process incoming data
  from 1280 Hz to 2560 Hz.
  (`#55 <https://github.com/SteveMacenski/ros2_ouster_drivers/issues/55>`_)
* Reordered in-memory data of point cloud data processor to row-major order
  (`#52 <https://github.com/SteveMacenski/ros2_ouster_drivers/issues/52>`_)
* Start tracking changes in CHANGELOG at 0.3.0
* Contributors: Steve Macenski, Tom Panzarella
