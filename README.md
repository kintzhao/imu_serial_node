imu_serial_node
=
a simple ros serial node for imu, such as mpu 6050.

you should change it by your serial Communication protocol.
  this package depend on serial: http://wiki.ros.org/rosserial/

#### Published Topics

* **`imu`** ([sensor_msgs::Imu])

	The resulting Imu orientation.

#### Published TF Transforms

*	The resulting orientation is published as a tf transform, the frame names can be set using the parameters.


#### Services

* **`set_zero_orientation`** ([std_srvs/Empty])

	This service sets the current orientation as the new zero orientation so that from now on only the difference to this orientation is sent.


#### Parameters

* **`port`** (string, default: "/dev/ttyACM0")

	The name of the serial port.

* **`time_offset_in_seconds`** (double, default: 0.0)

	This sets an offset which is added to the header timestamp of the imu-topic and the TF transforms. This can be used to synchronise the IMUs orientation values to values of another sensor.


* **`imu_frame_id`** (string, default: "imu_base")

	Sets the name of the base frame for imu messages.


* **`tf_parent_frame_id`** (string, default: "imu_base")

	Sets the name of the parent frame in the tf transform.


* **`tf_frame_id`** (string, default: "imu")

	Sets the name of the own frame in the tf transform.
