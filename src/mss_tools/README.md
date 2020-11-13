# mss_tools
ROS tools for the HCU Multi-Sensor-System 

## tf_points_global
![veloframe](https://i.imgur.com/grBI4w2.png)


**ptcl2_global_frame** - Der übergeordnete Frame, **default: map**. In den wir transformieren bzw. registrieren möchten (siehe Bild).
```cpp
104       transformStamped = tfBuffer.lookupTransform("--->map<---", local_frame, ros::Time(0));
```

**ptcl2_local_frame** - Der Frame(name) auf den wir uns beziehen. Welchen wir (mit samt den anhängenden Punkten) ins übergeordnete System transformieren wollen. In unserem Fall, **default: velodyne**
```cpp
104       transformStamped = tfBuffer.lookupTransform("map", --->local_frame<---, ros::Time(0));
```


**ptcl2_input_topic** - Die [PointCloud2-Message](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) die ins übergeordnete System Tranformiert werden soll, **default: /velodyne_points**

```cpp
92   ros::Subscriber sub_ptcl = nh_ptcl.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 10, &Ptcl::ptclCa    llback, &ptcl_object);
```
**ptcl2_output_topic** - Die [PointCloud2-Message](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) die die transformierte Punktwolke enthält, **default: /transformed_velodyne_points**
```cpp
98   ros::Publisher pub_trans_ptcl = nh_trans_ptcl.advertise<sensor_msgs::PointCloud2>("transformed_velodyne_point    s",1);
```


**drop_when_same_position** - Ein boolean der festlegt, ob Punktwolken auch dann weiterhin transfomiert werden sollen, wenn es kein aktuelles Positionsupdate für diese Punktwolke gibt, **default: true**. Bleibt das System stehen, oder wird das Prisma verloren, wir verhindert, dass Wilkürlich Punktwolken akkumuliert werden. Dieser Wert kann auf false gesetzt werden, um das Package zu testen, wenn etwa grade kein Tachymeter aufgebaut ist/aufgebaut werden soll.
```cpp
109       if(cloud_in.width && transformStamped.transform.translation.x != lastX){
```


## platform_tf

**pose_global_frame** - Der Frame auf den wir die Messungen beziehen, **default: map**.

**pose_local_frame** - Der Frame den wir aufgrunder der IMU und Tachymetermessung ausgeben, **default: prisma_frame**

**system_orientation_z** - Die Orientierung der Plattform ist zur initalisierung nicht bekannt, daher muss dieser Wert entweder messtechnisch ermittelt oder gut geschätzt werden, **default: 0** (degree).

**position_provider** - Das Topic, welches die Informationen über die aktuelle Position enthält, **default: /tachy_points** (aus dem Tachymeter).

**orientation_provider** - Das Topic, welches die Information über die Orientierung enthält, **default: /imu/data_raw**

## tachymeter_geocom

**tachymeter_position_topic** - Der Name unter dem die gemessene Position des Tachymeters als Topics zur Verfügung gestellt werden soll, **default: tachy_points** (Im direkten Zusammenhang zu position_provider aus platform_tf).

**position_local_frame** - Der lokale Frame des Tachymeters (wird bisher für nichts verwendet).

## xio_imu_driver

**xio_device_ip** - Die IP Adresse des Xio Devices, **default: 192.168.1.199**

**xio_orientation_topic** - Der Name unter dem die gemessene Verdrehung des xio Devices als Topics zur Verfügung gestellt wird, **default: imu/data_raw** (Im direkten Zusammenhang zu position_provider aus platform_tf).

## Frames

```cpp
 <node pkg="tf" type="static_transform_publisher" name="prisma_velodyne_tf"
        args="0.185 0 -0.10 0 -1.5707963264897 3.141592653589793 prisma_frame velodyne 300" />
 <node pkg="tf" type="static_transform_publisher" name="imu_map_tf"
        args="0 0 0 0 0 0 map imu 300" />
```

