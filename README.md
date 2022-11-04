# Install
```
cd ~/catkin_ws/src
git clone https://github.com/tmxkn1/brass_gazebo_battery.git
```

# Dependancy

libignition-math2-dev
```
sudo apt install libignition-math2-dev
```

# Build
```
cd ~/catkin_ws
catkin_make
```

# Use
An exmaple of the Jackal URDF can be found in `test\jackal_1.urdf`.

To add the plugin to your Jackal urdf, you need to add the following lines inside your model:

```

  <gazebo reference="base_link">
    <battery name="main_battery">
        <voltage>24</voltage>
    </battery>
  </gazebo>
  
  <gazebo>
    <plugin name="battery" filename="libbattery_discharge.so">
        <ros_node>battery_monitor_client</ros_node>
        <link_name>base_link</link_name>
        <battery_name>main_battery</battery_name>
        <constant_coef>24.8</constant_coef>
        <linear_coef>-3.1424</linear_coef>
        <initial_charge>11.25</initial_charge>
        <capacity>11.25</capacity>
        <resistance>0.061523</resistance>
        <smooth_current_tau>0.5499</smooth_current_tau>
        <charge_rate>0.2</charge_rate>
    </plugin>
  </gazebo>
  <gazebo>
    <plugin name="consumer" filename="libbattery_consumer.so">
        <link_name>base_link</link_name>
        <battery_name>main_battery</battery_name>
        <power_load>27.5</power_load>
    </plugin>
  </gazebo>
  <gazebo>
    <plugin name="motor_consumer" filename="libmotor_consumer.so">
        <link_name>base_link</link_name>
        <battery_name>main_battery</battery_name>
        <power_load_rate>1</power_load_rate>
        <consumer_idle_power>0.1</consumer_idle_power>
    </plugin>
  </gazebo>
  ```

# Published topics
```
/mobile_base/commands/charge_level
/mobile_base/commands/charge_level_mwh
/mobile_base/commands/battery_percentage
/mobile_base/commands/consumer/motor_power
```
- The remaining battery charge in Amp-hour (Ah).
- The remaining battery charge in milli-watt-hour (mWh).
- The remaining battery charge in percentage.
- The current power consumed by motors in Watt.

# Exposed services
```
/battery_monitor_client/battery_demo_model/set_charge
/battery_monitor_client/battery_demo_model/set_charge_rate
/battery_monitor_client/battery_demo_model/set_charging
/battery_monitor_client/battery_demo_model/set_model_coefficients
/battery_monitor_client/battery_demo_model/set_power_load
```
