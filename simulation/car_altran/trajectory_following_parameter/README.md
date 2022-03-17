# trajectory_following_parameter
Because matlab simulink cannot get ROS2 parameters.
Therefore this node is used to transfer the control parameters to publish topic.
Control parameters are in file [trajectory_following_parameter](config/trajectory_following_parameter.yaml)
### Publish:
<table>
    <tr>
        <td>Type</td>
        <td>Topic</td>
    </tr>
    <tr>
        <td>custom_messages/VehicleControlParameter</td>
        <td>'/carla/altran/following_control_parameters'</td>
    </tr>
</table>
