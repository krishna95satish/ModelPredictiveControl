# car_default
## Interfaces
### Publishes:
<table>
    <tr>
        <td>Type</td>
        <td>Topic</td>
    </tr>
    <tr>
        <td>CarlaEgoVehicleControl</td>
        <td>'/carla/default/vehicle_control_cmd'</td>
    </tr>
</table>

### Subscribes to:
<table>
    <tr>
        <td>Type</td>
        <td>Topic</td>
        <td>Callback</td>
    </tr>
    <tr>
        <td>Trajectory</td>
        <td>'/aorta/trajectory/filtered/default'</td>
        <td>trajectory_callback</td>
    </tr>
</table>

