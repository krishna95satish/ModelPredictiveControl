# car_emergency
## Interfaces
### Publishes:
<table>
    <tr>
        <td>Type</td>
        <td>Topic</td>
    </tr>
    <tr>
        <td>Path</td>
        <td>'/carla/emergency/waypoints'</td>
    </tr>
    <tr>
        <td>Float64</td>
        <td>'/carla/emergency/speed_command'</td>
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
        <td>'/aorta/trajectory/filtered/emergency'</td>
        <td>trajectory_callback</td>
    </tr>
</table>

