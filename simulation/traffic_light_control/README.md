# traffic_light_control
## Interfaces
### Publishes:
<table>
    <tr>
        <td>Type</td>
        <td>Topic</td>
    </tr>
    <tr>
        <td>TrafficLightControl</td>
        <td>'/aorta/control/traffic_light/control'</td>
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
        <td>TrafficLightProgram</td>
        <td>'/aorta/control/traffic_light/program'</td>
        <td>traffic_light_callback</td>
    </tr>
</table>

