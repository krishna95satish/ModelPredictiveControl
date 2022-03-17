# default_filter
## Interfaces
### Publishes:
<table>
    <tr>
        <td>Type</td>
        <td>Topic</td>
    </tr>
    <tr>
        <td>Trajectory</td>
        <td>filter_callback(topic='trajectory')</td>
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
        <td>'/aorta/trajectory/default'</td>
        <td>trajectory_callback</td>
    </tr>
</table>

