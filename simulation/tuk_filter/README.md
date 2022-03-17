# tuk_filter
## Interfaces
### Publishes:
<table>
    <tr>
        <td>Type</td>
        <td>Topic</td>
    </tr>
    <tr>
        <td>NavSatFix</td>
        <td>'/aorta/car_info/filtered/tuk/gnss'</td>
    </tr>
    <tr>
        <td>Trajectory</td>
        <td>'/aorta/trajectory/filtered/tuk'</td>
    </tr>
    <tr>
        <td>ObjectList</td>
        <td>'/aorta/car_info/filtered/tuk/objects'</td>
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
        <td>'/aorta/trajectory/tuk'</td>
        <td>trajectory_callback</td>
    </tr>
    <tr>
        <td>NavSatFix</td>
        <td>'/aorta/car_info/tuk/gnss'</td>
        <td>gnss_callback</td>
    </tr>
    <tr>
        <td>ObjectList</td>
        <td>'/aorta/car_info/tuk/objects'</td>
        <td>objects_callback</td>
    </tr>
</table>

