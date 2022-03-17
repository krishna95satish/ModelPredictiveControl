# altran_filter
## Interfaces
### Publishes:
<table>
    <tr>
        <td>Type</td>
        <td>Topic</td>
    </tr>
    <tr>
        <td>NavSatFix</td>
        <td>'/aorta/car_info/filtered/altran/gnss'</td>
    </tr>
    <tr>
        <td>Trajectory</td>
        <td>'/aorta/trajectory/filtered/altran'</td>
    </tr>
    <tr>
        <td>ObjectList</td>
        <td>'/aorta/car_info/filtered/altran/objects'</td>
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
        <td>'/aorta/trajectory/altran'</td>
        <td>filter_callback(topic='trajectory')</td>
    </tr>
    <tr>
        <td>NavSatFix</td>
        <td>'/aorta/car_info/altran/gnss'</td>
        <td>filter_callback(topic='gnss')</td>
    </tr>
    <tr>
        <td>ObjectList</td>
        <td>'/aorta/car_info/altran/objects'</td>
        <td>filter_callback(topic='objects')</td>
    </tr>
</table>

