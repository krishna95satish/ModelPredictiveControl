# edge_device
## Interfaces
### Publishes:
<table>
    <tr>
        <td>Type</td>
        <td>Topic</td>
    </tr>
    <tr>
        <td>ObjectList</td>
        <td>'/aorta/edge/objects'</td>
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
        <td>Image</td>
        <td>'/carla/edge_camera/image'</td>
        <td>image_callback</td>
    </tr>
    <tr>
        <td>CameraInfo</td>
        <td>'/carla/edge_camera/camera_info'</td>
        <td>camera_info_callback</td>
    </tr>
</table>

