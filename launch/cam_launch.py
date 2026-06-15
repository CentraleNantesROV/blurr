from simple_launch import SimpleLauncher
from ament_index_python.packages import get_package_prefix


def im_remap(src, start = {}):

    start.update(dict((src+sub, 'image'+sub)
                      for sub in ('', '/compressed', '/compressedDepth', '/theora', '/zstd')))
    return start


def generate_launch_description():

    sl = SimpleLauncher()

    has_camera_ros = False

    try:
        get_package_prefix('camera_ros')
        has_camera_ros = True
    except:
        pass


    cam_info = 'file://' + sl.find('blurr', 'calibration.yaml')
    dev = '/dev/video0'
    frame_id = 'camera'
    height = 600
    width = 800

    if has_camera_ros:
        sl.node('camera_ros', 'camera_node', name = 'camera',
                remappings = im_remap('~/image_raw', {'~/camera_info': 'camera_info'}),
                parameters = {'camera': 1,
                              'camera_info_url': cam_info,
                              'frame_id': frame_id,
                              'format': 'MJPEG',
                              'width': width,
                              'height': height})
    else:
        sl.node('v4l2_camera', 'v4l2_camera_node', name = 'camera',
                remappings = im_remap('image_raw'),
                parameters = {#'pixel_format': 'mjpg',
                        'video_device': dev,
                        'camera_frame_id': frame_id,
                        'image_size': [width,height],
                        'camera_info_url': cam_info})


    return sl.launch_description()
