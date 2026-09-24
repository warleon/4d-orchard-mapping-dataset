from sensor_msgs.msg import PointCloud2, PointField
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array

CLOUD_REGISTERED_FIELD = [
    {"name": "x", "offset": 0, "datatype": 7, "count": 1},
    {"name": "y", "offset": 4, "datatype": 7, "count": 1},
    {"name": "z", "offset": 8, "datatype": 7, "count": 1},
    {"name": "normal_x", "offset": 16, "datatype": 7, "count": 1},
    {"name": "normal_y", "offset": 20, "datatype": 7, "count": 1},
    {"name": "normal_z", "offset": 24, "datatype": 7, "count": 1},
    {"name": "intensity", "offset": 32, "datatype": 7, "count": 1},
    {"name": "curvature", "offset": 36, "datatype": 7, "count": 1},
]


def fix_point_cloud_message_fields(point_cloud_message: PointCloud2):
    original_fields_list = point_cloud_message.fields
    assert original_fields_list and len(original_fields_list) == len(
        CLOUD_REGISTERED_FIELD
    )
    corrected_fields_list = []
    for i in range(len(original_fields_list)):
        corrected_fields_list.append(
            PointField(
                name=CLOUD_REGISTERED_FIELD[i]["name"],
                offset=CLOUD_REGISTERED_FIELD[i]["offset"],
                datatype=CLOUD_REGISTERED_FIELD[i]["datatype"],
                count=CLOUD_REGISTERED_FIELD[i]["count"],
            )
        )
    point_cloud_message.fields = corrected_fields_list
    return point_cloud_message


def point_cloud_message_to_numpy(point_cloud_message: PointCloud2):
    fix_point_cloud_message_fields(point_cloud_message)
    return pointcloud2_to_xyz_array(point_cloud_message)
