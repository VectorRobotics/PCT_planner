import numpy as np
from sensor_msgs.msg import PointField


# For ROS2, PointField needs to be created differently
def create_point_field(name, offset, datatype, count):
    field = PointField()
    field.name = name
    field.offset = offset
    field.datatype = datatype
    field.count = count
    return field


POINT_FIELDS_XYZI = [
    create_point_field('x', 0, PointField.FLOAT32, 1),
    create_point_field('y', 4, PointField.FLOAT32, 1),
    create_point_field('z', 8, PointField.FLOAT32, 1),
    create_point_field('intensity', 12, PointField.FLOAT32, 1)
]


def GRID_POINTS_XYZI(resolution, dim_x, dim_y):
    index_proto = np.zeros((dim_x * dim_y, 2), dtype=int)
    lx = np.linspace(0, dim_x - 1, dim_x, dtype=int)
    ly = np.linspace(0, dim_y - 1, dim_y, dtype=int)
    ix, iy = np.meshgrid(lx, ly)
    index_proto[:, 0] = ix.flatten()
    index_proto[:, 1] = iy.flatten()

    point_proto = np.zeros((dim_x * dim_y, 4), dtype=np.float32)
    point_proto[:, :2] = index_proto[:, :2].astype(np.float32, copy=True)
    point_proto[:, 0] -= 0.5 * dim_x
    point_proto[:, 1] -= 0.5 * dim_y
    point_proto[:, :2] *= resolution
    point_proto[:, 3] = 1.0

    return index_proto, point_proto