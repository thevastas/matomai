import math
from typing import Dict, Deque
import open3d as o3d
import numpy as np
import laspy
from collections import deque
import matplotlib.pyplot as plt


class Node:
    def __init__(self, cube_dict: Dict[str, list]) -> None:
        self.cube_dict = cube_dict

    def draw_box(self, ax) -> None:
        xs = self.cube_dict["start"][0]
        ys = self.cube_dict["start"][1]
        zs = self.cube_dict["start"][2]
        xf = self.cube_dict["finish"][0]
        yf = self.cube_dict["finish"][1]
        zf = self.cube_dict["finish"][2]

        side = np.max([xf - xs, yf - ys, zf - zs])

        point_1 = [xs, ys, zs]
        point_2 = [xs + side, ys, zs]
        point_3 = [xs + side, ys + side, zs]
        point_4 = [xs, ys + side, zs]
        point_5 = [xs, ys, zs + side]
        point_6 = [xs + side, ys, zs + side]
        point_7 = [xs + side, ys + side, zs + side]
        point_8 = [xs, ys + side, zs + side]

        x = []
        y = []
        z = []

        order_of_movements = [
            point_1,
            point_2,
            point_3,
            point_4,
            point_1,
            point_5,
            point_6,
            point_7,
            point_8,
            point_5,
            point_8,
            point_4,
            point_3,
            point_7,
            point_3,
            point_2,
            point_6,
        ]
        for point in order_of_movements:
            x.append(point[0])
            y.append(point[1])
            z.append(point[2])

        ax.plot3D(x, y, z)
        return side

    def contains(self, x: float, y: float, z: float) -> bool:
        return (
            self.cube_dict["start"][0] < x
            and self.cube_dict["finish"][0] > x
            and self.cube_dict["start"][1] < y
            and self.cube_dict["finish"][1] > y
            and self.cube_dict["start"][2] < z
            and self.cube_dict["finish"][2] > z
        )


class Sphere:
    def __init__(self, x: float, y: float, z: float, radius: float) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.r = radius

    def contains(self, xc: float, yc: float, zc: float) -> bool:
        distance = math.sqrt(pow(xc - self.x, 2) + pow(yc - self.y, 2) + pow(zc - self.z, 2))
        return distance <= self.r


def limit_sphere(sphere: Sphere, point_cloud: np.ndarray):
    lists: Deque = deque()
    for point in point_cloud:
        if sphere.contains(point[0], point[1], point[2]):
            lists.append(point)
    point_cloud = np.array(lists)
    return point_cloud


def read_data(filename: str):
    las = laspy.read(filename)
    point_data = np.stack([las.X, las.Y, las.Z], axis=0).transpose((1, 0))
    return point_data, point_data.min(axis=0), point_data.max(axis=0)


def draw_octree_lib(point_cloud: np.ndarray) -> None:
    geom = o3d.geometry.PointCloud()
    geom.points = o3d.utility.Vector3dVector(point_cloud)
    octree = o3d.geometry.Octree(max_depth=6)
    octree.convert_from_point_cloud(geom, size_expand=0.01)
    sphere_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=1000)
    coords = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3000)
    o3d.visualization.draw_geometries([coords, octree, sphere_mesh])


def draw_octree_cube(ax, cube_dict: dict, point_cloud: np.ndarray, depth=None):
    if depth == None:
        depth = 0

    node = Node(cube_dict)
    node.draw_box(ax)

    displacement = cube_dict["finish"][0] + cube_dict["start"][0] / 2

    for point in point_cloud:
        if node.contains(point[0], point[1], point[2]) and depth <= 3:
            depth += 1
            cube_dict["finish"] = [displacement, displacement, displacement]
            return draw_octree_cube(ax, cube_dict, point_cloud, depth)
    # TODO: 8 nodes should be drawn with different locations based on displacement
    # The area needs to be limited by a sphere before beginning the search
    # The search of points in the node should be done by going through the whole list and only then breaking the branch
    # recursion is done until there are no more points left in the node or depth threshold is reached
    # if depth reached the threshold and there are still points in the node, the node should be colored


def draw_octree(filename: str, ax):
    point_cloud, minimal_values, maximal_values = read_data(filename)
    x_min = minimal_values[0]
    y_min = minimal_values[1]
    z_min = minimal_values[2]
    x_max = maximal_values[0]
    y_max = maximal_values[1]
    z_max = maximal_values[2]
    cube_dict = {"start": [x_min, y_min, z_min], "finish": [x_max, y_max, z_max]}
    draw_octree_cube(ax, cube_dict, point_cloud)


if __name__ == "__main__":
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    draw_octree("2743_1234.las", ax)
    plt.show()
