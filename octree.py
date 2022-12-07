import math
import open3d as o3d
import numpy as np
import laspy
from collections import deque
import matplotlib.pyplot as plt


class Cube:
    def __init__(self, cube_dict) -> None:
        self.cube_dict = cube_dict

    def draw_cube(self, ax) -> None:
        xs = self.cube_dict["start"][0]
        ys = self.cube_dict["start"][1]
        zs = self.cube_dict["start"][2]
        xf = self.cube_dict["finish"][0]
        yf = self.cube_dict["finish"][1]
        zf = self.cube_dict["finish"][2]

        a = np.max([xf - xs, yf - ys, zf - zs])

        p1 = [xs, ys, zs]
        p2 = [xs + a, ys, zs]
        p3 = [xs + a, ys + a, zs]
        p4 = [xs, ys + a, zs]
        p5 = [xs, ys, zs + a]
        p6 = [xs + a, ys, zs + a]
        p7 = [xs + a, ys + a, zs + a]
        p8 = [xs, ys + a, zs + a]

        x = []
        y = []
        z = []
        list_of_movements = [p1, p2, p3, p4, p1, p5, p6, p7, p8, p5, p8, p4, p3, p7, p3, p2, p6]
        for point in list_of_movements:
            x.append(point[0])
            y.append(point[1])
            z.append(point[2])

        r = np.array(x)
        s = np.array(y)
        t = np.array(z)
        ax.plot3D(r, s, t)

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


def limit_sphere(sphere: Sphere, point_cloud):
    lists = deque()
    for point in point_cloud:
        if sphere.contains(point[0], point[1], point[2]):
            lists.append(point)
    point_cloud = np.array(lists)
    return point_cloud


def read_data(filename: str):
    las = laspy.read(filename)
    point_data = np.stack([las.X, las.Y, las.Z], axis=0).transpose((1, 0))
    return point_data, point_data.min(axis=0), point_data.max(axis=0)


def draw_octree_lib(point_cloud: list) -> None:
    geom = o3d.geometry.PointCloud()
    geom.points = o3d.utility.Vector3dVector(point_cloud)
    octree = o3d.geometry.Octree(max_depth=6)
    octree.convert_from_point_cloud(geom, size_expand=0.01)
    sphere_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=1000)
    coords = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3000)
    o3d.visualization.draw_geometries([coords, octree, sphere_mesh])


def draw_octree_cube(ax, cube_dict: dict, point_cloud, depth=None):
    if depth == None:
        depth = 0
    initial_start = cube_dict["start"]
    initial_finish = cube_dict["finish"]
    cube = Cube(cube_dict)
    cube.draw_cube(ax)
    for point in point_cloud:
        if cube.contains(point[0], point[1], point[2]) and depth <= 3:
            print(depth)
            depth += 1
            cube_dict["finish"] = [
                (cube_dict["finish"][0] + cube_dict["start"][0]) / 2,
                (cube_dict["finish"][1] + cube_dict["start"][1]) / 2,
                (cube_dict["finish"][2] + cube_dict["start"][2]) / 2,
            ]
            # draw_octree_cube(ax, cube_dict, point_cloud)
            # cube_dict["start"] = cube_dict["finish"]
            # cube_dict["finish"] = initial_finish

            # draw_octree_cube(ax, cube_dict, point_cloud)
            # draw_octree_cube(ax, cube_dict, point_cloud)
            # draw_octree_cube(ax, cube_dict, point_cloud)
            # draw_octree_cube(ax, cube_dict, point_cloud)
            # draw_octree_cube(ax, cube_dict, point_cloud)
            # draw_octree_cube(ax, cube_dict, point_cloud)
            # draw_octree_cube(ax, cube_dict, point_cloud)

            return draw_octree_cube(ax, cube_dict, point_cloud, depth)
    return


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


fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
draw_octree("2743_1234.las", ax)

plt.show()
