"""
Performs collision detection on convex 2D polygons by means of the separating axis theorem (SAT).

The contents of this file are based of off a Python implementation of SAT created by Juan Antonio Aldea Armenteros.
The original version is available at https://github.com/JuantAldea/Separating-Axis-Theorem/. That code is under the
GNU General Public License, but I (Joel Eager) have received written permission to distribute this modified version
under the MIT license.
"""

# Obtained from: https://github.com/JoelEager/PythonInteropBenchmarking/blob/master/python_implementation.py


def edge_vector(point1, point2):
    """
    :return: A vector going from point1 to point2
    """
    return point2[0] - point1[0], point2[1] - point1[1]


def poly_to_edges(poly):
    """
    Runs edgeVector() on each point paired with the point after it in the poly
    :return: A list of the edges of the poly as vectors
    """
    return [edge_vector(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))]


def orthogonal(vector):
    """
    :return: A new vector which is orthogonal to the given vector
    """
    return vector[1], -vector[0]


def dot_product(vector1, vector2):
    """
    :return: The dot (or scalar) product of the two vectors
    """
    return vector1[0] * vector2[0] + vector1[1] * vector2[1]


def project(poly, axis):
    """
    :return: A vector showing how much of the poly lies along the axis
    """
    dots = [dot_product(point, axis) for point in poly]
    return min(dots), max(dots)


def overlap(projection1, projection2):
    """
    :return: Boolean indicating if the two projections overlap
    """
    return min(projection1) <= max(projection2) and min(projection2) <= max(projection1)


def has_collided(poly1, poly2):
    """
    Checks for a collision between two convex 2D polygons using separating axis theorem (SAT).
    :param poly1, poly2: The two polygons described as collections of points as tuples
        Example: ((x1, y1), (x2, y2), (x3, y3))
        Note: The points list must go in sequence around the polygon
    """
    edges = poly_to_edges(poly1) + poly_to_edges(poly2)
    axes = [orthogonal(edge) for edge in edges]

    for axis in axes:
        overlapping = overlap(project(poly1, axis), project(poly2, axis))

        if not overlapping:
            # The polys don't overlap on this axis so they can't be touching
            return False

    # The polys overlap on all axes so they must be touching
    return True
