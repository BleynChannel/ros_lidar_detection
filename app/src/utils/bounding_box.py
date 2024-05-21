import numpy as np

class BoundingBox:
    """ Simple data class representing a 3d box including, label, score and velocity. """

    def __init__(self, center, size, orientation, velocity=(np.nan, np.nan, np.nan)):
        """
        :param center: [<float>: 3]. Center of box given as x, y, z.
        :param size: [<float>: 3]. Size of box in width, length, height.
        :param orientation: <Quaternion>. Box orientation.
        :param velocity: [<float>: 3]. Box velocity in x, y, z direction.
        """
        assert not np.any(np.isnan(center))
        assert not np.any(np.isnan(size))
        assert len(center) == 3
        assert len(size) == 3

        self.center = np.array(center)
        self.wlh = np.array(size)
        self.orientation = orientation
        self.velocity = np.array(velocity)

    def corners(self, wlh_factor=1.0):
        """
        Returns the bounding box corners.
        :param wlh_factor: <float>. Multiply w, l, h by a factor to inflate or deflate the box.
        :return: <np.float: 3, 8>. First four corners are the ones facing forward.
            The last four are the ones facing backwards.
        """
        w, l, h = self.wlh * wlh_factor

        # 3D bounding box corners. (Convention: x points forward, y to the left, z up.)
        x_corners = l / 2 * np.array([1,  1,  1,  1, -1, -1, -1, -1])
        y_corners = w / 2 * np.array([1, -1, -1,  1,  1, -1, -1,  1])
        z_corners = h / 2 * np.array([1,  1, -1, -1,  1,  1, -1, -1])
        corners = np.vstack((x_corners, y_corners, z_corners))

        # Rotate
        corners = np.dot(self.orientation.rotation_matrix, corners)

        # Translate
        x, y, z = self.center
        corners[0, :] = corners[0, :] + x
        corners[1, :] = corners[1, :] + y
        corners[2, :] = corners[2, :] + z

        return corners
    
    def bounding(self, wlh_factor=1.0):
        """
        Returns the bounding box.
        :param wlh_factor: <float>. Multiply w, l, h by a factor to inflate or deflate the box.
        :return: <np.float: 2, 3>. The first corner is the smallest corner of the box.
            The second corner is the largest corner of the box.
        """
        w, l, h = self.wlh * wlh_factor

        # 3D bounding box corners. (Convention: x points forward, y to the left, z up.)
        x_corners = l / 2 * np.array([-1, 1])
        y_corners = w / 2 * np.array([-1, 1])
        z_corners = h / 2 * np.array([-1, 1])
        corners = np.vstack((x_corners, y_corners, z_corners))

        # Rotate
        corners = np.dot(self.orientation.rotation_matrix, corners)

        # Translate
        x, y, z = self.center
        corners[0, :] = corners[0, :] + x
        corners[1, :] = corners[1, :] + y
        corners[2, :] = corners[2, :] + z

        min_bound, max_bound = corners[:, 0], corners[:, 1]
        bound = np.vstack((min_bound, max_bound))
        return bound