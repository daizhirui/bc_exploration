"""costmap.py
Costmap class for exploration
"""
from __future__ import print_function, absolute_import, division

import cv2
import numpy as np

from bc_exploration.utilities.paths import get_exploration_dir
from bc_exploration.utilities.util import which_coords_in_bounds


class Costmap:
    """
    Costmap to be used for exploration.
    NOTE THIS IS NOT TO BE CONFUSED WITH CostMap2D
    THIS MAP IS USED FOR EXPLORATION
    CostMap2D was not used in an effort to keep this repo independent
    also the values are different because it makes visualization 10x easier,
    also when saving the map it looks like a map
    """

    FREE = 255
    OCCUPIED = 0
    UNEXPLORED = 127

    def __init__(self, data, resolution, origin):
        """
        Costmap object treated as a struct tying together the map data, resolution, and origin
        :param data array(N, M)[Union[uint8, float, int]]: 2d ndarray corresponding to the costmap data
        :param resolution float: resolution of the map in meters
        :param origin array(2)[float]: [x, y] origin of the costmap
        """
        # todo getters and setters, but for now we will treat as struct
        assert data.dtype == np.uint8
        self.data = data
        self.resolution = resolution
        self.origin = np.array(origin)

    def get_shape(self):
        """
        gets the shape of the data array of the map
        :return Tuple[int]: (row, col)
        """
        return self.data.shape

    def get_size(self):
        """
        gets the size in meters of each dimension of the map
        :return Tuple[float]: (x, y)
        """
        size = self.resolution * np.array(self.data.shape)
        size[[0, 1]] = size[[1, 0]]
        return tuple(size)

    def copy(self):
        """
        Returns a copy of the costmap
        :return Costmap: copy of the current costmap
        """
        return Costmap(self.data.copy(), self.resolution, self.origin.copy())

    def visualize(self, render_size, wait_key):
        """
        Simple visualizer for the costmap
        :param render_size Tuple(int): size to render the visualization window
        :param wait_key int: opencv wait key for visualization
        """
        cv2.namedWindow("map.visualize", cv2.WINDOW_GUI_NORMAL)
        cv2.imshow("map.visualize", self.data)
        cv2.resizeWindow("map.visualize", *render_size)
        cv2.waitKey(wait_key)

    def get_downscaled(self, desired_resolution):
        """
        Return a downscaled version of the costmap, makes sure to preserve obstacles and free space that is marked
        :param desired_resolution float: resolution in meters of the downscaled costmap
        :return Costmap: object
        """
        assert self.resolution <= desired_resolution
        scale_factor = self.resolution / desired_resolution
        scaled_shape = np.rint(np.array(self.data.shape) * scale_factor).astype(int)
        scaled_data = np.zeros(scaled_shape)

        scaled_occupied_coords = np.rint(
            np.argwhere(self.data == Costmap.OCCUPIED) * scale_factor
        ).astype(int)
        scaled_unexplored_coords = np.rint(
            np.argwhere(self.data == Costmap.UNEXPLORED) * scale_factor
        ).astype(int)
        scaled_free_coords = np.rint(
            np.argwhere(self.data == Costmap.FREE) * scale_factor
        ).astype(int)

        scaled_occupied_coords = scaled_occupied_coords[
            which_coords_in_bounds(scaled_occupied_coords, scaled_shape)
        ]
        scaled_unexplored_coords = scaled_unexplored_coords[
            which_coords_in_bounds(scaled_unexplored_coords, scaled_shape)
        ]
        scaled_free_coords = scaled_free_coords[
            which_coords_in_bounds(scaled_free_coords, scaled_shape)
        ]

        # order is important here, we want to make sure to keep the obstacles
        scaled_data[scaled_free_coords[:, 0], scaled_free_coords[:, 1]] = Costmap.FREE
        scaled_data[
            scaled_unexplored_coords[:, 0], scaled_unexplored_coords[:, 1]
        ] = Costmap.UNEXPLORED
        scaled_data[
            scaled_occupied_coords[:, 0], scaled_occupied_coords[:, 1]
        ] = Costmap.OCCUPIED

        return Costmap(
            data=scaled_data.astype(np.uint8),
            resolution=desired_resolution,
            origin=self.origin,
        )

    def save(self, filename):
        """
        saves the map to file relative to the exploration folder
        :param filename str: relative filepath of which to save
        """
        cv2.imwrite(get_exploration_dir() + "/" + filename, self.data)

    def to_brain_costmap(self):
        """
        Convert current costmap to a CostMap2D object that corresponds to brain's values.
        Note the origin is just the resolution scaled of the current costmap
        :return CostMap2D: brain formatted costmap
        """
        costmap_data = self.data.copy()
        costmap_data[
            self.data == Costmap.FREE
        ] = CostMap2D.FREE_SPACE  # pylint: disable=undefined-variable
        costmap_data[
            self.data == Costmap.OCCUPIED
        ] = CostMap2D.LETHAL_OBSTACLE  # pylint: disable=undefined-variable
        costmap_data[
            self.data == Costmap.UNEXPLORED
        ] = CostMap2D.NO_INFORMATION  # pylint: disable=undefined-variable

        return CostMap2D(
            data=np.flipud(costmap_data),
            resolution=self.resolution,  # pylint: disable=undefined-variable
            origin=self.origin.astype(np.float64),
        )

    @staticmethod
    def from_brain_costmap(
        brain_costmap,
    ):  # pylint: disable=extra-argument-docstring, undefined-variable
        """
        Convert a brain CostMap2D object to a Costmap object. We need to remove the padding that is given, and
        change the values to Costmap format
        :param brain_costmap CostMap2D: brain costmap object
        :return Costmap: exploration formatted costmap
        """
        brain_costmap_data = brain_costmap.get_data().astype(np.uint8)
        brain_costmap_resolution = brain_costmap.get_resolution()

        brain_costmap_data = np.flipud(brain_costmap_data)

        costmap_data = brain_costmap_data.copy()
        costmap_data[
            brain_costmap_data == CostMap2D.FREE_SPACE
        ] = Costmap.FREE  # pylint: disable=undefined-variable
        costmap_data[
            brain_costmap_data == CostMap2D.LETHAL_OBSTACLE
        ] = Costmap.OCCUPIED  # pylint: disable=undefined-variable
        costmap_data[
            brain_costmap_data == 100
        ] = Costmap.OCCUPIED  # another brain occupied value
        costmap_data[
            brain_costmap_data == CostMap2D.NO_INFORMATION
        ] = Costmap.UNEXPLORED  # pylint: disable=undefined-variable
        return Costmap(
            data=costmap_data,
            resolution=brain_costmap_resolution,
            origin=brain_costmap.get_origin(),
        )
