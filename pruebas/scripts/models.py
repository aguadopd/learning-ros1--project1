"""Data models for our nodes

Here we have classes to model our reality.

TODO:
- Write more tests. Here we only have some minimal doctests.
- Currently we cannot give orientation to our found objects.
- See if we could use a better Point abstraction. Maybe Numpy has one. Perhaps it was better to extend Tuple.
  - Actually we might be using vector everywhere, so this is probably done in ROS. We should
    search for the canonical way of working with vectors.
- Check the units of the world. Are units stable?
- We should load _max_identity_distance from somewhere else.
"""

# IMPORTS
# -------------------------------------------------------------------------------

# Python's Data Classes seems good for these cases
from dataclasses import dataclass, field

# For abstract classes
from abc import ABC, abstractmethod
from typing import Callable, ClassVar, List, Dict, Type

# Path manipulation
import os

# GLOBALS
# -------------------------------------------------------------------------------
# The following should provide '{package}/src/imgs_for_maps'
dir_of_this_file: str = os.path.dirname(os.path.realpath(__file__))
dir_of_imgs_for_maps: str = os.path.join(os.path.dirname(dir_of_this_file), "src", "imgs_for_map")


# -------------------------------------------------------------------------------
@dataclass
class Point:
    """A point in the 3D space. Unit-agnostic.

    >>> p1 = Point(1, -1, 3.0)
    >>> print(p1)
    Point(x=1, y=-1, z=3.0)
    >>> p2 = Point(x=1, y=1, z=4)
    >>> round(p1.distance_to(p2, "euclidean"),3)
    2.236

    """
    x: float
    y: float
    z: float

    # CLASS stuff
    # -------------------------------

    # INSTANCE stuff
    # -------------------------------
    # Types in 'quotes' are forward references for type hinting: https://www.python.org/dev/peps/pep-0484/#forward-references
    # These are needed since the class itself has not been created yet, so we cannot reference it.
    # More in https://stackoverflow.com/questions/54621348/how-to-refer-to-class-methods-when-defining-class-variables-in-python
    def _distance_euclidean(self: 'Point', to_: 'Point') -> float:
        """Euclidean distance from a point to another"""
        return (
            (to_.x - self.x)**2
            + (to_.y - self.y)**2
            + (to_.z - self.z)**2
            ) ** 0.5

    # A dictionary for dispatching distance functions from method 'distance_to'
    # 'ClassVar[]' is type hinting for class variables.
    # We could create distance classes too...
    _distance_functions: ClassVar[Dict[str, Callable]] = {
            "euclidean": _distance_euclidean,
            # "block":
            # "some_other_distance_metric"
            }

    def distance_to(self, other: 'Point', method: str = "euclidean") -> float:
        """Return (float) distance to another point using the chosen 'method'"""
        return self._distance_functions[method](self, other)


@dataclass
class TridimensionalModel(ABC):
    """Some tridimensional shape."""
    # An abstract property to  be implemented by any model
    @property
    @abstractmethod
    def _unitary_2d_image_path(self) -> str:
        """Path to the image that will be scaled and overlayed on a drawn map."""
        pass

@dataclass
class SquareBasedRectangularPrism(TridimensionalModel):
    """A tridimensional rectangular prism with a square base, parallel to the ground plane.

    Its position refers to the center of its base.
    """
    side: float
    height: float
    _unitary_2d_image_path: str = os.path.join(dir_of_imgs_for_maps, "square_based_rectangular_prism.png")

@dataclass
class FoundObject:
    """A 3D object found in the space.

    No orientation yet. We assume its axes are parallel to the reference axes.

    Parameters
    ----------
    reference_point
        A Point with the coordinates of some relevant feature of the object.
    label
        The object's type or class.
    """
    reference_point: Point
    # orientation_vector: Point = Point(x=0, y=1, z=0)  # To be implemented.
    label: str

@dataclass
class FoundSquareBasedRectangularPrism(SquareBasedRectangularPrism, FoundObject):
    """Found 3D rectangular prism with a square base, parallel to the ground.

    No orientation yet. We assume its axes are parallel to the reference axes.

    >>> fr = FoundSquareBasedRectangularPrism(Point(1,1,0), side=2, height=9, label="person")
    >>> fr.corners
    [Point(x=0.0, y=0.0, z=0), Point(x=2.0, y=0.0, z=0), Point(x=2.0, y=2.0, z=0), Point(x=0.0, y=2.0, z=0)]

    Parameters
    ----------
    reference_point
        A Point with the coordinates of the center of the base
    side
        Length of each side of the base
    height
        Length of any side other than the base or the top

    """

    @property
    def corners(self) -> List[Point]:
        """CCW-ordered list of 4 base corners. First one is closest to the origin."""
        p = self.reference_point  # alias
        half_side = self.side / 2
        bottom_left = Point(p.x-half_side, p.y-half_side, p.z)
        bottom_right = Point(p.x+half_side, p.y-half_side, p.z)
        top_right = Point(p.x+half_side, p.y+half_side, p.z)
        top_left = Point(p.x-half_side, p.y+half_side, p.z)
        return [bottom_left, bottom_right, top_right, top_left]


# -------------------------------------------------------------------------------

class ListOfFoundObjects(List):
    """ A list of objects found in the space


    >>> fr1 = FoundSquareBasedRectangularPrism(Point(0,1,0), side=2, height=9, label="person")
    >>> fr2 = FoundSquareBasedRectangularPrism(Point(2,2,0), side=2, height=9, label="person")
    >>> l = ListOfFoundObjects()
    >>> l._max_identity_distance = 0.1
    >>> l.update_list(fr1)
    >>> l.update_list(fr1)  # Same position, shouldn't add anything
    >>> len(l)
    1
    >>> l.update_list(fr2)
    >>> len(l)
    2
    >>> fr3 = FoundSquareBasedRectangularPrism(Point(0,1,0), side=2, height=9, label="lamp")
    >>> l.update_list(fr3)  # Another label, so it should be added despite being in the same place as fr1
    >>> len(l)
    3
    """

    _max_identity_distance = 0.1  # meters?


    def _is_object_already_in_list(self, object_: FoundObject) -> bool:

        def objects_are_the_same(ob1: FoundObject, ob2: FoundObject) -> bool:
            if (ob1.label == ob2.label) \
                and (ob1.reference_point.distance_to(ob2.reference_point) <= self._max_identity_distance):
                return True
            else:
                return False

        return True if any(objects_are_the_same(object_, existing) for existing in self) else False


    def update_list(self, object_: FoundObject) -> None:
        assert isinstance(object_, FoundObject), f"Only can add objects of type FoundObject"
        if not self._is_object_already_in_list(object_):
            super().append(object_)
        else:
            pass


    def append(self, object_: FoundObject) -> None:
        return self.update_list(object_)

    def extend(self, __iterable: 'ListOfFoundObjects') -> None:
        assert isinstance(__iterable, ListOfFoundObjects)
        return super().extend(__iterable)

    def __add__(self, x):
        raise NotImplementedError

