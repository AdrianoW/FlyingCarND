import numpy as np
from sklearn.neighbors import KDTree
from shapely.geometry import Polygon, Point

class Poly:

    def __init__(self, coords, height):
        self._polygon = Polygon(coords)
        self._height = height

    @property
    def height(self):
        return self._height

    @property
    def coords(self):
        return list(self._polygon.exterior.coords)[:-1]
    
    @property
    def area(self):
        return self._polygon.area

    @property
    def center(self):
        return (self._polygon.centroid.x, self._polygon.centroid.y)

    def contains(self, point):
        point = Point(point)
        return self._polygon.contains(point)

    def crosses(self, other):
        return self._polygon.crosses(other)



def extract_polygons(data):

    polygons = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        
        obstacle = [north - d_north, north + d_north, east - d_east, east + d_east]
        corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), (obstacle[1], obstacle[3]), (obstacle[1], obstacle[2])]
        
        # TODO: Compute the height of the polygon
        height = alt + d_alt

        p = Poly(corners, height)
        polygons.append(p)

    return polygons

class Sampler:

    def __init__(self, data):
        self._polygons = extract_polygons(data)
        self._xmin = np.min(data[:, 0] - data[:, 3])
        self._xmax = np.max(data[:, 0] + data[:, 3])

        self._ymin = np.min(data[:, 1] - data[:, 4])
        self._ymax = np.max(data[:, 1] + data[:, 4])

        self._zmin = 0
        # limit z-axis
        self._zmax = 20
        # Record maximum polygon dimension in the xy plane
        # multiply by 2 since given sizes are half widths
        # This is still rather clunky but will allow us to 
        # cut down the number of polygons we compare with by a lot.
        self._max_poly_xy = 2 * np.max((data[:, 3], data[:, 4]))
        centers = np.array([p.center for p in self._polygons])
        self._tree = KDTree(centers, metric='euclidean')

    def sample(self, num_samples):
        """Implemented with a k-d tree for efficiency."""
        xvals = np.random.uniform(self._xmin, self._xmax, num_samples)
        yvals = np.random.uniform(self._ymin, self._ymax, num_samples)
        zvals = np.random.uniform(self._zmin, self._zmax, num_samples)
        samples = list(zip(xvals, yvals, zvals))

        pts = []
        for s in samples:
            in_collision = False
            idxs = list(self._tree.query_radius(np.array([s[0], s[1]]).reshape(1, -1), r=self._max_poly_xy)[0])
            if len(idxs) > 0:
                for ind in idxs: 
                    p = self._polygons[int(ind)]
                    if p.contains(s) and p.height >= s[2]:
                        in_collision = True
            if not in_collision:
                pts.append(s)
                
        return pts
    
    def sample_in_margin(self, num_samples, start, goal, margin=50):
        
        def find_y_between_lines(num, m, b, b1, b2):
            '''
            Given a set of x, find random y that are inside parallel lines distance
            '''
            y_min = m*num+b1
            y_max = m*num+b2
            y= np.random.uniform(0,1, num.shape[0]) * (y_max - y_min) + y_min
            return y
        
        # find the max and min x to calculate y, making sure they are inside the grid
        min_x = np.min([start[1]-margin, goal[1]-margin])
        max_x = np.max([start[1]+margin, goal[1]+margin])
        print(min_x, self._xmin)
        print(max_x, self._xmax)
        min_x = np.max([min_x, self._xmin])
        max_x = np.min([max_x, self._xmax])
        xvals = np.random.uniform(min_x, max_x, num_samples)
        # yvals = np.random.uniform(self._ymin, self._ymax, num_samples)
        
        # find the slope between the start and end goal
        diff = np.array(start)[0:2] - np.array(goal)[0:2]
        slope = diff[1]/diff[0]
        b = start[0] - start[1]*slope

        # create 2 parallel lines that will limit the y space
        b1 = b - margin*np.sqrt(slope**2+1)
        b2 = b + margin*np.sqrt(slope**2+1)

        # create the random y and z according to the limits
        yvals = np.apply_along_axis(find_y_between_lines, 0, xvals, slope, b, b1, b2)
        zvals = np.random.uniform(self._zmin, self._zmax, num_samples)
        samples = list(zip(xvals, yvals, zvals))

        # for the generated points, check if they are hittig or no.
        pts = []
        for s in samples:
            in_collision = False
            idxs = list(self._tree.query_radius(np.array([s[0], s[1]]).reshape(1, -1), r=self._max_poly_xy)[0])
            if len(idxs) > 0:
                for ind in idxs: 
                    p = self._polygons[int(ind)]
                    if p.contains(s) and p.height >= s[2]:
                        in_collision = True
            if not in_collision:
                pts.append(s)
                
        return pts
        

    @property
    def polygons(self):
        return self._polygons

