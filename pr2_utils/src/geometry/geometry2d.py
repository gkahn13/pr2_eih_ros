import numpy as np
import p2t # polygon triangulation
from matplotlib import pyplot as plt

epsilon = 1e-5

class Polygon:
    """ Simple polygon """
    def __init__(self, points):
        """
        :param points: [[x0,y0],[x1,y1],...]
        """
        self.points = points
        
    def triangulate(self):
        """"
        Returns triangles that form the same shape as the polygon
        :return list of Triangle
        """
        p2t_points = [p2t.Point(p[0], p[1]) for p in self.points]
        cdt = p2t.CDT(p2t_points)
        p2t_triangles = cdt.triangulate()
        
        triangles = [Triangle([t.a.x, t.a.y], [t.b.x, t.b.y], [t.c.x, t.c.y]) for t in p2t_triangles]
        return triangles
    
    def plot(self, axes, color='r'):
        """
        :param axes: pyplot axes
        :param color: character or (r,g,b) [0,1]
        """
        x_list, y_list = list(), list()
        for x, y in (self.points + [self.points[0]]):
            x_list.append(x)
            y_list.append(y)
        axes.plot(x_list, y_list, color=color)

class ImagePlane:
    def __init__(self, p0, p1, p2, p3):
        """
        p0 ---- p1
        |       |
        p3 ---- p2
        :param p0, p1, p2, p3: each is 2d points (list/np.array) of corners of ImagePlane
        """
        self.p0, self.p1, self.p2, self.p3 = np.array(p0), np.array(p1), np.array(p2), np.array(p3)
        self.segments = [Segment(p0, p1), Segment(p1, p2), Segment(p2, p3), Segment(p3, p0)]
        
        # assuming ImagePlane is rectangle
        self.halfspaces =  [Halfspace((self.p0+self.p1)/2., self.p2-self.p1),
                            Halfspace((self.p1+self.p2)/2., self.p3-self.p2),
                            Halfspace((self.p2+self.p3)/2., self.p0-self.p3),
                            Halfspace((self.p3+self.p0)/2., self.p1-self.p0)]

    def truncate(self, triangles):
        
        triangle_segments, triangle_halfspaces = list(), list()
        for triangle in triangles:
            triangle_segments += triangle.segments()
            triangle_halfspaces += triangle.halfspaces()
        
        image_plane_segments, image_plane_halfspaces = list(), list()
        
        # find segments on border that are not in a triangle
        sort_keys = [lambda p: p[0], lambda p: -p[1], lambda p: -p[0], lambda p: p[1]]
        for segment, halfspace, sort_key in zip(self.segments, self.halfspaces, sort_keys):
            intersections = filter(lambda x: x is not None, [segment.intersection(s) for s in triangle_segments])
            intersections += [segment.p0, segment.p1]
            intersections = sorted(intersections, key=sort_key)
            
            intersection_segments = [Segment(intersections[i], intersections[i+1]) for i in xrange(len(intersections)-1)]
            for seg in intersection_segments:
                crosses_triangle = max([tri.is_inside((seg.p0+seg.p1)/2.) for tri in triangles] + [False])
                if not crosses_triangle:
                    image_plane_segments.append(seg)
                    image_plane_halfspaces.append(halfspace)
                    
        # find polygons while there still exit unused image_plane_segments
        polygons = list()
        while len(image_plane_segments) > 0:
            print('Number of image plane segments left: {0}'.format(len(image_plane_segments)))
            segments = image_plane_segments + triangle_segments
            halfspaces = image_plane_halfspaces + triangle_halfspaces
            
            polygon, segments_used = self.find_polygon(segments, halfspaces, segments[0], halfspaces[0])
            polygons.append(polygon)
            
            # filter out used segments from image_plane_segments
            new_image_plane_segments, new_image_plane_halfspaces = list(), list()
            for segment, halfspace in zip(image_plane_segments, image_plane_halfspaces):
                if segment not in segments_used:
                    new_image_plane_segments.append(segment)
                    new_image_plane_halfspaces.append(halfspace)
                    
            image_plane_segments = new_image_plane_segments
            image_plane_halfspaces = new_image_plane_halfspaces
            
        return polygons
        
            
    def find_polygon(self, segments, halfspaces, start_segment, start_halfspace):
        curr_segment = start_segment
        curr_halfspace = start_halfspace
        curr_point = start_segment.p0
        
        start_point = curr_point
        points = [start_point]
        
        segments_used = list()
        
        while True:
            print('\nCurrent segment: ({0}, {1})'.format(curr_segment.p0, curr_segment.p1))
            print('Current point: {0}'.format(curr_point))
            
            # find intersecting segment closest to curr_point
            closest_dist = np.inf
            intersect_segment, intersect_halfspace, intersect_point = None, None, None
            
            for other_segment, other_halfspace in zip(segments, halfspaces):
                if curr_halfspace != other_halfspace: # make sure not comparing to current segment
                    intersection = curr_segment.intersection(other_segment)
                    if intersection is not None:
                        dist = np.linalg.norm(intersection - curr_point)
                        if dist > epsilon and dist < closest_dist:
                            closest_dist = dist
                            intersect_segment = other_segment
                            intersect_halfspace = other_halfspace
                            intersect_point = intersection
                            
            assert intersect_segment is not None
            segments_used.append(intersect_segment)
            
            print('Intersecting segment: ({0}, {1})'.format(intersect_segment.p0, intersect_segment.p1))
            print('Intersect point: {0}'.format(intersect_point))
            
            # move along intersecting segment in direction of the normal
            if curr_halfspace.contains(intersect_segment.p0):
                next_segment = Segment(intersect_point, intersect_segment.p0)
            elif curr_halfspace.contains(intersect_segment.p1):
                next_segment = Segment(intersect_point, intersect_segment.p1)
            else:
                next_segment = intersect_segment
                
            next_halfspace = intersect_halfspace
            next_point = intersect_point
            
            # if back at the start, have found the whole polygon
            if np.linalg.norm(next_point - start_point) < epsilon:
                break
            points.append(next_point)
            
            curr_segment, curr_halfspace, curr_point = next_segment, next_halfspace, next_point
            
            #print('Press enter')
            #raw_input()
            
        return Polygon(points), segments_used
    
    def plot(self, axes, color='r'):
        """
        :param axes: pyplot axes
        :param color: character or (r,g,b) [0,1]
        """
        for segment in self.segments:
            segment.plot(axes, color=color)
            
            

class Triangle:
    def __init__(self, a, b, c):
        self.a, self.b, self.c = np.array(a), np.array(b), np.array(c)
        
    def closest_point_to(self, x):
        """
        Finds closest point on triangle to x
        by checking distances to the triangle edges
        
        :param p: 2d list or np.array
        :return if x is inside the triangle None, else 2d np.array of closest point
        """
        if self.is_inside(x):
            return None
        
        min_pt, min_dist = None, np.inf
        for s in self.segments():
            s_min_pt = s.closest_point_to(x)
            if np.linalg.norm(x - s_min_pt) < min_dist:
                min_dist = np.linalg.norm(x - s_min_pt)
                min_pt = s_min_pt
                
        return min_pt
    
    def is_inside(self, x):
        """
        :param x: 2d list or np.array
        :return True if x is inside, else False
        """
        total_area = self.area()
        area0 = Triangle(self.a, self.b, x).area()
        area1 = Triangle(self.b, self.c, x).area()
        area2 = Triangle(self.c, self.a, x).area()
        
        is_correct_area = np.abs(total_area - (area0 + area1 + area2)) < epsilon
        
        return is_correct_area
    
    def area(self):
        """
        :return float area
        """
        a, b, c = self.a, self.b, self.c
        return np.abs((c[0]*(a[1] - b[1]) + a[0]*(b[1] - c[1]) + b[0]*(c[1] - a[1])) / 2.0)
        
    def segments(self):
        """
        :return [edge0, edge1, edge2]
        """
        return (Segment(self.a, self.b), Segment(self.b, self.c), Segment(self.c, self.a))
    
    def halfspaces(self):
        """
        halfspaces of edges pointing outwards
        :return [hyp0, hyp1, hyp2]
        """
        segments = (Segment(self.a, self.b), Segment(self.b, self.c), Segment(self.c, self.a))
        other_points = (self.c, self.a, self.b)
        
        hspaces = list()
        for segment, other_point in zip(segments, other_points):
            origin = (segment.p0 + segment.p1)/2.
            
            colinear = segment.p1 - segment.p0
            normal = np.array([-colinear[1], colinear[0]])
            if np.dot(normal, other_point - origin) < 0:
                hspaces.append(Halfspace(origin, normal))
            else:
                hspaces.append(Halfspace(origin, -normal))
            
        return hspaces
    
    def plot(self, axes, color='r'):
        """
        :param axes: pyplot axes
        :param color: character or (r,g,b) [0,1]
        """
        axes.plot([self.a[0], self.b[0], self.c[0], self.a[0]], [self.a[1], self.b[1], self.c[1], self.a[1]], color=color)
        
    @staticmethod
    def random(min_x, max_x, min_y, max_y):
        return Triangle([np.random.uniform(min_x, max_x), np.random.uniform(min_y, max_y)],
                        [np.random.uniform(min_x, max_x), np.random.uniform(min_y, max_y)],
                        [np.random.uniform(min_x, max_x), np.random.uniform(min_y, max_y)])
     
# class Line:
#     def __init__(self, p0, p1):
#         self.p0, self.p1 = np.array(p0), np.array(p1)
#          
#     def closest_point_to(self, x):
#         """
#         min ||t*(p1-p0) + p0 - x||_{2}^{2}
#         
#         :param x: 2d list or np.array
#         :return 2d np.array of closest point on line to x
#         """
#         x = np.array(x)
#         v = self.p1 - self.p0
#         b = self.p0 - x
#         
#         t = -np.dot(v, b) / np.dot(v, v)
#         closest = t*(self.p1 - self.p0) + self.p0
#         return closest
#             
#     def intersection(self, other):
#         """
#         Finds intersection point with another segment
#         :param other: Segment
#         :return None if no intersection, else [x,y] of intersection 
#         """
#         p0_other, p1_other = other.p0, other.p1
#         
#         # w = p1 - p0
#         # v = p1_other - p0_other
#         # s*w + p0 = t*v + p_other
#         
#         w = self.p1 - self.p0
#         v = p1_other - p0_other
#         
#         A = np.vstack((w,v)).T
#         b = p0_other - self.p0
#         
#         if np.abs(np.linalg.det(A)) < epsilon:
#             return None
#         
#         soln = np.linalg.solve(A, b)
#         s, t = soln[0], -soln[1]
#         
#         intersection = s*w + self.p0
#         
#         if ((-epsilon <= s) and (s <= 1+epsilon) and (-epsilon <= t) and (t <= 1+epsilon)):
#             return intersection
#         else:
#             return None
#             
#     def plot(self, axes, color='r'):
#         """
#         :param axes: pyplot axes
#         :param color: character or (r,g,b) [0,1]
#         """
#         axes.plot([self.p0[0], self.p1[0]], [self.p0[1], self.p1[1]], color=color)
        
class Segment:
    def __init__(self, p0, p1):
        self.p0, self.p1 = np.array(p0), np.array(p1)
        
    def closest_point_to(self, x):
        """
        min_{0<=t<=1} ||t*(p1-p0) + p0 - x||_{2}^{2}
        
        :param x: 2d list or np.array
        :return 2d np.array of closest point on segment to x
        """
        x = np.array(x)
        v = self.p1 - self.p0
        b = self.p0 - x
        
        t = -np.dot(v, b) / np.dot(v, v)
        if (0 <= t <= 1):
            closest = t*(self.p1 - self.p0) + self.p0
            return closest
        else:
            if np.linalg.norm(x - self.p0) < np.linalg.norm(x - self.p1):
                return self.p0
            else:
                return self.p1
            
    def intersection(self, other):
        """
        Finds intersection point with another segment
        :param other: Segment
        :return None if no intersection, else [x,y] of intersection 
        """
        p0_other, p1_other = other.p0, other.p1
        
        # w = p1 - p0
        # v = p1_other - p0_other
        # s*w + p0 = t*v + p_other
        
        w = self.p1 - self.p0
        v = p1_other - p0_other
        
        A = np.vstack((w,v)).T
        b = p0_other - self.p0
        
        if np.abs(np.linalg.det(A)) < epsilon:
            return None
        
        soln = np.linalg.solve(A, b)
        s, t = soln[0], -soln[1]
        
        intersection = s*w + self.p0
        
        if ((-epsilon <= s) and (s <= 1+epsilon) and (-epsilon <= t) and (t <= 1+epsilon)):
            return intersection
        else:
            return None
        
    def is_endpoint(self, point):
        """
        :param point: 2d np.array
        :return True if point is an endpoint of the segment
        """
        return np.linalg.norm(self.p0 - point) < epsilon or np.linalg.norm(self.p1 - point) < epsilon
            
    def plot(self, axes, color='r'):
        """
        :param axes: pyplot axes
        :param color: character or (r,g,b) [0,1]
        """
        axes.plot([self.p0[0], self.p1[0]], [self.p0[1], self.p1[1]], color=color)
        
        
class Halfspace:
    def __init__(self, origin, normal):
        self.origin = np.array(origin, dtype=float)
        self.normal = np.array(normal, dtype=float)
        self.normal /= np.linalg.norm(self.normal)
        
    def contains(self, x):
        """
        :param x: 2d point as list or np.array
        :return True if x forms acute angle with plane normal, else False
        """
        return np.dot(self.normal, np.array(x) - self.origin) >= epsilon
    
    def plot(self, axes, color='r'):
        """
        Plots the normal
        
        :param axes: pyplot axes
        :param color: character or (r,g,b) [0,1]
        """
        x_list = [self.origin[0], self.origin[0] + self.normal[0]]
        y_list = [self.origin[1], self.origin[1] + self.normal[1]]
        axes.plot(x_list, y_list, color=color)
        
###########
#  TESTS  #
###########

def test_delaunay():
    from matplotlib import delaunay
    
    points = [(0,0), (10,0), (10,10), (0,10),
              (5,5), (8,3), (9,6)]
    
    x = [p[0] for p in points]
    y = [p[1] for p in points]
    
    circumcenters, edges, tri_points, tri_neighbors = delaunay.delaunay(x,y)
    
    triangles = list()
    for indices in tri_points:
        p0 = points[indices[0]]
        p1 = points[indices[1]]
        p2 = points[indices[2]]
        triangles.append(Triangle(p0, p1, p2))
    
    fig = plt.figure()
    axes = fig.add_subplot(111)
    
    for tri in triangles:
        tri.plot(axes, color='b')

    xmin, xmax = plt.xlim()
    ymin, ymax = plt.ylim()
    plt.xlim((xmin-1, xmax+1))
    plt.ylim((ymin-1, ymax+1))
        
    plt.show(block=False)
    
    import IPython
    IPython.embed()

def test_image_plane_truncate():
    image_plane = ImagePlane([0,10],[10,10],[10,0],[0,0])
    triangles = [Triangle.random(-2,12,-2,12) for i in xrange(5)]
    
    fig = plt.figure()
    axes = fig.add_subplot(111)
    
    image_plane.plot(axes, color='r')
    for tri in triangles:
        tri.plot(axes, color='b')
    
    xmin, xmax = plt.xlim()
    ymin, ymax = plt.ylim()
    plt.xlim((xmin-1, xmax+1))
    plt.ylim((ymin-1, ymax+1))
        
    plt.show(block=False)
        
    print('Image plane and triangles')
    print('Press enter to view truncation')
    raw_input()
    
    polygons = image_plane.truncate(triangles)
    
    axes.clear()
    for polygon in polygons:
        polygon.plot(axes, color='c')
        
    plt.xlim((xmin-1, xmax+1))
    plt.ylim((ymin-1, ymax+1))
    
    plt.show(block=False)
    
    print('Press enter')
    raw_input()

def test_triangle_halfspaces():
    #tri = Triangle([1,1],[2,1],[0,0])
    tri = Triangle([0,0],[1,0],[0,1])
    hspaces = tri.halfspaces()
    
    fig = plt.figure()
    axes = fig.add_subplot(111)
    tri.plot(axes, 'b')
    for hspace in hspaces:
        hspace.plot(axes)
        
    plt.show(block=False)
    raw_input()

def test_triangulate():
    poly = Polygon([[0,0],[-2,-1],[-1,-2],[1,-.5]])
    triangles = poly.triangulate()
    
    fig = plt.figure()
    axes = fig.add_subplot(111)
    for tri in triangles:
        tri.plot(axes, 'r')
        
    plt.show(block=False)
    raw_input()

def test_plotting():
    fig = plt.figure()
    axes = fig.add_subplot(111)
    
    seg = Segment([0,0], [-1,2])
    seg.plot(axes, 'g')
    
    tri = Triangle([1,1],[2,1],[0,0])
    tri.plot(axes, 'r')
    
    poly = Polygon([[0,0],[-2,-1],[-1,-2],[1,-.5]])
    poly.plot(axes,'b')
    
    plt.show(block=False)
    raw_input()
        
if __name__ == '__main__':
    #test_plotting()
    #test_triangulate()
    #test_triangle_halfspaces()
    test_image_plane_truncate()
    #test_delaunay()