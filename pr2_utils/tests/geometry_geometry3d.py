"""
Tests for geometry/geometry3d.py
"""

from geometry.geometry3d import *
from pr2_sim import simulator
        
def test_align_with():
    t = Triangle([0,0,1.2], [0,1,1.2], [1,0,1.2])
    
    t_rot, rot = t.align_with([0,0,1])
    
    print('t_rot:\n{0}\n{1}\n{2}'.format(t_rot.a, t_rot.b, t_rot.c))
        
def test_distance_to():
    t = Triangle([0,0,0], [0,1,0], [1,0,0])
    
    p = [0, 0, 1]
    dist = t.distance_to(p)
    print('Distance should be 1')
    print('Computed distance is: {0}'.format(dist))
    
    p = [-1, 0, 1]
    dist = t.distance_to(p)
    print('Distance should be sqrt(2)')
    print('Computed distance is: {0}'.format(dist))
        
def test_distance_to_plot():
    sim = simulator.Simulator(view=True)
    
    t = Triangle(np.random.rand(3), np.random.rand(3), np.random.rand(3))
    t.plot(sim)
    
    p = 2*np.random.rand(3)
    closest_pt = t.closest_point_to(p)
    
    sim.plot_point(p, color=(0,0,1))
    sim.plot_point(closest_pt)
    
    IPython.embed()
    
def test_pyramid_inside():
    sim = simulator.Simulator(view=True)
    
    base = [.5,0,0]
    a = [.6, .1, .5]
    b = [.4, .1, .5]
    c = [.4, -.1, .5]
    
    pyramid = Pyramid(base, a, b, c)
    pyramid.plot(sim)
    
    halfspaces = pyramid.halfspaces
    for h in halfspaces:
        h.plot(sim)
        
    for i in xrange(1000):
        p = [np.random.uniform(.4,.6),
             np.random.uniform(-.1,.1),
             np.random.uniform(0,.5)]
        is_inside = pyramid.is_inside(p)
        print('is_inside: {0}'.format(is_inside))
        sim.plot_point(p)
        if is_inside:
            raw_input()
        sim.clear_plots(1)
        
    
    IPython.embed()
         
def test_clip_triangle():
    sim = simulator.Simulator(view=True)
    
    base = [.5,0,0]
    a = [.6, .1, .5]
    b = [.4, .1, .5]
    c = [.4, -.1, .5]
    d = [.6, -.1, .5]
    
    frustum = RectangularPyramid(base, a, b, c, d)
    frustum.plot(sim)
    
#     triangle = Triangle([.5, 0, .25], [.5, 0, .6], [.52, .02, .25])
#     triangle = Triangle([.5, 0, .25], [.7, .5, .5], [.9, .1, .6])
    triangle = Triangle([.6, .1, .6], [.7, .2, .8], [.5, 0, .6])
    triangle.plot(sim, color=(0,0,1))
    
    print('Original triangle, press enter to clip')
    raw_input()
    sim.clear_plots(3)
    
    clipped_triangles = frustum.clip_triangle(triangle)
     
    print('Number of clipped triangles: {0}'.format(len(clipped_triangles)))
    for tri in clipped_triangles:
        tri.plot(sim, color=(0,1,0))
    
    print('Press enter to exit')
    raw_input()
    
def test_point():
    p = Point([1,2,3])
    q = Point([1,2,3.000001])
    
    print('p == q: {0}'.format(p == q))
    
    s = set()
    s.add(p)
    s.add(q)
    print('len(s): {0}'.format(len(s)))
    print('s:\n{0}'.format(s))
         
def test_pyramid_sd():
    sim = simulator.Simulator(view=True)
    
    base = [1,0,0]
    a = [1.5, .5, 2]
    b = [.5, .5, 2]
    c = [.5, -.5, 2]
    
    pyramid = Pyramid(base, a, b, c)
    pyramid.plot(sim, frame='base_link')
    
    halfspaces = pyramid.halfspaces
    for h in halfspaces:
        h.plot(sim)
        
    pos_step = .01
    delta_position = {'a' : [0, pos_step, 0],
                      'd' : [0, -pos_step, 0],
                      'w' : [pos_step, 0, 0],
                      'x' : [-pos_step, 0, 0],
                      '+' : [0, 0, pos_step],
                      '-' : [0, 0, -pos_step]}
    point = np.array(base, dtype=float)
    
    print('Move point around with keyboard to test signed-distance')
    char = ''
    while char != 'q':
        char = utils.Getch.getch()
        
        sim.clear_plots(1)
        if delta_position.has_key(char):
            point += np.array(delta_position[char])
        sim.plot_point(sim.transform_from_to(point, 'base_link', 'world'), size=.02, color=(0,1,0))
        
        is_inside = pyramid.is_inside(point)
        sd = pyramid.signed_distance(point)
        print('is inside: {0}'.format(is_inside))
        print('sd: {0}\n'.format(sd))

def test_plotting():
    sim = simulator.Simulator(view=True)
    
    sim.plot_triangle(([1,0,1],[1,0,2],[1.5,0,1.5]),color=(1,0,0),alpha=0.5)
    
    print('Press enter to exit')
    raw_input()

if __name__ == '__main__':
    #test_align_with()
    #test_distance_to()
    #test_distance_to_plot()
    #test_pyramid_inside()
    #test_clip_triangle()
    #test_point()
    #test_pyramid_sd()
    test_plotting()