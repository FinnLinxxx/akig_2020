import cv2
import numpy as np
import ruamel.yaml

# Define coordinates after mouse clicking
def click_event(event, x, y, flags, params):
    global points_list
    if event == cv2.EVENT_LBUTTONDOWN:
        # Print image coordinates in console
        print('Image coordinates: ' + str(x) + ', ' + str(y) + ' px')
        # Append coordinates to existing list
        points_list.append((x, y))

# Define image coordinates
def getImageCoordinates(size):
    # Open window with changeable size
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    # Read image
    img = cv2.imread('ig_2stock_map_30092020.pgm', -1)
    # Change size
    imgS = cv2.resize(img, (size, size))
    # Show downsized image
    cv2.imshow('image', imgS)
    # Get image coordiantes by mouse clicking
    cv2.setMouseCallback('image', click_event)
    # Close window after any key is pressed
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Calculate angle between two vectors
def angleVectors(point1, point2):
    inner_product = np.inner(point1, point2)
    angle = np.arccos(inner_product/(np.linalg.norm(point1)*np.linalg.norm(point2)))
    
    # Check if angle is greater or less than 180°
    if (point2[1] - point1[1]) < 0:
        angle = 2*np.pi - angle
    
    return angle

# Calculate orientation and scale between both systems and change yaml-file for desired visualization in rviz
def changeMapData(origin_px, map_point_px, world_vector, size):
    yaml = ruamel.yaml.YAML()
    # Read yaml-file
    with open('ig_2stock_map_30092020.yaml') as file:
        data = yaml.load(file)
    # Define yaml-layout
    yaml.default_flow_style = None
    
    # Scale of downsized window and resolution of image in m/px
    window_scale = 4000/size # Image has size of 4000x4000 px
    resolution = data['resolution']
    # Origin and map point in m
    origin = origin_px*window_scale*resolution
    map_point = map_point_px*window_scale*resolution
    # Vector in map system (left-handed)
    map_vector = map_point - origin
    # Change sign of y-coordinate of map_vector, yielding a right-handed coordinate system
    # (Import for angle determination)
    map_vector[1] = -map_vector[1]
    
    # Calculate angle and scale between both systems
    # Angles between vectors and respective x-axis
    x_axis = np.array([1., 0.])
    omega_map = angleVectors(x_axis, map_vector)
    omega_world = angleVectors(x_axis, world_vector)
    # Angle between both systems
    theta = omega_map - omega_world
    # Scale
    scale = np.sqrt(world_vector[0]**2 + world_vector[1]**2)/np.sqrt(map_vector[0]**2 + map_vector[1]**2)
    
    # Change yaml-file
    # Calculate appropriate origin, referred to rviz visualization
    origin_x = -origin[0]
    origin_y = -(200 - origin[1])
    origin_rviz = np.array([origin_x, origin_y])
    # Rotate origin and adjust scale
    R = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
    origin_rviz_rot_scaled = np.dot(R, origin_rviz)*scale
    # Change origin data
    data['origin'] = [origin_rviz_rot_scaled[0].tolist(), origin_rviz_rot_scaled[1].tolist(), -theta.tolist()]
    # Change scale data
    resolution_new = resolution*scale
    data['resolution'] = resolution_new
    
    # Create new yaml-file
    with open('ig_2stock_map_30092020_new.yaml', 'w') as file:
        yaml.dump(data, file)

if __name__ == '__main__':
    # Create empty point list and get image coordinates via getImageCoordinates function
    points_list = []
    # Size of window in px for getting image coordinates
    size = 700
    # Determine image coordinates
    getImageCoordinates(size)
    points = np.array(points_list)
    
    # Origin in px
    origin_px = points[0]
    # Map point in px
    map_point_px = points[1]
    # World vector in m, after asking user for input
    world_vector_x = float(input('x_coordinate of world vector: '))
    world_vector_y = float(input('y_coordinate of world vector: '))
    world_vector = np.array([world_vector_x, world_vector_y])
    
    # Calculate orientation and scale between both systems and change yaml-file for desired visualization in rviz
    changeMapData(origin_px, map_point_px, world_vector, size)


