import math


MODE = "shortcut"  # center, shortcut

MAX_SIGHT = 1.0


def dist(point1, point2):
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5


def rect(r, theta):
    x = r * math.cos(math.radians(theta))
    y = r * math.sin(math.radians(theta))
    return x, y


def polar(x, y):

    r = (x ** 2 + y ** 2) ** 0.5
    theta = math.degrees(math.atan2(y, x))
    return r, theta


def angle_mod_360(angle):
    n = math.floor(angle / 360.0)

    angle_between_0_and_360 = angle - n * 360.0

    if angle_between_0_and_360 <= 180.0:
        return angle_between_0_and_360
    else:
        return angle_between_0_and_360 - 360


def get_waypoints_ordered_in_driving_direction(params):
    if MODE == "shortcut":
        waypoints = get_shortcut_waypoints()
    else:
        waypoints = params["waypoints"]

    if params["is_reversed"]:  # driving clock wise.
        return list(reversed(waypoints))
    else:  # driving counter clock wise.
        return waypoints

def up_sample(waypoints, factor=10):
    p = waypoints
    n = len(p)

    return [[i / factor * p[int((j + 1) % n)][0] + (1 - i / factor) * p[j][0],
             i / factor * p[int((j + 1) % n)][1] + (1 - i / factor) * p[j][1],] for j in range(n) for i in range(factor)]


def get_target_point(params):
    waypoints = up_sample(get_waypoints_ordered_in_driving_direction(params), 20)
    car = [params["x"], params["y"]]
    distances = [dist(p, car) for p in waypoints]
    min_dist = min(distances)
    i_closest = distances.index(min_dist)
    n = len(waypoints)
    waypoints_starting_with_closest = [waypoints[(i + i_closest) % n] for i in range(n)]

    if MODE == "shortcut":
        sight = MAX_SIGHT * 0.8
    else:
        sight = MAX_SIGHT

    r = params["track_width"] * sight

    is_inside = [dist(p, car) < r for p in waypoints_starting_with_closest]
    i_first_outside = is_inside.index(False)

    if i_first_outside < 0:
        # this can only happen if we choose r as big as the entire track
        return waypoints[i_closest]

    return waypoints_starting_with_closest[i_first_outside]


def get_target_steering_degree(params):
    tx, ty = get_target_point(params)
    car_x = params["x"]
    car_y = params["y"]
    dx = tx - car_x
    dy = ty - car_y
    heading = params["heading"]

    _, target_angle = polar(dx, dy)

    steering_angle = target_angle - heading

    return angle_mod_360(steering_angle)


def score_steer_to_point_ahead(params):
    best_stearing_angle = get_target_steering_degree(params)
    steering_angle = params["steering_angle"]

    error = (
        steering_angle - best_stearing_angle
    ) / 60.0  # 60 degree is already really bad

    score = 1.0 - abs(error)

    return max(
        score, 0.01
    )  # optimizer is rumored to struggle with negative numbers and numbers too close to zero


def reward_function(params):
    return float(score_steer_to_point_ahead(params))


def get_shortcut_waypoints():
    return [[2.90990644, 0.98407875],
       [3.31990641, 0.98420088],
       [3.4199064 , 0.98423067],
       [3.62991234, 0.98429322],
       [4.18990633, 0.98446003],
       [4.49991226, 0.98455237],
       [4.5499063 , 0.98456726],
       [5.31991219, 0.98479663],
       [5.41991218, 0.98482642],
       [5.77991215, 0.98493365],
       [6.2553082 , 0.98873702],
       [6.42648362, 1.00861532],
       [6.43307007, 1.00938019],
       [6.58552838, 1.07519014],
       [6.65414056, 1.12115314],
       [6.76767029, 1.20752563],
       [6.86513054, 1.34568336],
       [6.88302339, 1.3793611 ],
       [6.97243225, 1.74776337],
       [6.96960177, 1.76566469],
       [6.96188883, 1.81444516],
       [6.86346611, 2.10632023],
       [6.7690622 , 2.22274942],
       [6.60065918, 2.36186218],
       [6.4314447 , 2.43673386],
       [6.08237788, 2.46239682],
       [5.92238881, 2.46107404],
       [5.72240248, 2.45942058],
       [5.67257125, 2.45900858],
       [5.20260337, 2.45512292],
       [5.01861984, 2.45668914],
       [4.96918566, 2.4619779 ],
       [4.90986465, 2.46832442],
       [4.39902628, 2.63606792],
       [4.01871064, 2.97147931],
       [3.85290613, 3.174899  ],
       [3.76129959, 3.28728782],
       [3.5341778 , 3.56593551],
       [3.44257126, 3.67832433],
       [3.30923655, 3.84190797],
       [3.09720703, 4.08122162],
       [3.02891715, 4.1234712 ],
       [2.99478647, 4.14414731],
       [2.88433502, 4.18227081],
       [2.77887123, 4.19261457],
       [2.78081412, 4.19242402],
       [2.50940672, 4.18829973],
       [2.25845167, 4.18096164],
       [1.99925809, 4.17338266],
       [1.74859519, 4.16605312],
       [1.31461304, 4.11409424],
       [1.2783205 , 4.08888491],
       [1.02484721, 3.77043495],
       [0.99661118, 3.59068639],
       [1.16676675, 2.78707146],
       [1.17861572, 2.73111083],
       [1.21104008, 2.57797629],
       [1.23142423, 2.4817055 ],
       [1.31634694, 2.08063044],
       [1.33863869, 1.97535044],
       [1.39027886, 1.73146298],
       [1.49782294, 1.28158859],
       [1.51857442, 1.22992349],
       [1.52968872, 1.20225212],
       [1.5434439 , 1.18662043],
       [1.61366728, 1.10681699],
       [1.63223282, 1.08571869],
       [1.64459173, 1.07881414],
       [2.03991248, 0.9838196 ],
       [2.74991242, 0.98403109],
       [2.90990644, 0.98407875]]