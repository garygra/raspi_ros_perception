import os
import sys
import time
import pdb

import rosbag
import imageio
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(suppress=True)


plan_files = [
    {
        "name": "plan1",
        "odo": "../out/plan1_odo.bag",
        "perc": "../out/plan1_perc.bag",
        "sim": "../out/traj1.txt",
        "vid": "../out/image_bags/plan1_vid.bag",
        "goal": [.5, -.5, 0],
    },
    {
        "name": "plan2",
        "odo": "../out/plan2_odo.bag",
        "perc": "../out/plan2_perc.bag",
        "sim": "../out/traj2.txt",
        "vid": "../out/image_bags/plan2_vid.bag",
        "goal": [-1, 0, 3.14],
    },
    {
        "name": "plan3",
        "odo": "../out/plan3_odo.bag",
        "perc": "../out/plan3_perc.bag",
        "sim": "../out/traj3.txt",
        "vid": "../out/image_bags/plan3_vid.bag",
        "goal": [0, 0, 3.14],
    },
    {
        "name": "plan4",
        "odo": "../out/plan4_odo.bag",
        "perc": "../out/plan4_perc.bag",
        "sim": "../out/traj4.txt",
        "vid": "../out/image_bags/plan4_vid.bag",
        "goal": [-1.5, .3, 3.14/2],
    },
]

cam_extrinsics = np.array([
    [-0.99955112,0.01122324,0.02777774,0.02823084],
    [-0.00464414,0.85792783,-0.51374922,-0.01628532],
    [-0.02959723,-0.51364761,-0.8574906,0.2125679],
    [0.,0.,0.,1.]
])

# x_converter = 0.865
# y_converter = 1.02

x_converter = 1
y_converter = 0.785


odom_topic = '/car/vesc/odom'
perc_topic = '/camera_logitech/pracsys/markers'
vid_topic = '/camera_logitech/pracsys/rgb_image'
mushr_tag_id = 0

plot_axes_dim = [-2.5, 2.5, -2.5, 2.5]
plot_fig_size = (16,16)

def get_odo_traj(bag_file):
    traj = []

    with rosbag.Bag(bag_file, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[odom_topic]):
            traj.append([msg.pose.pose.position.x,msg.pose.pose.position.y])

    return np.vstack(traj)

def perform_projection(x,y, rotation_vector, translation_vector):
    R_mark_to_cam, _ = cv2.Rodrigues(rotation_vector)
    T_mark_to_cam = np.eye(4)
    T_mark_to_cam[:3,:3] = R_mark_to_cam
    T_mark_to_cam[:3,3] = translation_vector

    T_robot = np.matmul(np.linalg.inv(cam_extrinsics), T_mark_to_cam)

    Pw_robot = T_robot[:,3]

    return Pw_robot[1]*y_converter, -Pw_robot[0]*x_converter

    # homogeneous_coordinates = np.concatenate((pixel_coordinates, [1]))

    # pdb.set_trace()

    # transformed_coordinates = np.dot(np.linalg.inv(rotation_matrix), homogeneous_coordinates - translation_vector)
    # print(transformed_coordinates)
    # transformed_coordinates += translation_vector

    # transformed_coordinates = transformed_coordinates[:2]/transformed_coordinates[2]
    # return transformed_coordinates[0], transformed_coordinates[1]

def get_perc_det_time(bag_file, raw=False):
    tag_traj = {}
    init_mushr_coordinates = None
    
    with rosbag.Bag(bag_file, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[perc_topic]):
            for tag_details in msg.markers:
                tag_id = tag_details.id
                if tag_id != 0: continue

def get_perc_traj(bag_file, raw=False):
    tag_traj = {}
    init_mushr_coordinates = None
    
    with rosbag.Bag(bag_file, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[perc_topic]):
            start_time = time.time()
            for tag_details in msg.markers:
                tag_id = tag_details.id
                if tag_id != 0: continue

                x_values = []
                x_values.append(tag_details.x1)
                x_values.append(tag_details.x2)
                x_values.append(tag_details.x3)
                x_values.append(tag_details.x4)

                y_values = []
                y_values.append(tag_details.y1)
                y_values.append(tag_details.y2)
                y_values.append(tag_details.y3)
                y_values.append(tag_details.y4)

                rot_vector = np.array([
                    tag_details.r1,
                    tag_details.r2,
                    tag_details.r3,
                ])

                trans_vector = np.array([
                    tag_details.t1,
                    tag_details.t2,
                    tag_details.t3,
                ])

                x = np.mean(x_values)
                y = np.mean(y_values)

                proj_x, proj_y = perform_projection(x,y, rot_vector, trans_vector) if not raw else (y, x)

                if tag_id not in tag_traj: tag_traj[tag_id] = []

                tag_traj[tag_id].append([proj_x,proj_y])

                if tag_id == mushr_tag_id and init_mushr_coordinates is None:
                    init_mushr_coordinates = np.array([proj_x, proj_y])

    for tag_id in tag_traj:
        traj = tag_traj[tag_id]
        traj = np.vstack(traj)
        traj -= init_mushr_coordinates # To set the origin at where the mushr begins
        tag_traj[tag_id] = traj
    return tag_traj

def get_sim_traj(sim_traj_file):
    traj = []
    with open(sim_traj_file , 'r') as file:
        lines = file.readlines()
        for line in lines:
            x, y, orientation = line.strip('\n').split(',')

            x = np.float64(x)
            y = np.float64(y)

            traj.append([x,y])

    return np.vstack(traj)

def get_perc_msg_time_plot(bag_file):
    init_time = None

    bucket_size = 1 * 1e9 # 1sec
    counts = []

    with rosbag.Bag(bag_file, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[perc_topic]):
            time_in_secs = int(msg.header.stamp.secs)
            time_in_nsecs = int(msg.header.stamp.nsecs) + (time_in_secs * 1e9)

            if init_time is None:
                init_time = time_in_nsecs

            nsecs_since_start = time_in_nsecs - init_time

            bucket_number = int(nsecs_since_start // bucket_size)

            while len(counts) < (bucket_number + 1):
                counts.append(0)
            
            is_mushr_detected = False

            for tag_details in msg.markers:
                if tag_details.id == mushr_tag_id:
                    is_mushr_detected = True
                    break

            if is_mushr_detected: counts[bucket_number] += 1

    return counts

def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8")
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return cv2.cvtColor(image_opencv, cv2.COLOR_BGR2RGB)

def get_video(vid_file):
    video = []
    with rosbag.Bag(vid_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[vid_topic]):
            image = imgmsg_to_cv2(msg)
            video.append(image)
    return video

def plot_odo(odo_traj):
    plt.plot(odo_traj[:,1],np.negative(odo_traj[:,0]), 'green', label='Odometry', linestyle=':')
    plt.plot(odo_traj[-1,1],np.negative(odo_traj[-1,0]), 'green', marker='x')

def plot_sim(sim_traj):
    plt.plot(sim_traj[-1,1],np.negative(sim_traj[-1,0]), 'red', marker='x')
    plt.plot(sim_traj[:,1],np.negative(sim_traj[:,0]), 'red', label='Simulation', linestyle='dashdot')


def plot_perc(perc_traj, show_all_tags=False, show_scatter=False):
    linestyle = '-'
    marker = 'x'
    markersize = 10

    if not show_all_tags:
        color = 'blue'
        plt.plot(perc_traj[mushr_tag_id][:, 1], np.negative(perc_traj[mushr_tag_id][:, 0]), color, label = 'Perception', linestyle=linestyle)
        plt.plot(perc_traj[mushr_tag_id][-1, 1], np.negative(perc_traj[mushr_tag_id][-1, 0]), color, marker = marker, markersize=markersize)
    else:
        plot_fn = plt.scatter if show_scatter else plt.plot
        for tag_id in perc_traj:
            label = f'Perception - Tag ID: {tag_id}' if tag_id != mushr_tag_id else 'Perception - MuSHR'
            plot_fn(perc_traj[tag_id][:, 1], np.negative(perc_traj[tag_id][:, 0]), label=label, linestyle=linestyle)

            if tag_id == mushr_tag_id:
                plt.plot(perc_traj[mushr_tag_id][-1, 1], np.negative(perc_traj[mushr_tag_id][-1, 0]), marker = marker, markersize=markersize)

def plot_init(axes_dim=plot_axes_dim):
    plt.figure(figsize=plot_fig_size)
    if axes_dim is not None:
        plt.axis(axes_dim)

def plot_goal(goal):
    radius = 0.25

    goal_x = goal[1]
    goal_y = -goal[0]

    mag = 2 * radius/1.2

    arrow_x = mag * np.sin(goal[2])
    arrow_y = -mag * np.cos(goal[2])

    plt.annotate('', xy=(goal_x + (arrow_x/2), goal_y + (arrow_y/2)),
     xycoords='data',
     xytext=(goal_x - (arrow_x/2), goal_y - (arrow_y/2)),
     textcoords='data',
     arrowprops=dict(arrowstyle= '-|>',
                     color='green',
                     lw=2,
                     ls='-')
    )

    theta = np.linspace( 0 , 2 * np.pi , 150 )
    circle_x = np.negative(radius * np.cos( theta ) + goal[0])
    circle_y = radius * np.sin( theta ) + goal[1]
    plt.plot(circle_y, circle_x, color='green')

    return goal_x, goal_y

def plot_save(file_name, plan_name, goal=None, show_goal=True):
    if show_goal and goal is not None:
        actual_goal_x, actual_goal_y = plot_goal(goal)
        plt.title(f'{plan_name} - Goal State: ({actual_goal_x}, {actual_goal_y}, {goal[2]*180/3.14} degrees)')
        plt.plot(0, 0, color='black', marker='o', label='Initial Point')

    plt.legend()
    plt.savefig(file_name)
    plt.close()

def plot_msg_hist(bag_file):
    init_time = None

    plot_values = []

    with rosbag.Bag(bag_file, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[perc_topic]):
            time_in_secs = (int(msg.header.stamp.nsecs)/1e9) + (int(msg.header.stamp.secs))

            if init_time is None:
                init_time = time_in_secs


            is_mushr_detected = False

            for tag_details in msg.markers:
                if tag_details.id == mushr_tag_id:
                    is_mushr_detected = True
                    break
            
            plot_values.append([
                time_in_secs - init_time,
                1 if is_mushr_detected else 0
            ])
    plot_values = np.array(plot_values)

    plt.scatter(plot_values[:,0], plot_values[:,1], s=1, marker='o')

def main():
    for plan_file_details in plan_files:
        plan_name = plan_file_details['name']
        odo_bag_file = plan_file_details['odo']
        perc_bag_file = plan_file_details['perc']
        sim_traj_file = plan_file_details['sim']
        vid_bag_file = plan_file_details['vid']
        goal = plan_file_details['goal']

        try:
            os.mkdir('./{}'.format(plan_name))
        except Exception:
            pass

        odo_traj = get_odo_traj(odo_bag_file)
        perc_traj = get_perc_traj(perc_bag_file)
        raw_perc_traj = get_perc_traj(perc_bag_file, raw=True)
        sim_traj = get_sim_traj(sim_traj_file)
        video = get_video(vid_bag_file)

        plot_init()
        plot_odo(odo_traj)
        plot_sim(sim_traj)
        plot_perc(perc_traj)
        plot_save('{}/all_traj.png'.format(plan_name), goal=goal, plan_name=plan_name)


        plot_init()
        plot_odo(odo_traj)
        plot_save('{}/odo.png'.format(plan_name), goal=goal, plan_name=plan_name)

        plot_init()
        plot_sim(sim_traj)
        plot_save('{}/sim.png'.format(plan_name), goal=goal, plan_name=plan_name)

        plot_init()
        plot_perc(perc_traj)
        plot_save('{}/perc.png'.format(plan_name), goal=goal, plan_name=plan_name)

        plot_init(None)
        plot_perc(raw_perc_traj, show_all_tags=True)
        plot_save(f'{plan_name}/raw_perc.png', goal=goal, plan_name=plan_name, show_goal=False)

        plot_init(None)
        plot_perc(raw_perc_traj, show_all_tags=True, show_scatter=True)
        plot_save(f'{plan_name}/raw_perc_scatter.png', goal=goal, plan_name=plan_name, show_goal=False)
        imageio.mimsave('{}/vid.mp4'.format(plan_name), video, fps=30)

        plot_init(None)
        plot_msg_hist(perc_bag_file)
        plot_save(f'{plan_name}/perc_msg_hist.png', plan_name=plan_name, show_goal=False)

        print(f'Done with {plan_name}')


def get_two_tags_dist():
    file_name = '/home/pracsys/raspi_ros_perception/out/2tags_hor2.bag'

    perc_traj = get_perc_traj(file_name)
    coords_52 = np.mean(perc_traj[52], axis=0)
    coords_50 = np.mean(perc_traj[50], axis=0)
    print(np.abs(coords_50 - coords_52))

    file_name = '/home/pracsys/raspi_ros_perception/out/2tags_ver2.bag'

    perc_traj = get_perc_traj(file_name)
    coords_52 = np.mean(perc_traj[52], axis=0)
    coords_50 = np.mean(perc_traj[50], axis=0)
    print(np.abs(coords_50 - coords_52))
if __name__ == '__main__':
    main()