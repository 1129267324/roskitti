#!/usr/bin/env python
# license removed for brevity
import os

from collections import deque

from data_utils import *
from publish_utils import *
DATA_PATH='/home/lc/RosKitti/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync/'
from cv_bridge import CvBridge
from kitti_util import *

def compute_3d_box_cam2(h, w, l, x, y, z, yaw):
    """
    Return : 3xn in cam2 coordinate
    """
    R = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]])
    x_corners = [l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2]
    y_corners = [0,0,0,0,-h,-h,-h,-h]
    z_corners = [w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2]
    corners_3d_cam2 = np.dot(R, np.vstack([x_corners,y_corners,z_corners]))
    corners_3d_cam2[0,:] += x
    corners_3d_cam2[1,:] += y
    corners_3d_cam2[2,:] += z
    return corners_3d_cam2
# the below 2 is belong to the def to calculate the distance between the car and other objects
def distance_point_to_segment(P,A,B):
    AP = P-A
    BP = P-B
    AB = B-A

    if np.dot(AB,AP)>=0 and np.dot(-AB,BP)>=0:
        return np.abs(np.cross(AP,AB))/np.linalg.norm(AB), np.dot(AP,AB)/np.dot(AB,AB)*AB+A

    d_PA = np.linalg.norm(AP)
    d_PB = np.linalg.norm(BP)
    if d_PA < d_PB:
        return d_PA, A 
    return d_PB, B

def min_distance_cuboids(cub1,cub2):
    minD = 1e5
    for i in range(4):
        for j in range(4):
            d, Q = distance_point_to_segment(cub1[i,:2], cub2[j,:2], cub2[j+1,:2])       
        if d < minD:
            minD = d
            minP = ego_car[i,:2]
            minQ = Q
    for i in range(4):
        for j in range(4):
            d,Q = distance_point_to_segment(cub1[i,:2], cub2[j,:2], cub2[j+1,:2])
        if d < minD:
            minD = d
            minP = corners_3d_velo[i,:2]
            minQ = Q
    return minP, minQ, minD
# 1-1 only for the car,the center of the car is (0,0)
# class Object():
#     def __init__(self):
#         self.locations = []
#         self.locations = deque(maxlen=20)
#     def update(self,displacement,yaw):
#         for i in range(len(self.locations)):
#             x0,y0=self.locations[i]
#             x1=x0*np.cos(yaw_change)+y0*np.sin(yaw_change)-displacement
#             y1=-x0*np.sin(yaw_change)+y0*np.cos(yaw_change)
#             self.locations[i]=np.array([x1,y1])

#         self.locations+=[np.array([0,0])]
#         #  extract the 20 from the all
#         # self.locations=self.locations[:20]

#         # self.locations.appendleft(np.array[[0,0]])
#     def reset(self):
#         self.locations=[]
#         # self.locations=deque(maxlen=20)

#     # 1-2 for the all objects
# class Object():
#     def __init__(self,center):
#         self.locations = []
#         self.locations.append(center)
#     def update(self,center,displacement,yaw):
#         for i in range(len(self.locations)):
#             x0,y0=self.locations[i]
#             x1=x0*np.cos(yaw_change)+y0*np.sin(yaw_change)-displacement
#             y1=-x0*np.sin(yaw_change)+y0*np.cos(yaw_change)
#             self.locations[i]=np.array([x1,y1])   
#         #if center is not None:        
#         self.locations.append(center)
#     def reset(self):
#         self.locations=[]
    # 1-3 for the all objects
class Object():
    def __init__(self,center):
        self.locations = deque(maxlen=20)
        self.locations.appendleft(center)
    def update(self,center,displacement,yaw):
        for i in range(len(self.locations)):
            x0,y0=self.locations[i]
            x1=x0*np.cos(yaw_change)+y0*np.sin(yaw_change)-displacement
            y1=-x0*np.sin(yaw_change)+y0*np.cos(yaw_change)
            self.locations[i]=np.array([x1,y1])   
        if center is not None:        
            self.locations.appendleft(center)
    def reset(self):
        self.locations=deque(maxlen=20)
        
if __name__ == '__main__':
    frame=0
    rospy.init_node('kitti_node', anonymous=True)
    cam_pub= rospy.Publisher('kitti_cam', Image, queue_size=10)
    pcl_pub= rospy.Publisher('kitti_point_cloud', PointCloud2, queue_size=10)    
    #ego_pub=rospy.Publisher('kitti_ego_car',MarkerArray,queue_size=10)
    ego_pub=rospy.Publisher('kitti_ego_car',Marker,queue_size=10)
    imu_pub=rospy.Publisher('kitti_imu',Imu,queue_size=10)
    gps_pub=rospy.Publisher('kitti_gps',NavSatFix,queue_size=10)
    #ego_pub=rospy.Publisher('kitti_ego_car',Marker,queue_size=10)
    model_pub=rospy.Publisher('kitti_car_model',Marker,queue_size=10)
    dist_pub=rospy.Publisher('kitti_dist',MarkerArray,queue_size=10)
    bridge=CvBridge()

    rate = rospy.Rate(10) # 10hz 

    dt_tracking=read_tracking('/home/lc/RosKitti/kitti/training/label_02/0000.txt')
    calib=Calibration('/home/lc/RosKitti/kitti/RawData/2011_09_26/',from_video=True)
    #3D data
    box3d_pub=rospy.Publisher('kitti_3d',MarkerArray,queue_size=10)
    loc_pub=rospy.Publisher('kitti_loc',MarkerArray,queue_size=10)

    # ego_car=Object()
    tracker={}   #track_id:object
    prev_imu_data = None
    
    while not rospy.is_shutdown():
        dt_tracking_frame=dt_tracking[dt_tracking.frame==frame]

        #boxes_2d=np.array(dt_tracking[dt_tracking['frame']==frame][['bbox_left','bbox_top','bbox_right','bbox_bottom']])
        boxes_2d=np.array(dt_tracking_frame[['bbox_left','bbox_top','bbox_right','bbox_bottom']])
        types=np.array(dt_tracking_frame['type'])
        #3D data
        boxes_3d=np.array(dt_tracking_frame[['height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']])
        track_ids=np.array(dt_tracking_frame['track_id'])

        corners_3d_velos=[ ]
        # for the other objects
        centers={ }   # track_id:center
        minPQDs=[]
        # for box_3d in boxes_3d:
        #     corners_3d_cam2=compute_3d_box_cam2(* box_3d)
        #     corners_3d_velo=calib.project_rect_to_velo(corners_3d_cam2.T)
        #     #calib.project_rec_to_velo(corners_3d_cam2.T)
        #     corners_3d_velos+=[corners_3d_velo]
        #  include the other objects
        for track_id,box_3d in zip(track_ids,boxes_3d):
            corners_3d_cam2=compute_3d_box_cam2(* box_3d)
            # the 8 coordinates
            corners_3d_velo=calib.project_rect_to_velo(corners_3d_cam2.T)
            #calib.project_rec_to_velo(corners_3d_cam2.T)
            # calculate the min distance between the car and other objects
            #minPQDs+=[min_distance_cuboids(EGOCAR,corners_3d_velo)]

            corners_3d_velos+=[corners_3d_velo]
            # figure the mean of the 8 coordinates
            centers[track_id]=np.mean(corners_3d_velo,axis=0)[:2]
        #corners_3d_velos+=[EGOCAR]
        types=np.append(types,'Car')
        track_ids=np.append(track_ids,-1)

        # add it to display the car
        centers[-1]=np.array([0,0])

        img=read_camera(os.path.join(DATA_PATH,'image_02/data/%010d.png'%frame))
        #publish_camera(cam_pub,bridge,img)
        publish_camera(cam_pub,bridge,img,boxes_2d,types)

        point_cloud=read_point_cloud(os.path.join(DATA_PATH,'velodyne_points/data/%010d.bin'%frame))
        publish_point_cloud(pcl_pub,point_cloud)
        # publish 3d data
        publish_3dbox(box3d_pub,corners_3d_velos,track_ids)
        
        publish_ego_car(ego_pub)
        publish_car_model(model_pub)

        imu_data=read_imu(os.path.join(DATA_PATH,'oxts/data/%010d.txt'%frame))
        publish_imu(imu_pub,imu_data)
        publish_gps(gps_pub,imu_data)
        # 1-1 only for the car
        # if prev_imu_data is not None:
        #     #gps_distances += [compute_great_circle_distance(imu_data.lat, imu_data.lon, prev_imu_data.lat, prev_imu_data.lon)]
        #     displacement=0.1*np.linalg.norm(imu_data[['vf','vl']])
        #     yaw_change=float(imu_data.yaw-prev_imu_data.yaw)
        #     ego_car.update(displacement,yaw_change)
        # 1-2  for the all objects
        if prev_imu_data is None:
            for track_id in centers:
                tracker[track_id]=Object(centers[track_id])
            #gps_distances += [compute_great_circle_distance(imu_data.lat, imu_data.lon, prev_imu_data.lat, prev_imu_data.lon)]
        else:
            displacement=0.1*np.linalg.norm(imu_data[['vf','vl']])
            yaw_change=float(imu_data.yaw-prev_imu_data.yaw)
            for track_id in centers:
                if track_id in tracker:
                    tracker[track_id].update(centers[track_id],displacement,yaw_change)
                else:
                    tracker[track_id]=Object(centers[track_id])
            for track_id in tracker:
                if track_id not in centers:
                    tracker[track_id].update(None,displacement,yaw_change)
            # ego_car.update(displacement,yaw_change)
        prev_imu_data = imu_data
        #1-1 publish the trajectory of the car
        # publish_loc(loc_pub,ego_car.locations)
        #1-2 publish the all objects
        publish_loc(loc_pub,tracker,centers)

        # rospy.loginfo("published")      
        rospy.loginfo("published frame %d"%frame)


        publish_dist(dist_pub,minPQDs)
        rate.sleep()
        frame +=1
        # 1-1
        # frame %=154
        # 1-2
        # if frame==154:
        #     frame=0
        #     ego_car.reset()
        # 1-3
        if frame==154:
            frame=0
            for track_id in tracker:
                tracker[track_id].reset()
