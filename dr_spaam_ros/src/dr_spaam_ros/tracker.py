# Stone Soup tracking tests

import threading
import queue
import numpy as np
import datetime


import threading
from abc import abstractmethod
from copy import copy
from datetime import timedelta

import rospy
import std_msgs.msg
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker

from stonesoup.detector.base import Detector
from stonesoup.base import Property
from stonesoup.buffered_generator import BufferedGenerator
from stonesoup.types.detection import Detection
from stonesoup.types.state import GaussianState
from stonesoup.types.array import StateVector



ID_MARKER_TOPIC = "/person_tracker_id"

POSE_ARRAY_TOPIC = "/person_tracker_poses"

class PersonTracker:
    def __init__(self):
        """ See jupyternotebook ./tracking_tests/2d_lidar_detection_test_10_Simulation_&_Tracking_Components.ipynb
        """
        
        # queue with people detections from dr_spaam
        self.person_detections_queue = queue.Queue()


        # published topics
        self.id_pub = rospy.Publisher(ID_MARKER_TOPIC, Marker, queue_size=10)

        self.pose_array_pub = rospy.Publisher(POSE_ARRAY_TOPIC, PoseArray, queue_size=10)

        # load logged file
        # format: list of detections, each detctions is a nx3 numpy arra (detx,dety,detprob)

        #filename = "/home/pblnav/logs/detections_11-19-2022-05-55-44.npz"
        # filename = "/home/pblnav/logs/detections_11-19-2022-05-56-29.npz"
        # filename = "/home/pblnav/logs/detections_11-19-2022-19-18-19.npz"

        # data = np.load(filename)

        # person_detections = [data[k] for k in data]
        # person_detections_timestamps = person_detections.pop()

        # person_detections = list(zip(person_detections,person_detections_timestamps))


        thread_track = threading.Thread(target=self.tracker_thread)
        thread_track.start()

        #thread_queue = threading.Thread(target=self.person_detections_queue_thread)
        #thread_queue.start()


    def tracker_thread(self):

        # ------------------
        # Create (online) detector
        # ------------------

        def person_detection_generator():
            
            while not rospy.is_shutdown():

                # wait for queue, set timeout so that it doen't prevent the node to be shut down with Ctrl-C
                try:
                    person_detection = self.person_detections_queue.get(timeout=1)
                except queue.Empty:
                    continue

                #print("got detection")

                yield  person_detection
                

        self.detector = LidarPersonDetector(person_detections=person_detection_generator())

       
        # ------------------
        # Transition model
        # ------------------

        from stonesoup.models.transition.linear import (
            CombinedLinearGaussianTransitionModel, ConstantVelocity)

        # state x_k = [x,x_vel,y,y_vel]
        # if the actual model is not very linear (as a person) best to set high variance (at least 0.5)

        variance = 0.5
        transition_model = CombinedLinearGaussianTransitionModel(
            [ConstantVelocity(variance), ConstantVelocity(variance)])


        # ------------------
        # Measurement model
        # ------------------

        from stonesoup.models.measurement.linear import LinearGaussian

        measurement_model_covariance = np.diag([0.25, 0.25])
        #measurement_model_covariance = np.diag([2.25, 2.25])
        measurement_model = LinearGaussian(ndim_state=4, mapping=[0, 2],noise_covar=measurement_model_covariance)


        # ------------------------------
        # Create the tracker components
        # ------------------------------
        # In this example a Kalman filter is used with global nearest neighbour (GNN) associator. Other options are, of course, available.

        # Predictor 

        from stonesoup.predictor.kalman import KalmanPredictor
        predictor = KalmanPredictor(transition_model)

        # Updater

        from stonesoup.updater.kalman import KalmanUpdater
        updater = KalmanUpdater(measurement_model)

        # Data associator
        from stonesoup.hypothesiser.distance import DistanceHypothesiser
        from stonesoup.measures import Mahalanobis
        hypothesiser = DistanceHypothesiser(predictor, updater, measure=Mahalanobis(), missed_distance=3)

        # Initialise the GNN with the hypothesiser.
        from stonesoup.dataassociator.neighbour import GNNWith2DAssignment
        data_associator = GNNWith2DAssignment(hypothesiser)

        # Track Initiator and Deleter

        # Create deleter - get rid of anything with a covariance trace greater than 2
        from stonesoup.deleter.error import CovarianceBasedDeleter
        covariance_limit_for_delete = 2
        deleter = CovarianceBasedDeleter(covar_trace_thresh=covariance_limit_for_delete)

        # Set a standard prior state and the minimum number of detections required to qualify for initiation
        s_prior_state = GaussianState([[0], [0], [0], [0]], np.diag([0, 0.5, 0, 0.5]))
        min_detections = 5

        # Initialise the initiator - use the 'full tracker' components specified above in the initiator.
        # But note that other ones could be used if needed.
        
        from stonesoup.initiator.simple import MultiMeasurementInitiator
        initiator = MultiMeasurementInitiator(
            prior_state=s_prior_state,
            measurement_model=measurement_model,
            deleter=deleter,
            data_associator=data_associator,
            updater=updater,
            min_points=min_detections
        ) 


        # ------------------------------
        # Create  and run the tracker
        # ------------------------------

        from stonesoup.tracker.simple import MultiTargetTracker

        self.tracker = MultiTargetTracker(
            initiator=initiator,
            deleter=deleter,
            detector=self.detector,
            data_associator=data_associator,
            updater=updater,
        )

        detections = set()
        tracks = set()

        for time, ctracks in self.tracker:
            detections.update(self.detector.detections)
            tracks.update(ctracks)
            print(time)

            # publish track information
            for track in ctracks: #Track istances which are currently  active!
                id = track.id

                id_marker_msg = id_to_marker(track)

                self.id_pub.publish(id_marker_msg)

    
            pose_array_msg = track_to_pose_array(ctracks)
            self.pose_array_pub.publish(pose_array_msg)


        # detections = set()
        # tracks = set()

        # for time, ctracks in tracker:
        #     #groundtruth.update(groundtruth_sim.groundtruth_paths)
        #     detections.update(detector.detections)
        #     tracks.update(ctracks)


        # Run tracker in thread (waits for person_detections generator to yield)

        #thread_track = threading.Thread(target=self.tracker_thread)
        #thread_track.start()

        #thread_queue = threading.Thread(target=self.person_detections_queue_thread)
        #thread_queue.start()


                




    def person_detections_queue_thread(self):
        
        while not rospy.is_shutdown():

            # wait for queue, set timeout so that it doen't prevent the node to be shut down with Ctrl-C
            try:
                person_detection = self.person_detections_queue.get(timeout=1)
            except queue.Empty:
                continue
            
            print(item[0])
            print(item[1])

    
            
   
class LidarPersonDetector(Detector):
    """ Stone Soup Detector wraper for 2d point detections (2d lidar person detector)
    Copied from Video Async Detector Abstract Class
   
    """
    
    person_detections: list = Property(
        doc="person_detections: generator or array of detections for each loop. each Detection is Tuple of a Nx3 numpy (x,y,confidence) and a timestamp [s]",
        default=[])


    def __init__(self,*args, **kwargs):
        super().__init__(None,*args, **kwargs)

        #self.person_detections = person_detections
        #self.person_detections_timestamps = person_detections_timestamps


    @BufferedGenerator.generator_method
    def detections_gen(self):
        """Returns a generator of detections for each frame.
        Yields
        ------
        : :class:`datetime.datetime`
            Datetime of current time step
        : set of :class:`~.Detection`
            Detections generated in the time step. The detection state vector is of the form
            ``(x, y)``, position of detection. Additionally, each detection carries the
            following meta-data fields:
            - ``confidence``: A float in the range ``(0, 1]`` indicating the detector's confidence
        """
        #timestamp = datetime.datetime.now()
              
        for person_det,stamp in self.person_detections:
            
            detections = set()
            
            timestamp = datetime.datetime.fromtimestamp(stamp)
            #timestamp += timedelta(seconds=0.1)
            
            for det in person_det:
                
                state_vector = StateVector([det[0], det[1]])
                metadata = {"confidence": det[2]}
                
                detection = Detection(state_vector=state_vector, #StateVector
                                  timestamp=timestamp, #datetime.datetime
                                  metadata=metadata) #dictionary of metadata items
                detections.add(detection)
                
            
            yield timestamp, detections



def id_to_marker(track):
    """
    @brief     Convert tracks to RViz marker msg. Marked with text
    """

    msg = Marker()
    msg.action = Marker.ADD
    msg.ns = "dr_spaam_ros"
    msg.id = 0
    msg.type = Marker.TEXT_VIEW_FACING

    # set quaternion so that RViz does not give warning
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0

    #position of last tracked position
    msg.pose.position.x = track.states[-1].state_vector[0]
    msg.pose.position.y = track.states[-1].state_vector[2]
    msg.pose.position.z = 0

    msg.scale.x = 0.03  # line width
    # red color
    msg.color.r = 1.0
    msg.color.a = 1.0

    # text
    msg.text = track.id

    msg.scale.x = 1
    msg.scale.y = 1
    msg.scale.z = 0.25


    msg.header = std_msgs.msg.Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "laser"

    return msg


def track_to_pose_array(tracks):
    """
    @brief     Convert tracks to poses msgs. Marked with text
    """

    pose_array = PoseArray()
    #print("New message")
    #print(dets_xy)
    #print("conf:",dets_cls)

    for track in tracks:

        p = Pose()
        p.position.x = track.states[-1].state_vector[0]
        p.position.y = track.states[-1].state_vector[2]
        p.position.z = 0.0
        pose_array.poses.append(p)

        break


    pose_array.header = std_msgs.msg.Header()
    pose_array.header.stamp = rospy.Time.now()
    pose_array.header.frame_id = "laser"

    return pose_array