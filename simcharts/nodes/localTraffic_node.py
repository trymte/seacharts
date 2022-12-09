import copy
import rclpy
import threading
import numpy as np
import simcharts.utils as utils
from rclpy.node import Node
from simcharts.utils.helper import *
from simcharts_interfaces.msg import ListOfAIS, AIS, ListOfVessels, Vessel
from simcharts_interfaces.srv import UpdateVesselInLocalTraffic, AddVesselToLocalTraffic, RemoveVesselFromLocalTraffic, ReplaceLocalTraffic

class LocalTrafficSubscriber(Node):
    def __init__(self):
        super().__init__('simcharts__local_traffic_subscriber')
        self._lock = threading.Lock()
        self._local_traffic = {}
        self.local_traffic_subscription = self.create_subscription(
            ListOfVessels,
            'simcharts/local_traffic',
            self._listener_callback,
            10
            )
        self.local_traffic_subscription  # prevent unused variable warning

    def _listener_callback(self, msg):
        with self._lock:
            self._local_traffic = {}
            self.get_logger().debug(f"Received {len(msg.local_traffic)} vessels")
            for vessel in msg.local_traffic:
                self._local_traffic[vessel.id] = vessel

    def get_local_traffic(self):
        ret = {}
        with self._lock:
            ret = copy.deepcopy(self._local_traffic)
        return ret

class LocalTrafficNode(Node):
    def __init__(self, settings, cli_args=None):
        super().__init__('simcharts__local_traffic_node', cli_args=cli_args)
        self._lock = threading.Lock()
        # Callback groups
        self.subscriber_cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.publisher_cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.srv_cb_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.local_traffic = {} # {"id": Ship()}
        self.origin = settings['origin']
        self.size = settings['size']
        
        # Publisher
        self.local_traffic_publisher = self.create_publisher(ListOfVessels, 'simcharts/local_traffic', 10, callback_group=self.publisher_cb_group)
        timer_period = settings['local_traffic_publish_timer']  # seconds
        self.timer = self.create_timer(timer_period, self._localTrafficPublishCallback, callback_group=self.publisher_cb_group)

        # Subscriber
        self.subscription = self.create_subscription(
            ListOfAIS,
            'simcharts_aisforwarder/latest_ais',
            self._ais_listener_callback,
            10,
            callback_group=self.subscriber_cb_group
            )
        self.subscription  # prevent unused variable warning

        # Services
        self.srv = self.create_service(AddVesselToLocalTraffic, 'simcharts/add_vessel_to_local_traffic', self._addVesselToLocalTrafficCallback, callback_group=self.srv_cb_group)
        self.srv = self.create_service(UpdateVesselInLocalTraffic, 'simcharts/update_vessel_in_local_traffic', self._updateVesselInLocalTrafficCallback, callback_group=self.srv_cb_group)
        self.srv = self.create_service(RemoveVesselFromLocalTraffic, 'simcharts/remove_vessel_from_local_traffic', self._removeVesselFromLocalTrafficCallback, callback_group=self.srv_cb_group)
        self.srv = self.create_service(ReplaceLocalTraffic, 'simcharts/replace_local_traffic', self._replaceLocalTraffic, callback_group=self.srv_cb_group)

    def getLocalTraffic(self) -> dict:
        copyTraffic = None
        with self._lock:
            copyTraffic = copy.deepcopy(self.local_traffic)
        return copyTraffic

    def _localTrafficPublishCallback(self):
        '''
        Publishes the local traffic
        '''
        self.get_logger().debug(f"Publishing {len(self.local_traffic)} vessels")
        msg = ListOfVessels()
        msg.timestamp = f"{getTimeStamp(self.get_clock())}"
        msg.local_traffic = list(self.local_traffic.values())
        self.local_traffic_publisher.publish(msg)

    def _ais_listener_callback(self, msg: ListOfAIS):
        '''
        Callback function for the latest_ais subscriber

        Converts AIS messages to Vessel messages and sends all vessels
        inside of the horizon to the localTrafficNode

        In:
            msg: (ListOfAIS) List of AIS messages from the AISforwarder
        '''
        self.get_logger().debug(f"Received {len(msg.ais_msgs)} AIS messages")
        vessels = []
        for m in msg.ais_msgs:
            vessel = self._AIS2Vessel(m)
            size = [self.size[0]*1.02, self.size[1]*1.02] # 2% buffer
            if not isInHorizon(vessel, size, self.origin): continue # 2% buffer
            vessels.append(vessel)
            self.local_traffic[vessel.id] = vessel
        
        self.get_logger().debug(f"{len(vessels)} vessels detected in horizon")
    
    def _longlat2utm(self, long: float, lat:float) -> tuple:
        '''
        Converts latitude and longitude to UTM coordinates
        '''
        return utils.geodesy.longlat2utm(long, lat)

    def _AIS2Vessel(self, aisMsg: AIS) -> Vessel:
        '''
        Converts an AIS message to a Vessel message
        '''
        vessel = Vessel()
        vessel.id = aisMsg.mmsi
        vessel.timestamp = aisMsg.timestamp
        vessel.vesselsimtype = 'AIS'
        vessel.x, vessel.y = self._longlat2utm(aisMsg.longitude, aisMsg.latitude)
        if vessel.id in self.local_traffic:
            vessel.length = self.local_traffic[vessel.id].length
            vessel.scale = self.local_traffic[vessel.id].scale
        else:
            vessel.length = float(np.random.randint(15, 75)) # Random length in meters
            vessel.scale = vessel.length / 80.0 # 80m is the length of the default ship model
        if aisMsg.sog == None or aisMsg.sog == "null":
            vessel.sog = 0.0
        else:
            vessel.sog = float(aisMsg.sog)
        if (aisMsg.cog == "null" or aisMsg.cog == None):
            vessel.cog = 0.0
        else:
            vessel.cog = float(aisMsg.cog)
        if (aisMsg.heading == "null" or aisMsg.heading == None):
            vessel.heading = 0.0
        else:
            vessel.heading = float(aisMsg.heading)
        if (aisMsg.rot == "null" or aisMsg.rot == None):
            vessel.rot = 0.0
        else:
            vessel.rot = float(aisMsg.rot)
        vessel.name = aisMsg.name
        vessel.shiptype = aisMsg.shiptype
        return vessel

    def _addVesselToLocalTrafficCallback(self, request: Vessel, response: bool) -> bool:
        '''
        Callback function for the add_vessel_to_local_traffic service

        Adds a new vessel to the local traffic list

        In:
            vessel: (Vessel) Vessel message
        Out:
            was_added: (Bool) True if vessel was added, False if vessel was already in the list
        '''
        self.get_logger().info('I heard: "%s"' % request)
        with self._lock:
            if request.vessel.id in self.local_traffic:
                response.was_added = False
            else:
                response.was_added = True
            self.local_traffic[request.id] = request.vessel
        return response

    def _updateVesselInLocalTrafficCallback(self, request: Vessel, response: bool) -> bool:
        '''
        Callback function for the update_vessel_in_local_traffic service

        Updates existing vessel in local traffic list

        In:
            vessel: (Vessel) Vessel message
        Out:
            was_updated: (Bool) True if vessel was updated, False if vessel was not in the list
        '''
        self.get_logger().debug('I heard: "%s"' % request)
        if request.vessel.id in self.local_traffic:
            response.was_updated = True
            self.local_traffic[request.vessel.id] = request.vessel
        else:
            response.was_updated = False
        return response
    
    def _removeVesselFromLocalTrafficCallback(self, request: str, response: Vessel) -> Vessel:
        '''
        Callback function for the remove_vessel_from_local_traffic service

        Removes vessel from local traffic list

        In:
            id: (String) id of the vessel to be removed
        Out:
            removed_vssel: (Vessel) Vessel message of the removed vessel
            was_removed: (Bool) True if vessel was removed, False if vessel was not in the list
        '''
        self.get_logger().debug('I heard: "%s"' % request)
        if request.id in self.local_traffic:
            response.removed_vssel = self.local_traffic.pop(request.id)
            response.was_removed = True
        else:
            response.was_removed = False
        return response
    
    def _replaceLocalTraffic(self, request: Vessel, response: Vessel) -> Vessel():
        '''
        Callback function for the replace_local_traffic service

        Replaces the local traffic list with a new list

        In:
            new_traffic: (Vessel[]) list of Vessel messages
        Out:
            timestamp: (Float) current time stamp
            old_traffic: (Vessel[]) list of the old traffic
        '''
        self.get_logger().debug('I heard: "%s"' % request)
        response.old_traffic = dictToList(self.local_traffic)
        response.timestamp = getTimeStamp(self.get_clock())
        self.local_traffic = request.new_traffic
        return response

def main(args=None):
    pass

if __name__ == '__main__':
    main()