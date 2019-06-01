# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math

import rospy

from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from gps_common.msg import GPSFix, GPSStatus
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from tf.transformations import quaternion_from_euler

from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
import libnmea_navsat_driver.parser


class RosNMEADriver(object):

    def __init__(self):
        self.fix_pub = rospy.Publisher('fix', NavSatFix, queue_size=1)
        self.vel_pub = rospy.Publisher('vel', TwistStamped, queue_size=1)
        self.extend_fix_pub = rospy.Publisher('extend_fix', GPSFix, queue_size=1)
        self.heading_pub = rospy.Publisher(
            'heading', QuaternionStamped, queue_size=1)
        self.time_ref_pub = rospy.Publisher(
            'time_reference', TimeReference, queue_size=1)

        self.time_ref_source = rospy.get_param('~time_ref_source', None)
        self.use_RMC = rospy.get_param('~useRMC', False)
        self.valid_fix = False

        # epe = estimated position error
        self.default_epe_quality0 = rospy.get_param('~epe_quality0', 1000000)
        self.default_epe_quality1 = rospy.get_param('~epe_quality1', 4.0)
        self.default_epe_quality2 = rospy.get_param('~epe_quality2', 0.1)
        self.default_epe_quality4 = rospy.get_param('~epe_quality4', 0.02)
        self.default_epe_quality5 = rospy.get_param('~epe_quality5', 4.0)
        self.default_epe_quality9 = rospy.get_param('~epe_quality9', 3.0)
        self.using_receiver_epe = False

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")

        """Format for this dictionary is the fix type from a GGA message as the key, with
        each entry containing a tuple consisting of a default estimated
        position error, a NavSatStatus value, and a NavSatFix covariance value."""
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }
        self.extend_fix = GPSFix()
        self.star_map_gps = []
        self.star_map_bd = []
        self.star_use_gps = []
        self.star_use_bd = []

    # Returns True if we successfully did something with the passed in
    # nmea_string
    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        if not check_nmea_checksum(nmea_string):
            rospy.logwarn("Received a sentence with an invalid checksum. " +
                          "Sentence was: %s" % repr(nmea_string))
            return False

        parsed_sentence = libnmea_navsat_driver.parser.parse_nmea_sentence(
            nmea_string)
        if not parsed_sentence:
            rospy.logdebug(
                "Failed to parse NMEA sentence. Sentence was: %s" %
                nmea_string)
            return False

        rospy.logdebug(parsed_sentence)

        if timestamp:
            current_time = timestamp
        else:
            current_time = rospy.get_rostime()
        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id
        current_time_ref = TimeReference()
        current_time_ref.header.stamp = current_time
        current_time_ref.header.frame_id = frame_id

        self.extend_fix.header.stamp = current_time
        self.extend_fix.header.frame_id = frame_id

        if self.time_ref_source:
            current_time_ref.source = self.time_ref_source
        else:
            current_time_ref.source = frame_id

        if not self.use_RMC and 'GGA' in parsed_sentence:
            current_fix.position_covariance_type = \
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            data = parsed_sentence['GGA']
            fix_type = data['fix_type']
            if not (fix_type in self.gps_qualities):
                fix_type = -1
            gps_qual = self.gps_qualities[fix_type]
            default_epe = gps_qual[0]
            current_fix.status.status = gps_qual[1]
            current_fix.position_covariance_type = gps_qual[2]

            if gps_qual > 0:
                self.valid_fix = True
            else:
                self.valid_fix = False

            current_fix.status.service = NavSatStatus.SERVICE_GPS

            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude

            # use default epe std_dev unless we've received a GST sentence with
            # epes
            if not self.using_receiver_epe or math.isnan(self.lon_std_dev):
                self.lon_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.lat_std_dev):
                self.lat_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.alt_std_dev):
                self.alt_std_dev = default_epe * 2

            hdop = data['hdop']
            current_fix.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
            current_fix.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
            current_fix.position_covariance[8] = (
                2 * hdop * self.alt_std_dev) ** 2  # FIXME

            self.fix_pub.publish(current_fix)

            # set extend fix
            self.extend_fix.status.header.stamp = current_time
            self.extend_fix.status.header.frame_id = frame_id
            self.extend_fix.status.status = gps_qual[1]
            self.extend_fix.status.satellites_used = data['num_satellites']
            self.extend_fix.status.motion_source = GPSStatus.SOURCE_GPS
            self.extend_fix.status.orientation_source = GPSStatus.SOURCE_GPS
            self.extend_fix.status.position_source = GPSStatus.SOURCE_GPS
            self.extend_fix.latitude = current_fix.latitude
            self.extend_fix.longitude = current_fix.longitude
            self.extend_fix.altitude = current_fix.altitude
            self.extend_fix.position_covariance = current_fix.position_covariance
            self.position_covariance_type = current_fix.position_covariance_type

            if not math.isnan(data['utc_time']):
                current_time_ref.time_ref = rospy.Time.from_sec(
                    data['utc_time'])
                self.last_valid_fix_time = current_time_ref
                self.time_ref_pub.publish(current_time_ref)
                self.extend_fix.time = current_time_ref.time_ref.to_sec()


        elif not self.use_RMC and 'VTG' in parsed_sentence:
            data = parsed_sentence['VTG']

            # Only report VTG data when you've received a valid GGA fix as
            # well.
            if self.valid_fix:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * \
                    math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * \
                    math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)
                self.extend_fix.track = data['true_course']
                self.extend_fix.speed = data['speed']
            self.extend_fix_pub.publish(self.extend_fix)

        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']

            # Only publish a fix from RMC if the use_RMC flag is set.
            if self.use_RMC:
                if data['fix_valid']:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                current_fix.status.service = NavSatStatus.SERVICE_GPS

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                current_fix.altitude = float('NaN')
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.fix_pub.publish(current_fix)

                if not math.isnan(data['utc_time']):
                    current_time_ref.time_ref = rospy.Time.from_sec(
                        data['utc_time'])
                    self.time_ref_pub.publish(current_time_ref)

            # Publish velocity from RMC regardless, since GGA doesn't provide
            # it.
            if data['fix_valid']:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * \
                    math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * \
                    math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)
                self.extend_fix.track = data['true_course']
        elif 'GST' in parsed_sentence:
            data = parsed_sentence['GST']

            # Use receiver-provided error estimate if available
            self.using_receiver_epe = True
            self.lon_std_dev = data['lon_std_dev']
            self.lat_std_dev = data['lat_std_dev']
            self.alt_std_dev = data['alt_std_dev']
        elif 'HDT' in parsed_sentence:
            data = parsed_sentence['HDT']
            if data['heading']:
                current_heading = QuaternionStamped()
                current_heading.header.stamp = current_time
                current_heading.header.frame_id = frame_id
                q = quaternion_from_euler(0, 0, math.radians(data['heading']))
                current_heading.quaternion.x = q[0]
                current_heading.quaternion.y = q[1]
                current_heading.quaternion.z = q[2]
                current_heading.quaternion.w = q[3]
                self.heading_pub.publish(current_heading)
        elif 'GSA' in parsed_sentence:
            data = parsed_sentence['GSA']
            self.star_use_gps = [data['sate_id1'], data['sate_id2'],
                data['sate_id3'], data['sate_id4'], data['sate_id5'], data['sate_id6'], data['sate_id7'],
                data['sate_id8'], data['sate_id9'], data['sate_id10'], data['sate_id11'], data['sate_id12']]
            self.star_use_gps = filter(lambda star: star != 0, self.star_use_gps)
            self.extend_fix.pdop = data['pdop']
            self.extend_fix.hdop = data['hdop']
            self.extend_fix.vdop = data['vdop']
        elif 'BDGSA' in parsed_sentence:
            data = parsed_sentence['BDGSA']
            self.star_use_bd = [data['sate_id1'], data['sate_id2'],
                data['sate_id3'], data['sate_id4'], data['sate_id5'], data['sate_id6'], data['sate_id7'],
                data['sate_id8'], data['sate_id9'], data['sate_id10'], data['sate_id11'], data['sate_id12']]
            self.star_use_bd = filter(lambda star: star != 0, self.star_use_bd)
            self.extend_fix.pdop = data['pdop']
            self.extend_fix.hdop = data['hdop']
            self.extend_fix.vdop = data['vdop']
            self.extend_fix.status.satellite_used_prn = self.star_use_gps + self.star_use_bd

        elif 'GSV' in parsed_sentence:
            data = parsed_sentence['GSV']
            if data['index'] == 1:
                self.star_map_gps = []
            self.star_map_gps.append({
                'id': data['id_satellites1'],
                'elevation': data['elevation_satellites1'],
                'azimuth': data['azimuth_satellites1'],
                'snr': data['snr1']
            })
            self.star_map_gps.append({
                'id': data['id_satellites2'],
                'elevation': data['elevation_satellites2'],
                'azimuth': data['azimuth_satellites2'],
                'snr': data['snr2']
            })
            self.star_map_gps.append({
                'id': data['id_satellites3'],
                'elevation': data['elevation_satellites3'],
                'azimuth': data['azimuth_satellites3'],
                'snr': data['snr3']
            })
            self.star_map_gps.append({
                'id': data['id_satellites4'],
                'elevation': data['elevation_satellites4'],
                'azimuth': data['azimuth_satellites4'],
                'snr': data['snr4']
            })
            self.star_map_gps = filter(lambda star: star['id'] != 0, self.star_map_gps)
        elif 'BDGSV' in parsed_sentence:
            data = parsed_sentence['BDGSV']
            if data['index'] == 1:
                self.star_map_bd = []
            self.star_map_bd.append({
                'id': data['id_satellites1'],
                'elevation': data['elevation_satellites1'],
                'azimuth': data['azimuth_satellites1'],
                'snr': data['snr1']
            })
            self.star_map_bd.append({
                'id': data['id_satellites2'],
                'elevation': data['elevation_satellites2'],
                'azimuth': data['azimuth_satellites2'],
                'snr': data['snr2']
            })
            self.star_map_bd.append({
                'id': data['id_satellites3'],
                'elevation': data['elevation_satellites3'],
                'azimuth': data['azimuth_satellites3'],
                'snr': data['snr3']
            })
            self.star_map_bd.append({
                'id': data['id_satellites4'],
                'elevation': data['elevation_satellites4'],
                'azimuth': data['azimuth_satellites4'],
                'snr': data['snr4']
            })
            self.star_map_bd = filter(lambda star: star['id'] != 0, self.star_map_bd)
            self.star_map = self.star_map_gps + self.star_map_bd
            if data['length'] == data['index']:
                self.extend_fix.status.satellites_visible = len(self.star_map)
                self.extend_fix.status.satellite_visible_prn = [star['id'] for star in self.star_map]
                self.extend_fix.status.satellite_visible_snr = [star['snr'] for star in self.star_map]
                self.extend_fix.status.satellite_visible_azimuth = [star['azimuth'] for star in self.star_map]
                self.extend_fix.status.satellite_visible_z = [star['elevation'] for star in self.star_map]
        else:
            return False

    """Helper method for getting the frame_id with the correct TF prefix"""

    @staticmethod
    def get_frame_id():
        frame_id = rospy.get_param('~frame_id', 'gps')
        """Add the TF prefix"""
        prefix = ""
        prefix_param = rospy.search_param('tf_prefix')
        if prefix_param:
            prefix = rospy.get_param(prefix_param)
            return "%s/%s" % (prefix, frame_id)
        else:
            return frame_id
