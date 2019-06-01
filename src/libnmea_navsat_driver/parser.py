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

import re
import time
import calendar
import math
import logging
logger = logging.getLogger('rosout')


def safe_float(field):
    try:
        return float(field)
    except ValueError:
        return float('NaN')


def safe_int(field):
    try:
        return int(field)
    except ValueError:
        return 0


def convert_latitude(field):
    return safe_float(field[0:2]) + safe_float(field[2:]) / 60.0


def convert_longitude(field):
    return safe_float(field[0:3]) + safe_float(field[3:]) / 60.0


def convert_time(nmea_utc):
    # Get current time in UTC for date information
    utc_struct = time.gmtime()  # immutable, so cannot modify this one
    utc_list = list(utc_struct)
    # If one of the time fields is empty, return NaN seconds
    if not nmea_utc[0:2] or not nmea_utc[2:4] or not nmea_utc[4:6]:
        return float('NaN')
    else:
        hours = int(nmea_utc[0:2])
        minutes = int(nmea_utc[2:4])
        seconds = int(nmea_utc[4:6])
        utc_list[3] = hours
        utc_list[4] = minutes
        utc_list[5] = seconds
        unix_time = calendar.timegm(tuple(utc_list))
        return unix_time


def convert_status_flag(status_flag):
    if status_flag == "A":
        return True
    elif status_flag == "V":
        return False
    else:
        return False


def convert_knots_to_mps(knots):
    return safe_float(knots) * 0.514444444444


# Need this wrapper because math.radians doesn't auto convert inputs
def convert_deg_to_rads(degs):
    return math.radians(safe_float(degs))


"""Format for this dictionary is a sentence identifier (e.g. "GGA") as the key, with a
list of tuples where each tuple is a field name, conversion function and index
into the split sentence"""
parse_maps = {
    "GGA": [
        ("fix_type", int, 6),
        ("latitude", convert_latitude, 2),
        ("latitude_direction", str, 3),
        ("longitude", convert_longitude, 4),
        ("longitude_direction", str, 5),
        ("altitude", safe_float, 9),
        ("mean_sea_level", safe_float, 11),
        ("hdop", safe_float, 8),
        ("num_satellites", safe_int, 7),
        ("utc_time", convert_time, 1),
    ],
    "RMC": [
        ("utc_time", convert_time, 1),
        ("fix_valid", convert_status_flag, 2),
        ("latitude", convert_latitude, 3),
        ("latitude_direction", str, 4),
        ("longitude", convert_longitude, 5),
        ("longitude_direction", str, 6),
        ("speed", convert_knots_to_mps, 7),
        ("true_course", convert_deg_to_rads, 8),
        ("datetime", str, 9),
        ("magnetic_variation", safe_float, 10),
        ("magnetic_variation_direction", str, 11),
    ],
    "GST": [
        ("utc_time", convert_time, 1),
        ("ranges_std_dev", safe_float, 2),
        ("semi_major_ellipse_std_dev", safe_float, 3),
        ("semi_minor_ellipse_std_dev", safe_float, 4),
        ("semi_major_orientation", safe_float, 5),
        ("lat_std_dev", safe_float, 6),
        ("lon_std_dev", safe_float, 7),
        ("alt_std_dev", safe_float, 8),
    ],
    "HDT": [
        ("heading", safe_float, 1),
    ],
    "VTG": [
        ("true_course", safe_float, 1),
        ("speed", convert_knots_to_mps, 5)
    ],
    "GSV": [
        ("length", safe_int, 1),
        ("index", safe_int, 2),
        ("num_satellites", safe_int, 3),
        ("id_satellites1", safe_int, 4),
        ("elevation_satellites1", safe_int, 5),
        ("azimuth_satellites1", safe_int, 6),
        ("snr1", safe_int, 7),
        ("id_satellites2", safe_int, 8),
        ("elevation_satellites2", safe_int, 9),
        ("azimuth_satellites2", safe_int, 10),
        ("snr2", safe_int, 11),
        ("id_satellites3", safe_int, 12),
        ("elevation_satellites3", safe_int, 13),
        ("azimuth_satellites3", safe_int, 14),
        ("snr3", safe_int, 15),
        ("id_satellites4", safe_int, 16),
        ("elevation_satellites4", safe_int, 17),
        ("azimuth_satellites4", safe_int, 18),
        ("snr4", safe_int, 19),
    ],
    "GSA": [
        ("sate_id1", safe_int, 3),
        ("sate_id2", safe_int, 4),
        ("sate_id3", safe_int, 5),
        ("sate_id4", safe_int, 6),
        ("sate_id5", safe_int, 7),
        ("sate_id6", safe_int, 8),
        ("sate_id7", safe_int, 9),
        ("sate_id8", safe_int, 10),
        ("sate_id9", safe_int, 11),
        ("sate_id10", safe_int, 12),
        ("sate_id11", safe_int, 13),
        ("sate_id12", safe_int, 14),
        ("pdop", safe_int, 15),
        ("hdop", safe_int, 16),
        ("vdop", safe_int, 17),
    ],
    "BDGSV": [
        ("length", safe_int, 1),
        ("index", safe_int, 2),
        ("num_satellites", safe_int, 3),
        ("id_satellites1", safe_int, 4),
        ("elevation_satellites1", safe_int, 5),
        ("azimuth_satellites1", safe_int, 6),
        ("snr1", safe_int, 7),
        ("id_satellites2", safe_int, 8),
        ("elevation_satellites2", safe_int, 9),
        ("azimuth_satellites2", safe_int, 10),
        ("snr2", safe_int, 11),
        ("id_satellites3", safe_int, 12),
        ("elevation_satellites3", safe_int, 13),
        ("azimuth_satellites3", safe_int, 14),
        ("snr3", safe_int, 15),
        ("id_satellites4", safe_int, 16),
        ("elevation_satellites4", safe_int, 17),
        ("azimuth_satellites4", safe_int, 18),
        ("snr4", safe_int, 19),
    ],
    "BDGSA": [
        ("sate_id1", safe_int, 3),
        ("sate_id2", safe_int, 4),
        ("sate_id3", safe_int, 5),
        ("sate_id4", safe_int, 6),
        ("sate_id5", safe_int, 7),
        ("sate_id6", safe_int, 8),
        ("sate_id7", safe_int, 9),
        ("sate_id8", safe_int, 10),
        ("sate_id9", safe_int, 11),
        ("sate_id10", safe_int, 12),
        ("sate_id11", safe_int, 13),
        ("sate_id12", safe_int, 14),
        ("pdop", safe_int, 15),
        ("hdop", safe_int, 16),
        ("vdop", safe_int, 17),
    ]
}


def parse_nmea_sentence(nmea_sentence):
    # Check for a valid nmea sentence

    if not re.match(
            r'(^\$GP|^\$GN|^\$GL|^\$IN|^\$BD).*\*[0-9A-Fa-f]{2}$', nmea_sentence):
        logger.debug(
            "Regex didn't match, sentence not valid NMEA? Sentence was: %s" %
            repr(nmea_sentence))
        return False
    fields = [field.strip(',') for field in nmea_sentence.split(',')]
    fields[-1] = fields[-1].split('*')[0]

    # Ignore the $ and talker ID portions (e.g. GP)
    sentence_type = fields[0][3:]

    if fields[0] == "$BDGSA":
        sentence_type = "BDGSA"
    if fields[0] == "$BDGSV":
        sentence_type = "BDGSV"

    if sentence_type not in parse_maps:
        logger.debug("Sentence type %s not in parse map, ignoring."
                     % repr(sentence_type))
        return False

    parse_map = parse_maps[sentence_type]

    parsed_sentence = {}
    for entry in parse_map:
        if entry[2] >= len(fields):
            parsed_sentence[entry[0]] = 0
        else:
            parsed_sentence[entry[0]] = entry[1](fields[entry[2]])

    return {sentence_type: parsed_sentence}
