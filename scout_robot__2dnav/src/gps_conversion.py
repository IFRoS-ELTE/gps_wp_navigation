#!/usr/bin/env python3

import utm

# from shapely.geometry import Point, Polygon


class LatLonToCartesianConverter(object):
    """Utility class to convert between latitude/longitude and cartesian coordinates.
    Cartesian x/y coordinates correspond to easting/northing relative to the anchor point.
    """

    def __init__(self, anchor_lat, anchor_lon):
        self.anchor_easting, self.anchor_northing, self.zone, self.zone_letter = (
            utm.from_latlon(anchor_lat, anchor_lon)
        )
        print(anchor_lat, anchor_lon)

        print(self.anchor_easting, self.anchor_northing, self.zone, self.zone_letter)

    def ll_to_cartesian(self, lat, lon):
        """
        converts latitude, longitude data to cartesian

        Parameters:
            latitude (float):
            longitude (float):

        Returns:
            x, y
        """
        easting, northing = utm.from_latlon(
            lat, lon, force_zone_number=self.zone, force_zone_letter=self.zone_letter
        )[0:2]
        return easting - self.anchor_easting, northing - self.anchor_northing

    def cartesian_to_ll(self, easting, northing):
        lat, lon = utm.to_latlon(
            easting + self.anchor_easting,
            northing + self.anchor_northing,
            self.zone,
            self.zone_letter,
        )
        return lat, lon
