#!/usr/bin/env python
ROS = True
try:
    import rospy
    from rospkg import RosPack
except ImportError:
    ROS = False
    print('Launching without ROS')

import sys
sys.path.insert(0, 'source')
import os
from lxml import etree
import argparse
import utm
from dict2sdf import GetSDF
from osm2dict import Osm2Dict
from getMapImage import getMapImage
from getOsmFile import getOsmFile
import numpy

def print_(input = str()):
    if ROS:
        rospy.loginfo(input)
    else:
        print(input)

def getUtmInWorld(lat, lon, osmFile):
    print_("Using file " + osmFile)
    f = open(osmFile, 'r')
    root = etree.fromstring(f.read())
    f.close()
    boundingbox = [root[0].get('minlon'),
                        root[0].get('minlat'),
                        root[0].get('maxlon'),
                        root[0].get('maxlat')]
    utm_coord = list()
    map_coord = [float(x) for x in boundingbox]
    utm_coord.extend(utm.from_latlon(map_coord[1], map_coord[0])[:2])
    utm_coord.extend(utm.from_latlon(map_coord[3], map_coord[2])[:2])

    robot_utm_coord = utm.from_latlon(lat, lon)[:]

    print(utm_coord)
    print(robot_utm_coord)
    return robot_utm_coord[0] - utm_coord[0],\
           robot_utm_coord[1] - utm_coord[1]

def parseOsm(**kwargs):
    flags = {'build' : [],\
             'outFile' : 'out_File.sdf',\
             'inputFile' : str(),\
             'directory' : str(),\
             'Osmfile' : 'map.osm'}
    boundingbox = list()
    osmDictionary = dict()

    for key, value in kwargs.items():
        if(key == 'outFile'):
            pass
        if(key == 'roads' and value == True):
            flags['build'].append('r')
        if(key == 'buildings' and value == True):
            flags['build'].append('b')
        if(key == 'models' and value == True):
            flags['build'].append('m')
        if(key == 'all' and value == True and (('roads' or 'buildings' or 'models') not in kwargs.keys())):
            flags['build'].append('a')
        if(key == 'inputFile' and value):
            flags['inputFile'] = value
        if(key == 'outFile' and value):
            flags['outFile'] = value
        if(key == 'Osmfile' and value):
            flags['Osmfile'] = value
        if(key == 'directory' and value):
            flags['directory'] = value
            if not os.path.exists(flags['directory']):
                os.makedirs(flags['directory'])    
    if(flags['inputFile']):
        print_("Using file " + flags['inputFile'])
        f = open(flags['inputFile'], 'r')
        root = etree.fromstring(f.read())
        f.close()
        boundingbox = [root[0].get('minlon'),
                            root[0].get('minlat'),
                            root[0].get('maxlon'),
                            root[0].get('maxlat')]
    else:
        print_('Downloading the osm data... ')

    osmDictionary = getOsmFile(boundingbox, 'map.osm',flags['inputFile'])
    osmRoads = Osm2Dict(boundingbox[0], boundingbox[1],
                    osmDictionary, flags['build'])

    print_("Extracting the map data for gazebo ...")
    #get Road and model details
    roadPointWidthMap, modelPoseMap, buildingLocationMap = osmRoads.getMapDetails()

    print_("Building sdf file ...")
    #Initialize the getSdf class
    sdfFile = GetSDF()

    #Set up the spherical coordinates
    sdfFile.addSky()
    sdfFile.setOffset(boundingbox)
    sdfFile.addGroundPlane(boundingbox)
    sdfFile.addSphericalCoords(boundingbox)
    #add Required models

    for model in modelPoseMap.keys():
        points = modelPoseMap[model]['points']
        sdfFile.addModel(modelPoseMap[model]['mainModel'],
                        model,
                        [points[0, 0], points[1, 0], points[2, 0]])
        

    for building in buildingLocationMap.keys():
        
        sdfFile.addBuilding(buildingLocationMap[building]['mean'],
                            buildingLocationMap[building]['points'],
                            building,
                            buildingLocationMap[building]['color'])

    #Include the roads in the map in sdf file
    for road in roadPointWidthMap.keys():
        sdfFile.addRoad(road)
        sdfFile.setRoadWidth(roadPointWidthMap[road]['width'], road)
        points = roadPointWidthMap[road]['points']
        for point in range(len(points[0, :])):
            sdfFile.addRoadPoint([points[0, point],
                                points[1, point],
                                points[2, point]],
                                road)

    #output sdf File
    sdfFile.writeToFile(flags['outFile'])
        
if __name__ == '__main__':
    print("Testing OSM to SDF/WORLD parser")
    parseOsm(all = True, inputFile = 'osm/vgu_map.osm', directory = 'bullshit')

