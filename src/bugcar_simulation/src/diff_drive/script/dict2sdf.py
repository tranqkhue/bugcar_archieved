##############################################################################
#Author: Tashwin Khurana
#Version: 1.0
#Package: gazebo_osm
#
#Description: GetSDF() class
#             Builds a sdf file by adding models and seting their properties,
#             roads and sets spherical coordinates for the world
##############################################################################

import lxml.etree as Et
import xml.dom.minidom as minidom
import numpy
import utm


class GetSDF:

    def __init__(self):
        self.sdf = Et.Element('sdf')
        self.sdf.set('version', "1.4")
        world = Et.SubElement(self.sdf, 'world')
        world.set('name', 'default')
        self.modelList = dict()
        self.offset = list()
    
    def setOffset(self, boundingbox):
        utm_coord = list()
        map_coord = [float(x) for x in boundingbox]
        utm_coord.extend(utm.from_latlon(map_coord[3], map_coord[2])[:2])
        utm_coord.extend(utm.from_latlon(map_coord[1], map_coord[0])[:2])

        #self.offset.extend([numpy.abs(utm_coord[0] - utm_coord[2])/2,\
        #                   numpy.abs(utm_coord[1] - utm_coord[3])/2])
        self.offset.extend([0,0])
        print(self.offset)
    def addSky(self):
        data_sky = """<scene>
      <sky>
        <clouds>
          <speed>12</speed>
        </clouds>
      </sky>
    </scene>"""
        self.sdf.xpath('//world')[0].append(Et.fromstring(data_sky))
    def addGroundPlane(self, boundingbox):
        data = """    <model name="ground">
      <link name="body">
          <collision name="geom">
              <geometry>
                  <plane>
                      <normal>0 0 1</normal>
                      <size></size>
                  </plane>
              </geometry>
              <surface>
                  <friction>
                    <ode>
                      <mu>100</mu>
                      <mu2>50</mu2>
                    </ode>
                  </friction>
              </surface>
          </collision>
      </link>
      <static>true</static>
    </model>
        """
        xml_data = Et.fromstring(data)

        size = xml_data.xpath('//size')
        size[0].text = str(self.offset[0]*2) + " " + str(self.offset[1]*2)
        self.sdf.xpath('//world')[0].append(xml_data)
        
        

    def addSphericalCoords(self, boundingbox):
        utm_coord = list()
        map_coord = [float(x) for x in boundingbox]
        utm_coord.extend(utm.from_latlon(map_coord[1], map_coord[0])[:2])
        utm_coord.extend(utm.from_latlon(map_coord[3], map_coord[2]))
        latVal, lonVal = utm.to_latlon(utm_coord[2] + numpy.abs(utm_coord[0] - utm_coord[2])/2,\
                       utm_coord[3] + numpy.abs(utm_coord[1] - utm_coord[3])/2,\
                       utm_coord[4], utm_coord[5])

        ''' Add the spherical coordinates for the map'''
        spherical_coordinates = Et.SubElement(self.sdf.find('world'),
                                              'spherical_coordinates')

        model = Et.SubElement(spherical_coordinates, 'surface_model')
        model.text = "EARTH_WGS84"

        lat = Et.SubElement(spherical_coordinates, 'latitude_deg')
        lat.text = str(latVal)

        lon = Et.SubElement(spherical_coordinates, 'longitude_deg')
        lon.text = str(lonVal)

        elevation = Et.SubElement(spherical_coordinates, 'elevation')
        elevation.text = str(0)

        heading = Et.SubElement(spherical_coordinates, 'heading_deg')
        heading.text = str(0)

    def includeModel(self, modelName):
        ''' Include models in gazebo database'''
        includeModel = Et.SubElement(self.sdf.find('world'), 'include')
        includeUri = Et.SubElement(includeModel, 'uri')
        includeUri.text = "model://" + modelName
        return includeModel

    def addModel(self, mainModel, modelName, pose):
        '''Add model with pose and the name taken as inputs'''

        includeModel = self.includeModel(mainModel)

        model = Et.SubElement(includeModel, 'name')
        model.text = modelName

        static = Et.SubElement(includeModel, 'static')
        static.text = 'true'

        modelPose = Et.SubElement(includeModel, 'pose')

        modelPose.text = (str(pose[0] - self.offset[0]) +
                          " " + str(pose[1] - self.offset[1]) +
                          " " + str(pose[2]) + " 0 0 0")

    def addRoad(self, roadName):
        '''Add road to sdf file'''
        road = Et.SubElement(self.sdf.find('world'), 'road')
        road.set('name', roadName)

    def setRoadWidth(self, width, roadName):
        ''' Set the width of the road specified by the road name'''
        allRoads = self.sdf.find('world').findall('road')

        roadWanted = [road for road in allRoads
                      if road.get('name') == roadName]

        roadWidth = Et.SubElement(roadWanted[0], 'width')
        roadWidth.text = str(10)

    def addRoadPoint(self, point, roadName):
        '''Add points required to build a road, specified by the roadname'''
        allRoads = self.sdf.find('world').findall('road')

        roadWanted = [road for road in allRoads
                      if road.get('name') == roadName]
        roadPoint = Et.SubElement(roadWanted[0], 'point')
        roadPoint.text = (str(point[0] - self.offset[0]) +
                          " " + str(point[1] - self.offset[1]) +
                          " " + str(point[2]))

    def addBuilding(self, mean, pointList, building_name, color):
        building = Et.SubElement(self.sdf.find('world'), 'model')
        building.set('name', building_name)
        static = Et.SubElement(building, 'static')
        static.text = 'true'
        mainPose = Et.SubElement(building, 'pose')

        mainPose.text = (str(mean[0, 0] - self.offset[0]) +
                         " " + str(mean[1, 0] - self.offset[1]) +
                         " " + str(mean[2, 0]) +
                         " 0 0 0")

        yaw = [numpy.arctan2((pointList[1, point] - pointList[1, point + 1]),
                             (pointList[0, point] - pointList[0, point + 1]))
               for point in range(numpy.size(pointList, 1)-1)]

        distance = [numpy.sqrt(((pointList[1, point] -
                                 pointList[1, point + 1])**2 +
                                (pointList[0, point] -
                                 pointList[0, point + 1])**2))
                    for point in range(numpy.size(pointList, 1)-1)]

        meanPoint = [[(pointList[0, point] +
                      pointList[0, point + 1])/2 - mean[0, 0],
                     (pointList[1, point] +
                      pointList[1, point + 1])/2 - mean[1, 0], 0]
                     for point in range(numpy.size(pointList, 1)-1)]

        for point in range(len(yaw)):

            link = Et.SubElement(building, 'link')
            link.set('name', (building_name + '_' + str(point)))
            collision = Et.SubElement(link, 'collision')
            collision.set('name', (building_name + '_' + str(point)))

            geometry = Et.SubElement(collision, 'geometry')
            box = Et.SubElement(geometry, 'box')
            Et.SubElement(box, 'size').text = str(distance[point]) + ' 0.2 10'

            visual = Et.SubElement(link, 'visual')
            visual.set('name', (building_name + '_' + str(point)))

            geometry = Et.SubElement(visual, 'geometry')
            box = Et.SubElement(geometry, 'box')
            Et.SubElement(box, 'size').text = str(distance[point]) + ' 0.2 10'

            material = Et.SubElement(visual, 'material')
            script = Et.SubElement(material, 'script')
            Et.SubElement(script, 'uri').text = ('file://media/materials/' +
                                                 'scripts/gazebo.material')
            Et.SubElement(script, 'name').text = 'Gazebo/' + color
            Et.SubElement(link, 'pose').text = (str(meanPoint[point][0]) +
                                                ' ' +
                                                str(meanPoint[point][1]) +
                                                ' 0 0 0 ' +
                                                str(yaw[point]))

    def writeToFile(self, filename):
        '''Write sdf file'''
        outfile = open(filename, "w")
        outfile.write(Et.tostring(self.sdf, pretty_print=True,
                                  xml_declaration=True))
        outfile.close()
