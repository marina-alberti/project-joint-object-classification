/*
 * ApiConvertRealWorldDB.hpp
 *
 *  Created on: Jan 22, 2014
 *      Author: marina
 *
 *  This class reads the dataset information from JSON data files
 *  (with annotations of the objects in the 3D scenes)
 *  and stores the data into the Internal Data Structure:
 *  'SceneInformation', 'Object'.
 */

#ifndef APICONVERTREALWORLDDB_HPP_
#define APICONVERTREALWORLDDB_HPP_

#include "Object.hpp"
#include "SceneInformation.hpp"
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <vector>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/common/angles.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


class ApiConvertRealWorldDB {

public:

  ApiConvertRealWorldDB();
  static void parseFileJSON(string, vector<SceneInformation> &);
  static void parseSceneJSON(boost::property_tree::ptree::value_type &, SceneInformation &);
  static vector<pcl::PointXYZ> addNoiseBoundingBox(vector<pcl::PointXYZ> boundingBoxVertices, int noiseAmount);

};

#endif /* APICONVERTSIMULATIONDB_HPP_ */
