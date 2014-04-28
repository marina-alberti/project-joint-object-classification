/*
 * ApiConvertKTHDB.hpp
 *
 *  Created on: Nov 18, 2013
 *      Author: marina
 *
 *  This class reads the dataset information from XML data files
 *  (with annotations of the objects in the 3D scenes
 *  in the format of the output of the KTH "manual annotation tool")
 *  and stores the data into the Internal Data Structure:
 *  'SceneInformation', 'Object'.
 */

#ifndef APICONVERTKTHDB_HPP_
#define APICONVERTKTHDB_HPP_

#include "Object.hpp"
#include "SceneInformation.hpp"
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <vector>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/common/angles.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef map<string, float> objectParametersKTH;

class ApiConvertKTHDB {

private:

  objectParametersKTH mapKTHparameters;
  void parseObject(boost::property_tree::ptree &parent, SceneInformation &);
  vector<pcl::PointXYZ> convertObjectParameters();
  void setmapKTHparameters(float, float, float, float, float, float, float, float, float);

public:

  void parseFileXML(string, SceneInformation &);
  void parseDir(string);

};



#endif /* APICONVERTKTHDB_HPP_ */
