/*
 * ApiConvertServiceFormat.hpp
 *
 *  Created on: Jan 9, 2014
 *      Author: marina
 *
 *  This class receives the data information in the format defined by the
 *  the ROS (Robot Operating System) service request,
 *  which contains geometry data of a set of 3D objects in a 3D scene,
 *  and stores the data into the Internal Data Structure: 'SceneInformation',
 *  'Object'.
 */

#ifndef APICONVERTSERVICEFORMAT_HPP_
#define APICONVERTSERVICEFORMAT_HPP_

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


class ApiConvertServiceFormat {

public:

	ApiConvertServiceFormat();
	static void parseScene(vector<string> objectList, vector<vector<pcl::PointXYZ> > bbox, vector<pcl::PointXYZ> pose, SceneInformation & currentScene);
};


#endif /* APICONVERTSERVICEFORMAT_HPP_ */
