/*
 * ApiSceneSingleObjectFeature.hpp
 *
 *  Created on: Nov 18, 2013
 *      Author: marina
 *
 *  This class provides an API to extract
 *  'single object' features from individual objects
 *  in a scene ('SceneInformation').
 */

#ifndef APISCENESINGLEOBJECTFEATURE_HPP_
#define APISCENESINGLEOBJECTFEATURE_HPP_

#include "ApiFeatureExtractionSingleObject.hpp"
#include "SceneSingleObjectFeature.hpp"
#include "SceneInformation.hpp"
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/common/angles.h>
#include <algorithm>
#include <cmath>
#include "math.h"


using namespace std;

class ApiFeatureExtractionSceneSingleObject {

private:



public:

static void extract(SceneInformation &, SceneSingleObjectFeature & );
static void extractNoReference(SceneInformation & scene, SceneSingleObjectFeature & out);

};


#endif /* APISCENESINGLEOBJECTFEATURE_HPP_ */
