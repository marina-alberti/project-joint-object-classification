/*
 * Object.cpp
 *
 *  Created on: Nov 18, 2013
 *      Author: marina
 */

#include "Object.hpp"

#define DEBUG 0

Object::Object() {
  objectName = "";
  predictedObjectID = -1;
  actualObjectID = -1;
  instanceName = "";
  categoryName = "";
}


/*
 * @description:
 * The function sets the field "objectName", the field "categoryName"
 * and the field "actualObjectID".
*/
void Object::setObjectName(string inputName) {

  objectName = inputName;


  const char * nameChar = inputName.c_str();
  if ( strcmp(nameChar, "Monitor") == 0 || strcmp(nameChar, "monitor") == 0 || strcmp(nameChar, "Screen") == 0 || strcmp(nameChar, "Monitor2") == 0) {
    actualObjectID = 0;
    categoryName = "Monitor";
  }
  if ( strcmp(nameChar, "Keyboard") == 0 || strcmp(nameChar, "keyboard") == 0 ) {
    actualObjectID = 1;
    categoryName = "Keyboard";
  }
  if (strcmp(nameChar, "Mouse") == 0 || strcmp(nameChar, "mouse") == 0 ) {
    actualObjectID = 2;
    categoryName = "Mouse";
  }
  if (strcmp(nameChar, "Mug") == 0 || strcmp(nameChar, "Cup") == 0 || strcmp(nameChar, "mug") == 0) {
    actualObjectID = 3;
    categoryName = "Cup";
  }
  if (strcmp(nameChar, "Lamp") == 0 || strcmp(nameChar, "Lamp2") == 0) {
    actualObjectID = 4;
    categoryName = "Lamp";
  }
  if (strcmp(nameChar, "Laptop") == 0 || strcmp(nameChar, "laptop") == 0) {
    actualObjectID = -1;
    categoryName = "Laptop";
  }
  if (strcmp(nameChar, "Pen") == 0 || strcmp(nameChar, "Pen2") == 0 || strcmp(nameChar, "Pen3") == 0 || strcmp(nameChar, "pen") == 0 || strcmp(nameChar, "Pencil") == 0) {
    actualObjectID = 5;
    categoryName = "Pencil";
  }

  // additional object categories present in the simulated scenes
  if (strcmp(nameChar, "Book") == 0) {
    actualObjectID = 7;
    categoryName = "Book";
  }

  if (strcmp(nameChar, "Bottle") == 0) {
    actualObjectID = 8;
    categoryName = "Bottle";
  }
  if (strcmp(nameChar, "Calculator") == 0) {
    actualObjectID = 9;
    categoryName = "Calculator";
  }
  if (strcmp(nameChar, "PC") == 0) {
    actualObjectID = 10;
    categoryName = "PC";
  }
  if (strcmp(nameChar, "Glass") == 0) {
    actualObjectID = 11;
    categoryName = "Glass";
  }
  if (strcmp(nameChar, "Headphone") == 0) {
    actualObjectID = 12;
    categoryName = "Headphone";
  }
  if (strcmp(nameChar, "Keys") == 0) {
    actualObjectID = 13;
    categoryName = "Keys";
  }
  if (strcmp(nameChar, "MobilePhone") == 0) {
    actualObjectID = 14;
    categoryName = "MobilePhone";
  }
  if (strcmp(nameChar, "Stapler") == 0) {
    actualObjectID = 15;
    categoryName = "Stapler";
  }
  if (strcmp(nameChar, "Telephone") == 0) {
    actualObjectID = 16;
    categoryName = "Telephone";
  }

}

void Object::setInstanceName(string inputName) {

  instanceName = inputName;

}


void Object::setCategoryName(string inputName) {

  categoryName = inputName;
  actualObjectID = convertStringToIntCategoryLabel(inputName);

}

/*
 * @description:
 * This function sets the 'boundingBox' data field given in input a
 * vector of PCL points corresponding to the 8 bounding box vertices.
 */
void Object::setBoundingBox(vector<pcl::PointXYZ> inputBoundingBox)   {

  for (int i = 0; i < inputBoundingBox.size(); i++) {
	  pcl::PointXYZ tmp = inputBoundingBox[i];

	  boundingBox.push_back(inputBoundingBox[i]);
  }
}


void Object::setCentroid() {
	float x_mean = (boundingBox.points[0].x + boundingBox.points[1].x + boundingBox.points[2].x + boundingBox.points[3].x + boundingBox.points[4].x + boundingBox.points[5].x + boundingBox.points[6].x + boundingBox.points[7].x) / 8;
	float y_mean = (boundingBox.points[0].y + boundingBox.points[1].y + boundingBox.points[2].y + boundingBox.points[3].y + boundingBox.points[4].y + boundingBox.points[5].y + boundingBox.points[6].y + boundingBox.points[7].y) / 8;
	float z_mean = (boundingBox.points[0].z + boundingBox.points[1].z + boundingBox.points[2].z + boundingBox.points[3].z + boundingBox.points[4].z + boundingBox.points[5].z + boundingBox.points[6].z + boundingBox.points[7].z) / 8;
	centroid.x = x_mean;
	centroid.y = y_mean;
	centroid.z = z_mean;
}


void Object::setCentroidPoint(pcl::PointXYZ in) {
	centroid = in;
}

pcl::PointCloud<pcl::PointXYZ> Object::getBoundingBox(){
  if (DEBUG) {
    for (int i = 0; i < boundingBox.size(); i++ ) {
      cout << "Vertex "<< i << endl;
      cout << "x: " << boundingBox.points[i].x ;
      cout << endl ;
      cout << "y: " << boundingBox.points[i].y;
      cout << endl ;
      cout << "z: " << boundingBox.points[i].z;
      cout << endl ;
    }
  }
  return boundingBox;
}

pcl::PointXYZ Object::getCentroid() {
  if (DEBUG) {
    cout << "Centroid: "<< endl;
    cout << "x: " << centroid.x;
    cout << endl ;
    cout << "y: " << centroid.y;
    cout << endl ;
    cout << "z: " << centroid.z;
    cout << endl ;
  }
  return centroid;
}

void Object::setObjectParameters(vector<pcl::PointXYZ> inputbbox, string inputInstanceName, string inputCategoryName) {

  Object::setBoundingBox(inputbbox);
  Object::setInstanceName(inputInstanceName);
  Object::setCategoryName(inputCategoryName);

}
