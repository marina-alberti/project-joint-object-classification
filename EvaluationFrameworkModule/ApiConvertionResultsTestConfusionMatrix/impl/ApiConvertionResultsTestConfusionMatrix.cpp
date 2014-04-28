/*
 * ApiConvertionResultsTestConfusionMatrix.cpp
 *
 *  Created on: Dec 19, 2013
 *      Author: marina
 */

#include "ApiConvertionResultsTestConfusionMatrix.hpp"




void ApiConvertionResultsTestConfusionMatrix::convertResultsToMatrix(path myPath, SceneInformation scene, ConfusionMatrix & cMatrix, vector<int> categoryList) {

	cMatrix.setConfusionMatrix(categoryList);
	vector<Object> objectList = scene.getObjectList();

	for (vector<Object>::iterator it = objectList.begin(); it != objectList.end(); ++it) {

		int objectInstanceId = (*it).getInstanceID();
		int categoryLabelActual = (*it).getActualObjectID();
		int categoryLabelPredicted = myPath[objectInstanceId];

		if (categoryLabelPredicted != -1) {

			cMatrix.incrementConfusionMatrix(categoryLabelActual, categoryLabelPredicted);
		}
	}
}
