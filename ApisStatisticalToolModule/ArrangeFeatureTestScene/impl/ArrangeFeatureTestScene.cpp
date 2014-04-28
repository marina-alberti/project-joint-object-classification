/*
 * ArrangeFeatureTestScene.cpp
 *
 *  Created on: Nov 25, 2013
 *      Author: marina
 */

#include "ArrangeFeatureTestScene.hpp"

#define DEBUG 0

	/*
	 * @input:
	 * scene-level features (both from 'single object' and 'object pair'):
	 *  - 'SceneSingleObjectFeature'
	 *  - 'SceneObjectPairFeature'
	 * @output:
	 * - information about 'single object features' and 'object pair features'
	 *   at the scene level is arranged and stored in the data fields of this class,
	 *   organized to call the methods for the Test phase.
	 */
void ArrangeFeatureTestScene::arrangeTestFeatures(SceneSingleObjectFeature & sof, SceneObjectPairFeature & opf) {

	listSOF = sof.getListSingleObjectFeature();
	listOPF = opf.getListObjectPairFeature();

	// Gets the number of test objects in the test scene
	int nTestObject = listSOF.size();
	int i = 0;
	int j = 0;
	int countVector = 0;

	if (DEBUG) {
		cout << "nTestObject = "  << nTestObject << endl;
	}

	for (int i = 0; i < nTestObject; i++) {
		vector<ObjectPairFeature> tmp;
		for (int j = 0; j < nTestObject; j++) {

			if (i != j) {
				tmp.push_back(listOPF.at(countVector));
				countVector++;
			}
			else {
				ObjectPairFeature empty;
				tmp.push_back(empty);
			}
		}
		matrixOPF.push_back(tmp);
	}
}
