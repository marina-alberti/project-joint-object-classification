/*
 * ArrangeFeatureTestScene.hpp
 *
 *  Created on: Nov 25, 2013
 *      Author: marina
 */

#ifndef ARRANGEFEATURETESTSCENE_HPP_
#define ARRANGEFEATURETESTSCENE_HPP_

#include "SingleObjectFeature.hpp"
#include "ObjectPairFeature.hpp"
#include "SceneSingleObjectFeature.hpp"
#include "SceneObjectPairFeature.hpp"




class ArrangeFeatureTestScene {

private:

	vector<SingleObjectFeature> listSOF;
	vector<ObjectPairFeature> listOPF;
	vector<vector<ObjectPairFeature> > matrixOPF;

public:

	void arrangeTestFeatures(SceneSingleObjectFeature &, SceneObjectPairFeature &);

	vector<SingleObjectFeature> getListSOF()  { return listSOF; }
	vector<ObjectPairFeature> getListOPF() { return listOPF; }
	vector<vector<ObjectPairFeature> > getMatrixOPF() { return matrixOPF; }

};


#endif /* ARRANGEFEATURETESTSCENE_HPP_ */
