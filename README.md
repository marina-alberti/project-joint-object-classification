project-joint-object-classification
===================================

This project deals with the joint classification of a set of 3D objects in indoor 
scenes by exploiting information about objects 3D spatial configuration.
The goal is to jointly predict the object cateogory labels of a set of objects in 
a test scene. The geometric information about the objects is provided to the system 
in the form of 3D cuboid bounding boxes.
In a training phase, the system learns models of the 3D spatial arrangment / 
spatial relations of pairs of objects, and spatial characteristics of individual 
objects of different categories, using a training dataset. The system can read data 
from different input datasets: XML files with real-world data with object 
annotations (output of KTH '3D Annotation Tool'), JSON files with simulated 
data, JSON files with real-world data.

Main set of components
===================================

See the figure ./'System_diagram.pdf'.

- APIs Convertion:

  These classes parse the XML / JSON files with the dataset object annotations,
  read and convert the annotated data into the internal data
  structure, consisting of a scene with a set of objects.

- Database Handling Module:

  Classes: 'DatabaseInformation', 'SceneInformation', 'Object', representing 
  the internal data structure.

- Feature Extraction Module:

  APIs to extract features from individual objects and from pairs of objects in 
  the scenes and classes that store these features at the level of: database, 
  scene, object / object pair.

- Api Database Frequencies:

  Given the 'DatabaseInformation' of a set of scenes, these APIs compute the 
  occurrence frequencies of the object categories and the   occurrence frequencies 
  of the object categories co-occurrences (pair of categories).

- Apis Statistical Tool Module:

  Contains 'Training' class to do training of Gaussian Mixture Models (GMMs) 
  for a set of object   categories / category pairs, 'Test' class with different algorithms 
  to predict the object category labels in the inference phase, 'Statistical_Tool_Api' 
  with static functions to learn GMM models and to compute GMM likelihood values,
  interface to store and load the learned model parameters to / from file. 
 
- Evaluation Framework Module:

  Module for performance evaluation of the object class prediction task.
  Computation of confusion matrix, precision and recall per object category.
  Provides also a cross-validation framework for extensive training and test experiments.

System requirements
===================================

- PCL library v. 1.4
- OpenCV library v. opencv-2.4.6.1 or higher
- CMake v. 2.6

To build the project
===================================

cd /yourPath/project-joint-object-classification/

mkdir build

cd build

cmake ..

make


Usage
===================================

See the test cases ('TestCases') for different examples of training and test of the joint 
object classification method and cross validation experiments.



