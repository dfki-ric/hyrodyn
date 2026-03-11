# Coding style for HyRoDyn

This documents gives an overview of the coding style used in the software package Hybrid Robot Dynamics (HyRoDyn) and also the general goals of HyRoDyn.

If you are considering contributing to this library please read the whole document.

## General Purpose of HyRoDyn

Hybrid Robot Dynamics (HyRoDyn) is an analytical and modular software workbench written in C++ for solving kinematics and dynamics of highly complex series-parallel hybrid robots. The main idea behind HyRoDyn is to store the closed form solutions to the loop closure constraints in a configurable mechanism library which is identified by its type (for e.g. 1-RRPR, 2SPU+1U, 2SPRR+1U, 6-UPS).  Based on submechanisms defined in a hybrid robot, HyRoDyn can modularly compose the loop closure function of the overall system in an automated way. The resulting loop closure Jacobian has a block diagonal structure that can be exploited in the computation of various forward and inverse kinematics and dynamics algorithms. HyRoDyn is implemented in C++ and utilizes recursive O(n) multi-body dynamics  algorithms  for  tree  type  systems  from  the Rigid Body Dynamics Library (RBDL) based on Featherstone's algorithms. Presently, closed form solutions to mechanisms such as 1-RRPR, 2-SPU+1U, 2-SPRR+1U, 6-RUS, 6-UPS, parallelogram chains, and numerical methods are available in its submechanism libraries and HyRoDyn can be used to analytically solve the kinematics and dynamics of arbitrary series-parallel hybrid robots composed of these submechanism modules. Actuation of the robot can be arbitrarily selected.

Just like its parent library, the algorithmic parts of HyRoDyn's code try to follow the algorithmic or mathematical notations instead of wrapping algorithms in elegant programming patterns. 

### Aims and Non-Aims of HyRoDyn

This is what HyRoDyn aims to be:

* HyRoDyn aims to be lean (think before adding an unnecessary dependency) 
* HyRoDyn aims to be easily integrated into other projects (no framework dependence to RoCK or ROS)
* HyRoDyn aims to be suitable as a foundation for sophisticated control architectures
* HyRoDyn gives you access to its internals and provides only a thin abstraction layer over the actual computation

And this is what HyRoDyn is ***not*** about:

* HyRoDyn is ***not*** a fully fledged simulator with collision detection or fancy graphics or a control toolbox. 
* HyRoDyn does not keep you from screwing up things.

Multibody dynamics is a complicated subject and in this codebase the preference is mathematical and algorithmic clarity over elegant software architecture.

## Licensing

HyRoDyn is published under the very permissive zlib license that gives you a lot of freedom in the use of full library or parts of it. The core part of the library is solely using this license but addons may use different licenses. 

There is no formal contributor license agreement for this project. Instead when you submit patches or create a pull request it is assumed that you have the rights to transfer the corresponding code to the HyRoDyn project and that you are okay that the code will be published as part of HyRoDyn.

## Data Storage

HyRoDyn tries to avoid dynamic allocations and prefers contiguous memory storage such as in ```std::vectors``` over possibly fragmented memory as in ```std::list``` or heap allocated tree structures.

Where possible we use the Structure-of-Arrays (SOA) approach to store data, e.g. the velocities v of all bodies is stored in an array (```std::vector```) of ```SpatialVector```s in the ```Model``` structure.

## Naming Conventions

1. Structs and classes use CamelCase, e.g. ```ConstraintSet```
2. Struct and class members use the lowerCamelCase convention, e.g.
  ```Model::dofCount```.
  Exceptions are:
    1. The member variable is a mathematical symbol in an algorithm reference, E.g. ```S``` is commonly used to denote the joint motion subspace, then we use the algorithm notation. For mathematical
    symbols we also allow the underscore ```_``` to denote a subscript.
    2. Specializations of existing variables may be prefixed with an identifier, followed by a underscore. E.g. ```Model::S``` is the default storage for joint motion subspaces, however for the          specialized 3-DOF joints it uses the prefix ```multdof3_``` and are therefore stored in 
```Model::multdof3_S```.
3. Only the first letter of an acronym is using a capital letter, e.g. degree of freedom (DOF) would be used as ```jointDofCount```, or ```dofCount```.
4. Variables that are not member variables use the ```snake_case``` convention.

### Examples

    struct Model {
      std::vector<SpatialVector> v;          // ok, v is an  
      std::vector<SpatialVector> S;          // ok, S is commonly used in a reference algorithm
      std::vector<double> u;                 // ok
      std::vector<Vector3d> multdof3_u;      // ok, 3-dof specialization of Model::u

      std::vector<unsigned int> mJointIndex; // NOT OK: invalid prefix
      unsigned int DOFCount;                 // NOT OK: only first letter of abbreviation should be in upper case
      double error_tol;                      // NOT OK: use camelCase instead of snake_case
      void CalcPositions();                  // NOT OK: camelCase for member variables and function must start with lower-case name

    };

## Error Handling

HyRoDyn will fail loudly and abort if an error occurs. This allows you to spot errors early on.

Code must compile without warnings with all compiler warnings enabled.

## Const Correctness

This code uses const correctness, i.e. parameters that are not expected to change must be specified as const. Use const references whenever possible. For some optional variables we use pointers, but when possible use references.

## Working with Eigen::VectorXd, Eigen::MatrixXd etc.

Use dynamic Vectors or Matrices only when they are absolutely necessary and unavoidable. When working with such types, make sure you initiliaze them with zeros and assign them a size whenever possible. Otherwise it may lead to garbage computations in many machines.

## Documentation

Most importantly the code should be readable to someone who is familiar with multibody dynamics, especially with Featherstone's notation. The documentation should mainly serve to clarify the API in terms of doxygen comments. Within the code itself comments may be used to emphasize on ideas behind it or to clarify sections. But in general it is best to write readable code in the first place as comments easily become deprecated.

The doxygen comments should be written in the header files and not in the ```.cpp``` files.

## Testing

All code contributions must provide unit tests. HyRoDyn uses Google tests as a testing framework. You can find the unit tests in ```test``` folder. Many small tests that check single features are preferred over large tests that test multiple things simultaneously.

Bugfixes ideally come with a test case that reproduce the bug.

### Working on a new feature

1. Clone the hyrodyn repository, 
2. implement the new feature, 
3. make sure it is properly documented, 
4. implement a unit test which ensures its correctness, 
5. run the remaining tests and 
6. if everything works fine create a pull request.



