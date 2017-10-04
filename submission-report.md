# Extended Kalman Filter Project

[video1]: ./video1.mp4

## Content of the Submission and Usage
This submission includes the following c++ files:
* main.cpp: the main function that communicates with the simulator and drive the estimation process using UKF.
* filter/ParticleFilter.h, filter/ParticleFilter.cpp: contain the particle filter implementation
* utils/helper_functions.h: contains some helper functions
* map/Map.h defines landmark map
* map/Partition2D.h contains an implementation of a 2D partition for speeding up finds of nearest landmarks.

### Usage
By default, the program will use 1000 particles. However, it can be launched with different number of particles and noise:

    ./particle_filter [-parts number] [-stdgps x y yaw] [-stdland| x y]

Where the command line options are described as follows:

* -parts: specifies the number of particles to use
* -stdgps: specifies the x, y, and yaw noise of GPS measurements
* -stdland, specify the x, and y noise of landmark measurements

The program will listen on port 4567 for an incoming simulator connection. Only one simulator should be connected at anytime, though the program does not prohibit it. To start a new simulator, terminate the existing one first, then start a new one.

To start the simulator:

#### On Windows

    term2_sim9.exe

#### Other platforms:

    ./term2_sim9

#### Repeat simulations
The program can run one simulation only, it needs to be restarted in order to restart a simulation. 

#### Build
For Windows, Bash on Ubuntu on Windows should be used. Both gcc and clang can be used to build the program.

To build the program, invoke the following commands on the bash terminal:
```
mkdir build
cd build
cmake ..
make
```

The program can be built to output more information for disgnosis purposes by defining **VERBOSE_OUT** macro.
In addition, the program can be built to perform sanity tests on the 2D partitioning algorithm by defining **TEST_PARTITION** macro.

#### Build API Documentation
The documentation for functions, classes and methods are included in the header files in Doxygen format. To generate Api documentation with the included doxygen.cfg:

1. Make sure doxygen is installed
2. cd to the root folder of this project
3. Issue the following command:

    doxygen doxygen.cfg

## The Implementation

### The source code structure
The src folder contains three folders for the source files:
. filter: contains ParticleFilter.h, ParticleFilter.cpp files which implement Particle Filter.
. map: contains Map.h which defines landmark maps, and Partition2D.h which implements a simple 2D partition algorithm for speeding up finding of landmarks for a given observation.
. utils: contains helper_functions.h that defines a set of helper functions. Refer to [API documentation](api/html/helper__functions_8h.html) for the list of helper functions.

##### File names
In this project, I made a class to have its own .h, and .cpp files. More specifically, a class source file contains exactly one class, and has the same name as the class it contains.

#### The libs folder
The libs folder contains Eigen and json.hpp required by the program.

## ParticleFilter class
The class implements the **init()**, **prediction()**, **updateWeights()**, and **resample()** methods for the initialization, prediction, update, and resample process of Particle Filters.

The API documentation for ParticleFilter can be found [here](api/html/classParticleFilter.html).

### Initialization
During the initialization process, the **init()** method is invoked to create a set of particles with their x, y location and yaw angle set to the initial GPS reading perturbed with some random noise according to the GPS noise settings.

### Prediction
During the prediction stage, each particle's location and yaw are updated according to the delta time, velocity, and yaw rate. The new location and yaw angle are then perturbed according to the GPS noise settings.

**0 Yaw Rate**
0 yaw rate needs to be handled differently to avoid division by 0.

### Update Weights
In the update stage, weights of each particle is updated in the **updateWeights()** method. It first converts each observation's coordinate from the vehicle's coordinate system to the map's coordinate system. The it use Partition2D to find the nearest landmark for each observation.

Then it computes the probability of each observation according to the distance deviation between the nearest landmark and the observation.

Finally the weight of each particle is obtained as the product of the probabilities of all the observations obtained above.

#### Handling 0 weights
When the deviation is large, say 2 or more, the probability may become very low, and can result in 0 weights. When this happen to all particles, the filter will fail to produce useful result. This was observed in my early tests, and the vehicle was able to escape eventually.

Fortunately, since weights are relative, we could avoid this problem by flattening the distribution. In the implementation, I divide the exponent of the Gaussian distribution by 10, that is I flatten the distribution by an order of magnitude.

### Resampling
After the weight of each particle is updated, we resample the particles according to their weights.

## Partition2D class
The brute-force approach to find the closest landmark given an observation is to iterate through all the landmarks, and find the one with the smallest distance to the observation. For n landmarks, m samples, and s measurements, this will take n*m*s steps for each cycle.

A more efficient, simple approach is to partition the 2D space into a 2D grid. Each cell is associated with the landmarks that are in it. To find the nearest landmark to a location, x, y, we perform the followings:

1. Compute the cell containing the location
2. If there are landmarks associated with the cell, the one closest to the location is returned
3. Otherwise, we search a bigger window surrounding the cell. That is, we will search 1 cell first, then 3 by 3 cells, then 5 by 5, 7 by 7 cells ... and so on. We can repeat this process until we find the nearest one. However, in practice, it is safe to terminate the process earlier and assume a very low probability due to the fact that even if we eventually find one, the distance will too large.

Adaptive subsivisions that partition the space into a hierarchy of cells depending on the complexity of a cell is not used in this implementation for simplicity reason.

## Results
With the implementation, the program has been successfully tested against the simulator.
The execution of the third scenario is recorded in [this video](video1.mp4).

The following table shows the X, Y, and Yaw errors against the number of particles.

|        |   500    |  1000   |  2000   |  4000   |
|:-------|:--------:|:-------:|:-------:|:-------:|
|  Time  |  49.14   |  49.74  |  48.98  |  97.34  |
|  X     |  0.165   |  0.158  |  0.154  |  0.149  | 
|  Y     |  0.109   |  0.106  |  0.106  |  0.102  | 
|  Yaw   |  0.004   |  0.004  |  0.004  |  0.004  | 

It shows that the errors for x, and y is reduced slightly when more particles are used, but the yaw error stays the same. In the case of 4000 particles, the simulation tooks double the time to completed.

### Effectiveness of Space Partition
As for how effectively the space partition is, the average number of landmark search is around **1.015** per observation. This is very effective comparing to the brute-force approach that will require **42** per observation.



