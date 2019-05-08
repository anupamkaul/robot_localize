//#include "src/matplotlibcpp.h"//Graph Library
#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <stdexcept> // throw errors
#include <random> //C++ 11 Random Numbers

//namespace plt = matplotlibcpp;
using namespace std;

// We assume a square Map available to the robot that is 100 * 100
// The Map contains 8 landmarks percievable & measurable by its on-board sensors

// The sensors will be noisy, and we will simulate that noise for the purposes
// of this program

// Landmarks
double landmarks[8][2] = { { 20.0, 20.0 }, { 20.0, 80.0 }, { 20.0, 50.0 },
    { 50.0, 20.0 }, { 50.0, 80.0 }, { 80.0, 80.0 },
    { 80.0, 20.0 }, { 80.0, 50.0 } };

// Map size in meters 

double world_size = 100.0;

// Random Generators
random_device rd;
mt19937 gen(rd());

// Global Functions
double mod(double first_term, double second_term);
double gen_real_random();

class Robot {
public:
    Robot()
    {
        // Constructor
        x = gen_real_random() * world_size; // robot's x coordinate
        y = gen_real_random() * world_size; // robot's y coordinate
        orient = gen_real_random() * 2.0 * M_PI; // robot's orientation

        forward_noise = 0.0; //noise of the forward movement
        turn_noise = 0.0; //noise of the turn
        sense_noise = 0.0; //noise of the sensing
    }

    void set(double new_x, double new_y, double new_orient)
    {
        // Set robot new position and orientation
        if (new_x < 0 || new_x >= world_size)
            throw std::invalid_argument("X coordinate out of bound");
        if (new_y < 0 || new_y >= world_size)
            throw std::invalid_argument("Y coordinate out of bound");
        if (new_orient < 0 || new_orient >= 2 * M_PI)
            throw std::invalid_argument("Orientation must be in [0..2pi]");

        x = new_x;
        y = new_y;
        orient = new_orient;
    }

    void set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise)
    {
        // Simulate noise, often useful in particle filters
        forward_noise = new_forward_noise;
        turn_noise = new_turn_noise;
        sense_noise = new_sense_noise;
    }

    vector<double> sense()
    {
        // Measure the distances from the robot toward the landmarks
        vector<double> z(sizeof(landmarks) / sizeof(landmarks[0]));
        double dist;

        for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++) {
            dist = sqrt(pow((x - landmarks[i][0]), 2) + pow((y - landmarks[i][1]), 2));
            dist += gen_gauss_random(0.0, sense_noise);
            z[i] = dist;
        }
        return z;
    }

    Robot move(double turn, double forward)
    {
        if (forward < 0)
            throw std::invalid_argument("Robot cannot move backward");

        // turn, and add randomness to the turning command
        orient = orient + turn + gen_gauss_random(0.0, turn_noise);
        orient = mod(orient, 2 * M_PI);

        // move, and add randomness to the motion command
        double dist = forward + gen_gauss_random(0.0, forward_noise);
        x = x + (cos(orient) * dist);
        y = y + (sin(orient) * dist);

        // cyclic truncate
        x = mod(x, world_size);
        y = mod(y, world_size);

        // set particle <---- Note Carefully !!
        Robot res;
        res.set(x, y, orient);
        res.set_noise(forward_noise, turn_noise, sense_noise);

        return res;
    }

    string show_pose()
    {
        // Returns the robot current position and orientation in a string format
        return "[x=" + to_string(x) + " y=" + to_string(y) + " orient=" + to_string(orient) + "]";
    }

    string read_sensors()
    {
        // Returns all the distances from the robot toward the landmarks
        vector<double> z = sense();
        string readings = "[";
        for (int i = 0; i < z.size(); i++) {
            readings += to_string(z[i]) + " ";
        }
        readings[readings.size() - 1] = ']';

        return readings;
    }

    double measurement_prob(vector<double> measurement)
    {
        // Calculates how likely a measurement should be
        double prob = 1.0;
        double dist;

        for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++) {
            dist = sqrt(pow((x - landmarks[i][0]), 2) + pow((y - landmarks[i][1]), 2));
            prob *= gaussian(dist, sense_noise, measurement[i]);
        }

        return prob;
    }

    double x, y, orient; //robot poses
    double forward_noise, turn_noise, sense_noise; //robot noises

private:
    double gen_gauss_random(double mean, double variance)
    {
        // Gaussian random
        normal_distribution<double> gauss_dist(mean, variance);
        return gauss_dist(gen);
    }

    double gaussian(double mu, double sigma, double x)
    {
        // Probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(-(pow((mu - x), 2)) / (pow(sigma, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(sigma, 2)));
    }
};

// Functions
double gen_real_random()
{
    // Generate real random between 0 and 1
    uniform_real_distribution<double> real_dist(0.0, 1.0); //Real
    return real_dist(gen);
}

double mod(double first_term, double second_term)
{
    // Compute the modulus
    return first_term - (second_term)*floor(first_term / (second_term));
}

double evaluation(Robot r, Robot p[], int n)
{
    //Calculate the mean error of the system
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        //the second part is because of world's cyclicity
        double dx = mod((p[i].x - r.x + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double dy = mod((p[i].y - r.y + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double err = sqrt(pow(dx, 2) + pow(dy, 2));
        sum += err;
    }
    return sum / n;
}
double max(double arr[], int n)
{
    // Identify the max element in an array
    double max = 0;
    for (int i = 0; i < n; i++) {
        if (arr[i] > max)
            max = arr[i];
    }
    return max;
}
/*
void visualization(int n, Robot robot, int step, Robot p[], Robot pr[])
{
	//Draw the robot, landmarks, particles and resampled particles on a graph
	
    //Graph Format
    plt::title("MCL, step " + to_string(step));
    plt::xlim(0, 100);
    plt::ylim(0, 100);

    //Draw particles in green
    for (int i = 0; i < n; i++) {
        plt::plot({ p[i].x }, { p[i].y }, "go");
    }

    //Draw resampled particles in yellow
    for (int i = 0; i < n; i++) {
        plt::plot({ pr[i].x }, { pr[i].y }, "yo");
    }

    //Draw landmarks in red
    for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++) {
        plt::plot({ landmarks[i][0] }, { landmarks[i][1] }, "ro");
    }
    
    //Draw robot position in blue
    plt::plot({ robot.x }, { robot.y }, "bo");

	//Save the image and close the plot
    plt::save("./Images/Step" + to_string(step) + ".png");
    plt::clf();
}
*/

int main()
{ 
    // Instantiate a robot object from the Robot class
    Robot myrobot;

    // Simulate sensor noise..
    myrobot.set_noise(5.0, 0.1, 5.0);
    
    myrobot.set(30.0,  50.0, M_PI/2.0);
    cout << "Initial robot pose: " << myrobot.show_pose() << endl;

    myrobot.move(-M_PI / 2.0, 15.0);
    // show distance from landmarks
    cout << myrobot.read_sensors() << endl; 

    myrobot.move(-M_PI / 2.0, 10.0);
    // show distance from landmarks
    cout << myrobot.read_sensors() << endl; 

    // Print out the new robot position and orientation
    cout << "New pose: " << myrobot.show_pose() << endl;

    // Printing the distance from the robot toward the eight landmarks
    cout << "Robot's distance from landmarks: " << myrobot.read_sensors() << endl;

    // MCL Algo: 

    // A clever implementation of a recursive bayes estimation algo,
    // where the Bel(x)s (PDFs) are represented by samples, to which "importance weights"
    // are attached. Using bayes markovian character, new updates are calculated based on
    // weights (1 state diff only) and the "new samples" basically represent new belief ...

    // Instantiate 1000 Particles each with random position and orientation
    int n = 1000; // maxParticles
    Robot p[n]; //robotParticles assigned random values via constructor

    // Loop over the set of particles. For each particle, add random noise
    for (int i = 0; i < n; i++) {

        // For each particle, add random noise (the same random noise to every particle)
        // (i.e. configure "equally bad and noisy" sensor on every fake robot particle ...
        p[i].set_noise(0.05, 0.05, 5.0);
        
        // Print each particle's pose on a single line
        // cout << "MCL Particle " << i << " Pose data: " << p[i].show_pose() << endl;
    }

    //Re-initialize myrobot object and Initialize a measurment vector
    myrobot = Robot();
    vector<double> z;

    //Move the robot and sense the environment afterwards
    //These are the actual measurements of landmarks made by the "real" robot..

    myrobot = myrobot.move(0.1, 5.0); // <-- Note Carefully, a new particle is created!
    z = myrobot.sense();  // collect the actual measurement in a vector

    // Now, simulate exact same motion for each 'fake' robot particle:

    // Create a new particle set 'p2'
    // Rotate each particle by 0.1 and move it forward by 5.0
    // Assign p2 to p and print the particle poses, each on a single line

    Robot p2[n]; 

    for (int i = 0; i < n; i++) {
        p2[i] = p[i].move(0.1, 5.0); // <-- Note carefully, a default copy constructor is avoided since move creates a new particle everytime !!
        p[i] =  p2[i];  
        cout << "MCL Particle (post move) " << i << " Pose data: " << p[i].show_pose() << endl;
    }

    // Now Generate particle weights depending on robot's measurement
    // Print particle weights, each on a single line

    // Here is a wieght-vector that holds the importance-weight values of the 1000 particles
    double w[n];  // shouldn't weight be private data / character of each fake robot particle?

    // to calculat the weight of each particle, we will compare the robot's actual measurements (in z)
    // to the predicted measurements (of landmarks) of that particle. The magnitude of the difference of measurement
    // will signify the wieght. (The closer the measurements are, the larger the weights, thereby increasing the 
    // probability that those particles will likely be picked-up on the next time-stamp)

    cout << " Wieght of Actual robot: " << myrobot.measurement_prob(z);

    // generate particle weights depending on robot's measurements
    for (int i = 0; i < n; i++) {

        //vector<double> pz = p[i].sense(); // measure landmark distances that this fake particle sees

        // since the sensor measurements now contain 8 distinct values of landmarks that this fake-one sees,
        // how does one compare these set of 8 values to the "real" 8 values observed by the actual robot?

        // For this, look at 'measurement_prob' function defined above. This generates a likelihood probability
        // of the goodness of a measurement by again comparing it to the actual landmarks and returns a gaussian-noise
        // based single value probability

        w[i] = p[i].measurement_prob(z); 
        cout << w[i] << endl;
        cout << "MCL Particle " << i << " Weight: " << w[i] << endl;
    }

   //resample the particles with a sample probability proportional to the importance weight

    Robot p3[n];
    int index = gen_real_random() * n;
    //cout << index << endl;

    double beta = 0.0;
    double mw = max(w, n);
    //cout << mw;
    for (int i = 0; i < n; i++) {
        beta += gen_real_random() * 2.0 * mw;
        while (beta > w[index]) {
            beta -= w[index];
            index = mod((index + 1), n);
        }
        p3[i] = p[index];
    }
    for (int k=0; k < n; k++) {
        p[k] = p3[k];
        cout << p[k].show_pose() << endl;
    }

    return 0;
}
