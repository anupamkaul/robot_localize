#include <iostream>
#include <math.h>
#include <vector>
using namespace std;

// There are 8 sonar range-finder sensors in this project.

// Sensor characteristic: Min and Max ranges of the beams
double Zmax = 5000, Zmin = 170; // sensor range limits, assumed for our calculations

// Defining free cells(lfree), occupied cells(locc), unknown cells(l0) Log Odds Values
double l0 = 0, locc = 0.4, lfree = -0.4;

// Grid unit dimensions - or cell dimensions
double gridWidth = 100, gridHeight = 100;

// Map dimensions - full map dimensions
double mapWidth = 30000, mapHeight = 15000;

// Robot size with respect to the map 
double robotXOffset = mapWidth / 5, robotYOffset = mapHeight / 3;

// Defining an l vector to store the log odds values of each cell
// This is a vector of vectors, and is the basic data rep of an occupancy grid map

vector< vector<double> > l(mapWidth/gridWidth, vector<double>(mapHeight/gridHeight));

double inverseSensorModel(double x, double y, double theta, double xi, double yi, double sensorData[])
{
    // (see InverseSensor.png)

    // Defining Sensor Characteristics
    double Zk, thetaK, sensorTheta;
    double minDelta = -1;
    double alpha = 200, beta = 20; // beta - opening cone angle, alpha = obstacle width ~ cell width

    //******************TODO: Compute r and phi**********************//
   

    //Scaling Measurement to [-90 -37.5 -22.5 -7.5 7.5 22.5 37.5 90] (8 sonar range-finders)

    double r = sqrt(pow(xi - x , 2) + pow(yi -y, 2));
    double phi = atan2(yi - y, xi -x) - theta;
    
    for (int i = 0; i < 8; i++) {
        if (i == 0) {
            sensorTheta = -90 * (M_PI / 180);
        }
        else if (i == 1) {
            sensorTheta = -37.5 * (M_PI / 180);
        }
        else if (i == 6) {
            sensorTheta = 37.5 * (M_PI / 180);
        }
        else if (i == 7) {
            sensorTheta = 90 * (M_PI / 180);
        }
        else {
            sensorTheta = (-37.5 + (i - 1) * 15) * (M_PI / 180);
        }

        if (fabs(phi - sensorTheta) < minDelta || minDelta == -1) {
            Zk = sensorData[i];
            thetaK = sensorTheta;
            minDelta = fabs(phi - sensorTheta);
        }
    }

    //******************TODO: Evaluate the three cases**********************//
    // You also have to consider the cells with Zk > Zmax or Zk < Zmin as unkown states
    
    if ((r > min(Zmax, Zk + alpha/2)) || ( fabs(phi - thetaK) > beta/2) || (Zk > Zmax) || (Zk < Zmin) ) {
        return l0;
    }
    else if ((Zk < Zmax) && ((r - Zk) < alpha/2)) {
        return locc; 
    }
    else if (r <= Zk) {
        return lfree;  
    } 
}

// The occupancy grid takes a precise Localized Pose, and sensorData as its inputs

// It generates a Grid, center-masses every cell, and checks what area falls under the 
// perceptual field of measurements.

void occupancyGridMapping(double Robotx, double Roboty, double Robottheta, double sensorData[])
{
    // Main Occupancy Grid Mapping Algorithm

    // Generate a grid (size 300 * 150) and then loop through all the cells
    for (int x = 0; x < mapWidth / gridWidth; x++) {

        for (int y = 0; y < mapHeight / gridHeight; y++) { // col first, per x

            // Calculate the center of mass (xi, yi) of each cell ..

            double xi = x * gridWidth + gridWidth / 2 - robotXOffset;
            double yi = -(y * gridHeight + gridHeight / 2) + robotYOffset;

            // Check if each cell falls under the perceptual range of measurements
            // Do that by just comparing hyp. distance to a limit

            if (sqrt(pow(xi - Robotx, 2) + pow(yi - Roboty, 2)) <= Zmax) {

                // if it falls under perception of sensors, then update its map inclusion (occupied, empty or unknown? - with shades)
                l[x][y] = l[x][y] + inverseSensorModel(Robotx, Roboty, Robottheta, xi, yi, sensorData) - l0;
            }
        }
    }   
    
}

int main()
{
    double timeStamp;
    double measurementData[8];
    double robotX, robotY, robotTheta;

    FILE* posesFile = fopen("poses.txt", "r");
    FILE* measurementFile = fopen("measurement.txt", "r");

    // Scanning the files and retrieving measurement and poses at each timestamp
    while (fscanf(posesFile, "%lf %lf %lf %lf", &timeStamp, &robotX, &robotY, &robotTheta) != EOF) {
        fscanf(measurementFile, "%lf", &timeStamp);
        for (int i = 0; i < 8; i++) {
            fscanf(measurementFile, "%lf", &measurementData[i]);
        }
        occupancyGridMapping(robotX, robotY, (robotTheta / 10) * (M_PI / 180), measurementData);
    }
    
    // Displaying the map
    for (int x = 0; x < mapWidth / gridWidth; x++) {
        for (int y = 0; y < mapHeight / gridHeight; y++) {
            cout << l[x][y] << " ";
        }
    }
  
    cout << endl;
  
    return 0;
}

