// A single dimension kalman filter

// The Measurement Update step:

// 2 Gaussians w.r.t measurements (of position) - one represents the prior belief, and the second represents the actual measurements from sensors.
// Since both have uncertainties, hence both are represented by unimodal gaussian distributions. From these 2 gaussians we calculate the posterior gaussian.

// The posterior belief gaussian gets a more confident value because it takes the gaussian-distributed uncertainties (variances and mean) into account using the formulas below.

// If u and sigma2 are mean and variance of prior gaussian pdf (belief)
// And v and r2 are mean and variance of measurement gaussian pdf
// then the new posterior is another gaussian that is more accurate. Its mean and variances are:

// new-mean (posterior) = ( u * r2 + v * sigma2) / (r2 + sigma2)    -- multiply the opposite means and variances
// new-variance (posterior) = `1 / ( (1 / sigma2) + ( 1 / r2) ) 

// This is what happens during the Measurement Update part of the Kalman Filter

// ----------------------------------------------------------------------------

// State prediction takes places after an inevitably uncertain motion (control). 
// Since measurement and prediction are iterative cycles in the kalman filter, so the 
// Posterior gaussian from measurement step is now considered as the prior belief of the state (position) for the 
// state prediction step. (i.e, the robot's best estimate of its current location)..

// The robot now executes a command (control step0 - move forward by xyz meters. The resultant gaussian is centered around xyz as mean with a variance abc.
// Calculating the state estimate is very simple:

// new mean = prior's mean + motion's mean
// new variance = prior's variance + motion's variance

#include <iostream>
#include <math.h>
#include <tuple> 

using namespace std;

double new_mean, new_var;

tuple<double, double> measurement_update(double mean1, double var1, double mean2, double var2)
{

    // remember bodmas

    new_mean = ( mean1 * var2 + mean2 * var1 ) / (var2 + var1); 
    new_var = 1 / ( (1 / var1) + ( 1 / var2) ); 

    return make_tuple(new_mean, new_var);
}

tuple<double, double> state_prediction(double mean1, double var1, double mean2, double var2)
{
    new_mean = mean1 + mean2;
    new_var =  var1 + var2;
    return make_tuple(new_mean, new_var);
}


int main()
{
   // Measurements and measurement variances

   // Let's assume data for 5 time cycles, and further assume that the 
   // measurement variance is constant to start with.

   double measurements[5] = { 5, 6, 7, 9, 10 };
   double measurement_sig = 4;

   // Motions and motion variances (for 5 time cycles again)
   double motion[5] = { 1, 1, 2, 1, 1 };
   double motion_sig = 2;

   // Kalman needs an initial state (a random guess) so here it is:
   double mu = 0;
   double sig = 1000;

   // Loop through all the measurements
   for (int i = 0; i < sizeof(measurements) / sizeof(measurements[0]); i++) {

       // Apply a measurement update
       tie(mu, sig) = measurement_update(mu, sig, measurements[i], measurement_sig);
       printf("iteration %i measurement update: [%f, %f]\n", i, mu, sig);

       // Apply a state prediction
       tie(mu, sig) = state_prediction(mu, sig, motion[i], motion_sig);
       printf("iteration %i state estimate: [%f, %f]\n\n", i, mu, sig);
   }

   return 0;
}
