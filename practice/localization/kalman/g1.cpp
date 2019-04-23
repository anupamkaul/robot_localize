// simple function to calculate gaussian (the famous normal or bell curve) of a variable x's value,
// given mean and variance of the expected gaussian curve.

// Based on expected uncertainty, the gaussian curve sets a clear limit to the expected values.
// the peak of the curve represents the most likely value, called mean. The width of the 
// curve is called the variance. Obviously smaller variances means better accuracy and prediction.

// If  mu = gaussian mean
// and sigma-squared = gaussian variance
// the the probablilty distribution of variable x (as a gaussian PDF) is:

// p(x) = ( 1/ sq.root(2 * pi * sigma-squared) ) * exponential ( - (x - mu)squared / 2 * sigma-squared

// The formula contains an exponential of a quadratic function. Compares the value of x to mu, and when x = mu, exponential is e(power 0) = 1. 
// The constant in front of the eqn is a necessary normalizing factor.

#include <iostream>
#include <math.h>

using namespace std;

double f(double mu, double sigma2, double x)
{
  // why does this not work - all it does is 1/2 instead of 0.5 ... (because BODMAS .. the division operator takes precedence. Yuck.)

  //double prob = (1.0 / sqrt(2.0 * M_PI * sigma2)) * exp ( - (pow( (x - mu), 2.0) / 2 * sigma2)); // - incorrect (bodmas error)
  double prob = (1.0 / sqrt(2.0 * M_PI * sigma2)) * exp ( - (pow( (x - mu), 2.0) / (2 * sigma2))); // - correct - group the 2 * sigma to have it happen first.

   // double prob = (1.0 / sqrt(2.0 * M_PI * sigma2)) * exp ( - 0.5 *  (pow( (x - mu), 2.0) / sigma2)); (ref, correct)

    return prob;
}

int main()
{
    cout << f(10.0, 4.0, 8.0) << endl;
    return 0;
}
