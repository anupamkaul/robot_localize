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

int main()
{
   tie(new_mean, new_var) = measurement_update(10, 8, 13, 2);
   printf("[%f, %f]\n", new_mean, new_var);
   return 0;
}
