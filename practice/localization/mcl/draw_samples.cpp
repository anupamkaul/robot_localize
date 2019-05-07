#include <iostream>
using namespace std;

double w[] = { 0.6, 1.2, 2.4, 0.6, 1.2 };//Can also change this to a vector
double sum = 0;

// Define a  ComputeProb function and compute the Probabilities

double computeProb(double w[], int len) {

  // calculate sum of weights
  for (int i=0; i < len; i++) {
      sum += w[i];    
  }

  // the normalized weights now are the probabilities ! 
  for (int i=0; i < len; i++) {
      w[i] /= sum;    
      cout << "P" << i + 1 << " = " << w[i] << endl;
  }

  // note that the sum of the new normalized weights (or probs) will
  // always be 1.
} 

int main()
{
    //TODO: Print Probabilites each on a single line:
    //P1=Value
    //:
    //P5=Value
    
    computeProb(w, sizeof(w) / sizeof(w[0]));   
    return 0;
}

