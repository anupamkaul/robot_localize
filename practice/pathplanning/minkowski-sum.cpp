/* 

Purpose : Define minkowski sum of two triangles - (minkowski-sum-initial.png)
the Red represents the Robot and the Blue represents the obstacle.

* main() - Define the coordinates of triangle A and B in 2D vectors
* minkowski_sum() - Compute the Minkowski Sun of two vectors
* delete_duplicate() - Check for duplicate coordinates inside a 2D vector and delete them

*/


#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

// We create a vector of vectors. The subvector has (x, y), i.e. 2 elements constantly

// Print 2D vectors coordinate values
void print2DVector(vector<vector<int> > vec)
{

    // sort the vector for grading purpose
    sort(vec.begin(), vec.end());
    for (int i = 0; i < vec.size(); ++i) {
        for (int j=0; j < vec[0].size(); ++j) {  //(vec[0].size = 2, so toggle between printing element 1 and 2 of that subvector
            cout << vec[i][j] << " ";
        }
        cout << endl;
    }
}

// Check for duplicate coordinates inside a 2D vector and delete them
vector<vector<int> > delete_duplicate(vector<vector<int> > C)
{

    // Sort the C vector
    sort(C.begin(), C.end());

    // Initialize a non duplicated vector
    vector<vector<int> > Cret;
    for (int i=0; i < C.size() - 1; i++) {
        if (C[i] != C[i+1]) {
            Cret.push_back(C[i]);
        }
    }

    Cret.push_back(C[C.size() - 1]);
    return Cret;
}


// Compute the Minkowski Sum of two vectors
vector<vector<int> > minkowski_sum(vector<vector<int> > A, vector<vector<int> > B)
{

    vector<vector<int> > C;

    for (int i = 0; i < A.size(); i++) {
        for (int j = 0; j < B.size(); j++) {
 
            // [0] dim is x value, [1] dim is y value.. so add X with X, and Y with Y       
            vector<int> Ci = { A[i][0] + B[j][0], A[i][1] + B[j][1] }; 
            C.push_back(Ci);
        }
    }
    
    C = delete_duplicate(C);
    return C;
    
}


int main()
{

   // Define the coordinates of triangle A and B using 2D vectors
   
   // Blue triangle
   vector<vector<int> > A(3, vector<int>(2));
   A = {{0, -1}, {0, 1}, {1, 0}, };

   // Red triangle
   vector<vector<int> > B(3, vector<int>(2));
   B = {{0, 0}, {1, 1}, {1, -1}, };

   // Compute the minkowski sum of triangle A and B

   vector<vector<int> > C;
   C = minkowski_sum(A, B);

   cout << "A: \n"; print2DVector(A); cout << endl;
   cout << "B: \n"; print2DVector(B); cout << endl;
   cout << "Minkowski Sum (C): \n"; print2DVector(C); cout << endl;

   cout << endl;

   return 0;

}



