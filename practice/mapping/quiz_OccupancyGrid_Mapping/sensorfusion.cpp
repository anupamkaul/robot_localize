#include <iostream>
#include <math.h>
using namespace std;

/* when sensors have different characteristics, can see different obstacles etc,
   then a good approach is to let every sensor generate its own map, and then
   we combine the maps.

   either use de-morgan's laws (1 - product of probs), or simply OR the values,
   such that obstacles are identified by the lowest obs-range sensor
*/

const int mapWidth =  2;
const int mapHeight = 2;

void sensorFusion(double m1[][mapWidth], double m2[][mapWidth])
{
    //*#############TODO: Code the Sensor Fusion Function############*//

    // Fuse the measurments of the two maps and print the resulting 
    //map in a matrix form:
    //a  b
    //c  d
 
    double sfmap[mapHeight][mapWidth];

    for(int i = 0; i < mapWidth; i++) {

        for(int j=0; j < mapHeight; j++) {
  
            //sfmap[i][j] = m1[i][j] | m2[i][j]; (bitwise ORs don't work on double or floats 
                                               // unless I handle sign, exponent and mantissa parts out)

            // Use De-Morgan's product rule
            sfmap[i][j] = 1 - (1 - m1[i][j]) * (1 - m2[i][j]);
            cout << " " << sfmap[i][j];
        }
        cout << endl;
    }   
}

int main()
{

    double m1[mapHeight][mapWidth] = { { 0.9, 0.6 }, { 0.1, 0.5 } };
    double m2[mapHeight][mapWidth] = { { 0.3, 0.4 }, { 0.4, 0.3 } };
    sensorFusion(m1, m2);

    return 0;
}

