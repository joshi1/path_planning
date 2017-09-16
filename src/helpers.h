#ifndef HELPERS_H_
#define HELPERS_H_

#include <vector>
#include "map.h"
#include "vehicle.h"
#include "debug.h"



extern std::tuple <vector <double>, vector <double> >
  tx_to_vehicle_coordinates(vector<double> x,
			    vector <double> y,
			    double ref_x,
			    double ref_y,
			    double ref_yaw);

extern bool
doubleEquals(double left, double right, double epsilon=0.0000001);

extern bool
doubleLess(double left, double right, double epsilon=0.0000001, bool orequal = false);

extern bool
doubleGreater(double left, double right, double epsilon=0.0000001, bool orequal = false);


/*
 * Logistic function: A function that returns a value 
 *  - between 0 and 1 for x in the range [0, infinity] and 
 *  - between -1 to 1 for x in the range [-infinity, infinity].
 *
*/
inline double
logistic(double x)
{
  return (2.0/(1 + exp(-x)) - 1);
}

/*
 * Differentiate: Calculates the derivative of a polynomial and returns
 *  the corresponding coefficients.
*/
inline vector <double>
differentiate(vector <double> coeffs)
{
  vector <double> new_coeffs;
  for (int i = 1; i < coeffs.size(); i++){
    new_coeffs.push_back((i+1)*coeffs[i]);
  }
  return new_coeffs;
}

/*
 *Takes the coefficients of a polynomial as the first input parameter
 * and computes the function of the second variable. Time in our case.
 */
inline double
compute_poly(vector <double> coeffs, double t)
{
  double total = 0;
  for (int i = 0; i < coeffs.size(); i++) {
    total += coeffs[i]*pow(t, i);
  }
  return total;
}


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


inline std::tuple <vector <double>, vector <double> >
tx_to_vehicle_coordinates(vector<double> x,
			  vector <double> y,
			  double ref_x,
			  double ref_y,
			  double ref_yaw)
{
  vector <double> new_x;
  vector <double> new_y;

  for(int i = 0; i < x.size(); i++)
    {
      double shift_x = x[i] - ref_x;
      double shift_y = y[i] - ref_y;
      
      new_x.push_back(shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
      new_y.push_back(shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
    }

  return std::make_tuple(new_x, new_y);
}

inline bool doubleEquals(double left, double right, double epsilon) {
  return (fabs(left - right) < epsilon);
}

inline bool doubleLess(double left, double right, double epsilon, bool orequal) {
  if (fabs(left - right) < epsilon) {
    return (orequal);
  }
  return (left < right);
}

inline bool doubleGreater(double left, double right, double epsilon, bool orequal) {
  if (fabs(left - right) < epsilon) {
    return (orequal);
  }
  return (left > right);
}




#endif //HELPERS_H
