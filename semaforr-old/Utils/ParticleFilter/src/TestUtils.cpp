#include <iostream> 
#include "../include/Utils.h" 

using namespace std; 

int main (int argc, char** argv){
  int firstAngle, secondAngle; 

  char cont; 

  do {
    cout << "Enter first angle: " ; 
    cin >> firstAngle ; 
    cout << "Enter second angle: " ; 
    cin >> secondAngle; 
    
    double angleDiff = Utils::calcAngleDifference(Utils::toRadians(firstAngle), Utils::toRadians(secondAngle)); 
    cout << "The difference is : " << Utils::toDegrees(angleDiff) << endl; 
    
    cout << "Hit 'y' if you want to test again" << endl;
    cin >> cont; 
    
  } while ( cont == 'y' ) ; 
  
  return 0; 
}
