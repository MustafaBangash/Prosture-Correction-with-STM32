#include <iostream>
#include <cmath>

bool alert();

bool alert(int neck_angle, int upper_angle, int lower_angle){
    if(180-(upper_angle-lower_angle)<175){
        return 1;
    }
    else if(abs(neck_angle-upper_angle)>5){
        return 1;
    }
    return 0;
}