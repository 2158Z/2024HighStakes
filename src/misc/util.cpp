namespace util{
    double clamp(double value, double min, double max){
        if (value < min){
            return min;
        }
        if (value > max){
            return max;
        }
        return value;
    }

    float sgn(float value){
        if (value < 0){
            return -1;
        } else if (value > 0){
            return 1;
        } else {
            return 0;
        }
    }
}