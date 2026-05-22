#include "bpmp_utils/bpmp_utils.h"
#include <cmath>  // (Yunwoo) Added for std::pow

namespace bpmp_tracker{
    double interpolate(const vector<double> &xData, const vector<double> &yData, const double &x){
        int size = (int)xData.size();
        if(xData.size()<2)
            return yData[0];
    
        if(x<xData[0])
            return yData[0];
    
        if(x>xData[size-1])
            return yData.back();
    
        double xL= xData[0], yL = yData[0], xR = xData.back(), yR = yData.back();
    
        for(int i =0;i<size-2;i++){
            if(xData[i]<=x and x<xData[i+1]){
                xL = xData[i]; yL = yData[i];
                xR = xData[i+1], yR = yData[i+1];
                break;
            }
        }
        double dydx = (yR-yL)/(xR-xL);
        return yL + dydx * (x-xL);
    }

    int factorial(int num){
        if (num<=1) return 1;
        return num*factorial(num-1);
    }
    int nchooser(int n, int r) {
        if (n < 0 or r < 0)
            return 0;
        if (n == r)
            return 1;
        return n_choose_r[n][r];
    }
    double getBernsteinValue(double bern_ctrl_pts[], double t, double t0, double tf, int poly_order){
        double value = 0.0;
        for (int i = 0; i < poly_order + 1; i++) {
            value += bern_ctrl_pts[i] * double(nchooser(poly_order, i)) * std::pow(t - t0, i) *
                     std::pow(tf - t, poly_order - i) / std::pow(tf - t0, poly_order);
        }
        return value;
    }
}