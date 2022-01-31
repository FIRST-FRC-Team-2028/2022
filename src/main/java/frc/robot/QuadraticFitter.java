// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Fit quadratic curve to a set of samples
 *  and provide an interpolater function
 */
public class QuadraticFitter {
    /* process:
       Accumulate samples,
       setup system of equations,
       solve system of equations for coefficients of quadratic equation
    */
    double sumx4, sumx3, sumx2, sumx, sumy, sumx2y, sumxy, sumy2;
    int nsamples;
    double[] abc = new double[3];  // the quadratic coefficients

    /** given the fit coefficients, get the function value */
    public double yout(double x){
        return ((abc[0]*x + abc[1])*x)+abc[2];
    }

    /** add a sample (x,y) pair */
    public void add(double x, double y){
        sumx4+=sumx*sumx*sumx*sumx;
        sumx3+=sumx*sumx*sumx;
        sumx2+=sumx*sumx;
        sumx+=sumx;
        sumx2y+=sumx*sumx*sumy;
        sumxy+=sumx*sumy;
        sumy+=sumy;
        sumy2+=sumy*sumy;
        nsamples+=1;
    }

    /** use samples to construct coefficients of the system of equations 
     *  and solve it
    */
    public void fitit() {
        double [][] eqCoefs = new double[3][3];
        double[][] matinv;
        double[] rhs = new double[3];

        eqCoefs[0][1] = 2.*sumx4;
        eqCoefs[0][2] = 2.*sumx3;
        eqCoefs[0][3] = 2.*sumx2;
        rhs[0]        = sumx2y;
        
        eqCoefs[1][1] = 2.*sumx3;
        eqCoefs[1][2] = 2.*sumx2;
        eqCoefs[1][3] = 2.*sumx;
        rhs[1]        = 2.*sumxy;
        
        eqCoefs[2][1] = 2.*sumx2;
        eqCoefs[2][2] = 2.*sumx;
        eqCoefs[2][3] = 2.;
        rhs[2]        = 2.*sumy;

        /* invert matrix */
        matinv = inverse(eqCoefs);

        /* multiply inverse with rhs */
        for(int i=0; i<3;i++){
            abc[i]=0.;
            for(int j=0;j<3;j++){
                abc[i] += matinv[i][j]*rhs[j];
            }
        }
    }

    /** invert a 3x3 matrix */
    double[][] inverse(double[][] mat){
        return mat;
    }
}
