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
        return ((abc[0]*x + abc[1])*x+abc[2]);
    }

    /** add a sample (x,y) pair */
    public void add(double x, double y){
        sumx4+=x*x*x*x;
        sumx3+=x*x*x;
        sumx2+=x*x;
        sumx+=x;
        sumx2y+=x*x*y;
        sumxy+=x*y;
        sumy+=y;
        sumy2+=y*y;
        nsamples+=1;
        //System.out.println("after "+nsamples);
        /*System.out.println("x4,x3,x2,x,x2y,xy,y,y2: "
        +sumx4+" "
        +sumx3+" "
        +sumx2+" "
        +sumx+" "
        +sumx2y+" "
        +sumxy+" "
        +sumy+" "
        +sumy2+" "
        ); */
    }

    /** construct coefficients of the system of equations 
     *  and solve it
    */
    public void fitit() {
        fitit(false);
    }
    public void fitit(boolean debug) {
            double [][] eqCoefs = new double[3][3];
        double[][] matinv;
        double[] rhs = new double[3];

        if (nsamples <3) {
            System.out.println("NEED MORE THAN "+nsamples+" TO DEFINE QUADRATIC");
            return;
        }
        /*eqCoefs[0][0] = 2.*sumx4;
        eqCoefs[0][1] = 2.*sumx3;
        eqCoefs[0][2] = 2.*sumx2;
        
        eqCoefs[1][0] = 2.*sumx3;
        eqCoefs[1][1] = 2.*sumx2;
        eqCoefs[1][2] = 2.*sumx;
        
        eqCoefs[2][0] = 2.*sumx2;
        eqCoefs[2][1] = 2.*sumx;
        eqCoefs[2][2] = 2.*nsamples;*/

        eqCoefs[0][0] = 2.*sumx4;
        eqCoefs[1][0] = 2.*sumx3;
        eqCoefs[2][0] = 2.*sumx2;
        
        eqCoefs[0][1] = 2.*sumx3;
        eqCoefs[1][1] = 2.*sumx2;
        eqCoefs[2][1] = 2.*sumx;
        
        eqCoefs[0][2] = 2.*sumx2;
        eqCoefs[1][2] = 2.*sumx;
        eqCoefs[2][2] = 2.*nsamples;

        rhs[0]        = 2.*sumx2y;
        rhs[1]        = 2.*sumxy;
        rhs[2]        = 2.*sumy;

        /* example from https://en.wikipedia.org/wiki/Gaussian_elimination#:~:text=In%20mathematics%2C%20Gaussian%20elimination%2C%20also,the%20corresponding%20matrix%20of%20coefficients */
        /*eqCoefs[0][0] = 2.;
        eqCoefs[0][1] = 1;
        eqCoefs[0][2] = -1;
        eqCoefs[1][0] = -3.;
        eqCoefs[1][1] = -1;
        eqCoefs[1][2] = 2;
        eqCoefs[2][0] = -2.;
        eqCoefs[2][1] = 1;
        eqCoefs[2][2] = 2;
        rhs[0]=8;
        rhs[1]=-11;
        rhs[2]=-3; */
        if (debug) {
            for (int i=0;i<3;i++){
                System.out.println(
                    eqCoefs[i][0]+" "
                   +eqCoefs[i][1]+" "
                   +eqCoefs[i][2]+" "
                                  );
            }
        }

        /* invert matrix */
        matinv = inverse(eqCoefs, debug);
        if(debug){
            for (int i=0;i<3;i++){
                System.out.println(
                    matinv[i][0]+" "
                   +matinv[i][1]+" "
                   +matinv[i][2]+" "
                                  );
            }
        }

        /* multiply inverse with rhs */
        for(int i=0; i<3;i++){
            abc[i]=0.;
            for(int j=0;j<3;j++){
                abc[i] += matinv[i][j]*rhs[j];
            }
        }
        //if (debug)
           System.out.println(String.format("fitter coefficients a = %f, b=%f, c=%f",abc[0],abc[1],abc[2]));
    }

    /** invert a 3x3 matrix */
    static double[][] inverse(double a[][] ){
        return inverse(a, false);
    }
    static double[][] inverse(double a[][] , boolean debug){
        int n = a.length;
        double x[][] = new double[n][n];
        double b[][] = new double[n][n];
        int index[] = new int[n];
        for (int i=0; i<n; ++i)
            b[i][i] = 1;

 // Transform the matrix into an upper triangle
        gaussian(a, index);

 // Update the matrix b[i][j] with the ratios stored
        for (int i=0; i<n-1; ++i)
            for (int j=i+1; j<n; ++j)
                for (int k=0; k<n; ++k)
                    b[index[j]][k]
                            -= a[index[j]][i]*b[index[i]][k];

 // Perform backward substitutions
        for (int i=0; i<n; ++i)
        {
            x[n-1][i] = b[index[n-1]][i]/a[index[n-1]][n-1];
            for (int j=n-2; j>=0; --j)
            {
                x[j][i] = b[index[j]][i];
                for (int k=j+1; k<n; ++k)
                {
                    x[j][i] -= a[index[j]][k]*x[k][i];
                }
                x[j][i] /= a[index[j]][j];
            }
        }
        return x;
    }

    static void gaussian(double a[][], int index[])
    {
        int n = index.length;
        double c[] = new double[n];

 // Initialize the index
        for (int i=0; i<n; ++i)
            index[i] = i;

 // Find the rescaling factors, one from each row
        for (int i=0; i<n; ++i)
        {
            double c1 = 0;
            for (int j=0; j<n; ++j)
            {
                double c0 = Math.abs(a[i][j]);
                if (c0 > c1) c1 = c0;
            }
            c[i] = c1;
        }

 // Search the pivoting element from each column
        int k = 0;
        for (int j=0; j<n-1; ++j)
        {
            double pi1 = 0;
            for (int i=j; i<n; ++i)
            {
                double pi0 = Math.abs(a[index[i]][j]);
                pi0 /= c[index[i]];
                if (pi0 > pi1)
                {
                    pi1 = pi0;
                    k = i;
                }
            }

   // Interchange rows according to the pivoting order
            int itmp = index[j];
            index[j] = index[k];
            index[k] = itmp;
            for (int i=j+1; i<n; ++i)
            {
                double pj = a[index[i]][j]/a[index[j]][j];

 // Record pivoting ratios below the diagonal
                a[index[i]][j] = pj;

 // Modify other elements accordingly
                for (int l=j+1; l<n; ++l)
                    a[index[i]][l] -= pj*a[index[j]][l];
            }
        }
    }
}
