/*
 *  Code originally from http://playground.arduino.cc/Code/MatrixMath
 *  Edited starting on 2/7/16 by buckbaskin
 */
double pi = 3.14159;
 //Robot variable
float alphA_first = -pi/2 ;  
float alphA_second = pi/2;
float alphA_third = -pi/2;
float alphA_final = 0;
float d1 = 0;
float d2 = 0;
float d3 = 0;
float df = 0;
float r1 = 95;
float r2 = 25;
float r3 = 95;
float rf = 25;

void getMatrix(float theta, float alpha, float d, float r, float A_out[][N]){
    A_out[0][0] = cos(theta);
    A_out[0][1] = -sin(theta) * cos(alpha);
    A_out[0][2] = sin(theta) * sin(alpha);
    A_out[0][3] = r * cos(theta);
    A_out[1][0] = sin(theta);
    A_out[1][1] = cos(theta) * cos(alpha);
    A_out[1][2] = -cos(theta) * sin(alpha);
    A_out[1][3] = r * sin(theta);
    A_out[2][0] = 0;
    A_out[2][1] = sin(alpha);
    A_out[2][2] = cos(alpha);
    A_out[2][3] = d;
    A_out[3][0] = 0;
    A_out[3][1] = 0;
    A_out[3][2] = 0;
    A_out[3][3] = 1;
}


void forwardKinetic(float A1[][N], float A2[][N], float A3[][N], float A4[][N], float A_out[][N]){

    float A_temp[N][N];  
    Matrix.Multiply((float*)A1,(float*)A2,N,N,N,(float*)A_temp);
    float A_temp2[N][N]; 
    Matrix.Multiply((float*)A_temp,(float*)A3,N,N,N,(float*)A_temp2);
    
    Matrix.Multiply((float*)A_temp2,(float*)A4,N,N,N,(float*)A_out);

}


void getPose(float A[][N], float* x, float* y, float* z){
    float source[N];
    source[0] = 0;
    source[1] = 0;
    source[2] = 0;
    source[3] = 1;
    float A_temp[N];  
    Matrix.Multiply((float*)A,(float*)source,N,N,1,(float*)A_temp);

    // Matrix.Print((float*)A_temp,N,1,"a_tmep");
    *x = A_temp[0];
    *y = A_temp[1];
    *z = A_temp[2];    
}

getPose(float t1, float t2, float t3, float tf, float* x, float* y, float *z){
    float A_first[N][N];
    float A_second[N][N];
    float A_third[N][N];
    float A_final[N][N]; 
    float Akinetic[N][N]; 

    getMatrix(t1, alphA_first, d1, r1, A_first);
    // Matrix.Print((float*)A_first,N,N,"a1");
    getMatrix(t2, alphA_second, d2, r2, A_second);
    getMatrix(t3, alphA_third, d3, r3, A_third);
    getMatrix(tf, alphA_final, df, rf, A_final);

    forwardKinetic(A_first, A_second, A_third, A_final, Akinetic);
    getPose(Akinetic, x, y, z);
}