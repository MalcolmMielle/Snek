
float calc_Distance(float thetA_first, float thetA_second, float thetA_third, float thetA_final, float x_toward, float y_toward, float z_toward){
  float x, y, z;
  getPose(thetA_first, thetA_second, thetA_third, thetA_final, &x, &y, &z);

  float x_dist = x - x_toward;
  float y_dist = y - y_toward;
  float z_dist = z - z_toward;
  float norm = (x_dist*x_dist)+(y_dist*y_dist)+(z_dist*z_dist);

  return sqrt(norm);
}

float calc_DistancePRINT(float thetA_first, float thetA_second, float thetA_third, float thetA_final, float x_toward, float y_toward, float z_toward){
  float x, y, z;
  
  getPose(1.27 + pi, -0.00, 0.57, 0.00, &x, &y, &z);

  Serial.print("pose : ");
  
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.println(z); 

  Serial.print("posetoward : ");
  
  Serial.print(x_toward);
  Serial.print(" ");
  Serial.print(y_toward);
  Serial.print(" ");
  Serial.println(z_toward); 

  float x_dist = x - x_toward;
  float y_dist = y - y_toward;
  float z_dist = z - z_toward;
  float norm = (x_dist*x_dist)+(y_dist*y_dist)+(z_dist*z_dist);

  Serial.print("norm erre : ");
  Serial.println(sqrt(norm));

  return sqrt(norm);
}

bool positif(double a){
    if(a >= 0){
        return true;
    }
    else{
        return false;
    }
}

float boundAngle(float a){
    if(a >= 2*pi){
        a = a - (2 * pi);
    }
    if(a < 0){
        a = a + (2 * pi);
    }
    return a;
}

double grad(float x_toward, float y_toward, float z_toward, float* theta_a_out, float* theta_b_out, float* theta_c_out, float* theta_d_out){


    int max_iter = 0;

    float theta_a = 0;
    float theta_b = 0;
    float theta_c = 0;
    float theta_d = 0;

    float theta_a_prev = theta_a;
    float theta_b_prev = theta_b;
    float theta_c_prev = theta_c;
    float theta_d_prev = theta_d;

    float step = 0.017/2;

    float speeda = 0;
    float speedb = 0;
    float speedc = 0;
    float speedd = 0;

    float dist = calc_Distance(theta_a, theta_b, theta_c, theta_d, x_toward, y_toward, z_toward);

    //init
    float gradient_a = calc_Distance(theta_a + step, theta_b, theta_c, theta_d, x_toward, y_toward, z_toward) - calc_Distance(theta_a - step, theta_b, theta_c, theta_d, x_toward, y_toward, z_toward);   

    float gradient_b = calc_Distance(theta_a, theta_b + step, theta_c, theta_d, x_toward, y_toward, z_toward) - calc_Distance(theta_a, theta_b - step, theta_c, theta_d, x_toward, y_toward, z_toward);   

    float gradient_c = calc_Distance(theta_a, theta_b, theta_c + step, theta_d, x_toward, y_toward, z_toward) - calc_Distance(theta_a, theta_b, theta_c - step, theta_d, x_toward, y_toward, z_toward);

    float gradient_d = calc_Distance(theta_a, theta_b, theta_c, theta_d + step, x_toward, y_toward, z_toward) - calc_Distance(theta_a, theta_b, theta_c, theta_d - step, x_toward, y_toward, z_toward);

    float old_gradient_a = gradient_a;
    float old_gradient_b = gradient_b;
    float old_gradient_c = gradient_c;
    float old_gradient_d = gradient_d;

    // Serial.println("grad ");
    // Serial.println(gradient_a);
    // Serial.println(gradient_b);
    // Serial.println(gradient_c);
    // Serial.println(gradient_d);

    Serial.println("START");

    while (dist > 10 && max_iter < 100)
    {
        max_iter++;
        float x2, y2, z2;
        getPose(theta_a, theta_b, theta_c, theta_d, &x2, &y2, &z2);

        Serial.print("Now : "); 
        Serial.print(theta_a);
        Serial.print(" ");
        Serial.print(theta_b);
        Serial.print(" ");
        Serial.print(theta_c);
        Serial.print(" ");
        Serial.println(theta_d);
        Serial.print("gives : "); 
        Serial.print(x2);
        Serial.print(" ");
        Serial.print(y2);
        Serial.print(" ");
        Serial.println(z2);
        Serial.print("dist ");
        Serial.println(dist);

        calc_DistancePRINT(theta_a, theta_b, theta_c, theta_d, x_toward, y_toward, z_toward);

        gradient_a = calc_Distance(theta_a + step, theta_b, theta_c, theta_d, x_toward, y_toward, z_toward) - calc_Distance(theta_a - step, theta_b, theta_c, theta_d, x_toward, y_toward, z_toward);   

        gradient_b = calc_Distance(theta_a, theta_b + step, theta_c, theta_d, x_toward, y_toward, z_toward) - calc_Distance(theta_a, theta_b - step, theta_c, theta_d, x_toward, y_toward, z_toward);   

        gradient_c = calc_Distance(theta_a, theta_b, theta_c + step, theta_d, x_toward, y_toward, z_toward) - calc_Distance(theta_a, theta_b, theta_c - step, theta_d, x_toward, y_toward, z_toward);

        gradient_d = calc_Distance(theta_a, theta_b, theta_c, theta_d + step, x_toward, y_toward, z_toward) - calc_Distance(theta_a, theta_b, theta_c, theta_d - step, x_toward, y_toward, z_toward);

        // Serial.println("grad ");
        // Serial.println(gradient_a);
        // Serial.println(gradient_b);
        // Serial.println(gradient_c);
        // Serial.println(gradient_d);
        // have we gone past it?

// Should be like this :
        // if(positif(old_gradient_a) != positif(gradient_a)){
        //     theta_a -= speeda * old_gradient_a / (gradient_a - old_gradient_a);
        //     speeda = 0;
        // }
        // else{
            
        //     speeda += gradient_a;
        // }

        if(positif(old_gradient_a) != positif(gradient_a)){
            theta_a = theta_a_prev;
            speeda = 0;
        }
        else{
            
            speeda += (gradient_a * step) / 2;
        }

        if(positif(old_gradient_b) != positif(gradient_b)){
            theta_b = theta_b_prev;
            speedb = 0;
        }
        else{
            speedb += (gradient_b * step) / 2;
        }
        
        if(positif(old_gradient_c) != positif(gradient_c)){
            theta_c = theta_c_prev;
            speedc = 0;
        }
        else{
            speedc += (gradient_c * step) / 2;
        }
        
        if(positif(old_gradient_d) != positif(gradient_d)){
            theta_d = theta_d_prev;
            speedd = 0;
        }
        else{
            speedd += (gradient_d * step) / 2;
        }

        // Serial.println("speed ");
        // Serial.println(speeda);
        // Serial.println(speedb);
        // Serial.println(speedc);
        // Serial.println(speedd);

        // move

        theta_a_prev = theta_a;
        theta_b_prev = theta_b;
        theta_c_prev = theta_c;
        theta_d_prev = theta_d;

        theta_a -= speeda;
        theta_b -= speedb;
        theta_c -= speedc;
        theta_d -= speedd;

        boundAngle(theta_a);
        boundAngle(theta_b);
        boundAngle(theta_c);
        boundAngle(theta_d);        
        

        if(speeda == 0 && speedb == 0 && speedc == 0 && speedd == 0){
            break;
        }

        dist = calc_Distance(theta_a, theta_b, theta_c, theta_d, x_toward, y_toward, z_toward);
        

        old_gradient_a = gradient_a;
        old_gradient_b = gradient_b;
        old_gradient_c = gradient_c;
        old_gradient_d = gradient_d;
    }

    *theta_a_out = theta_a;
    *theta_b_out = theta_b;
    *theta_c_out = theta_c;
    *theta_d_out = theta_d;


}

