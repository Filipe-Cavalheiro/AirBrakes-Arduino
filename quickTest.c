#include <stdlib.h>
#include <stdio.h>
#include <math.h>

typedef struct {//AB = Air Brake; CD = Coefficient of drag
  int setpoint;
  int Kp;
  int Ki;
  int Kd;
  double mass;
  double rocket_area;
  double AB_area;
  double rocket_CD;
  double AB_CD;
} *Parameters, _Parameters;

Parameters makeParameters(int setpoint, int Kp, int Ki, int Kd, double rocket_area, double AB_area, double rocket_CD, double AB_CD){
    Parameters param = (Parameters)malloc(sizeof(_Parameters));
    param->setpoint = setpoint;
    param->Kp = Kp;
    param->Ki = Ki;
    param->Kd = Kd;
    param->rocket_area = rocket_area;
    param->AB_area = AB_area;
    param->rocket_CD = rocket_CD;
    param->AB_CD = AB_CD;
    return param;
}

typedef struct {
  double height;
  double velocity;
  double vertVelocity;
  double mass;
} *Rocket, _Rocket;

Rocket makeRocket(double height, double velocity, double vertVelocity, double mass){
    Rocket rocket= (Rocket)malloc(sizeof(_Rocket));
    rocket->height = height;
    rocket->velocity = velocity;
    rocket->vertVelocity = vertVelocity;
    rocket->mass = mass;
    return rocket;
}

typedef struct {
  int step;
  double inter;
  double diff; 
  double prev_err;
  double prev_predApogee;
  double tau;
} *Miscellaneous, _Miscellaneous;

Miscellaneous makeMiscellaneous(int step, double inter, double diff, double prev_err, double prev_predApogee, double tau){
    Miscellaneous misc= (Miscellaneous)malloc(sizeof(_Miscellaneous));
    misc->step = step;
    misc->inter = inter;
    misc->diff = diff;
    misc->prev_err = prev_err;
    misc->prev_predApogee = prev_predApogee;
    misc->tau = tau;
    return misc;
}

double airDensity(double altitude) 
/**
 * Using a linear approximation (Y = -1.05^{-4} + 1.225) finds the air density for the given altitude.
 * 
 * @param altitude
 * 
 * @return The air density*/{
    return -0.000105 * altitude + 1.225;
}

double dragSurface(double altitude, double velocity, double area, double C_drag)
/**
 * Finds the drag force of a surfaces given the current velocity, 
 * altitude and area.
 * 
 * @return The drag given altitude, velocity, and area.*/{
    return (C_drag * airDensity(altitude)* velocity * velocity * area / 2);
}

double requiredDrag(Rocket rocket, Parameters param, Miscellaneous misc)
/**
 * Computes required drag using a PID controller.
 * 
 * @param rocket structure that contains rocket variables;
 * @param param parameters of the simulation;
 * @param misc miscellaneous variables to be updates TODO: categorize correctly
 * 
 * @return required drag
 */
{
    //TODO open Ab only after x time or when aceleration is 0 or < 0

    // Initial conditions
    double out = 0;
    double alt = rocket->height;
    double velocity = rocket->velocity;
    double vertVelocity = rocket->vertVelocity;
    double mass = rocket->mass;

    double gravity = 9.80665; /** TODO: pass gravity to depend on height*/
    double refArea = param->rocket_area;

    double termVelocity = sqrt((2 * mass * gravity) / (param->rocket_CD * refArea * 1.225));
    double predApogee = alt + (((pow(termVelocity, 2)) / (2 * gravity))
            * log((pow(vertVelocity, 2) + pow(termVelocity, 2)) / (pow(termVelocity, 2))));

    // PID Controller
    double minInte = dragSurface(predApogee, velocity, param->rocket_area, 0.5);
    double maxInte = dragSurface(predApogee, velocity, param->AB_area, 1.17) + minInte; 
    
    // Error function
    double err = param->setpoint - predApogee;

    // Proportional term
    double prop = -param->Kp * err;

    // Integral term
    misc->inter += 0.5 * param->Ki * misc->step * (err + misc->prev_err);
    
    // Differential term
    misc->diff = (-2 * param->Kd * (predApogee - misc->prev_predApogee) + (2 * misc->tau - misc->step) * misc->diff) / (2 * misc->tau + misc->step);

    // Anti-wind up (integral clamping)
    if (misc->inter > maxInte)
        misc->inter = maxInte;
    else if (misc->inter < minInte)
        misc->inter = minInte;
    
    // Output
    out = prop + misc->inter + misc->diff;

    // Update memory
    misc->prev_err = err;
    misc->prev_predApogee = predApogee;

    return out;
}

double airBrakeArea(double drag, double velocity, double airDensity, double AB_CD)
/**
 * Using the drag formula we can calculate the area of the air brake
 * 
 * @return air brake Area*/{
    return (drag * 2 / (velocity * velocity * airDensity * AB_CD));
}

double airbrakeForce(Rocket rocket, Parameters param, Miscellaneous misc) {
    double reqDrag = requiredDrag(rocket, param, misc);
    double maxDrag = dragSurface(rocket->height, rocket->velocity, param->AB_area, param->AB_CD);
    if (reqDrag > maxDrag){
        reqDrag = maxDrag;
    } else if (reqDrag < 0){
        reqDrag = 0;
    }
    return reqDrag;
}

#define MAX_AREA 0.004484

float servoAngle(double area) 
/**
 * Using a linear approximation (Y = 40142.73 * area) finds the servo angle for the given area.
 * 
 * @param area AB area
 * 
 * @return servo angle for the given area*/{
    if(area < 0)
        return 0;
    if(area > MAX_AREA)
        return 180;
    return 40142.73 * area;
}

#define MAX_ROWS 875
#define MAX_COLS 4

int getcsvFile(double values[MAX_ROWS][MAX_COLS], char* fileLocation){
    FILE *file = fopen(fileLocation, "r");
    if(file == NULL){
        perror("Error opening file");
        return -1;
    }

    int rows = 0;

    // Read the CSV file
    char line[256];  // Adjust the buffer size as needed
    while (fgets(line, sizeof(line), file) != NULL) {
        // Skip lines starting with #
        if (line[0] == '#') {
            continue;
        }

        // Parse the CSV values
        if (sscanf(line, "%lf,%lf,%lf,%lf", &values[rows][0], &values[rows][1], &values[rows][2], &values[rows][3]) == 4){
            rows++;
            if (rows >= MAX_ROWS){
                printf("Too many rows, increase MAX_ROWS.\n");
                break;
            }
        }
    }

    fclose(file);

    // Display the read values
    /*printf("Read values:\n");
    for (int i = 0; i < rows; i++){
        printf("%lf,%lf,%lf,%lf\n", values[i][0], values[i][1], values[i][2], values[i][3]);
    }*/
    return 1;
}

int main(){
    Parameters param = makeParameters(3000, 8, 1, 1, 0.01767, MAX_AREA, 0.5, 1.17);
    Rocket rocket = makeRocket(0, 0, 0, 23088.0);
    Miscellaneous misc = makeMiscellaneous(0, 0, 0, 0, 0, 1.0);

    double values[MAX_ROWS][MAX_COLS];

    if(getcsvFile(values, "C:\\Users\\caval\\Documents\\Universidade\\PIIC\\test.csv") == -1)
        return -1;

    for(int i = 0; i < MAX_ROWS; ++i){
        rocket->height = values[i][1];
        rocket->vertVelocity = values[i][2];
        rocket->velocity = values[i][3];

        double AB_drag = airbrakeForce(rocket, param, misc);

        double area = airBrakeArea(AB_drag, rocket->velocity, airDensity(rocket->height), param->AB_CD); 
        double angle = servoAngle(area);
        printf("Velocity: %lf Area: %lf Angle: %f\n", values[i][3], area, angle);
    }
    return 1;
}