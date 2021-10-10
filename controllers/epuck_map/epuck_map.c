#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/supervisor.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

#define ROBOT_RADIUS 0.035 // meters.
#define RANGE_MIN 0.005+ROBOT_RADIUS // 0.5 cm + ROBOT_RADIUS.
#define RANGE_MAX 0.05+ROBOT_RADIUS // 5 cm + ROBOT_RADIUS. 

typedef struct {

WbDeviceTag left_motor;
WbDeviceTag right_motor;  
WbFieldRef position;
WbFieldRef rotation;
WbDeviceTag ps[8];

}rb_speed;  

bool detect_obstacle_ahead(float d[8]) {
  return ( (d[0] < RANGE_MAX/2.0) || 
           (d[1] < RANGE_MAX/2.0) || 
           (d[6] < RANGE_MAX/2.0) || 
           (d[7] < RANGE_MAX/2.0));
}

float convert_intensity_to_meters(float prox) {
  float dist = 0.0;
  if (prox > 0.0)
    dist = 0.5/sqrt(prox)+ROBOT_RADIUS;
  else
    dist = RANGE_MAX;
  if (dist > RANGE_MAX)
    dist = RANGE_MAX;
  return dist;
}
 
void salvar_posicao_distancias(FILE *log, const double * p, const double * r, float d[8]){
    fprintf(log, "%f %f %f ", p[0], p[2], (r[1] < 0 ? -r[3] : r[3]) );
    for(int i = 0; i < 8; i++) {
      fprintf(log, "%f ", d[i]);
    }
    fprintf(log, "\n");
    fflush(log);
}


int main(int argc, char **argv) {
  FILE *log = fopen("log.csv", "w");
  if (!log)
    exit(1);
    
  rb_speed robot_speed () {
    
    rb_speed sp;
    
    
  wb_robot_init();
  sp.left_motor = wb_robot_get_device("left wheel motor");
  sp.right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(sp.left_motor, INFINITY);
  wb_motor_set_position(sp.right_motor, INFINITY);
  wb_motor_set_velocity(sp.left_motor, 0.1 * MAX_SPEED);
  wb_motor_set_velocity(sp.right_motor, 0.1 * MAX_SPEED);
  
  char ps_id[4];
  for(int i = 0; i < 8; i++){
    sprintf(ps_id, "ps%d", i);
    sp.ps[i] = wb_robot_get_device(ps_id);
    wb_distance_sensor_enable(sp.ps[i], TIME_STEP);
  }
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("EPUCK");
  if (robot_node == NULL) {
    fprintf(stderr, "No DEF EPUCK node found in the current world file\n");
    exit(1);
  }
  sp.position = wb_supervisor_node_get_field(robot_node, "translation");
  sp.rotation = wb_supervisor_node_get_field(robot_node, "rotation");
 
    return sp;
  }
  
  rb_speed inter = robot_speed ();
  
  while (wb_robot_step(TIME_STEP) != -1) {
    const double *position = wb_supervisor_field_get_sf_vec3f(inter.position);
    const double *rotation = wb_supervisor_field_get_sf_rotation(inter.rotation);
       
    float dist[8];
    for(int i = 0; i < 8; i++) {
      dist[i] = convert_intensity_to_meters(wb_distance_sensor_get_value(inter.ps[i]));
    }

    salvar_posicao_distancias(log, position, rotation, dist);

    if ( detect_obstacle_ahead(dist) )
    {
      wb_motor_set_velocity(inter.left_motor, 0.2 * MAX_SPEED);
      wb_motor_set_velocity(inter.right_motor, -0.2 * MAX_SPEED);
    }
    if ( !detect_obstacle_ahead(dist) )
    {
      wb_motor_set_velocity(inter.left_motor, 0.5 * MAX_SPEED);
      wb_motor_set_velocity(inter.right_motor, 0.5 * MAX_SPEED);
    }
  }
   
  fclose(log);

  wb_robot_cleanup();

  return 0;
}
