/****************************************************************/
/** xrobot.c: simulates and renders mobile manipulator         **/
/**           version of Roger-the-Crab                        **/
/** author:   Grupen                                           **/
/** date:     April, 2010                                      **/
/****************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "Xkw/Xkw.h"

#include "include/roger.h"
#include "include/simulate.h"
#include "include/control.h"
#include "include/modes.h"
#include "include/evaluations.h"
#define NXBINS            160
#define NYBINS             64

int ServerPorts[MaxRobotNum] = {8000, 8001, 8002};

Robot Rogers[MaxRobotNum];
RogerBody RogersBodys[MaxRobotNum];
PolyBall objects_total[MaxInertialObjectsNum];

History history[MaxRobotNum][MAX_HISTORY]; // a diagnostic tool to illustrate a trajectory
int history_ptr[MaxRobotNum];

int numRoger = 0;
int numObjects = 0; // number of inertial objects
int numToy = 1;
int socknew[MaxRobotNum];



// simulator data structures that comprise Roger
Base mobile_base_home;
Eye eyes_home[NEYES];
Arm arms_home[NARMS][NARM_FRAMES];


Base mobile_base_home_1;
Eye eyes_home_1[NEYES];
Arm arms_home_1[NARMS][NARM_FRAMES];
Base mobile_base_home_2;
Eye eyes_home_2[NEYES];
Arm arms_home_2[NARMS][NARM_FRAMES];
Base mobile_base_home_3;
Arm arms_home_3[NARMS][NARM_FRAMES];


PolyBall toy; extern PolyBall toy_home;
// CIRCLE, TRIANGLE, SQUARE, TRANGULAR receptical, SQUARE receptical
// constants in simulate.h
// N_TOY_TYPES = 5 total number of unique toys in the toybox
// MAX_TOYS = 10 maximum number of active toys allowed
eObject toybox[N_TOY_TYPES];
eObject active_toys[MAX_TOYS];
int input_toy_type = 0; // 0<=input_toy_type<NTOY_TYPES, initialized 0 (CIRCLE)
int n_active_toys = 0; // the current number of user instantiated toys 


SimColor world_colors[113];

int draw_visual = FALSE;

//extern init_control_flag;
int init_control_flag = TRUE;

// global Boolean reset flag - eliminates user defined obstacles and goals
// reconfigures Roger and single object to starting configuration
int reset;

// global environment variable (Arena or Developmental)
int kEnvironment;

// global simulator clock
double simtime = 0.0;
#define CIRCLE         0      /* object "shape" identifier */
#define TRIANGLE       1 

PolyBall toy_home = {
                   CIRCLE,    // object id (CIRCLE||TRIANGLE)
                        1,    // number of vertices
                   R_BALL,    // circle radius
                      0.0,    // spoke_length
                   M_BALL,    // mass
                   I_BALL,    // mass moment of inertia
{ 0.0, (2 * MAX_Y), 0.0 },    // position in world coordinates 
                              // (initially outside/north of drawable canvas
        { 0.0, 0.0, 0.0 },    // velocity in world coordinates
        { 0.0, 0.0, 0.0 } };  // default external forces

void x_canvas_proc(), x_start_proc(), x_params_proc(),
     x_input_mode_arena_proc(), x_input_mode_dev_proc(),
     x_control_mode_arena_proc(), x_input_mode_2_proc(),
     x_control_mode_2_proc();
void x_quit_proc(), x_timer_proc(), x_visualize_proc();
void x_sock_1_proc(), x_sock_2_proc();
void initialize_control_mode();
void SIMmatXvec(), SIMmatXmat(), SIMinv_transform(), SIMcopy_matrix();
void simulate_object(), simulate_arm(), simulate_base(), simulate_eyes();
void SocketCommunicate(), SocketInit();
void usleep();
void InitializeArenaEvironment();
void InitializeDevelopmentalEvironment();
void x_control_mode_proc(), x_room_proc();
void simulate_object_polyball(), simulate_roger(), HandleDrawingRequests();
void matrix_mult();

// --------------------------

Display          *display;
Window           window;
Pixmap           pixmap;
XtAppContext     app_con;
GC               gc;
int              screen;
Widget           canvas_w, input_mode_1_w, control_mode_1_w, params_w,
start_w, popup_w = NULL;
Widget       input_mode_2_w, control_mode_2_w, pause_w = NULL;
Widget       input_mode_w, control_mode_w, room_w;
Widget           intensity_w, stream_w;
XtIntervalId     timer = 0;
int              width = (int)(1.0* WIDTH), height = (int)(1.0*HEIGHT), depth;
unsigned long    foreground, background;

int width_pixmap = (int)((float)(WIDTH)* PIX_MAP_SIZE_RATIO);
int height_pixmap = (int)((float)(HEIGHT)* PIX_MAP_SIZE_RATIO);
float zoom = 1;

int evalType = -1;

PongEval pong_evaluator;

// Helper functions for converting world coordinates to pixmap
int ConvertWorld2PixmapX(float scale, float num, int environment) {
  if (environment == ARENA) {
    return W2DX(scale, num);
  }
  else {
    return W2DX_DEV(scale, num);
  }
}

int ConvertWorld2PixmapY(float scale, float num, int environment) {
  if (environment == ARENA) {
    return W2DY(scale, num);
  }
  else {
    return W2DY_DEV(scale, num);
  }
}

int ConvertWorld2PixmapR(float scale, float num, int environment) {
  if (environment == ARENA) {
    return W2DR(scale, num);
  }
  else {
    return W2DR_DEV(scale, num);
  }
}

// Helper functions for converting pixmap to world coordinates
double ConvertPixmap2WorldX(float scale, int num, int environment) {
  if (environment == ARENA) {
    return D2WX(scale, num);
  }
  else {
    return D2WX_DEV(scale, num);
  }
}

double ConvertPixmap2WorldY(float scale, int num, int environment) {
  if (environment == ARENA) {
    return D2WY(scale, num);
  }
  else {
    return D2WY_DEV(scale, num);
  }
}

double ConvertPixmap2WorldR(float scale, int num, int environment) {
  if (environment == ARENA) {
    return D2WR(scale, num);
  }
  else {
    return D2WR_DEV(scale, num);
  }
}

void x_init_colors()
{
  int i;

  //  printf("initializing grey scale colors..."); fflush(stdout);
  for (i = 0; i <= 100; i++) { // 0 => black; 100 => white
    sprintf(world_colors[i].name, "grey%d", i);
    world_colors[i].display_color =
      XkwParseColor(display, world_colors[i].name);
    world_colors[i].red = (int)i*2.55;
    world_colors[i].green = (int)i*2.55;
    world_colors[i].blue = (int)i*2.55;
  }
  strcpy(world_colors[101].name, "dark red");
  world_colors[101].display_color = XkwParseColor(display, "dark red");
  world_colors[101].red = 139;
  world_colors[101].green = 0;
  world_colors[101].blue = 0;

  strcpy(world_colors[102].name, "red");
  world_colors[102].display_color = XkwParseColor(display, "red");
  world_colors[102].red = 255;
  world_colors[102].green = 0;
  world_colors[102].blue = 0;

  strcpy(world_colors[103].name, "hot pink");
  world_colors[103].display_color = XkwParseColor(display, "hot pink");
  world_colors[103].red = 255;
  world_colors[103].green = 105;
  world_colors[103].blue = 180;

  strcpy(world_colors[104].name, "navy");
  world_colors[104].display_color = XkwParseColor(display, "navy");
  world_colors[104].red = 65;
  world_colors[104].green = 105;
  world_colors[104].blue = 225;

  strcpy(world_colors[105].name, "blue");
  world_colors[105].display_color = XkwParseColor(display, "blue");
  world_colors[105].red = 0;
  world_colors[105].green = 0;
  world_colors[105].blue = 255;

  strcpy(world_colors[106].name, "light sky blue");
  world_colors[106].display_color = XkwParseColor(display, "light sky blue");
  world_colors[106].red = 250;
  world_colors[106].green = 128;
  world_colors[106].blue = 114;

  strcpy(world_colors[107].name, "dark green");
  world_colors[107].display_color = XkwParseColor(display, "dark green");
  world_colors[107].red = 244;
  world_colors[107].green = 164;
  world_colors[107].blue = 96;

  strcpy(world_colors[108].name, "green");
  world_colors[108].display_color = XkwParseColor(display, "green");
  world_colors[108].red = 0;
  world_colors[108].green = 255;
  world_colors[108].blue = 0;

  strcpy(world_colors[109].name, "light green");
  world_colors[109].display_color = XkwParseColor(display, "light green");
  world_colors[109].red = 46;
  world_colors[109].green = 139;
  world_colors[109].blue = 87;

  strcpy(world_colors[110].name, "gold");
  world_colors[110].display_color = XkwParseColor(display, "gold");
  world_colors[110].red = 160;
  world_colors[110].green = 82;
  world_colors[110].blue = 45;

  strcpy(world_colors[111].name, "yellow");
  world_colors[111].display_color = XkwParseColor(display, "yellow");
  world_colors[111].red = 255;
  world_colors[111].green = 255;
  world_colors[111].blue = 0;

  strcpy(world_colors[112].name, "light goldenrod");
  world_colors[112].display_color = XkwParseColor(display, "light goldenrod");
  world_colors[112].red = 192;
  world_colors[112].green = 192;
  world_colors[112].blue = 192;
}

/************************ BEGIN EVAL SPECIFIC FUNCTIONS ***************************/

void initialize_pong_object() {

  toy.position[X] = 0;
  toy.position[Y] = 0;

  srand((unsigned int)(time(NULL)));
  double N1 = ((double)(rand() % 1000 + 1)) / 1000.0;
  double N2 = ((double)(rand() % 1000 + 1)) / 1000.0;

  double vx = (N1 - 0.5) * 20;
  double vy = (N2 - 0.5) * 20;
  toy.velocity[X] = vx;
  toy.velocity[Y] = vy;
  toy.net_extForce[X] = toy.net_extForce[Y] = toy.net_extForce[THETA] = 0.0;
}

void reset_pong() {

  void initialize_simulator(), place_object(), make_images(), draw_all();

  printf("reset called\n");
  reset = TRUE;
  initialize_simulator(reset); // initializes world boundaries,

  int i;
  for (int i = 0; i < numRoger; ++i) {
    initialize_control_mode(&Rogers[i]);
  }

  place_object(WIDTH / 2, HEIGHT / 2);
  initialize_pong_object();

  for (i = 0; i < numRoger; ++i) {
    simulate_roger(&RogersBodys[i].mobile_base, RogersBodys[i].arms,
       RogersBodys[i].eyes);
  }

  simulate_object_polyball(&toy);

  make_images(1);
  make_images(2);
  draw_all();
}

void evalManager() {

  int pong_fault_detection(),pong_game_over_detection(),pong_goal_detection();
  void reset_pong_timers(), reset_pong_timers_and_score();

  if (evalType == PONG) {
    double middle = MIN_X + (MAX_X - MIN_X) / 2;
    int fault1 = pong_fault_detection(MIN_X, MAX_X, MIN_Y, MAX_Y, middle,
                                      objects_total[BASE].position[X],
                                      objects_total[BASE].position[Y], R_BASE,
                                      objects_total[ARM1].position[X],
                                      objects_total[ARM1].position[Y],
                                      objects_total[ARM2].position[X],
                                      objects_total[ARM2].position[Y], R_TACTILE,
                                      1, &pong_evaluator);
    int fault2 = pong_fault_detection(MIN_X, MAX_X, MIN_Y, MAX_Y, middle,
                                      objects_total[N_BODY_ROBOT+BASE].position[X],
                                      objects_total[N_BODY_ROBOT+BASE].position[Y], R_BASE,
                                      objects_total[N_BODY_ROBOT+ARM1].position[X],
                                      objects_total[N_BODY_ROBOT+ARM1].position[Y],
                                      objects_total[N_BODY_ROBOT+ARM2].position[X],
                                      objects_total[N_BODY_ROBOT+ARM2].position[Y], R_TACTILE,
                                      2, &pong_evaluator);
    if (!fault1 && !fault2) {
      double ball_x = objects_total[numRoger * N_BODY_ROBOT].position[X];
      if (pong_goal_detection(MIN_X, MAX_X, ball_x, middle, R_GOAL, DT, &pong_evaluator)) {
        reset_pong_timers(&pong_evaluator);
        usleep(1000000);
        reset_pong();
      }
    }
    else {
      reset_pong_timers(&pong_evaluator);
      usleep(1000000);
      reset_pong();
    }
    if (pong_game_over_detection(&pong_evaluator)) {
      reset_pong_timers_and_score(&pong_evaluator);
      usleep(1000000);
      reset_pong();
      XkwSetWidgetLabel(start_w, "New Game");
      XtRemoveTimeOut(timer);
      timer = 0;
    }
  }
  //extend with other eval types here
}

/************************ END EVAL SPECIFIC FUNCTIONS ***************************/


double motor_model(tau, omega, tau_s, omega_0)
double tau, omega, tau_s, omega_0;
{
  int i;
  double tau_max, tau_min;

  if (omega >= 0) { // motor velocity positive
    tau_min = -tau_s;
    tau_max = tau_s - (tau_s / omega_0)*omega;
  }
  else { // motor velocity negative
    tau_min = -tau_s - (tau_s / omega_0)*omega;
    tau_max = tau_s;
  }
  if (tau < tau_min) tau = tau_min;
  if (tau > tau_max) tau = tau_max;

  return(tau);
}

void place_object(x,y,id)
double x,y;
int id;
{
  if (id==CIRCLE) {
    toy.id = id;
    toy.N = 1;
    toy.Rsphere = R_BALL;;
    toy.radius = 0.0;
    toy.mass = M_BALL;
    toy.moi = I_BALL;
    toy.position[X] = x;
    toy.position[Y] = y;
    toy.position[THETA] = 0.0;
    toy.velocity[X] = toy.velocity[Y] = toy.velocity[THETA] = 0.0;
    toy.net_extForce[X] = toy.net_extForce[Y] = toy.net_extForce[THETA] = 0.0;
  }
  else if (id==TRIANGLE) {
    toy.id = id;
    toy.N = 3;
    toy.Rsphere = 0.1; //R_VERTEX;;
    toy.radius = R_SPOKE;
    toy.mass = M_TRIANGLE;
    toy.moi = I_TRIANGLE;
    toy.position[X] = x;
    toy.position[Y] = y;
    toy.position[THETA] = 0.0;
    toy.velocity[X] = toy.velocity[Y] = toy.velocity[THETA] = 0.0;
    toy.net_extForce[X] = toy.net_extForce[Y] = toy.net_extForce[THETA] = 0.0;
  }
}   

void x_params_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  Rogers[0].enter_param_event = TRUE;
}


int change_input_mode_dev()
{
  static int input_mode;

  input_mode = (input_mode + 1) % N_INPUT_MODES_DEV;
  //init_input_flag = TRUE;
  return (input_mode);
}

int change_input_mode_arena()
{
  static int input_mode;

  input_mode = (input_mode + 1) % N_INPUT_MODES_ARENA;
  //init_input_flag = TRUE;
  return (input_mode);
}


void x_input_mode_dev_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  Rogers[0].input_mode = change_input_mode_dev();

  switch (Rogers[0].input_mode) {
  case JOINT_ANGLE_INPUT:
    XkwSetWidgetLabel(input_mode_w, "Input: Joint angles"); break;
  case BASE_GOAL_INPUT:
    XkwSetWidgetLabel(input_mode_w, "Input: Base goal"); break;
  case ARM_GOAL_INPUT:
    XkwSetWidgetLabel(input_mode_w, "Input: Arm goals"); break;
  case BALL_INPUT:
    XkwSetWidgetLabel(input_mode_w, "Input: Ball position"); break;
  case MAP_INPUT:
    XkwSetWidgetLabel(input_mode_w, "Input: Map Editor"); break;
  default: break;
  }
}

void x_input_mode_arena_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  int i;

  int current_input_mode = change_input_mode_arena();
  for (i = 0; i < numRoger; ++i) {
    Rogers[i].input_mode = current_input_mode;
  }

  switch (Rogers[0].input_mode) {
  case BALL_INPUT_ARENA:
    XkwSetWidgetLabel(input_mode_1_w, "Input: Ball position"); break;
  case MAP_INPUT_ARENA:
    XkwSetWidgetLabel(input_mode_1_w, "Input: Map Editor"); break;
  default: break;
  }
}

int change_control_mode_dev()
{
  static int control_mode;

  control_mode = (control_mode + 1) % N_CONTROL_MODES;
  init_control_flag = TRUE;
  return (control_mode);
}

int change_control_mode_arena()
{
  static int control_mode;

  control_mode = (control_mode + 1) % N_CONTROL_MODES;
  init_control_flag = TRUE;
  return (control_mode);
}


void x_control_mode_arena_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  int i, current_control_mode; 

  current_control_mode = change_control_mode_arena();

  for (i = 0; i < numRoger; ++i) {
    Rogers[i].control_mode = current_control_mode;
  }


  switch (Rogers[0].control_mode) {
     case PROJECT1:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 1-MotorUnits"); break;
     case PROJECT2:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 2-ArmKinematics"); break;
     case PROJECT3:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 3-Vision"); break;
     case PROJECT4:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 4-SearchTrack"); break;
     case PROJECT5:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 5-StereoKinematics"); break;
     case PROJECT6:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 6-Kalman"); break;
     case PROJECT7:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 7-ChasePunch"); break;
     case PROJECT8:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 8-PathPlanning"); break;
     case PROJECT9:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 9-PONG"); break;
     case PROJECT10:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 10-Model"); break;
     case PROJECT11:
       XkwSetWidgetLabel(control_mode_1_w, "Control: 11-Belief"); break;
     default: break;
  }
  //call init here makes it independent of timer running
  for (i = 0; i < numRoger; ++i) {
    initialize_control_mode(&Rogers[i]);
  }

}



void x_quit_proc(w, client_data, call_data)
Widget w;
XtPointer call_data, client_data;
{
  XFreePixmap(display, pixmap);
  XFreeGC(display, gc);
  XtDestroyApplicationContext(app_con);
  exit(0);
}

void x_canvas_proc(w, client_data, call_data)
Widget w;
XtPointer call_data, client_data;
{
  XEvent *event = ((XEvent *)call_data);
  char text[10];
  KeySym key;
  double x, y, theta1, theta2;
  int i, j, xbin, ybin, k;
  int c;
  static Dimension nwidth = WIDTH, nheight = HEIGHT;

  // Original width and height of current environemtn window
  double env_width, env_height;
  if (kEnvironment == ARENA) {
    // nwidth = WIDTH;
    // nheight = HEIGHT;
    env_width = (double)WIDTH;
    env_height = (double)HEIGHT;
  } else {
    // nwidth = WIDTH_DEV;
    // nheight = HEIGHT_DEV;
    env_width = (double)WIDTH_DEV;
    env_height = (double)HEIGHT_DEV;
  }
  void x_expose(), x_clear();

  switch (event->type) {
  case ConfigureNotify:
    nwidth = event->xconfigure.width;
    nheight = event->xconfigure.height;
    // Calculate the zoom scale for scaling the windows and graphics
    float zoom_width = (double)nwidth / env_width;
    float zoom_height = (double)nheight / env_height;
    zoom = zoom_width > zoom_height ? zoom_height : zoom_width;

    // Limit the zoom scale between 1.0 and PIX_MAP_SIZE_RATIO
    zoom = zoom > 1.0 ? zoom : 1.0;
    zoom = zoom > (float)(PIX_MAP_SIZE_RATIO) ? (float)(PIX_MAP_SIZE_RATIO) : zoom;

    for (k = 0; k < numRoger; ++k) {
      Rogers[k].graphics.zoom = zoom;
    }

    break;
  case Expose:
    if (nwidth == width && nheight == height)
      x_expose();
    else {
      width = nwidth; height = nheight;
      x_expose();
    }
    break;

  case ButtonPress:
    for (k = 0; k < numRoger; ++k) {
      Rogers[k].button_reference[X] = ConvertPixmap2WorldX(zoom, event->xbutton.x, kEnvironment);
      Rogers[k].button_reference[Y] = ConvertPixmap2WorldY(zoom, event->xbutton.y, kEnvironment);
      Rogers[k].button_event = event->xbutton.button;
    }

    break;

  case ButtonRelease:
    break;

  case KeyPress:
    c = XLookupString((XKeyEvent *)event, text, 10, &key, 0);
    if (c == 1)
      switch (text[0]) {
      case 'h':
        break;
      case 'c':
        x_clear();
        x_expose();
        break;
      case 'q':
        x_quit_proc(w, client_data, call_data);
      }
  }
}

void x_draw_line(color, start_x, start_y, end_x, end_y)
int color;
double start_x, start_y, end_x, end_y;
{
  XSetForeground(display, gc, world_colors[color].display_color);
  XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, start_x, kEnvironment), ConvertWorld2PixmapY(zoom, start_y, kEnvironment),
    ConvertWorld2PixmapX(zoom, end_x, kEnvironment), ConvertWorld2PixmapY(zoom, end_y, kEnvironment));
}

/*
void x_draw_circle(color, center_x, center_y, radius, fill)
int color, fill;
double center_x, center_y, radius;
{
XSetForeground(display, gc, world_colors[color].display_color);
draw_circle(ConvertWorld2PixmapX(zoom,center_x),ConvertWorld2PixmapY(zoom,center_y),ConvertWorld2PixmapR(zoom,radius),fill);
}
*/

void x_expose()
{
  XCopyArea(display, pixmap, window, gc, 0, 0, width, height, 0, 0);
}

void x_clear()
{
  XSetForeground(display, gc, background);
  XFillRectangle(display, pixmap, gc, 0, 0, width_pixmap, height_pixmap);
}

#define STEP         0.01
#define STREAM_SPACE 0


void x_visualize_proc(w, client_data, call_data)
{
  if (draw_visual == TRUE) draw_visual=FALSE;
  else draw_visual = TRUE;
}



void mark_used(ii, jj, aux)
int ii, jj;
int aux[NBINS][NBINS];
{
  int j, k;
  double dist;

  for (j = -STREAM_SPACE; j <= STREAM_SPACE; ++j) {
    for (k = -STREAM_SPACE; k <= STREAM_SPACE; ++k) {
      dist = sqrt(SQR((double)j) + SQR((double)k));
      if ((dist < (2.0*STREAM_SPACE + 1.0)) &&
        ((ii + j) >= 0) && ((ii + j) < NBINS) &&
        ((jj + k) >= 0) && ((jj + k) < NBINS))
        aux[ii + j][jj + k] = TRUE;
    }
  }
}


void write_interface(reset)
int reset;
{
  int i, j, k;

  for (k = 0; k < numRoger; ++k) {
    // pass in afferents (read only)
    Rogers[k].eye_theta[0] = RogersBodys[k].eyes[0].theta;
    Rogers[k].eye_theta_dot[0] = RogersBodys[k].eyes[0].theta_dot;
    Rogers[k].eye_theta[1] = RogersBodys[k].eyes[1].theta;
    Rogers[k].eye_theta_dot[1] = RogersBodys[k].eyes[1].theta_dot;
    for (i = 0; i<NPIXELS; ++i) {
      Rogers[k].image[LEFT][i][RED_CHANNEL] =
        world_colors[RogersBodys[k].eyes[LEFT].image[i]].red;
      Rogers[k].image[LEFT][i][GREEN_CHANNEL] =
        world_colors[RogersBodys[k].eyes[LEFT].image[i]].green;
      Rogers[k].image[LEFT][i][BLUE_CHANNEL] =
        world_colors[RogersBodys[k].eyes[LEFT].image[i]].blue;
      Rogers[k].image[RIGHT][i][RED_CHANNEL] =
        world_colors[RogersBodys[k].eyes[RIGHT].image[i]].red;
      Rogers[k].image[RIGHT][i][GREEN_CHANNEL] =
        world_colors[RogersBodys[k].eyes[RIGHT].image[i]].green;
      Rogers[k].image[RIGHT][i][BLUE_CHANNEL] =
        world_colors[RogersBodys[k].eyes[RIGHT].image[i]].blue;
    }
    Rogers[k].arm_theta[0][0] = RogersBodys[k].arms[0][1].theta;
    Rogers[k].arm_theta[0][1] = RogersBodys[k].arms[0][2].theta;
    Rogers[k].arm_theta[1][0] = RogersBodys[k].arms[1][1].theta;
    Rogers[k].arm_theta[1][1] = RogersBodys[k].arms[1][2].theta;
    Rogers[k].arm_theta_dot[0][0] = RogersBodys[k].arms[0][1].theta_dot;
    Rogers[k].arm_theta_dot[0][1] = RogersBodys[k].arms[0][2].theta_dot;
    Rogers[k].arm_theta_dot[1][0] = RogersBodys[k].arms[1][1].theta_dot;
    Rogers[k].arm_theta_dot[1][1] = RogersBodys[k].arms[1][2].theta_dot;
    Rogers[k].ext_force[0][X] = RogersBodys[k].arms[0][NARM_FRAMES - 1].extForce[X];
    Rogers[k].ext_force[0][Y] = RogersBodys[k].arms[0][NARM_FRAMES - 1].extForce[Y];
    Rogers[k].ext_force[1][X] = RogersBodys[k].arms[1][NARM_FRAMES - 1].extForce[X];
    Rogers[k].ext_force[1][Y] = RogersBodys[k].arms[1][NARM_FRAMES - 1].extForce[Y];

    Rogers[k].base_position[0] = RogersBodys[k].mobile_base.x;
    Rogers[k].base_position[1] = RogersBodys[k].mobile_base.y;
    Rogers[k].base_position[2] = RogersBodys[k].mobile_base.theta;
    Rogers[k].base_velocity[0] = RogersBodys[k].mobile_base.x_dot;
    Rogers[k].base_velocity[1] = RogersBodys[k].mobile_base.y_dot;
    Rogers[k].base_velocity[2] = RogersBodys[k].mobile_base.theta_dot;
    Rogers[k].ext_force_body[X] = -RogersBodys[k].mobile_base.extForce[X];
    Rogers[k].ext_force_body[Y] = -RogersBodys[k].mobile_base.extForce[Y];

    // zero efferents (write only)
    Rogers[k].eye_torque[0] = Rogers[k].eye_torque[1] = 0.0;
    Rogers[k].arm_torque[0][0] = Rogers[k].arm_torque[0][1] = Rogers[k].arm_torque[1][0] =
      Rogers[k].arm_torque[1][1] = 0.0;
    Rogers[k].wheel_torque[0] = Rogers[k].wheel_torque[1] = 0.0;
    Rogers[k].simtime = simtime;
  }
}

void read_interface()
{
  int k;

  for (k = 0; k < numRoger; ++k) {
    // pass back torques (write only)
    RogersBodys[k].arms[0][1].torque =
      motor_model(Rogers[k].arm_torque[0][0], Rogers[k].arm_theta_dot[0][0],
        SHOULDER_TS, SHOULDER_W0);
    RogersBodys[k].arms[1][1].torque =
      motor_model(Rogers[k].arm_torque[1][0], Rogers[k].arm_theta_dot[1][0],
        SHOULDER_TS, SHOULDER_W0);
    RogersBodys[k].arms[0][2].torque =
      motor_model(Rogers[k].arm_torque[0][1], Rogers[k].arm_theta_dot[0][1],
        ELBOW_TS, ELBOW_W0);
    RogersBodys[k].arms[1][2].torque =
      motor_model(Rogers[k].arm_torque[1][1], Rogers[k].arm_theta_dot[1][1],
        ELBOW_TS, ELBOW_W0);

    RogersBodys[k].eyes[0].torque =
      motor_model(Rogers[k].eye_torque[0], Rogers[k].eye_theta_dot[0], EYE_TS, EYE_W0);
    RogersBodys[k].eyes[1].torque =
      motor_model(Rogers[k].eye_torque[1], Rogers[k].eye_theta_dot[1], EYE_TS, EYE_W0);

    RogersBodys[k].mobile_base.wheel_torque[0] =
      motor_model(Rogers[k].wheel_torque[0], Rogers[k].wheel_theta_dot[0],
        WHEEL_TS, WHEEL_W0);
    RogersBodys[k].mobile_base.wheel_torque[1] =
      motor_model(Rogers[k].wheel_torque[1], Rogers[k].wheel_theta_dot[1],
        WHEEL_TS, WHEEL_W0);
  }
}

/* forward kinematics in base frame **************************************/
void sim_fwd_kinematics(arm_id, theta1, theta2, x, y)
int arm_id;
double theta1, theta2;
double *x, *y;
{
  *x = L_ARM1 * cos(theta1) + L_ARM2 * cos(theta1 + theta2);
  *y = L_ARM1 * sin(theta1) + L_ARM2 * sin(theta1 + theta2);

  if (arm_id == LEFT) *y += ARM_OFFSET;
  else *y -= ARM_OFFSET;
}

void sim_arm_Jacobian(theta1, theta2, Jacobian)
double theta1, theta2;
double Jacobian[2][2];
{
  Jacobian[0][0] = -L_ARM1*sin(theta1) - L_ARM2*sin(theta1 + theta2);
  Jacobian[0][1] = -L_ARM2*sin(theta1 + theta2);
  Jacobian[1][0] = L_ARM1*cos(theta1) + L_ARM2*cos(theta1 + theta2);
  Jacobian[1][1] = L_ARM2*cos(theta1 + theta2);
}

//#define NBODY 4 // single ball, two hands, roger's body




void copy_object(i, obj, objects)
int i;
PolyBall * obj;
PolyBall objects[NBODY];
{
  obj->id = objects[i].id;
  obj->N = objects[i].N;
  obj->Rsphere = objects[i].Rsphere;
  obj->radius = objects[i].radius;
  obj->mass = objects[i].mass;
  obj->moi = objects[i].moi;
  obj->position[X] = objects[i].position[X];
  obj->position[Y] = objects[i].position[Y];
  obj->position[THETA] = objects[i].position[THETA];

  obj->velocity[X] = objects[i].velocity[X];
  obj->velocity[Y] = objects[i].velocity[Y];
  obj->velocity[THETA] = objects[i].velocity[THETA];

  obj->net_extForce[X] = objects[i].net_extForce[X];
  obj->net_extForce[Y] = objects[i].net_extForce[Y];
  obj->net_extForce[THETA] = objects[i].net_extForce[THETA];
}



void update_objects()
{
  double pb[4], pw[4]; // homogeneous position vectors in base and world coords
  double vb[4], vw[4]; // homogeneous velocity vectors in base and world coords
  double x, y, J[2][2];
  int i;

  void sim_fwd_kinematics(), sim_arm_Jacobian();

  int base_counter = 0;
  for (i = 0; i < numRoger; ++i) {
    base_counter = i * N_BODY_ROBOT;

    /****************************** 0: BASE ************************************/
    objects_total[base_counter + BASE].N = 1;
    objects_total[base_counter + base_counter + BASE].Rsphere = R_BASE;
    objects_total[base_counter + BASE].radius = 0.0;
    objects_total[base_counter + BASE].mass = M_BASE;
    objects_total[base_counter + BASE].moi = I_BASE;

    objects_total[base_counter + BASE].position[X] = RogersBodys[i].mobile_base.x;
    objects_total[base_counter + BASE].position[Y] = RogersBodys[i].mobile_base.y;
    objects_total[base_counter + BASE].position[THETA] = RogersBodys[i].mobile_base.theta;

    objects_total[base_counter + BASE].velocity[X] = RogersBodys[i].mobile_base.x_dot;
    objects_total[base_counter + BASE].velocity[Y] = RogersBodys[i].mobile_base.y_dot;
    objects_total[base_counter + BASE].velocity[THETA] = RogersBodys[i].mobile_base.theta_dot;

    objects_total[base_counter + BASE].net_extForce[X] = objects_total[base_counter + BASE].net_extForce[Y] =
      objects_total[base_counter + BASE].net_extForce[THETA] = 0.0;

    /************************** 1: LEFT ARM ************************************/
    // left hand position in world coordinates
    sim_fwd_kinematics(LEFT, RogersBodys[i].arms[LEFT][1].theta, RogersBodys[i].arms[LEFT][2].theta, &x, &y);
    pb[0] = x; pb[1] = y; pb[2] = 0.0; pb[3] = 1.0;
    matrix_mult(4, 4, RogersBodys[i].mobile_base.wTb, 1, pb, pw);

    // left hand velocity relative to base written in base coordinates
    //          (figure 2 in RogerDynamics document defines frames)
    // ^w(v)_10 = wRb[ ^b(v)_7 +  J_arm theta_dot_arm ]
    //    = ^w(v)_b + wRb[ -ARM_OFFSET omega_0 xhat_b + J_arm theta_dot_arm ]
    sim_arm_Jacobian(RogersBodys[i].arms[LEFT][1].theta, RogersBodys[i].arms[LEFT][2].theta, J);
    vb[X] = J[0][0] * RogersBodys[i].arms[LEFT][1].theta_dot +
      J[0][1] * RogersBodys[i].arms[LEFT][2].theta_dot - ARM_OFFSET*RogersBodys[i].mobile_base.theta_dot;
    vb[Y] = J[1][0] * RogersBodys[i].arms[LEFT][1].theta_dot + J[1][1] * RogersBodys[i].arms[LEFT][2].theta_dot;
    vb[2] = 0.0;
    vb[3] = 0.0; // homogeneous vector

    matrix_mult(4, 4, RogersBodys[i].mobile_base.wTb, 1, vb, vw);
    //  v[1][X] = base->x_dot + v_w[X];
    //  v[1][Y] = base->y_dot + v_w[Y];
    //  R[1] = R_TACTILE;

    objects_total[base_counter + ARM1].N = 1;
    objects_total[base_counter + ARM1].Rsphere = R_TACTILE;
    objects_total[base_counter + ARM1].radius = 0.0;
    objects_total[base_counter + ARM1].mass = M_ARM1;
    objects_total[base_counter + ARM1].moi = I_ARM1;

    objects_total[base_counter + ARM1].position[X] = pw[X];
    objects_total[base_counter + ARM1].position[Y] = pw[Y];
    objects_total[base_counter + ARM1].position[THETA] = 0.0;
    // hand orientation is not relevant to system dynamics

    objects_total[base_counter + ARM1].velocity[X] = RogersBodys[i].mobile_base.x_dot + vw[X];
    objects_total[base_counter + ARM1].velocity[Y] = RogersBodys[i].mobile_base.y_dot + vw[Y];
    objects_total[base_counter + ARM1].velocity[THETA] = 0.0;
    // angular acceleration in the hand is not relevant to system dynamics

    objects_total[base_counter + ARM1].net_extForce[X] = objects_total[base_counter + ARM1].net_extForce[Y] =
      objects_total[base_counter + ARM1].net_extForce[THETA] = 0.0;

    /************************** 2: RIGHT ARM ************************************/
    // right hand position in world coordinates
    sim_fwd_kinematics(RIGHT, RogersBodys[i].arms[RIGHT][1].theta, RogersBodys[i].arms[RIGHT][2].theta, &x, &y);
    pb[0] = x; pb[1] = y; pb[2] = 0.0; pb[3] = 1.0;
    matrix_mult(4, 4, RogersBodys[i].mobile_base.wTb, 1, pb, pw);

    // left hand velocity relative to base written in base coordinates
    //          (figure 2 in RogerDynamics document defines frames)
    // ^w(v)_10 = wRb[ ^b(v)_7 +  J_arm theta_dot_arm ]
    //    = ^w(v)_b + wRb[ -ARM_OFFSET omega_0 xhat_b + J_arm theta_dot_arm ]
    sim_arm_Jacobian(RogersBodys[i].arms[RIGHT][1].theta, RogersBodys[i].arms[RIGHT][2].theta, J);
    vb[X] = J[0][0] * RogersBodys[i].arms[RIGHT][1].theta_dot +
      J[0][1] * RogersBodys[i].arms[RIGHT][2].theta_dot - ARM_OFFSET*RogersBodys[i].mobile_base.theta_dot;
    vb[Y] = J[1][0] * RogersBodys[i].arms[RIGHT][1].theta_dot + J[1][1] * RogersBodys[i].arms[RIGHT][2].theta_dot;
    vb[2] = 0.0;
    vb[3] = 0.0; // homogeneous vector

    matrix_mult(4, 4, RogersBodys[i].mobile_base.wTb, 1, vb, vw);

    objects_total[base_counter + ARM2].N = 1;
    objects_total[base_counter + ARM2].Rsphere = R_TACTILE;
    objects_total[base_counter + ARM2].radius = 0.0;
    objects_total[base_counter + ARM2].mass = M_ARM1;
    objects_total[base_counter + ARM2].moi = I_ARM1;

    objects_total[base_counter + ARM2].position[X] = pw[X];
    objects_total[base_counter + ARM2].position[Y] = pw[Y];
    objects_total[base_counter + ARM2].position[THETA] = 0.0;
    // hand orientation not relevant to system dynamics

    objects_total[base_counter + ARM2].velocity[X] = RogersBodys[i].mobile_base.x_dot + vw[X];
    objects_total[base_counter + ARM2].velocity[Y] = RogersBodys[i].mobile_base.y_dot + vw[Y];
    objects_total[base_counter + ARM2].velocity[THETA] = 0.0;
    // angular acceleration of hand is not relevant to system dynamics

    objects_total[base_counter +ARM2].net_extForce[X] = objects_total[base_counter + ARM2].net_extForce[Y] =
    objects_total[base_counter + ARM2].net_extForce[THETA] = 0.0;

  }
  base_counter = base_counter + N_BODY_ROBOT;

  /****************** 3: TOY OBJECT - CIRCLE || TRIANGLE *********************/
  objects_total[base_counter].id = toy.id;
  objects_total[base_counter].N = toy.N;
  objects_total[base_counter].Rsphere = toy.Rsphere;
  objects_total[base_counter].radius = toy.radius;
  objects_total[base_counter].mass = toy.mass;
  objects_total[base_counter].moi = toy.moi;

  objects_total[base_counter].position[X] = toy.position[X];
  objects_total[base_counter].position[Y] = toy.position[Y];
  objects_total[base_counter].position[THETA] = toy.position[THETA];

  objects_total[base_counter].velocity[X] = toy.velocity[X];
  objects_total[base_counter].velocity[Y] = toy.velocity[Y];
  objects_total[base_counter].velocity[THETA] = toy.velocity[THETA];

  objects_total[base_counter].net_extForce[X] = objects_total[base_counter].net_extForce[Y] =
  objects_total[base_counter].net_extForce[THETA] = 0.0;
}


void compute_external_forces()
{
  int i, j, ii, jj, row, col, thetai, thetaj;
  double ri[2], rj[2];
  double dr[2], mag, theta, dij, rx, ry, fx, fy, tz;
  PolyBall obji, objj;

  void copy_object(), update_objects(), SIMfwd_kinematics(), SIMarm_Jacobian();

  // Define delta_x, delta_y, and r_obstacle size given the environment
  float x_delta, y_delta, r_obstacle;
  float max_x, max_y, min_x, min_y;
  if (kEnvironment == DEVELOPMENTAL) {
    x_delta = XDELTA_DEV;
    y_delta = YDELTA_DEV;
    r_obstacle = R_OBSTACLE_DEV;
    max_x = MAX_X_DEV;
    min_x = MIN_X_DEV;
    max_y = MAX_Y_DEV;
    min_y = MIN_Y_DEV;
  }
  else {
    x_delta = XDELTA;
    y_delta = YDELTA;
    r_obstacle = R_OBSTACLE;
    max_x = MAX_X;
    min_x = MIN_X;
    max_y = MAX_Y;
    min_y = MIN_Y;
  }


  // copy the current pos/vel info from device (mobile_base, arms, and toy
  // structures) into inertial "PolyBall objects[NBODY]" array
  update_objects();

 

  for (i = 0; i<(numObjects); ++i) { // compute force on body i by body j
    for (j = (i + 1); j < (numObjects + 1) ; ++j) {

      copy_object(i, &obji, objects_total); copy_object(j, &objj, objects_total);

      // Last body is the occupancy grid - sum compression      
      if (j == numObjects) { // needs to be upgraded too
              //  printf("checking body i=%d bouncing on OBSTACLE j=%d\n", i,j);
        for (ii = 0; ii<obji.N; ++ii) {
          thetai = (double)ii * (2 * M_PI / obji.N);

          // position of the first vertex
          //    theta = (double)k*(2.0*M_PI/3.0);
          //    xt = RT*cos((double)k*one_twenty); 
          //    yt = RT*sin((double)k*one_twenty); 
          ri[X] = objects_total[i].position[X]
            + (objects_total[i].radius * cos(thetai))
            * cos(objects_total[i].position[THETA])
            - (objects_total[i].radius * sin(thetai))
            * sin(objects_total[i].position[THETA]);
          ri[Y] = objects_total[i].position[Y]
            + (objects_total[i].radius * cos(thetai))
            * sin(objects_total[i].position[THETA])
            + (objects_total[i].radius * sin(thetai))
            * cos(objects_total[i].position[THETA]);

          for (row = 0; row<NBINS; ++row) {
            for (col = 0; col<NBINS; ++col) {
              if (Rogers[0].world_map.occupancy_map[row][col] == OBSTACLE) {
                dr[X] = ri[X] - (min_x + (col + 0.5)*x_delta);
                dr[Y] = ri[Y] - (max_y - (row + 0.5)*y_delta);
                mag = sqrt(SQR(dr[X]) + SQR(dr[Y]));
                dij = MAX(0.0, (obji.Rsphere + r_obstacle - mag));
                fx = K_COLLIDE*dij*(dr[X] / mag);
                fy = K_COLLIDE*dij*(dr[Y] / mag);

                rx = obji.radius * cos(obji.position[THETA] + thetai);
                ry = obji.radius * sin(obji.position[THETA] + thetai);
                tz = rx * fy - fx * ry;

                objects_total[i].net_extForce[X] += fx;
                objects_total[i].net_extForce[Y] += fy;
                objects_total[i].net_extForce[THETA] += tz;


              }
            }
          }
        }

        //  printf("\tforce on body i=%d  f = [%6.4lf %6.4lf %6.4lf]\n", i,
        //           objects[i].net_extForce[X], objects[i].net_extForce[Y], 
        //         objects[i].net_extForce[THETA]);
      }
      else { // j not the occupacy grid: BASE || ARM#1 || ARM#2 || circle object
        for (ii = 0; ii<obji.N; ++ii) {
          thetai = (double)ii * (2 * M_PI / obji.N);

          // position of the first vertex
          //    theta = (double)k*(2.0*M_PI/3.0);
          //    xt = RT*cos((double)k*one_twenty); 
          //    yt = RT*sin((double)k*one_twenty); 
          ri[X] = obji.position[X] +
            obji.radius * cos(obji.position[THETA] + thetai);
          ri[Y] = obji.position[Y] +
            obji.radius * sin(obji.position[THETA] + thetai);

          for (jj = 0; jj<objj.N; ++jj) {
            thetaj = (double)jj * (2 * M_PI / objj.N);

            rj[X] = objj.position[X] +
              objj.radius * cos(objj.position[THETA] + thetaj);
            rj[Y] = objj.position[Y] +
              objj.radius * sin(objj.position[THETA] + thetaj);

            dr[X] = ri[X] - rj[X];
            dr[Y] = ri[Y] - rj[Y];
            mag = sqrt(SQR(dr[X]) + SQR(dr[Y]));
            dij = MAX(0.0, (obji.Rsphere + objj.Rsphere - mag));
            fx = K_COLLIDE*dij*(dr[X] / mag);
            fy = K_COLLIDE*dij*(dr[Y] / mag);

            rx = obji.radius * cos(obji.position[THETA] + thetai);
            ry = obji.radius * sin(obji.position[THETA] + thetai);
            tz = rx * fy - fx * ry;

            objects_total[i].net_extForce[X] += fx;
            objects_total[i].net_extForce[Y] += fy;
            objects_total[i].net_extForce[THETA] += tz;

            rx = objj.radius * cos(objj.position[THETA] + thetaj);
            ry = objj.radius * sin(objj.position[THETA] + thetaj);
            tz = rx * fy - fx * ry;

            objects_total[j].net_extForce[X] -= fx;
            objects_total[j].net_extForce[Y] -= fy;
            objects_total[j].net_extForce[THETA] -= tz;
          }
        }
      }
    }
  }


  // Update the RogersBodys with the calculated collision forces
  int base_counter = 0;
  for (i = 0; i < numRoger; ++i) {
    base_counter = i * N_BODY_ROBOT;

    // BASE
    RogersBodys[i].mobile_base.extForce[X] = objects_total[base_counter + BASE].net_extForce[X];
    RogersBodys[i].mobile_base.extForce[Y] = objects_total[base_counter + BASE].net_extForce[Y];
  
    // ARM #1
    //  reality check: why do you need the negative of fb?
    RogersBodys[i].arms[LEFT][NARM_FRAMES - 1].extForce[X] = -objects_total[base_counter + ARM1].net_extForce[X];
    RogersBodys[i].arms[LEFT][NARM_FRAMES - 1].extForce[Y] = -objects_total[base_counter + ARM1].net_extForce[Y];

    // ARM #2
    //  reality check: why do you need the negative of fb?
    RogersBodys[i].arms[RIGHT][NARM_FRAMES - 1].extForce[X] = -objects_total[base_counter + ARM2].net_extForce[X];
    RogersBodys[i].arms[RIGHT][NARM_FRAMES - 1].extForce[Y] = -objects_total[base_counter + ARM2].net_extForce[Y];
  }
  base_counter = base_counter + N_BODY_ROBOT;
  
  // TOY OBJECT
  toy.net_extForce[X] = objects_total[base_counter].net_extForce[X];
  toy.net_extForce[Y] = objects_total[base_counter].net_extForce[Y];
  toy.net_extForce[THETA] = objects_total[base_counter].net_extForce[THETA];

  if (VERBOSE) { printf("exiting compute_external_forces()\n"); fflush(stdout); }
}

  /****************** BEGIN EVALUATION DEPENDENT DRAW FUNCTIONS *********************/

void draw_pong_score()
{
  char buffer[64];
  int n;

  n = sprintf(buffer, "Time: %6.3lf", pong_evaluator.possession_time_1);
  XSetForeground(display, gc, foreground);
  XDrawString(display, pixmap, gc,
    W2DX(zoom, MIN_X - 1.0), W2DY(zoom, 1.0), buffer, n);

  n = sprintf(buffer, "Time: %6.3lf", pong_evaluator.possession_time_2);
  XSetForeground(display, gc, foreground);
  XDrawString(display, pixmap, gc,
    W2DX(zoom, MAX_X + 0.5), W2DY(zoom, 1.0), buffer, n);

  n = sprintf(buffer, "Goal: %d", pong_evaluator.goal_1);
  XSetForeground(display, gc, foreground);
  XDrawString(display, pixmap, gc,
    W2DX(zoom, MIN_X - 1.0), W2DY(zoom, 1.3), buffer, n);

  n = sprintf(buffer, "Goal: %d", pong_evaluator.goal_2);
  XSetForeground(display, gc, foreground);
  XDrawString(display, pixmap, gc,
    W2DX(zoom, MAX_X + 0.5), W2DY(zoom, 1.3), buffer, n);

  n = sprintf(buffer, "Roger1");
  XSetForeground(display, gc, foreground);
  XDrawString(display, pixmap, gc,
    W2DX(zoom, MIN_X - 1.0), W2DY(zoom, 1.6), buffer, n);

  n = sprintf(buffer, "Roger2");
  XSetForeground(display, gc, foreground);
  XDrawString(display, pixmap, gc,
    W2DX(zoom, MAX_X + 0.5), W2DY(zoom, 1.6), buffer, n);

}

  /****************** END EVALUATION DEPENDENT DRAW FUNCTIONS *********************/

void draw_circle(cu, cv, r, fill)
int cu, cv, r, fill;
{
  if (fill == NOFILL)
    XDrawArc(display, pixmap, gc, cu - r, cv - r, 2 * r, 2 * r, 0, 64 * 360);
  else
    XFillArc(display, pixmap, gc, cu - r, cv - r, 2 * r, 2 * r, 0, 64 * 360);
}

void draw_frames_dev()
{
#define FRAME_L 0.04
#define FRAME_T 0.045

  XSetForeground(display, gc, foreground);

  // the Cartesian frame
  /* x-axis */
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, 0.0, kEnvironment), ConvertWorld2PixmapY(zoom, 0.0, kEnvironment),
    ConvertWorld2PixmapX(zoom, (FRAME_L*4.0), kEnvironment), ConvertWorld2PixmapY(zoom, 0.0, kEnvironment));
  XDrawString(display, pixmap, gc, ConvertWorld2PixmapX(zoom, FRAME_T*4.0, kEnvironment), ConvertWorld2PixmapY(zoom, 0.0, kEnvironment), "x", 1);

  /* y-axis */
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, 0.0, kEnvironment), ConvertWorld2PixmapY(zoom, 0.0, kEnvironment),
    ConvertWorld2PixmapX(zoom, 0.0, kEnvironment), ConvertWorld2PixmapY(zoom, FRAME_L*4.0, kEnvironment));
  XDrawString(display, pixmap, gc, ConvertWorld2PixmapX(zoom, 0.0, kEnvironment), ConvertWorld2PixmapY(zoom, FRAME_T*4.0, kEnvironment), "y", 1);

  // the LEFT CSpace frame
  /* q1-axis */
  XDrawLine(display, pixmap, gc,
    T12LD(zoom, 0.0), T22LD(zoom, 0.0),
    T12LD(zoom, FRAME_L*2.0*M_PI), T22LD(zoom, 0.0));
  XDrawString(display, pixmap, gc, T12LD(zoom, FRAME_T*2.0*M_PI), T22LD(zoom, 0.0), "q1", 2);

  /* q2-axis */
  XDrawLine(display, pixmap, gc,
    T12LD(zoom, 0.0), T22LD(zoom, 0.0),
    T12LD(zoom, 0.0), T22LD(zoom, FRAME_L*2.0*M_PI));
  XDrawString(display, pixmap, gc, T12LD(zoom, 0.0), T22LD(zoom, FRAME_T*2.0*M_PI), "q2", 2);

  XDrawString(display, pixmap, gc, T12LD(zoom, -0.75), T22LD(zoom, -3.5), "left", 4);
  XDrawString(display, pixmap, gc, T12LD(zoom, 0.25), T22LD(zoom, -3.5), "/", 1);
  XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
  XDrawString(display, pixmap, gc, T12LD(zoom, -0.2), T22LD(zoom, -3.5), "arm", 3);
  XSetForeground(display, gc, world_colors[EYE_COLOR].display_color);
  XDrawString(display, pixmap, gc, T12LD(zoom, 0.35), T22LD(zoom, -3.5), "eye", 3);

  XSetForeground(display, gc, foreground);
  // the RIGHT CSpace frame
  /* q1-axis */
  XDrawLine(display, pixmap, gc,
    T12RD(zoom, 0.0), T22RD(zoom, 0.0), T12RD(zoom, FRAME_L*2.0*M_PI), T22RD(zoom, 0.0));
  XDrawString(display, pixmap, gc, T12RD(zoom, FRAME_T*2.0*M_PI), T22RD(zoom, 0.0), "q1", 2);

  /* q2-axis */
  XDrawLine(display, pixmap, gc,
    T12RD(zoom, 0.0), T22RD(zoom, 0.0), T12RD(zoom, 0.0), T22RD(zoom, FRAME_L*2.0*M_PI));
  XDrawString(display, pixmap, gc, T12RD(zoom, 0.0), T22RD(zoom, FRAME_T*2.0*M_PI), "q2", 2);

  XDrawString(display, pixmap, gc, T12RD(zoom, -0.85), T22RD(zoom, -3.5), "right", 5);
  XDrawString(display, pixmap, gc, T12RD(zoom, 0.25), T22RD(zoom, -3.5), "/", 1);
  XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
  XDrawString(display, pixmap, gc, T12RD(zoom, -0.15), T22RD(zoom, -3.5), "arm", 3);
  XSetForeground(display, gc, world_colors[EYE_COLOR].display_color);
  XDrawString(display, pixmap, gc, T12RD(zoom, 0.4), T22RD(zoom, -3.5), "eye", 3);

#undef FRAME_L // 0.04
#undef FRAME_T // 0.045
}


void draw_frames()
{
#define FRAME_L 0.04
#define FRAME_T 0.045

  XSetForeground(display, gc, foreground);

  // the Cartesian frame
  /* x-axis */
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, 0.0, kEnvironment), ConvertWorld2PixmapY(zoom, 0.0, kEnvironment),
    ConvertWorld2PixmapX(zoom, (FRAME_L*4.0), kEnvironment), ConvertWorld2PixmapY(zoom, 0.0, kEnvironment));
  XDrawString(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, FRAME_T*4.0, kEnvironment), ConvertWorld2PixmapY(zoom, 0.0, kEnvironment), "x", 1);

  /* y-axis */
  XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, 0.0, kEnvironment), ConvertWorld2PixmapY(zoom, 0.0, kEnvironment),
    ConvertWorld2PixmapX(zoom, 0.0, kEnvironment), ConvertWorld2PixmapY(zoom, FRAME_L*4.0, kEnvironment));
  XDrawString(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, 0.0, kEnvironment), ConvertWorld2PixmapY(zoom, FRAME_T*4.0, kEnvironment), "y", 1);


#undef FRAME_L // 0.04
#undef FRAME_T // 0.045
}

void draw_frame(xform)
double xform[4][4]; {
#define FRAME_L 0.04
#define FRAME_T 0.045

  XSetForeground(display, gc, foreground);

  /* x-axis */
  XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, xform[0][3], kEnvironment),
    ConvertWorld2PixmapY(zoom, xform[1][3], kEnvironment),
    ConvertWorld2PixmapX(zoom, xform[0][3] + FRAME_L*xform[0][0], kEnvironment),
    ConvertWorld2PixmapY(zoom, xform[1][3] + FRAME_L*xform[1][0], kEnvironment));
  XDrawString(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, xform[0][3] + FRAME_T*xform[0][0], kEnvironment),
    ConvertWorld2PixmapY(zoom, xform[1][3] + FRAME_T*xform[1][0], kEnvironment), "x", 1);

  /* y-axis */
  XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, xform[0][3], kEnvironment),
    ConvertWorld2PixmapY(zoom, xform[1][3], kEnvironment),
    ConvertWorld2PixmapX(zoom, xform[0][3] + FRAME_L*xform[0][1], kEnvironment),
    ConvertWorld2PixmapY(zoom, xform[1][3] + FRAME_L*xform[1][1], kEnvironment));
  XDrawString(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, xform[0][3] + FRAME_T*xform[0][1], kEnvironment),
    ConvertWorld2PixmapY(zoom, xform[1][3] + FRAME_T*xform[1][1], kEnvironment), "y", 1);

#undef FRAME_L // 0.04
#undef FRAME_T // 0.045
}

void draw_boundaries_dev()
{
  /******************************************************************/
  /**  draw world                                                  **/
  XSetForeground(display, gc, foreground);
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, MIN_X_DEV, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y_DEV, kEnvironment),
    ConvertWorld2PixmapX(zoom, MAX_X_DEV, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y_DEV, kEnvironment));
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, MAX_X_DEV, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y_DEV, kEnvironment),
    ConvertWorld2PixmapX(zoom, MAX_X_DEV, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y_DEV, kEnvironment));
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, MAX_X_DEV, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y_DEV, kEnvironment),
    ConvertWorld2PixmapX(zoom, MIN_X_DEV, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y_DEV, kEnvironment));
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, MIN_X_DEV, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y_DEV, kEnvironment),
    ConvertWorld2PixmapX(zoom, MIN_X_DEV, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y_DEV, kEnvironment));

  /* draw LEFT boundaries */
  XSetForeground(display, gc, foreground);
  XDrawLine(display, pixmap, gc,
    T12LD(zoom, T1_MIN), T22LD(zoom, T2_MAX), T12LD(zoom, T1_MAX), T22LD(zoom, T2_MAX));
  XDrawLine(display, pixmap, gc,
    T12LD(zoom, T1_MAX), T22LD(zoom, T2_MAX), T12LD(zoom, T1_MAX), T22LD(zoom, T2_MIN));
  XDrawLine(display, pixmap, gc,
    T12LD(zoom, T1_MAX), T22LD(zoom, T2_MIN), T12LD(zoom, T1_MIN), T22LD(zoom, T2_MIN));
  XDrawLine(display, pixmap, gc,
    T12LD(zoom, T1_MIN), T22LD(zoom, T2_MIN), T12LD(zoom, T1_MIN), T22LD(zoom, T2_MAX));

  /* draw RIGHT boundaries */
  XSetForeground(display, gc, foreground);
  XDrawLine(display, pixmap, gc,
    T12RD(zoom, T1_MIN), T22RD(zoom, T2_MAX), T12RD(zoom, T1_MAX), T22RD(zoom, T2_MAX));
  XDrawLine(display, pixmap, gc,
    T12RD(zoom, T1_MAX), T22RD(zoom, T2_MAX), T12RD(zoom, T1_MAX), T22RD(zoom, T2_MIN));
  XDrawLine(display, pixmap, gc,
    T12RD(zoom, T1_MAX), T22RD(zoom, T2_MIN), T12RD(zoom, T1_MIN), T22RD(zoom, T2_MIN));
  XDrawLine(display, pixmap, gc,
    T12RD(zoom, T1_MIN), T22RD(zoom, T2_MIN), T12RD(zoom, T1_MIN), T22RD(zoom, T2_MAX));
}


void draw_boundaries()
{
  /******************************************************************/
  /**  draw world                                                  **/
  XSetForeground(display, gc, foreground);
  XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, MIN_X, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y, kEnvironment),
    ConvertWorld2PixmapX(zoom, MAX_X, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y, kEnvironment));
  XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, MAX_X, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y, kEnvironment),
    ConvertWorld2PixmapX(zoom, MAX_X, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y, kEnvironment));
  XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, MAX_X, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y, kEnvironment),
    ConvertWorld2PixmapX(zoom, MIN_X, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y, kEnvironment));
  XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, MIN_X, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y, kEnvironment),
    ConvertWorld2PixmapX(zoom, MIN_X, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y, kEnvironment));

  XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, MIN_X + (MAX_X - MIN_X) / 2, kEnvironment), ConvertWorld2PixmapY(zoom, MIN_Y, kEnvironment),
    ConvertWorld2PixmapX(zoom, MIN_X + (MAX_X - MIN_X) / 2, kEnvironment), ConvertWorld2PixmapY(zoom, MAX_Y, kEnvironment));
}

// draw the NBINSxNBINS potential maps in their respective areas of the canvas
void draw_potential_maps_dev()
{
  int i, j, Cart_grey_index, left_arm_grey_index, right_arm_grey_index;
  double x, y, t1, t2;
  double Cart_bin_potential, left_arm_bin_potential, right_arm_bin_potential;

  for (i = 0; i < NBINS; ++i) {
    y = MAX_Y_DEV - i*YDELTA;
    t2 = T2_MAX - i*TDELTA;
    for (j = 0; j < NBINS; ++j) {
      x = MIN_X_DEV + j*XDELTA_DEV;
      t1 = T1_MIN + j*TDELTA;
      // user map grey level fill
      Cart_bin_potential = Rogers[0].world_map.potential_map[i][j];
      left_arm_bin_potential = Rogers[0].arm_map[LEFT].potential_map[i][j];
      right_arm_bin_potential = Rogers[0].arm_map[RIGHT].potential_map[i][j];

      // 0 <= grey indices <= 100
      Cart_grey_index = (int)(Cart_bin_potential * 100.0);
      left_arm_grey_index = (int)(left_arm_bin_potential * 100.0);
      right_arm_grey_index = (int)(right_arm_bin_potential * 100.0);

      // Cartesian Map
      // fill is either:
      //   a grey level depicting the user defined potential
      XSetForeground(display, gc, world_colors[Cart_grey_index].display_color);
      //   a user map perceived obstacle color, or
      if (Rogers[0].world_map.occupancy_map[i][j] == OBSTACLE)
        XSetForeground(display, gc,
          world_colors[Rogers[0].world_map.color_map[i][j]].display_color);
      else if (Rogers[0].world_map.occupancy_map[i][j] == DILATED_OBSTACLE)
        XSetForeground(display, gc,
          world_colors[Rogers[0].world_map.color_map[i][j]].display_color);
      //   a user defined goal
      else if (Rogers[0].world_map.occupancy_map[i][j] == GOAL)
        XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
      XFillRectangle(display, pixmap, gc,
        ConvertWorld2PixmapX(zoom, x, kEnvironment), ConvertWorld2PixmapY(zoom, y, kEnvironment),
        (ConvertWorld2PixmapR(zoom, XDELTA_DEV, kEnvironment) + 1), (ConvertWorld2PixmapR(zoom, YDELTA_DEV, kEnvironment) + 1));

      //      // each real obstacle should be outlined in the obstacle color
      //      if (real_world.occupancy_map[i][j] == OBSTACLE) {
      //  XSetForeground(display, gc,
      //         world_colors[real_world.color_map[i][j]].display_color);
      //  x = MIN_X + j*XDELTA; y = MAX_Y - i*YDELTA;
      //  XDrawRectangle(display, pixmap, gc, ConvertWorld2PixmapX(zoom,x), ConvertWorld2PixmapY(zoom,y),
      //         (ConvertWorld2PixmapR(zoom,XDELTA)), (ConvertWorld2PixmapR(zoom,YDELTA)));
      //      }

      // Left Arm Map
      XSetForeground(display, gc,
        world_colors[left_arm_grey_index].display_color);
      if (Rogers[0].arm_map[LEFT].occupancy_map[i][j] == OBSTACLE)
        XSetForeground(display, gc, world_colors[RED].display_color);
      else if (Rogers[0].arm_map[LEFT].occupancy_map[i][j] == GOAL)
        XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
      XFillRectangle(display, pixmap, gc,
        T12LD(zoom, t1), T22LD(zoom, t2), (T2DR(zoom, TDELTA) + 1), (T2DR(zoom, TDELTA) + 1));

      // Right Arm Map
      XSetForeground(display, gc,
        world_colors[right_arm_grey_index].display_color);
      if (Rogers[0].arm_map[RIGHT].occupancy_map[i][j] == OBSTACLE)
        XSetForeground(display, gc, world_colors[RED].display_color);
      else if (Rogers[0].arm_map[RIGHT].occupancy_map[i][j] == GOAL)
        XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
      XFillRectangle(display, pixmap, gc,
        T12RD(zoom, t1), T22RD(zoom, t2), (T2DR(zoom, TDELTA) + 1), (T2DR(zoom, TDELTA) + 1));
    }
  }
}

// draw the NBINSxNBINS potential maps in their respective areas of the canvas
void draw_potential_maps() {
  int i, j, k, Cart_grey_index, left_arm_grey_index, right_arm_grey_index;
  double x, y;
  double Cart_bin_potential, left_arm_bin_potential, right_arm_bin_potential;

  for (k = 0; k < numRoger; ++k) {
    for (i = 0; i < NBINS; ++i) {
      y = MAX_Y - i*YDELTA;
      for (j = 0; j < NBINS; ++j) {
        x = MIN_X + j*XDELTA;
        // user map grey level fill
        Cart_bin_potential = Rogers[k].world_map.potential_map[i][j];
        left_arm_bin_potential = Rogers[k].arm_map[LEFT].potential_map[i][j];
        right_arm_bin_potential = Rogers[k].arm_map[RIGHT].potential_map[i][j];

        // 0 <= grey indices <= 100
        Cart_grey_index = (int)(Cart_bin_potential * 100.0);
        left_arm_grey_index = (int)(left_arm_bin_potential * 100.0);
        right_arm_grey_index = (int)(right_arm_bin_potential * 100.0);

        // Cartesian Map
        // fill is either:
        //   a grey level depicting the user defined potential
        XSetForeground(display, gc, world_colors[Cart_grey_index].display_color);
        //   a user map perceived obstacle color, or
        if (Rogers[k].world_map.occupancy_map[i][j] == OBSTACLE)
          XSetForeground(display, gc,
            world_colors[Rogers[k].world_map.color_map[i][j]].display_color);
        else if (Rogers[k].world_map.occupancy_map[i][j] == DILATED_OBSTACLE)
          XSetForeground(display, gc,
            world_colors[Rogers[k].world_map.color_map[i][j]].display_color);
        //   a user defined goal
        else if (Rogers[k].world_map.occupancy_map[i][j] == GOAL)
          XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
        XFillRectangle(display, pixmap, gc, ConvertWorld2PixmapX(zoom, x, kEnvironment), ConvertWorld2PixmapY(zoom, y, kEnvironment),
          (ConvertWorld2PixmapR(zoom, XDELTA, kEnvironment) + 1), (ConvertWorld2PixmapR(zoom, YDELTA, kEnvironment) + 1));
      }
    }
  }
}


void draw_object_poly(obj)
PolyBall obj;
{
  int k;
  double xw, yw, theta;

  XSetForeground(display, gc, world_colors[OBJECT_COLOR].display_color);

  for (k = 0; k<obj.N; ++k) {
    theta = (double)k * 2.0*M_PI / obj.N;
    xw = obj.position[X]
      + cos(obj.position[THETA]) * (obj.radius*cos(theta))
      - sin(obj.position[THETA]) * (obj.radius*sin(theta));
    yw = obj.position[Y]
      + sin(obj.position[THETA]) * (obj.radius*cos(theta))
      + cos(obj.position[THETA]) * (obj.radius*sin(theta));
    draw_circle(ConvertWorld2PixmapX(zoom, xw, kEnvironment), ConvertWorld2PixmapY(zoom, yw, kEnvironment),
      ConvertWorld2PixmapR(zoom, obj.Rsphere, kEnvironment), FILL);
  }
}

void draw_object(obj)
Obj obj;
{
  XSetForeground(display, gc, world_colors[OBJECT_COLOR].display_color);
  draw_circle(ConvertWorld2PixmapX(zoom, obj.position[X], kEnvironment), ConvertWorld2PixmapY(zoom, obj.position[Y], kEnvironment),
    ConvertWorld2PixmapR(zoom, R_OBJ, kEnvironment), FILL);
}


void draw_toy(toy, fill)
eObject toy;
int fill;
{
  int i, j, n;
  double x0, y0, x1, y1;
  XPoint points[12];

  if (VERBOSE) printf("         inside draw_toy()\n");
  // BLACK: 0   RED: 102    BLUE: 105    GREEN: 108    
  if (toy.N == 1) { //special case: CIRCLE
    XSetForeground(display, gc, toy.color[0].display_color);
    if (VERBOSE) {
      if (fill == FILL) printf("            i=0: drawing CIRCLE FILL\n");
      else printf("            i=0: drawing CIRCLE NOFILL\n");
    }
    draw_circle(ConvertWorld2PixmapX(zoom, toy.pos[0], kEnvironment), ConvertWorld2PixmapY(zoom, toy.pos[1], kEnvironment),
      ConvertWorld2PixmapR(zoom, R_TOY, kEnvironment), fill);
  }
  else if ((toy.N > 2) && (toy.N <= 12)) {// polygon w/N vertices
    if (fill == FILL) {
      XSetForeground(display, gc, toy.color[0].display_color);
      // XFillRectangle(display, pixmap, gc,
      //            ConvertWorld2PixmapX(zoom,x), ConvertWorld2PixmapY(zoom,y),
      //      (ConvertWorld2PixmapR(zoom,XDELTA) + 1), (ConvertWorld2PixmapR(zoom,YDELTA) + 1));
      if (VERBOSE) {
        if (toy.N == 3)
          printf("            i=1: drawing TRIANGLE FILL\n");
        if (toy.N == 4)
          printf("            i=2: drawing SQUARE FILL\n");
        if (toy.N == 11)
          printf("            i=3: drawing TRIANGLE RECEPTICAL FILL\n");
        if (toy.N == 12)
          printf("            i=4: drawing SQUARE RECEPTICAL FILL\n");
      }
      for (j = 0; j<toy.N; ++j) {
        points[j].x = (short)ConvertWorld2PixmapX(zoom, (toy.pos[X] + toy.vertices[j][X]), kEnvironment);
        points[j].y = (short)ConvertWorld2PixmapY(zoom, (toy.pos[Y] + toy.vertices[j][Y]), kEnvironment);
      }

      XFillPolygon(display, pixmap, gc, points,
        toy.N, Complex, CoordModeOrigin);
    }
    else {
      if (VERBOSE) {
        if (toy.N == 3)
          printf("            i=1: drawing TRIANGLE NOFILL\n");
        if (toy.N == 4)
          printf("            i=2: drawing SQUARE NOFILL\n");
        if (toy.N == 11)
          printf("            i=3: drawing TRIANGLE RECEPTICAL NOFILL\n");
        if (toy.N == 12)
          printf("            i=4: drawing SQUARE RECEPTICAL NOFILL\n");
      }
    }
    for (j = 0; j<toy.N; ++j) {
      XSetForeground(display, gc, toy.color[j].display_color);
      x0 = toy.pos[X] + toy.vertices[j][X];
      y0 = toy.pos[Y] + toy.vertices[j][Y];
      if (j == toy.N - 1) {
        x1 = toy.pos[X] + toy.vertices[0][X];
        y1 = toy.pos[Y] + toy.vertices[0][Y];
      }
      else {
        x1 = toy.pos[X] + toy.vertices[j + 1][X];
        y1 = toy.pos[Y] + toy.vertices[j + 1][Y];
      }
      XDrawLine(display, pixmap, gc,
        ConvertWorld2PixmapX(zoom, x0, kEnvironment), ConvertWorld2PixmapY(zoom, y0, kEnvironment), ConvertWorld2PixmapX(zoom, x1, kEnvironment), ConvertWorld2PixmapY(zoom, y1, kEnvironment));
    }
  }
  else { // N=2
    printf("illegal polygon\n");
    //      return;
  }

  XSetForeground(display, gc, world_colors[GREEN].display_color);
  /* x-axis */
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, (toy.pos[X] - 0.05), kEnvironment),
    ConvertWorld2PixmapY(zoom, toy.pos[Y], kEnvironment),
    ConvertWorld2PixmapX(zoom, (toy.pos[X] + 0.05), kEnvironment),
    ConvertWorld2PixmapY(zoom, toy.pos[Y], kEnvironment));

  /* y-axis */
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, toy.pos[X], kEnvironment),
    ConvertWorld2PixmapY(zoom, (toy.pos[Y] - 0.05), kEnvironment),
    ConvertWorld2PixmapX(zoom, toy.pos[X], kEnvironment),
    ConvertWorld2PixmapY(zoom, (toy.pos[Y] + 0.05), kEnvironment));
}


void draw_active_toys()
{
  int i, j, n;
  double x0, y0, x1, y1;
  XPoint points[12];

  // BLACK: 0   RED: 102    BLUE: 105    GREEN: 108    
  if (VERBOSE) printf("  inside draw_active_toys()\n");
  for (i = 0; i<n_active_toys; ++i) {
    draw_toy(active_toys[i], FILL);
  }
  if (VERBOSE) printf("\n");
}

void draw_toybox()
{
  int i;

  // BLACK: 0   RED: 102    BLUE: 105    GREEN: 108    
  if (VERBOSE)  printf("   inside draw_toybox():\n");
  for (i = 0; i<N_TOY_TYPES; ++i) {
    if (i == input_toy_type) {
      if (VERBOSE) printf("      before draw_toy()\n");
      draw_toy(toybox[i], FILL);
      if (VERBOSE) printf("      returned from draw_toy()\n");
    }
    else {
      if (VERBOSE) printf("      before draw_toy()\n");
      draw_toy(toybox[i], NOFILL);
      if (VERBOSE) printf("      returned from draw_toy()\n");
    }
  }
  if (VERBOSE) printf("\n");
}

void draw_eye(base, eye)
Base base;
Eye eye;
{
  double px, py;
  double rx, ry, from_x, from_y, to_x, to_y;
  double lambda_x, lambda_y;
  int xbin, ybin;

  // Definie delta_x and delta_y size given the environment
  float x_delta, y_delta;
  double max_x, min_x, max_y, min_y;

  if (kEnvironment == DEVELOPMENTAL) {
    x_delta = XDELTA_DEV;
    y_delta = YDELTA_DEV;
    max_x = MAX_X_DEV;
    min_x = MIN_X_DEV;
    max_y = MAX_Y_DEV;
    min_y = MIN_Y_DEV;
  }
  else {
    x_delta = XDELTA;
    y_delta = YDELTA;
    max_x = MAX_X;
    min_x = MIN_X;
    max_y = MAX_Y;
    min_y = MIN_Y;
  }

  px = base.wTb[0][0] * eye.position[0] + base.wTb[0][1] * eye.position[1] +
    base.wTb[0][3];
  py = base.wTb[1][0] * eye.position[0] + base.wTb[1][1] * eye.position[1] +
    base.wTb[1][3];

  from_x = px; from_y = py;
  rx = cos(base.theta + eye.theta);
  ry = sin(base.theta + eye.theta);

  //trace the eye direction till you hit an obstacle
  to_x = from_x;
  to_y = from_y;

  while (to_x < max_x && to_x > min_x && to_y < max_y && to_y > min_y) {
    //get bin for location
    ybin = (int)((max_y - to_y) / y_delta);
    xbin = (int)((to_x - min_x) / x_delta);

    //check for obstacle collision
    if (Rogers[0].world_map.occupancy_map[ybin][xbin] == OBSTACLE) break;

    to_x += rx * 0.001;
    to_y += ry * 0.001;
  }

  XSetForeground(display, gc, world_colors[GAZE_COLOR].display_color);
  XDrawLine(display, pixmap, gc,
	    ConvertWorld2PixmapX(zoom, from_x, kEnvironment),
	    ConvertWorld2PixmapY(zoom, from_y, kEnvironment),
	    ConvertWorld2PixmapX(zoom, to_x, kEnvironment),
	    ConvertWorld2PixmapY(zoom, to_y, kEnvironment));

  XSetForeground(display, gc, foreground);
  draw_circle(ConvertWorld2PixmapX(zoom, px, kEnvironment),
	      ConvertWorld2PixmapY(zoom, py, kEnvironment),
	      ConvertWorld2PixmapR(zoom, R_EYE, kEnvironment), NOFILL);
  draw_circle(ConvertWorld2PixmapX(zoom, px + (R_EYE - 0.8*R_PUPIL)*rx, 
				   kEnvironment),
	      ConvertWorld2PixmapY(zoom, py + (R_EYE - 0.8*R_PUPIL)*ry,
				   kEnvironment),
	      ConvertWorld2PixmapR(zoom, R_PUPIL, kEnvironment), FILL);
}

void draw_image_dev(eye)
int eye;
{
  register int i, color, dx;
  float scaled_left_image_x;
  float scaled_right_image_x;

  // Scale the position of image_x such that the image center is scaled linearly with zoom factor
  scaled_left_image_x = zoom * LEFT_IMAGE_X + ((float)(IMAGE_WIDTH) / 2.0) * (zoom - 1);
  scaled_right_image_x = zoom * RIGHT_IMAGE_X + ((float)(IMAGE_WIDTH) / 2.0) * (zoom - 1);

  XSetForeground(display, gc, foreground);
  if (eye == LEFT)
    XDrawRectangle(display, pixmap, gc,
    (scaled_left_image_x - 1), (zoom*IMAGE_Y - 1),
      (IMAGE_WIDTH + 1), (PIXEL_HEIGHT + 1));
  else if (eye == RIGHT)
    XDrawRectangle(display, pixmap, gc,
    (scaled_right_image_x - 1), (zoom*IMAGE_Y - 1),
      (IMAGE_WIDTH + 1), (PIXEL_HEIGHT + 1));

  for (i = 0, dx = 0; i < NPIXELS; i++, dx += PIXEL_WIDTH) {
    color = RogersBodys[0].eyes[eye].image[i];
    XSetForeground(display, gc, world_colors[color].display_color);

    if (eye == LEFT)
      XFillRectangle(display, pixmap, gc,
      (scaled_left_image_x + dx), zoom*IMAGE_Y, PIXEL_WIDTH, PIXEL_HEIGHT);
    else if (eye == RIGHT)
      XFillRectangle(display, pixmap, gc,
      (scaled_right_image_x + dx), zoom*IMAGE_Y, PIXEL_WIDTH, PIXEL_HEIGHT);
  }
}

void draw_image(eye, rogerID)
int eye;
int rogerID;
{
  int i, color, dx;
  float scaled_left_image_x_1;
  float scaled_right_image_x_1;
  float scaled_left_image_x_2;
  float scaled_right_image_x_2;

  // Scale the position of image_x such that the image center is scaled linearly with zoom factor
  scaled_left_image_x_1 = zoom * LEFT_IMAGE_X_1 + ((float)(IMAGE_WIDTH) / 2.0) * (zoom - 1);
  scaled_right_image_x_1 = zoom * RIGHT_IMAGE_X_1 + ((float)(IMAGE_WIDTH) / 2.0) * (zoom - 1);
  scaled_left_image_x_2 = zoom * LEFT_IMAGE_X_2 + ((float)(IMAGE_WIDTH) / 2.0) * (zoom - 1);
  scaled_right_image_x_2 = zoom * RIGHT_IMAGE_X_2 + ((float)(IMAGE_WIDTH) / 2.0) * (zoom - 1);


  if (rogerID == 0) {
    XSetForeground(display, gc, foreground);
    if (eye == LEFT)
      XDrawRectangle(display, pixmap, gc, scaled_left_image_x_1 - 1, (zoom*IMAGE_Y - 1),
        IMAGE_WIDTH + 1, PIXEL_HEIGHT + 1);
    else if (eye == RIGHT)
      XDrawRectangle(display, pixmap, gc, scaled_right_image_x_1 - 1, (zoom*IMAGE_Y - 1) - 1,
        IMAGE_WIDTH + 1, PIXEL_HEIGHT + 1);

    for (i = 0, dx = 0; i < NPIXELS; i++, dx += PIXEL_WIDTH) {
      color = RogersBodys[rogerID].eyes[eye].image[i];
      //    XSetForeground(display, gc, color);

      // Commented out by Dan.
      //printf("color=%d\n", color);

      //    if (color > 99) XSetForeground(display, gc, background);
      //    else if (intensity >= 0)
      //      XSetForeground(display, gc, image_color[intensity]);
      //    if (color == ARM_COLOR)
      //      XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
      //    else if (color == ARM_COLOR)
      //      XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
      //    else if (color == OBJECT_COLOR)
      //      XSetForeground(display, gc,
      //         world_colors[OBJECT_COLOR].display_color);
      XSetForeground(display, gc, world_colors[color].display_color);

      if (eye == LEFT)
        XFillRectangle(display, pixmap, gc, scaled_left_image_x_1 + dx, zoom*IMAGE_Y,
          PIXEL_WIDTH, PIXEL_HEIGHT);
      else if (eye == RIGHT)
        XFillRectangle(display, pixmap, gc, scaled_right_image_x_1 + dx, zoom*IMAGE_Y,
          PIXEL_WIDTH, PIXEL_HEIGHT);
    }
  }
  else if (rogerID == 1) {
    XSetForeground(display, gc, foreground);
    if (eye == LEFT)
      XDrawRectangle(display, pixmap, gc, scaled_left_image_x_2 - 1, (zoom*IMAGE_Y - 1),
        IMAGE_WIDTH + 1, PIXEL_HEIGHT + 1);
    else if (eye == RIGHT)
      XDrawRectangle(display, pixmap, gc, scaled_right_image_x_2 - 1, (zoom*IMAGE_Y - 1),
        IMAGE_WIDTH + 1, PIXEL_HEIGHT + 1);

    for (i = 0, dx = 0; i < NPIXELS; i++, dx += PIXEL_WIDTH) {
      color = RogersBodys[rogerID].eyes[eye].image[i];
      //    XSetForeground(display, gc, color);

      // Commented out by Dan.
      //printf("color=%d\n", color);

      //    if (color > 99) XSetForeground(display, gc, background);
      //    else if (intensity >= 0)
      //      XSetForeground(display, gc, image_color[intensity]);
      //    if (color == ARM_COLOR)
      //      XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
      //    else if (color == ARM_COLOR)
      //      XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
      //    else if (color == OBJECT_COLOR)
      //      XSetForeground(display, gc,
      //         world_colors[OBJECT_COLOR].display_color);
      XSetForeground(display, gc, world_colors[color].display_color);

      if (eye == LEFT)
        XFillRectangle(display, pixmap, gc, scaled_left_image_x_2 + dx, zoom*IMAGE_Y,
          PIXEL_WIDTH, PIXEL_HEIGHT);
      else if (eye == RIGHT)
        XFillRectangle(display, pixmap, gc, scaled_right_image_x_2 + dx, zoom*IMAGE_Y,
          PIXEL_WIDTH, PIXEL_HEIGHT);
    }
  }
}

void draw_roger(mobile_base, arms, eyes, rogerID)
Base mobile_base;
Arm arms[NARMS][NARM_FRAMES];
Eye eyes[NEYES];
int rogerID;
{
  int i, j, k;
  double r_b[4], r_w[4], fhat[2];
  double theta1, theta2, mag;
  double temp0[4][4], temp1[4][4];
  XPoint rect[4];
  void draw_history();

  /******************************************************************/
  /* draw mobile base */
  XSetForeground(display, gc, foreground);
  draw_circle(ConvertWorld2PixmapX(zoom, mobile_base.wTb[0][3], kEnvironment),
    ConvertWorld2PixmapY(zoom, mobile_base.wTb[1][3], kEnvironment), ConvertWorld2PixmapR(zoom, R_BASE, kEnvironment), NOFILL);

  // draw contact forces on object from body
  mag = sqrt(SQR(mobile_base.extForce[X]) + SQR(mobile_base.extForce[Y]));

  if (mag > 0.0) {
    fhat[X] = mobile_base.extForce[X] / mag;
    fhat[Y] = mobile_base.extForce[Y] / mag;

    XDrawLine(display, pixmap, gc,
      ConvertWorld2PixmapX(zoom, mobile_base.x - R_BASE*fhat[X], kEnvironment),
      ConvertWorld2PixmapY(zoom, mobile_base.y - R_BASE*fhat[Y], kEnvironment),
      ConvertWorld2PixmapX(zoom, mobile_base.x - (R_BASE + 0.08)*fhat[X], kEnvironment),
      ConvertWorld2PixmapY(zoom, mobile_base.y - (R_BASE + 0.08)*fhat[Y], kEnvironment));

    //    XDrawLine(display, pixmap, gc,
    //      ConvertWorld2PixmapX(zoom, mobile_base.x + R_BASE*cos(mobile_base.contact_theta)),
    //      ConvertWorld2PixmapY(zoom, mobile_base.y + R_BASE*sin(mobile_base.contact_theta)),
    //      ConvertWorld2PixmapX(zoom, mobile_base.x + R_BASE*cos(mobile_base.contact_theta)
    //     + 0.08*mobile_base.extforce[Y]/mag),
    //      ConvertWorld2PixmapY(zoom, mobile_base.y + R_BASE*sin(mobile_base.contact_theta)
    //     - 0.08*mobile_base.extforce[X]/mag));
  }

  //  draw_wheels();
  r_b[0] = R_BASE / 2.0; r_b[1] = R_BASE + WHEEL_THICKNESS; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  draw_circle(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment), ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment), ConvertWorld2PixmapR(zoom, WHEEL_THICKNESS, kEnvironment),
    FILL);
  r_b[0] = -R_BASE / 2.0; r_b[1] = R_BASE + WHEEL_THICKNESS; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  draw_circle(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment), ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment), ConvertWorld2PixmapR(zoom, WHEEL_THICKNESS, kEnvironment),
    FILL);

  r_b[0] = R_BASE / 2.0; r_b[1] = R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[0].x = (short)(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment));
  rect[0].y = (short)(ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment));

  r_b[0] = R_BASE / 2.0; r_b[1] = (R_BASE + 2 * WHEEL_THICKNESS); r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[1].x = (short)(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment));
  rect[1].y = (short)(ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment));

  r_b[0] = -R_BASE / 2.0; r_b[1] = (R_BASE + 2 * WHEEL_THICKNESS); r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[2].x = (short)(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment));
  rect[2].y = (short)(ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment));

  r_b[0] = -R_BASE / 2.0; r_b[1] = R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[3].x = (short)(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment));
  rect[3].y = (short)(ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment));

  XFillPolygon(display, pixmap, gc, rect, 4, Convex, CoordModeOrigin);

  r_b[0] = R_BASE / 2.0; r_b[1] = -R_BASE - WHEEL_THICKNESS; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  draw_circle(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment), ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment), ConvertWorld2PixmapR(zoom, WHEEL_THICKNESS, kEnvironment), FILL);
  r_b[0] = -R_BASE / 2.0; r_b[1] = -R_BASE - WHEEL_THICKNESS; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  draw_circle(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment), ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment), ConvertWorld2PixmapR(zoom, WHEEL_THICKNESS, kEnvironment), FILL);

  r_b[0] = R_BASE / 2.0; r_b[1] = -R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[0].x = (short)(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment));
  rect[0].y = (short)(ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment));

  r_b[0] = R_BASE / 2.0; r_b[1] = -(R_BASE + 2 * WHEEL_THICKNESS); r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[1].x = (short)(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment));
  rect[1].y = (short)(ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment));

  r_b[0] = -R_BASE / 2.0; r_b[1] = -(R_BASE + 2 * WHEEL_THICKNESS); r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[2].x = (short)(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment));
  rect[2].y = (short)(ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment));

  r_b[0] = -R_BASE / 2.0; r_b[1] = -R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[3].x = (short)(ConvertWorld2PixmapX(zoom, r_w[0], kEnvironment));
  rect[3].y = (short)(ConvertWorld2PixmapY(zoom, r_w[1], kEnvironment));

  XFillPolygon(display, pixmap, gc, rect, 4, Convex, CoordModeOrigin);
  /******************************************************************/
  /* draw eyes */
  for (i = 0; i < NEYES; i++)
  {
    draw_eye(mobile_base, eyes[i]);

    /******************************************************************/
    /* draw displays **************************************************/
    /* draw coordinate in configuration space for left and right eyes */
    if (kEnvironment == DEVELOPMENTAL) {
      XSetForeground(display, gc, world_colors[EYE_COLOR].display_color);
      if (i == LEFT)
        XFillRectangle(display, pixmap, gc, T12LD(zoom, eyes[i].theta),
        T22LD(zoom, 0.0), (T2DR(zoom, TDELTA) + 1), (T2DR(zoom, TDELTA) + 1));
      else if (i == RIGHT)
        XFillRectangle(display, pixmap, gc, T12RD(zoom, eyes[i].theta),
        T22RD(zoom, 0.0), (T2DR(zoom, TDELTA) + 1), (T2DR(zoom, TDELTA) + 1));
    }
  }
  /******************************************************************/
  /* draw arms */
  for (j = 0; j<NARMS; j++) {
    XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
    SIMmatXmat(mobile_base.wTb, arms[j][0].iTj, temp0);

    for (i = 1; i<NARM_FRAMES; i++) {
      SIMmatXmat(temp0, arms[j][i].iTj, temp1);
      XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, temp0[0][3], kEnvironment),
        ConvertWorld2PixmapY(zoom, temp0[1][3], kEnvironment), ConvertWorld2PixmapX(zoom, temp1[0][3], kEnvironment),
        ConvertWorld2PixmapY(zoom, temp1[1][3], kEnvironment));
      if (i == (NARM_FRAMES - 1))
        draw_circle(ConvertWorld2PixmapX(zoom, temp1[0][3], kEnvironment), ConvertWorld2PixmapY(zoom, temp1[1][3], kEnvironment),
          ConvertWorld2PixmapR(zoom, R_TACTILE, kEnvironment), FILL);
      else {
        draw_circle(ConvertWorld2PixmapX(zoom, temp1[0][3], kEnvironment), ConvertWorld2PixmapY(zoom, temp1[1][3], kEnvironment),
          ConvertWorld2PixmapR(zoom, R_JOINT, kEnvironment), NOFILL);
        SIMcopy_matrix(temp1, temp0);
      }
    }

    // draw endpoint forces
    mag = sqrt(SQR(arms[j][NARM_FRAMES - 1].extForce[X]) +
      SQR(arms[j][NARM_FRAMES - 1].extForce[Y]));
    if (mag>0.0) {
      XSetForeground(display, gc, foreground);
      fhat[X] = arms[j][NARM_FRAMES-1].extForce[X]/mag;
      fhat[Y] = arms[j][NARM_FRAMES-1].extForce[Y]/mag;
      XDrawLine(display, pixmap, gc,
        ConvertWorld2PixmapX(zoom, temp1[0][3] + R_TACTILE*fhat[X], kEnvironment),
        ConvertWorld2PixmapY(zoom, temp1[1][3] + R_TACTILE*fhat[Y], kEnvironment),
        ConvertWorld2PixmapX(zoom, temp1[0][3] + 2*R_TACTILE*fhat[X], kEnvironment),
        ConvertWorld2PixmapY(zoom, temp1[1][3] + 2*R_TACTILE*fhat[Y], kEnvironment));
    }

    /******************************************************************/
    /* draw displays **************************************************/
    /* draw coordinate in configuration space for left and right arms */
    if (kEnvironment == DEVELOPMENTAL) {
      if (j == LEFT)
        XFillRectangle(display, pixmap, gc,
           T12LD(zoom, arms[j][1].theta), T22LD(zoom, arms[j][2].theta),
           (T2DR(zoom, TDELTA)+1), (T2DR(zoom, TDELTA)+1));
      else if (j == RIGHT)
        XFillRectangle(display, pixmap, gc,
           T12RD(zoom, arms[j][1].theta), T22RD(zoom, arms[j][2].theta),
           (T2DR(zoom, TDELTA)+1), (T2DR(zoom, TDELTA)+1));
    }
  }
  if (HISTORY) draw_history(rogerID);

  /* visual images *************************************************/
  if (kEnvironment == ARENA) {
    if (numRoger <= 2) {
      draw_image(LEFT, rogerID);
      draw_image(RIGHT, rogerID);
    }

  } else {
    draw_image_dev(LEFT);
    draw_image_dev(RIGHT);
  }
}


void draw_all_dev()
{
  int n;
  char buffer[64];
  void draw_frames_dev(), draw_boundaries_dev(), draw_potential_maps_dev();
  void draw_toybox(), draw_active_toys();

  x_clear();

  int i;

  // draw localizability/manipulability ellipses on top (i.e. render them last)
  // draw harmonic function streamlines on bottom (i.e. render them first)

  //  draw_ellipse(manipulator(LEFT));
  //  draw_ellipse(manipulator(RIGHT));

  // XSetForeground(display, gc, goal_color);
  //  draw_ellipse(observation);
  //  draw_ellipse(spatial_goals[CENTROID]);
  //  draw_ellipse(spatial_goals[LEFT_EDGE]);
  //  draw_ellipse(spatial_goals[RIGHT_EDGE]);

  //  if (p_index==6) draw_ellipse(grasp_goal);
  //  XSetForeground(display, gc, target_color);
  //  draw_ellipse(target);

  draw_potential_maps_dev();
  draw_boundaries_dev();
  draw_frames_dev();
  // if (VERBOSE) printf("before draw_toybox()\n");
  // draw_toybox();

 
  draw_object_poly(toy);

  for (i = 0; i < numRoger; ++i) {
    draw_roger(RogersBodys[i].mobile_base, RogersBodys[i].arms, RogersBodys[i].eyes, i);
  }
  

  XSetForeground(display, gc, foreground);
  n = sprintf(buffer, "total elapsed time = %6.3lf", simtime);
  XDrawString(display, pixmap, gc, ConvertWorld2PixmapX(zoom, 2.2, kEnvironment), ConvertWorld2PixmapY(zoom, 1.65, kEnvironment), buffer, n);

  if (draw_visual) {
    n = sprintf(buffer, "VISUALIZE: ON");
    XDrawString(display, pixmap, gc, ConvertWorld2PixmapX(zoom, 2.6, kEnvironment), ConvertWorld2PixmapY(zoom, 1.8, kEnvironment), buffer, n);
  }
  else {
    n = sprintf(buffer, "VISUALIZE: OFF");
    XDrawString(display, pixmap, gc, ConvertWorld2PixmapX(zoom, 2.6, kEnvironment), ConvertWorld2PixmapY(zoom, 1.8, kEnvironment), buffer, n);
  }

  if (COUPLED_DYNAMICS) {
    n = sprintf(buffer, "WHOLE-BODY DYNAMICS");
    XDrawString(display, pixmap, gc, ConvertWorld2PixmapX(zoom, -3.625, kEnvironment), ConvertWorld2PixmapY(zoom, 1.8, kEnvironment), buffer, n);
  }
  else {
    n = sprintf(buffer, "BLOCK DIAGONAL DYNAMICS");
    XDrawString(display, pixmap, gc, ConvertWorld2PixmapX(zoom, -3.875, kEnvironment), ConvertWorld2PixmapY(zoom, 1.8, kEnvironment), buffer, n);
  }

  if (ACTUATE_BASE) {
    n = sprintf(buffer, "BASE: ON");
    XDrawString(display, pixmap, gc, ConvertWorld2PixmapX(zoom, -4.0, kEnvironment), ConvertWorld2PixmapY(zoom, 1.65, kEnvironment), buffer, n);
  }
  else {
    n = sprintf(buffer, "BASE: OFF");
    XDrawString(display, pixmap, gc, ConvertWorld2PixmapX(zoom, -4.0, kEnvironment), ConvertWorld2PixmapY(zoom, 1.65, kEnvironment), buffer, n);
  }

  if (ACTUATE_ARMS) {
    n = sprintf(buffer, "ARMS: ON");
    XDrawString(display, pixmap, gc, ConvertWorld2PixmapX(zoom, -3.3125, kEnvironment), ConvertWorld2PixmapY(zoom, 1.65, kEnvironment), buffer, n);
  }
  else {
    n = sprintf(buffer, "ARMS: OFF");
    XDrawString(display, pixmap, gc, ConvertWorld2PixmapX(zoom, -3.3125, kEnvironment), ConvertWorld2PixmapY(zoom, 1.65, kEnvironment), buffer, n);
  }

  if (ACTUATE_EYES) {
    n = sprintf(buffer, "EYES: ON");
    XDrawString(display, pixmap, gc, ConvertWorld2PixmapX(zoom, -2.625, kEnvironment), ConvertWorld2PixmapY(zoom, 1.65, kEnvironment), buffer, n);
  }
  else {
    n = sprintf(buffer, "EYES: OFF");
    XDrawString(display, pixmap, gc, ConvertWorld2PixmapX(zoom, -2.625, kEnvironment), ConvertWorld2PixmapY(zoom, 1.65, kEnvironment), buffer, n);
  }

  HandleDrawingRequests(&Rogers[0]);
  x_expose();
}

void draw_all()
{
  int n, i;
  char buffer[64];
  void draw_potential_maps(), draw_object(), draw_roger(), draw_eval();

  x_clear();

  draw_potential_maps();
  draw_boundaries();
  draw_frames();
  draw_object_poly(toy);

  for (i = 0; i < numRoger; ++i) {
    draw_roger(RogersBodys[i].mobile_base, RogersBodys[i].arms, RogersBodys[i].eyes, i);
  }

  draw_eval();

  n = sprintf(buffer, "total elapsed time = %6.3lf", simtime);
  XSetForeground(display, gc, foreground);
  XDrawString(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, 3.0, kEnvironment), ConvertWorld2PixmapY(zoom, 1.8, kEnvironment), buffer, n);

  for (i = 0; i < numRoger; ++i) {
    if(socknew[i] > 0) {
      HandleDrawingRequests(&Rogers[i]);
    }
  }


  x_expose();
}

void draw_eval() {
  if (evalType == PONG) {
    draw_pong_score();
  }
  // extend for more types of evaluations
}

void draw_history(rogerID)
int rogerID;
{
  int h;
  // Definie delta_x and delta_y size given the environment
  float x_delta, y_delta;
  if (kEnvironment == DEVELOPMENTAL) {
    x_delta = XDELTA_DEV;
    y_delta = YDELTA_DEV;
  }
  else {
    x_delta = XDELTA;
    y_delta = YDELTA;
  }

  // draw history of all Cartesian arm postures
  XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);

  for (h = 0; h < history_ptr[rogerID]; ++h) {
    // draw Cartesian history of the mobile platform
    XFillRectangle(display, pixmap, gc,
      ConvertWorld2PixmapX(zoom, (history[rogerID][h].base_pos[0] - x_delta / 4.0), kEnvironment),
      ConvertWorld2PixmapY(zoom, (history[rogerID][h].base_pos[1] - y_delta / 4.0), kEnvironment),
      ConvertWorld2PixmapR(zoom, x_delta / 2.0, kEnvironment),
      ConvertWorld2PixmapR(zoom, y_delta / 2.0, kEnvironment));
  }
}

#define VMAG 0.5

// observations in world coordinates
void draw_estimate0(scale, est)
double scale;
Estimate est;
{
  draw_circle(ConvertWorld2PixmapX(ZOOM_SCALE, est.state[X], kEnvironment),
    ConvertWorld2PixmapY(ZOOM_SCALE, est.state[Y], kEnvironment),
    ConvertWorld2PixmapR(ZOOM_SCALE, R_BASE, kEnvironment), NOFILL);

  x_draw_line(BLUE, est.state[X], est.state[Y],
    (est.state[X] + VMAG*est.state[XDOT]),
    (est.state[Y] + VMAG*est.state[YDOT]));
}

#define FRAME_L 0.08

// observations in world coordinates
void draw_observation(obs)
Observation obs;
{
  double a, b, c, root[2];

  double dx, dy, mag, eigenvalue[2], eigenvector[2][2];
  double ref_w[4], ref_b[4], wTb[4][4], bTw[4][4];
  double theta, dx0, dy0, dx1, dy1;

  printf("inside draw_observation()\n");
  printf("x=%6.4lf y=%6.4lf \n\n", obs.pos[X], obs.pos[Y]);

  //printf("   %lf %lf\n", obs.cov[0][0], obs.cov[0][1]);
  //printf("   %lf %lf\n", obs.cov[1][0], obs.cov[1][1]);

  // DRAW THE CURRENT ESTIMATED STATE VARIABLES AND THEIR VELOCITIES
  //  draw_circle(ConvertWorld2PixmapX(zoom,est.state[X]), ConvertWorld2PixmapY(zoom,est.state[Y]),
  //            ConvertWorld2PixmapR(zoom,R_JOINT), FILL);

  // cov = [A  B] => det |JJt| = a(lambda)^2 + b(lambda) + c
  //       [B  C]
  a = 1.0;
  b = -(obs.cov[0][0] + obs.cov[1][1]);
  c = obs.cov[0][0] * obs.cov[1][1] - obs.cov[1][0] * obs.cov[0][1];

  root[0] = (-b + sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);
  root[1] = (-b - sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);

  if (fabs(obs.cov[1][0]) > 0.0001) {
    dy = 1.0;
    dx = -(obs.cov[1][1] - root[0]) / obs.cov[1][0];

    mag = sqrt(SQR(dx) + SQR(dy));
    //    eigenvalue[0] = scale*sqrt(root[0]);
    eigenvalue[0] = sqrt(root[0]);
    eigenvector[0][0] = dx / mag;
    eigenvector[0][1] = dy / mag;

    // the eigenvector for root 1
    //    eigenvalue[1] = scale*sqrt(root[1]);
    eigenvalue[1] = sqrt(root[1]);
    eigenvector[1][0] = -eigenvector[0][1];
    eigenvector[1][1] = eigenvector[0][0];
  }
  // when ball is directly in front of Roger:
  else {
    // set eigenvalue 0 to be the greater root
    if (fabs(root[0]) > fabs(root[1])) {
      eigenvalue[0] = sqrt(root[0]);
      eigenvalue[1] = sqrt(root[1]);
    }
    else {
      eigenvalue[0] = sqrt(root[1]);
      eigenvalue[1] = sqrt(root[0]);
    }

    // when cov item A > cov item C, Roger is facing either of the green
    // walls; when cov item A < cov item C, Roger is facing either of the
    // blue walls
    if (fabs(obs.cov[0][0]) > fabs(obs.cov[1][1])) {
      eigenvector[0][0] = 1.0;
      eigenvector[0][1] = 0.0;
      eigenvector[1][0] = -0.0;
      eigenvector[1][1] = 1.0;
    }
    else {
      eigenvector[0][0] = 0.0;
      eigenvector[0][1] = 1.0;
      eigenvector[1][0] = -1.0;
      eigenvector[1][1] = 0.0;
    }
  }

  // all observations will be displayed in green goal_color
  XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);

  // draw cross hair
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, (obs.pos[X] - (FRAME_L / 2.0)*eigenvector[0][X]), kEnvironment),
    ConvertWorld2PixmapY(zoom, (obs.pos[Y] - (FRAME_L / 2.0)*eigenvector[0][Y]), kEnvironment),
    ConvertWorld2PixmapX(zoom, (obs.pos[X] + (FRAME_L / 2.0)*eigenvector[0][X]), kEnvironment),
    ConvertWorld2PixmapY(zoom, (obs.pos[Y] + (FRAME_L / 2.0)*eigenvector[0][Y]), kEnvironment));
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, (obs.pos[X] - (FRAME_L / 2.0)*eigenvector[1][X]), kEnvironment),
    ConvertWorld2PixmapY(zoom, (obs.pos[Y] - (FRAME_L / 2.0)*eigenvector[1][Y]), kEnvironment),
    ConvertWorld2PixmapX(zoom, (obs.pos[X] + (FRAME_L / 2.0)*eigenvector[1][X]), kEnvironment),
    ConvertWorld2PixmapY(zoom, (obs.pos[Y] + (FRAME_L / 2.0)*eigenvector[1][Y]), kEnvironment));
  dx0 = eigenvalue[0] * eigenvector[0][X];
  dy0 = eigenvalue[0] * eigenvector[0][Y];

  //printf("observation cov:\n");
  //printf("\t%lf %lf\n\t%lf %lf\n", obs.cov[0][0], obs.cov[0][1], obs.cov[1][0], obs.cov[1][1]);

  for (theta = 0.0; theta <= 2 * M_PI; theta += M_PI / 20.0) {
    dx1 = (eigenvalue[0] * cos(theta))*eigenvector[0][X] +
      (eigenvalue[1] * sin(theta))*eigenvector[1][X];
    dy1 = (eigenvalue[0] * cos(theta))*eigenvector[0][Y] +
      (eigenvalue[1] * sin(theta))*eigenvector[1][Y];
    XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, (obs.pos[X] + dx0), kEnvironment),
      ConvertWorld2PixmapY(zoom, (obs.pos[Y] + dy0), kEnvironment),
      ConvertWorld2PixmapX(zoom, (obs.pos[X] + dx1), kEnvironment),
      ConvertWorld2PixmapY(zoom, (obs.pos[Y] + dy1), kEnvironment));
    dx0 = dx1;
    dy0 = dy1;
  }
}

// estimates in world coordinates
void draw_estimate(scale, est)
double scale;
Estimate est;
{
  double a, b, c, root[2];
  double dx, dy, mag, eigenvalue[2], eigenvector[2][2];
  double ref_w[2], ref_b[2], wTb[4][4], bTw[4][4];
  double theta, dx0, dy0, dx1, dy1;

  printf("inside draw_estimate()\n");
  printf("x=%6.4lf y=%6.4lf xdot=%6.4lf ydot=%6.4lf\n\n", est.state[X], est.state[Y], est.state[XDOT], est.state[YDOT]);

  //printf("   %lf %lf\n", est.cov[0][0], est.cov[0][1]);
  //printf("   %lf %lf\n", est.cov[1][0], est.cov[1][1]);

  // cov = [A  B] => det |JJt| = a(lambda)^2 + b(lambda) + c
  //       [B  C]
  a = 1.0;
  b = -(est.cov[0][0] + est.cov[1][1]);
  c = est.cov[0][0] * est.cov[1][1] - est.cov[1][0] * est.cov[0][1];

  root[0] = (-b + sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);
  root[1] = (-b - sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);

  if (fabs(est.cov[1][0]) > 0.000001) {
    dy = 1.0;
    dx = -(est.cov[1][1] - root[0]) / est.cov[1][0];

    mag = sqrt(SQR(dx) + SQR(dy));
    //      eigenvalue[0] = scale*0.0221*sqrt(root[0]);
    eigenvalue[0] = scale*sqrt(root[0]);
    eigenvector[0][0] = dx / mag;
    eigenvector[0][1] = dy / mag;

    // the eigenvector for root 1
    //      eigenvalue[1] = scale*0.0221*sqrt(root[1]);
    eigenvalue[1] = scale*sqrt(root[1]);
    eigenvector[1][0] = -eigenvector[0][1];
    eigenvector[1][1] = eigenvector[0][0];
  }
  else {
    // set eigenvalue[0] to be the greater root
    if (fabs(root[0]) > fabs(root[1])) {
      eigenvalue[0] = scale*sqrt(root[0]);
      eigenvalue[1] = scale*sqrt(root[1]);
    }
    else {
      eigenvalue[0] = scale*sqrt(root[1]);
      eigenvalue[1] = scale*sqrt(root[0]);
    }

    // when cov item A > cov item C, Roger is facing either of the green
    // walls; when cov item A < cov item C, Roger is facing either of the
    // blue walls
    if (fabs(est.cov[0][0]) > fabs(est.cov[1][1])) {
      eigenvector[0][0] = 1.0;
      eigenvector[0][1] = 0.0;
      eigenvector[1][0] = -0.0;
      eigenvector[1][1] = 1.0;
    }
    else {
      eigenvector[0][0] = 0.0;
      eigenvector[0][1] = 1.0;
      eigenvector[1][0] = -1.0;
      eigenvector[1][1] = 0.0;
    }
  }

  // all "estimates" will be displayed in blue
  XSetForeground(display, gc, world_colors[BLUE].display_color);


  // draw cross hair
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, (est.state[X] - (FRAME_L / 2.0)*eigenvector[0][X]), kEnvironment),
    ConvertWorld2PixmapY(zoom, (est.state[Y] - (FRAME_L / 2.0)*eigenvector[0][Y]), kEnvironment),
    ConvertWorld2PixmapX(zoom, (est.state[X] + (FRAME_L / 2.0)*eigenvector[0][X]), kEnvironment),
    ConvertWorld2PixmapY(zoom, (est.state[Y] + (FRAME_L / 2.0)*eigenvector[0][Y]), kEnvironment));
  XDrawLine(display, pixmap, gc,
    ConvertWorld2PixmapX(zoom, (est.state[X] - (FRAME_L / 2.0)*eigenvector[1][X]), kEnvironment),
    ConvertWorld2PixmapY(zoom, (est.state[Y] - (FRAME_L / 2.0)*eigenvector[1][Y]), kEnvironment),
    ConvertWorld2PixmapX(zoom, (est.state[X] + (FRAME_L / 2.0)*eigenvector[1][X]), kEnvironment),
    ConvertWorld2PixmapY(zoom, (est.state[Y] + (FRAME_L / 2.0)*eigenvector[1][Y]), kEnvironment));
  dx0 = eigenvalue[0] * eigenvector[0][X];
  dy0 = eigenvalue[0] * eigenvector[0][Y];

  //printf("estimate cov:\n");
  //printf("\t%lf %lf\n\t%lf %lf\n", est.cov[0][0], est.cov[0][1], est.cov[1][0], est.cov[1][1]);

  for (theta = 0.0; theta <= 2 * M_PI; theta += M_PI / 20.0) {
    dx1 = (eigenvalue[0] * cos(theta))*eigenvector[0][X] +
      (eigenvalue[1] * sin(theta))*eigenvector[1][X];
    dy1 = (eigenvalue[0] * cos(theta))*eigenvector[0][Y] +
      (eigenvalue[1] * sin(theta))*eigenvector[1][Y];
    XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, (est.state[X] + dx0), kEnvironment),
      ConvertWorld2PixmapY(zoom, (est.state[Y] + dy0), kEnvironment),
      ConvertWorld2PixmapX(zoom, (est.state[X] + dx1), kEnvironment),
      ConvertWorld2PixmapY(zoom, (est.state[Y] + dy1), kEnvironment));
    dx0 = dx1;
    dy0 = dy1;
  }
}

void draw_ellipse(est)
Estimate est;
{
  double m[2][2], a, b, c, root[2];
  double dx, dy, mag, eigenvalue[2], eigenvector[2][2];
  double theta, dx0, dy0, dx1, dy1;

  //  printf("observation time = %lf  time = %lf\n", est.t, simtime);

  //if ((est.t == simtime)) {

  // DRAW THE CURRENT ESTIMATED STATE VARIABLES AND THEIR VELOCITIES
  draw_circle(ConvertWorld2PixmapX(zoom, est.state[X], kEnvironment), ConvertWorld2PixmapY(zoom, est.state[Y], kEnvironment),
    ConvertWorld2PixmapR(zoom, R_JOINT, kEnvironment), FILL);

  m[0][0] = est.cov[0][0];
  m[0][1] = est.cov[0][1];
  m[1][0] = est.cov[1][0];
  m[1][1] = est.cov[1][1];

  // cov = [A  B] => det |JJt| = a(lambda)^2 + b(lambda) +c
  //       [B  C]
  a = 1.0;
  b = -(m[0][0] + m[1][1]);
  c = m[0][0] * m[1][1] - m[1][0] * m[0][1];

  root[0] = (-b + sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);
  root[1] = (-b - sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);

  // the eigenvector for root 0
  dy = 1.0;
  dx = -m[0][1] / (m[0][0] - root[0]);
  mag = sqrt(SQR(dx) + SQR(dy));
  eigenvalue[0] = sqrt(root[0]);
  eigenvector[0][0] = dx / mag;
  eigenvector[0][1] = dy / mag;

  // the eigenvector for root 1
  dy = 1.0;
  dx = -m[0][1] / (m[0][0] - root[1]);
  mag = sqrt(SQR(dx) + SQR(dy));
  eigenvalue[1] = sqrt(root[1]);
  eigenvector[1][0] = dx / mag;
  eigenvector[1][1] = dy / mag;

  dx0 = eigenvalue[0] * eigenvector[0][X];
  dy0 = eigenvalue[0] * eigenvector[0][Y];
  for (theta = M_PI / 20.0; theta < 2 * M_PI; theta += M_PI / 20.0) {
    dx1 = (eigenvalue[0] * cos(theta))*eigenvector[0][X] +
      (eigenvalue[1] * sin(theta))*eigenvector[1][X];
    dy1 = (eigenvalue[0] * cos(theta))*eigenvector[0][Y] +
      (eigenvalue[1] * sin(theta))*eigenvector[1][Y];
    XDrawLine(display, pixmap, gc, ConvertWorld2PixmapX(zoom, (est.state[X] + dx0), kEnvironment),
      ConvertWorld2PixmapY(zoom, (est.state[Y] + dy0), kEnvironment),
      ConvertWorld2PixmapX(zoom, (est.state[X] + dx1), kEnvironment),
      ConvertWorld2PixmapY(zoom, (est.state[Y] + dy1), kEnvironment));
    dx0 = dx1;
    dy0 = dy1;
  }
  // }
}

/* the streamline visualization is not yet implemented in this version */
// void draw_streamlines(roger, color)
// Robot* roger;
// int color;
// {
//   void sor();
//   double from[2], to[2];

//   int i, row, col, current_row, current_col, already_used[NBINS][NBINS], count;
//   double grad[2], oldx, oldy, x, y, mag, compute_gradient();
//   void mark_used(), draw_current_streamline();

//   /* make sure the harmonic function is fully converged */
//   sor(roger);
  
//   // initialize auxilliary structure for controlling the
//   // density of streamlines rendered
//   for (row=0; row<NBINS; ++row) {
//     for (col=0; col<NBINS; ++col) {
//       already_used[row][col] = FALSE;
//     }
//   }

//   XSetForeground(display, gc, world_colors[GREEN].display_color);

//   // if [col,row] is FREESPACE and at least one of its neighbors 
//   // is OBSTACLE (could be DIRICHLET and NEUMANN), then draw a streamline
//   for (row=1;row<(NBINS-1); ++row) {
//     for (col=1;col<(NBINS-1); ++col) {
//       if ((roger->world_map.occupancy_map[row][col] == FREESPACE) &&
//     ((roger->world_map.occupancy_map[row-1][col-1] == OBSTACLE) ||
//      (roger->world_map.occupancy_map[row-1][col] == OBSTACLE)   ||
//      (roger->world_map.occupancy_map[row-1][col+1] == OBSTACLE) ||
//      (roger->world_map.occupancy_map[row][col-1] == OBSTACLE)   ||
//      (roger->world_map.occupancy_map[row][col+1] == OBSTACLE)   ||
//      (roger->world_map.occupancy_map[row+1][col-1] == OBSTACLE) ||
//      (roger->world_map.occupancy_map[row+1][col] == OBSTACLE)   ||
//      (roger->world_map.occupancy_map[row+1][col+1] == OBSTACLE) ) &&
//     (!already_used[row][col]) ) {
    
//   /* follow a stream line */
//   oldx = MIN_X + (col+0.5)*XDELTA;
//   oldy = MAX_Y - (row+0.5)*YDELTA;

//   count = 0;
//   current_row = row;
//   current_col = col;
//   while ((roger->world_map.occupancy_map[current_row][current_col]!=GOAL)
//          && (mag=compute_gradient(oldx, oldy, roger, grad))
//          && (count++ < 500)) {
    
//     x = oldx - STEP*grad[X];
//     y = oldy - STEP*grad[Y];

//     XDrawLine(display, pixmap, gc, W2DX(zoom,oldx), W2DY(zoom,oldy),
//         W2DX(zoom,x), W2DY(zoom,y));
//     /*  circle((int)x,(int)y,2,background);  */
//     oldx=x; oldy=y;
//     current_row = (int)(y - MAX_Y)/YDELTA;
//     current_col = (int)(MIN_X - x)/XDELTA;
//   }

//   //  mark_used((col+1),(row+1),already_used); 
//   mark_used(row, col, already_used); 
//       }
//     }
//   }
//   //  draw_history();
//   x_expose();
// }
      


typedef struct _visible_object {
  double dx, dy;
  double radius;
  int color;
} VisibleObject;

void insertion_sort(vob, sort, num)
VisibleObject *vob;
int *sort, num;
{
  int i, j, temp;

  for (i = 0; i<num; i++) sort[i] = i;

  for (i = 1; i<num; i++) {
    j = i - 1;
    temp = sort[i];
    while ((j >= 0) && (sqrt(SQR(vob[sort[j]].dx) + SQR(vob[sort[j]].dy)) <=
      sqrt(SQR(vob[temp].dx) + SQR(vob[temp].dy)))) {
      sort[j + 1] = sort[j];
      j--;
    }
    sort[j + 1] = temp;
  }
}


void pinhole_camera(vob, i, rogerID)
VisibleObject vob;
int i;
int rogerID;
{
  int j, low_bin_index, high_bin_index, paint_image;
  double phi, alpha;

  phi = atan2(vob.dy, vob.dx) - RogersBodys[rogerID].eyes[i].theta; // eye frame heading eye->object


    //  printf("      phi for eye %d = %6.4lf\n", i, phi);
  alpha = atan2(vob.radius, sqrt(SQR(vob.dx)+SQR(vob.dy) - SQR(vob.radius)));

  paint_image = FALSE;

  if (fabs(phi + alpha) < FOV) { /* CCW edge of the feature projects in FOV */
    high_bin_index = (int) (NPIXELS / 2.0 * (1.0 + tan(phi + alpha)));
    paint_image = TRUE;
  }
  else high_bin_index = 127;

  if (fabs(phi - alpha) < FOV) { /* CW edge of the feature projects in FOV */
    low_bin_index = (int) (NPIXELS / 2.0 * (1.0 + tan(phi - alpha)));
    paint_image = TRUE;
  }
  else low_bin_index = 0;

  // low_bin_index = MAX(low_bin_index,0);
  // high_bin_index = MIN(high_bin_index,(NPIXELS-1));

  if (paint_image) {
    for (j = low_bin_index; j <= high_bin_index; j++) {
      RogersBodys[rogerID].eyes[i].image[j] = vob.color;
    }
  }
}


void make_images(rogerID)
int rogerID;
{
  int i, j, eye, o_index, k; // intensity[3];
  double x, y;
  double p_b[4], p_w[4], bTw[4][4];


  // Define delta_x, delta_y, and r_obstacle size given the environment
  float x_delta, y_delta, r_obstacle;
  float max_x, max_y, min_x, min_y;
  if (kEnvironment == DEVELOPMENTAL) {
    x_delta = XDELTA_DEV;
    y_delta = YDELTA_DEV;
    r_obstacle = R_OBSTACLE_DEV;
    max_x = MAX_X_DEV;
    min_x = MIN_X_DEV;
    max_y = MAX_Y_DEV;
    min_y = MIN_Y_DEV;
  }
  else {
    x_delta = XDELTA;
    y_delta = YDELTA;
    r_obstacle = R_OBSTACLE;
    max_x = MAX_X;
    min_x = MIN_X;
    max_y = MAX_Y;
    min_y = MIN_Y;
  }


  int sort[NBINS*NBINS + (MaxRobotNum * N_VISIBLEOBJ_ROBOT) + MaxToyNum];
  VisibleObject vobject[NBINS*NBINS + (MaxRobotNum * N_VISIBLEOBJ_ROBOT) + MaxToyNum];


  /* initialize image white */
  // make sure eye's images keep changing when roger is moving
  for (i = 0; i < NPIXELS; i++) {
    RogersBodys[rogerID].eyes[LEFT].image[i] = RogersBodys[rogerID].eyes[RIGHT].image[i] = 100;
  }

    
  for (eye = LEFT; eye <= RIGHT; ++eye) {
    // Visual objects counter
    o_index = 0;

    // Go through visual body parts of all robots first (arm1, arm2)
    for (k = 0; k < numRoger; ++k) {
      //    first 3 elements in the range array are the hands and the ball
      /* LEFT_ARM - in body frame */
      sim_fwd_kinematics(LEFT, RogersBodys[k].arms[LEFT][1].theta, RogersBodys[k].arms[LEFT][2].theta, &x, &y);
      // convert to eye coordinates
      vobject[o_index].dx = x - RogersBodys[rogerID].eyes[eye].position[X];
      vobject[o_index].dy = y - RogersBodys[rogerID].eyes[eye].position[Y];
      vobject[o_index].radius = R_JOINT;
      vobject[o_index++].color = ARM_COLOR;

      /* RIGHT_ARM - in body frame */
      sim_fwd_kinematics(RIGHT, RogersBodys[k].arms[RIGHT][1].theta, RogersBodys[k].arms[RIGHT][2].theta, &x, &y);
      vobject[o_index].dx = x - RogersBodys[rogerID].eyes[eye].position[X];
      vobject[o_index].dy = y - RogersBodys[rogerID].eyes[eye].position[Y];
      vobject[o_index].radius = R_JOINT;
      vobject[o_index++].color = ARM_COLOR;
    }


    /* OBJECT */
    SIMinv_transform(RogersBodys[rogerID].mobile_base.wTb, bTw);
    p_w[0] = toy.position[X]; p_w[1] = toy.position[Y];
    p_w[2] = 0.0; p_w[3] = 1.0;
    SIMmatXvec(bTw, p_w, p_b);
    vobject[o_index].dx = p_b[X] - RogersBodys[rogerID].eyes[eye].position[X];
    vobject[o_index].dy = p_b[Y] - RogersBodys[rogerID].eyes[eye].position[Y];
    vobject[o_index].radius = toy.Rsphere;
    vobject[o_index++].color = OBJECT_COLOR;


    // after the first three, the rest are colored obstacles in the occupancy
    // grid
    // points at the next empty element of the range array
    for (i = 0; i<NBINS; ++i) {
      y = max_y - (i + 0.5)*y_delta;
      for (j = 0; j<NBINS; ++j) {
        if (Rogers[0].world_map.occupancy_map[i][j] == OBSTACLE) {
          p_w[0] = min_x + (j + 0.5)*x_delta; p_w[1] = y;
          p_w[2] = 0.0; p_w[3] = 1.0;
          SIMmatXvec(bTw, p_w, p_b);
          vobject[o_index].dx = p_b[X] - RogersBodys[rogerID].eyes[eye].position[X];
          vobject[o_index].dy = p_b[Y] - RogersBodys[rogerID].eyes[eye].position[Y];
          vobject[o_index].radius = r_obstacle;
          vobject[o_index++].color = Rogers[0].world_map.color_map[i][j];
        }
      }
    }

    // printf("o_index: %d\n", o_index);
    insertion_sort(vobject, sort, o_index);
    for (i = 0; i<o_index; ++i) {
      pinhole_camera(vobject[sort[i]], eye, rogerID);
    }
  }
}


void initialize_simulator(rst)
int rst;
{
  void initialize_inertial_objects();
  void initialize_roger(); void intialize_history();
  int i;

  if (rst) {
    initialize_roger();

    for (i = 0; i < numRoger; ++i) {
      Rogers[i].environment = kEnvironment;
      Rogers[i].graphics.zoom = zoom;
    }

    initialize_inertial_objects();

    // all params and GUI position for the 5 objects in the toybox
    // initialize_toybox();

    /************************************************************************/
    // initialize the volatile elements (afferents and efferents)
    // of the applications interface for user control code
    write_interface(rst);
  }
}

void intialize_history() {
  int i;
  for (i = 0; i < MaxRobotNum; ++i) {
    history_ptr[i] = 0;
  }
}

void initialize_roger()
{
  int i, j, k, l;

  // Developmental environment *************************
  if (kEnvironment == DEVELOPMENTAL) {
  /************************************************************************/
    // MOBILE BASE
    for (i = 0; i<4; ++i) {
      for (j = 0; j<4; ++j) {
        RogersBodys[0].mobile_base.wTb[i][j] = mobile_base_home.wTb[i][j];
      }
    }

    RogersBodys[0].mobile_base.x = mobile_base_home.x;
    RogersBodys[0].mobile_base.x_dot = mobile_base_home.x_dot;
    RogersBodys[0].mobile_base.y = mobile_base_home.y;
    RogersBodys[0].mobile_base.y_dot = mobile_base_home.y_dot;
    RogersBodys[0].mobile_base.theta = mobile_base_home.theta;
    RogersBodys[0].mobile_base.theta_dot = mobile_base_home.theta_dot;
    for (i = 0; i<2; ++i) {
      RogersBodys[0].mobile_base.wheel_torque[i] = mobile_base_home.wheel_torque[i];
    }
    RogersBodys[0].mobile_base.contact_theta = mobile_base_home.contact_theta;
    for (i = 0; i<2; ++i) {
      RogersBodys[0].mobile_base.extForce[i] = mobile_base_home.extForce[i];
    }

    /************************************************************************/
    // LEFT AND RIGHT EYE
    for (i = LEFT; i <= RIGHT; ++i) {
      RogersBodys[0].eyes[i].position[X] = eyes_home[i].position[X];
      RogersBodys[0].eyes[i].position[Y] = eyes_home[i].position[Y];
      RogersBodys[0].eyes[i].theta = eyes_home[i].theta;
      RogersBodys[0].eyes[i].theta_dot = eyes_home[i].theta_dot;
      for (j = 0; j<NPIXELS; ++j) {
        RogersBodys[0].eyes[i].image[j] = eyes_home[i].image[j];
      }
      RogersBodys[0].eyes[i].torque = eyes_home[i].torque;
    }

    /************************************************************************/
    // LEFT AND RIGHT ARM
    for (i = 0; i<NARMS; ++i) {
      for (j = 0; j<NARM_FRAMES; ++j) {
        for (k = 0; k<4; ++k) {
          for (l = 0; l<4; ++l) {
            RogersBodys[0].arms[i][j].iTj[k][l] = arms_home[i][j].iTj[k][l];
          }
        }
        RogersBodys[0].arms[i][j].dof_type = arms_home[i][j].dof_type;
        RogersBodys[0].arms[i][j].axis = arms_home[i][j].axis;
        RogersBodys[0].arms[i][j].theta = arms_home[i][j].theta;
        RogersBodys[0].arms[i][j].theta_dot = arms_home[i][j].theta_dot;
        RogersBodys[0].arms[i][j].torque = arms_home[i][j].torque;
        RogersBodys[0].arms[i][j].extForce[0] = arms_home[i][j].extForce[0];
        RogersBodys[0].arms[i][j].extForce[1] = arms_home[i][j].extForce[1];
      }
    }

  // Arena environment ****************************************************
  } else {

    // *******************************************************************
    // Roger#0 ***********************************************************
    // *******************************************************************
      /************************************************************************/
    // MOBILE BASE
    for (i = 0; i<4; ++i) {
      for (j = 0; j<4; ++j) {
        RogersBodys[0].mobile_base.wTb[i][j] = mobile_base_home_1.wTb[i][j];
      }
    }

    RogersBodys[0].mobile_base.x = mobile_base_home_1.x;
    RogersBodys[0].mobile_base.x_dot = mobile_base_home_1.x_dot;
    RogersBodys[0].mobile_base.y = mobile_base_home_1.y;
    RogersBodys[0].mobile_base.y_dot = mobile_base_home_1.y_dot;
    RogersBodys[0].mobile_base.theta = mobile_base_home_1.theta;
    RogersBodys[0].mobile_base.theta_dot = mobile_base_home_1.theta_dot;
    for (i = 0; i<2; ++i) {
      RogersBodys[0].mobile_base.wheel_torque[i] = mobile_base_home_1.wheel_torque[i];
    }
    RogersBodys[0].mobile_base.contact_theta = mobile_base_home_1.contact_theta;
    for (i = 0; i<2; ++i) {
      RogersBodys[0].mobile_base.extForce[i] = mobile_base_home_1.extForce[i];
    }

    /************************************************************************/
    // LEFT AND RIGHT EYE
    for (i = LEFT; i <= RIGHT; ++i) {
      RogersBodys[0].eyes[i].position[X] = eyes_home[i].position[X];
      RogersBodys[0].eyes[i].position[Y] = eyes_home[i].position[Y];
      RogersBodys[0].eyes[i].theta = eyes_home[i].theta;
      RogersBodys[0].eyes[i].theta_dot = eyes_home[i].theta_dot;
      for (j = 0; j<NPIXELS; ++j) {
        RogersBodys[0].eyes[i].image[j] = eyes_home[i].image[j];
      }
      RogersBodys[0].eyes[i].torque = eyes_home[i].torque;
    }

    /************************************************************************/
    // LEFT AND RIGHT ARM
    for (i = 0; i<NARMS; ++i) {
      for (j = 0; j<NARM_FRAMES; ++j) {
        for (k = 0; k<4; ++k) {
          for (l = 0; l<4; ++l) {
            RogersBodys[0].arms[i][j].iTj[k][l] = arms_home_1[i][j].iTj[k][l];
          }
        }
        RogersBodys[0].arms[i][j].dof_type = arms_home_1[i][j].dof_type;
        RogersBodys[0].arms[i][j].axis = arms_home_1[i][j].axis;
        RogersBodys[0].arms[i][j].theta = arms_home_1[i][j].theta;
        RogersBodys[0].arms[i][j].theta_dot = arms_home_1[i][j].theta_dot;
        RogersBodys[0].arms[i][j].torque = arms_home_1[i][j].torque;
        RogersBodys[0].arms[i][j].extForce[0] = arms_home_1[i][j].extForce[0];
        RogersBodys[0].arms[i][j].extForce[1] = arms_home_1[i][j].extForce[1];
      }
    }

    // *******************************************************************
    // Roger#1 ***********************************************************
    // *******************************************************************
      /************************************************************************/
    // MOBILE BASE
    for (i = 0; i<4; ++i) {
      for (j = 0; j<4; ++j) {
        RogersBodys[1].mobile_base.wTb[i][j] = mobile_base_home_2.wTb[i][j];
      }
    }

    RogersBodys[1].mobile_base.x = mobile_base_home_2.x;
    RogersBodys[1].mobile_base.x_dot = mobile_base_home_2.x_dot;
    RogersBodys[1].mobile_base.y = mobile_base_home_2.y;
    RogersBodys[1].mobile_base.y_dot = mobile_base_home_2.y_dot;
    //printf("orig MBHT: %lf", mobile_base_home_2.theta);
    //RogersBodys[1].mobile_base.theta = mobile_base_home_2.theta;
    //RogersBodys[1].mobile_base.theta = -mobile_base_home_2.theta;
    RogersBodys[1].mobile_base.theta = -M_PI;
    //printf("new MBHT: %lf", RogersBodys[1].mobile_base.theta);
    RogersBodys[1].mobile_base.theta_dot = mobile_base_home_2.theta_dot;
    for (i = 0; i<2; ++i) {
      RogersBodys[1].mobile_base.wheel_torque[i] = mobile_base_home_2.wheel_torque[i];
    }
    RogersBodys[1].mobile_base.contact_theta = mobile_base_home_2.contact_theta;
    for (i = 0; i<2; ++i) {
      RogersBodys[1].mobile_base.extForce[i] = mobile_base_home_2.extForce[i];
    }

    /************************************************************************/
    // LEFT AND RIGHT EYE
    for (i = LEFT; i <= RIGHT; ++i) {
      RogersBodys[1].eyes[i].position[X] = eyes_home[i].position[X];
      RogersBodys[1].eyes[i].position[Y] = eyes_home[i].position[Y];
      RogersBodys[1].eyes[i].theta = eyes_home[i].theta;
      RogersBodys[1].eyes[i].theta_dot = eyes_home[i].theta_dot;
      for (j = 0; j<NPIXELS; ++j) {
        RogersBodys[1].eyes[i].image[j] = eyes_home[i].image[j];
      }
      RogersBodys[1].eyes[i].torque = eyes_home[i].torque;
    }

    /************************************************************************/
    // LEFT AND RIGHT ARM
    for (i = 0; i<NARMS; ++i) {
      for (j = 0; j<NARM_FRAMES; ++j) {
        for (k = 0; k<4; ++k) {
          for (l = 0; l<4; ++l) {
            RogersBodys[1].arms[i][j].iTj[k][l] = arms_home_2[i][j].iTj[k][l];
          }
        }
        RogersBodys[1].arms[i][j].dof_type = arms_home_2[i][j].dof_type;
        RogersBodys[1].arms[i][j].axis = arms_home_2[i][j].axis;
        RogersBodys[1].arms[i][j].theta = arms_home_2[i][j].theta;
        RogersBodys[1].arms[i][j].theta_dot = arms_home_2[i][j].theta_dot;
        RogersBodys[1].arms[i][j].torque = arms_home_2[i][j].torque;
        RogersBodys[1].arms[i][j].extForce[0] = arms_home_2[i][j].extForce[0];
        RogersBodys[1].arms[i][j].extForce[1] = arms_home_2[i][j].extForce[1];
      }
    }


    // *******************************************************************
    // Roger#2 ***********************************************************
    // *******************************************************************
      /************************************************************************/
    // MOBILE BASE
    for (i = 0; i<4; ++i) {
      for (j = 0; j<4; ++j) {
        RogersBodys[2].mobile_base.wTb[i][j] = mobile_base_home_3.wTb[i][j];
      }
    }

    RogersBodys[2].mobile_base.x = mobile_base_home_3.x;
    RogersBodys[2].mobile_base.x_dot = mobile_base_home_3.x_dot;
    RogersBodys[2].mobile_base.y = mobile_base_home_3.y;
    RogersBodys[2].mobile_base.y_dot = mobile_base_home_3.y_dot;
    RogersBodys[2].mobile_base.theta = mobile_base_home_3.theta;
    RogersBodys[2].mobile_base.theta_dot = mobile_base_home_3.theta_dot;
    for (i = 0; i<2; ++i) {
      RogersBodys[2].mobile_base.wheel_torque[i] = mobile_base_home_3.wheel_torque[i];
    }
    RogersBodys[2].mobile_base.contact_theta = mobile_base_home_3.contact_theta;
    for (i = 0; i<2; ++i) {
      RogersBodys[2].mobile_base.extForce[i] = mobile_base_home_3.extForce[i];
    }

    /************************************************************************/
    // LEFT AND RIGHT EYE
    for (i = LEFT; i <= RIGHT; ++i) {
      RogersBodys[2].eyes[i].position[X] = eyes_home[i].position[X];
      RogersBodys[2].eyes[i].position[Y] = eyes_home[i].position[Y];
      RogersBodys[2].eyes[i].theta = eyes_home[i].theta;
      RogersBodys[2].eyes[i].theta_dot = eyes_home[i].theta_dot;
      for (j = 0; j<NPIXELS; ++j) {
        RogersBodys[2].eyes[i].image[j] = eyes_home[i].image[j];
      }
      RogersBodys[2].eyes[i].torque = eyes_home[i].torque;
    }

    /************************************************************************/
    // LEFT AND RIGHT ARM
    for (i = 0; i<NARMS; ++i) {
      for (j = 0; j<NARM_FRAMES; ++j) {
        for (k = 0; k<4; ++k) {
          for (l = 0; l<4; ++l) {
            RogersBodys[2].arms[i][j].iTj[k][l] = arms_home_3[i][j].iTj[k][l];
          }
        }
        RogersBodys[2].arms[i][j].dof_type = arms_home_3[i][j].dof_type;
        RogersBodys[2].arms[i][j].axis = arms_home_3[i][j].axis;
        RogersBodys[2].arms[i][j].theta = arms_home_3[i][j].theta;
        RogersBodys[2].arms[i][j].theta_dot = arms_home_3[i][j].theta_dot;
        RogersBodys[2].arms[i][j].torque = arms_home_3[i][j].torque;
        RogersBodys[2].arms[i][j].extForce[0] = arms_home_3[i][j].extForce[0];
        RogersBodys[2].arms[i][j].extForce[1] = arms_home_3[i][j].extForce[1];
      }
    }

  }
}

// define geometrical and inertial parameters for the NBODY dynamical system
void initialize_inertial_objects()
{
  double pb[4], pw[4]; // homogeneous position vectors in base and world coords
  double vb[4], vw[4]; // homogeneous velocity vectors in base and world coords
  double x, y, J[2][2];
  int i;

  void sim_fwd_kinematics(), sim_arm_Jacobian();

  // initialize an array "PolyBall objects[NBODY]" that support computing 
  //    collision forces between "bodies" which are rigid body arrangements
  //    of elastic circular objects moving in the Cartesian world plane
  //  typedef struct _polyball {
  //    int id;                // CIRCLE || TRIANGLE
  //    int N;                 // a  composite of N elastic spheres
  //    double Rsphere;        // the radius of the elastic spheres
  //    double radius;         // distance from sphere center to body frame
  //    double mass;           // total mass of polyball
  //    double total_moment;   // moment of inertia (mass*SQR(radius) )
  //    double position[3];    // position (x,y,theta) of the object
  //    double velocity[3];    // velocity of the object
  //    double net_extForce[3]; // from collisions with other objects
  //  } PolyBall;

  int base_counter = 0;
  for (i = 0; i < numRoger; ++i) {
    base_counter = i * N_BODY_ROBOT;

    /****************************** 0: BASE ************************************/
    objects_total[base_counter + BASE].N = 1;
    objects_total[base_counter + BASE].Rsphere = R_BASE;
    objects_total[base_counter + BASE].radius = 0.0;
    objects_total[base_counter + BASE].mass = M_BASE;
    objects_total[base_counter + BASE].moi = I_BASE;

    objects_total[base_counter + BASE].position[X] = RogersBodys[i].mobile_base.x;
    objects_total[base_counter + BASE].position[Y] = RogersBodys[i].mobile_base.y;
    objects_total[base_counter + BASE].position[THETA] = RogersBodys[i].mobile_base.theta;

    objects_total[base_counter + BASE].velocity[X] = RogersBodys[i].mobile_base.x_dot;
    objects_total[base_counter + BASE].velocity[Y] = RogersBodys[i].mobile_base.y_dot;
    objects_total[base_counter + BASE].velocity[THETA] = RogersBodys[i].mobile_base.theta_dot;

    objects_total[base_counter + BASE].net_extForce[X] = objects_total[base_counter + BASE].net_extForce[Y] =
      objects_total[base_counter + BASE].net_extForce[THETA] = 0.0;

    /************************** 1: LEFT ARM ************************************/
    // left hand position in world coordinates
    sim_fwd_kinematics(LEFT, RogersBodys[i].arms[LEFT][1].theta, RogersBodys[i].arms[LEFT][2].theta, &x, &y);
    pb[0] = x; pb[1] = y; pb[2] = 0.0; pb[3] = 1.0;
    matrix_mult(4, 4, RogersBodys[i].mobile_base.wTb, 1, pb, pw);

    // left hand velocity relative to base written in base coordinates
    //          (figure 2 in RogerDynamics document defines frames)
    // ^w(v)_10 = wRb[ ^b(v)_7 +  J_arm theta_dot_arm ]
    //    = ^w(v)_b + wRb[ -ARM_OFFSET omega_0 xhat_b + J_arm theta_dot_arm ]
    sim_arm_Jacobian(RogersBodys[i].arms[LEFT][1].theta, RogersBodys[i].arms[LEFT][2].theta, J);
    vb[X] = J[0][0] * RogersBodys[i].arms[LEFT][1].theta_dot +
      J[0][1] * RogersBodys[i].arms[LEFT][2].theta_dot - ARM_OFFSET*RogersBodys[i].mobile_base.theta_dot;
    vb[Y] = J[1][0] * RogersBodys[i].arms[LEFT][1].theta_dot + J[1][1] * RogersBodys[i].arms[LEFT][2].theta_dot;
    vb[2] = 0.0;
    vb[3] = 0.0; // homogeneous vector

    matrix_mult(4, 4, RogersBodys[i].mobile_base.wTb, 1, vb, vw);
    //  v[1][X] = base->x_dot + v_w[X];
    //  v[1][Y] = base->y_dot + v_w[Y];
    //  R[1] = R_TACTILE;

    objects_total[base_counter + ARM1].N = 1;
    objects_total[base_counter + ARM1].Rsphere = R_TACTILE;
    objects_total[base_counter + ARM1].radius = 0.0;
    objects_total[base_counter + ARM1].mass = M_ARM1;
    objects_total[base_counter + ARM1].moi = I_ARM1;

    objects_total[base_counter + ARM1].position[X] = pw[X];
    objects_total[base_counter + ARM1].position[Y] = pw[Y];
    objects_total[base_counter + ARM1].position[THETA] = 0.0;
    // hand orientation is not relevant to system dynamics

    objects_total[base_counter + ARM1].velocity[X] = RogersBodys[i].mobile_base.x_dot + vw[X];
    objects_total[base_counter + ARM1].velocity[Y] = RogersBodys[i].mobile_base.y_dot + vw[Y];
    objects_total[base_counter + ARM1].velocity[THETA] = 0.0;
    // angular acceleration in the hand is not relevant to system dynamics

    objects_total[base_counter + ARM1].net_extForce[X] = objects_total[base_counter + ARM1].net_extForce[Y] =
      objects_total[base_counter + ARM1].net_extForce[THETA] = 0.0;

    /************************** 2: RIGHT ARM ************************************/
    // right hand position in world coordinates
    sim_fwd_kinematics(RIGHT, RogersBodys[i].arms[RIGHT][1].theta, RogersBodys[i].arms[RIGHT][2].theta, &x, &y);
    pb[0] = x; pb[1] = y; pb[2] = 0.0; pb[3] = 1.0;
    matrix_mult(4, 4, RogersBodys[i].mobile_base.wTb, 1, pb, pw);

    // left hand velocity relative to base written in base coordinates
    //          (figure 2 in RogerDynamics document defines frames)
    // ^w(v)_10 = wRb[ ^b(v)_7 +  J_arm theta_dot_arm ]
    //    = ^w(v)_b + wRb[ -ARM_OFFSET omega_0 xhat_b + J_arm theta_dot_arm ]
    sim_arm_Jacobian(RogersBodys[i].arms[RIGHT][1].theta, RogersBodys[i].arms[RIGHT][2].theta, J);
    vb[X] = J[0][0] * RogersBodys[i].arms[RIGHT][1].theta_dot +
      J[0][1] * RogersBodys[i].arms[RIGHT][2].theta_dot - ARM_OFFSET*RogersBodys[i].mobile_base.theta_dot;
    vb[Y] = J[1][0] * RogersBodys[i].arms[RIGHT][1].theta_dot + J[1][1] * RogersBodys[i].arms[RIGHT][2].theta_dot;
    vb[2] = 0.0;
    vb[3] = 0.0; // homogeneous vector

    matrix_mult(4, 4, RogersBodys[i].mobile_base.wTb, 1, vb, vw);

    objects_total[base_counter + ARM2].N = 1;
    objects_total[base_counter + ARM2].Rsphere = R_TACTILE;
    objects_total[base_counter + ARM2].radius = 0.0;
    objects_total[base_counter + ARM2].mass = M_ARM1;
    objects_total[base_counter + ARM2].moi = I_ARM1;

    objects_total[base_counter + ARM2].position[X] = pw[X];
    objects_total[base_counter + ARM2].position[Y] = pw[Y];
    objects_total[base_counter + ARM2].position[THETA] = 0.0;
    // hand orientation not relevant to system dynamics

    objects_total[base_counter + ARM2].velocity[X] = RogersBodys[i].mobile_base.x_dot + vw[X];
    objects_total[base_counter + ARM2].velocity[Y] = RogersBodys[i].mobile_base.y_dot + vw[Y];
    objects_total[base_counter + ARM2].velocity[THETA] = 0.0;
    // angular acceleration of hand is not relevant to system dynamics

    objects_total[base_counter +ARM2].net_extForce[X] = objects_total[base_counter + ARM2].net_extForce[Y] =
    objects_total[base_counter + ARM2].net_extForce[THETA] = 0.0;

  }
  base_counter = base_counter + N_BODY_ROBOT;

  /****************** 3: TOY OBJECT - CIRCLE || TRIANGLE *********************/
  objects_total[base_counter].id = toy.id = toy_home.id;
  objects_total[base_counter].N = toy.N = toy_home.N;
  objects_total[base_counter].Rsphere = toy.Rsphere = toy_home.Rsphere;
  objects_total[base_counter].radius = toy.radius = toy_home.radius;
  objects_total[base_counter].mass = toy.mass = toy_home.mass;
  objects_total[base_counter].moi = toy.moi = toy_home.moi;

  objects_total[base_counter].position[X] = toy.position[X] = toy_home.position[X];
  objects_total[base_counter].position[Y] = toy.position[Y] = toy_home.position[Y];
  objects_total[base_counter].position[THETA] = toy.position[THETA] = toy_home.position[THETA];

  objects_total[base_counter].velocity[X] = toy.velocity[X] = toy_home.velocity[X];
  objects_total[base_counter].velocity[Y] = toy.velocity[Y] = toy_home.velocity[Y];
  objects_total[base_counter].velocity[THETA] = toy.velocity[THETA] = toy_home.velocity[THETA];

  objects_total[base_counter].net_extForce[X] = objects_total[base_counter].net_extForce[Y] =
  objects_total[base_counter].net_extForce[THETA] = 0.0;

}

//check if input is in cartesian space area
int isCartesianInput(x_input, y_input, environment, x, y) 
double x_input, y_input;
int environment;
double *x, *y;
{
  if (environment == ARENA) {
    if (x_input<(MIN_X + XDELTA) || x_input>(MAX_X - XDELTA) || y_input<(MIN_Y + YDELTA) || y_input>(MAX_Y- YDELTA) ) {
      //printf("Location out of bounds!!!\n");
      return FALSE;
    }
  } else {
    if (x_input<(MIN_X_DEV + XDELTA_DEV) || x_input>(MAX_X_DEV - XDELTA_DEV) || y_input<(MIN_Y_DEV + YDELTA_DEV) || y_input>(MAX_Y_DEV - YDELTA_DEV) ) {
      //printf("Location out of bounds!!!\n");
      return FALSE;
    }
  }
  
  *x = x_input;
  *y = y_input;

  return TRUE;
}

// Handles drawing requests that are made on the project side and 
// sent to the simulator
void HandleDrawingRequests(roger)
Robot* roger;
{
  int i, j;

  if (draw_visual) {
    // Address draw_observation() requests
    for (i = 0; i < roger->drawing_request.obs_number; i++) {
      draw_observation(roger->drawing_request.obs_args[i].obs);
    }

    // Address draw_estimate() requests
    for (i = 0; i < roger->drawing_request.est_number; i++) {
      draw_estimate(roger->drawing_request.est_args[i].scale, roger->drawing_request.est_args[i].est);
    }

    // Address draw_history() requests
    if (roger->drawing_request.draw_history == TRUE) {
      // Only available in Developmental environment
      if(kEnvironment == DEVELOPMENTAL) {
        draw_history(0);
      }
    }
    

    // Address draw_steamline() requests
    if (roger->drawing_request.draw_streamline == TRUE) {
      // TODO: Modify the draw_streamline() in the project side
      // to send a list of line segment endpoints along with the
      // drawing_request data structure and draw them here

    }
  }


  // Reset requests
  roger->drawing_request.obs_number = 0;
  roger->drawing_request.est_number = 0;
  roger->drawing_request.draw_history = FALSE;
  roger->drawing_request.draw_streamline = FALSE;

}


// Handles map editing and bal placement interactions through the GUI
void HandleGuiInteraction(roger)
Robot * roger;
{
  int xbin, ybin;
  //cartesian coordinates
  double x, y;
  
  if (kEnvironment == DEVELOPMENTAL) {
      if (roger->button_event) {
      switch(roger->input_mode) {

        case BALL_INPUT:
          //check if inputs are valid
          if (isCartesianInput(roger->button_reference[X],
             roger->button_reference[Y], kEnvironment, &x, &y) == FALSE) {
            break;
          } else {
            if (roger->button_event == LEFT_BUTTON) {
              place_object(x,y,CIRCLE);
            } else if (roger->button_event == RIGHT_BUTTON) {
              place_object(x,y,TRIANGLE);
            }
            break;
          }

        case MAP_INPUT:
          if (isCartesianInput(roger->button_reference[X],
                 roger->button_reference[Y], kEnvironment, &x, &y) == FALSE)
          break;

          //      int xbin, ybin; already defined above
          printf("Map editor input - x: %4.3f, y: %4.3f - button: %d\n",
           x, y, roger->button_event);

          if (kEnvironment == ARENA) {
            xbin = (x - MIN_X) / XDELTA;
            ybin = NBINS - (y - MIN_Y) / YDELTA;
          } else {
            xbin = (x - MIN_X_DEV) / XDELTA_DEV;
            ybin = NBINS - (y - MIN_Y_DEV) / YDELTA_DEV;
          }


          if ((xbin<0) || (xbin>(NBINS-1)) || (ybin<0) || (ybin>(NBINS-1))) {
          printf("Out of the boundary!!!\n");
              }
          else {
            if (roger->button_event==LEFT_BUTTON) {// obstacles in Cartesian space
              if (roger->world_map.occupancy_map[ybin][xbin] == OBSTACLE) {
                printf("deleting an obstacle xbin=%d  ybin=%d\n", xbin, ybin);
                fflush(stdout);
                roger->world_map.occupancy_map[ybin][xbin] = FREESPACE;
                //      delete_bin_bumper(xbin,ybin);
              }
              else if (roger->world_map.occupancy_map[ybin][xbin] == FREESPACE) {
                printf("inserting an obstacle xbin=%d  ybin=%d\n", xbin, ybin);
                fflush(stdout);
                roger->world_map.occupancy_map[ybin][xbin] = OBSTACLE;
                roger->world_map.potential_map[ybin][xbin] = 1.0;
                roger->world_map.color_map[ybin][xbin] = LIGHTBLUE;
              }
            }
            else if (roger->button_event == MIDDLE_BUTTON) { }
            else if (roger->button_event == RIGHT_BUTTON) {
              if (roger->world_map.occupancy_map[ybin][xbin] == GOAL) {
                printf("deleting an goal xbin=%d  ybin=%d\n", xbin, ybin);
                fflush(stdout);
                roger->world_map.occupancy_map[ybin][xbin] = FREESPACE;
              }
              else if (roger->world_map.occupancy_map[ybin][xbin] == FREESPACE) {
                printf("inserting an goal xbin=%d  ybin=%d\n", xbin, ybin);
                fflush(stdout);
                roger->world_map.occupancy_map[ybin][xbin] = GOAL;
                roger->world_map.potential_map[ybin][xbin] = 0.0;
                }
              }

          }
        
        break;
        default:
        break;
      }
    }

  } else {
    if (roger->button_event) {
      switch(roger->input_mode) {

        case BALL_INPUT_ARENA:
          //check if inputs are valid
          if (isCartesianInput(roger->button_reference[X],
             roger->button_reference[Y], kEnvironment, &x, &y) == FALSE) {
            break;
          } else {
            if (roger->button_event == LEFT_BUTTON) {
              place_object(x,y,CIRCLE);
            } else if (roger->button_event == RIGHT_BUTTON) {
              place_object(x,y,TRIANGLE);
            }
            break;
          }

        case MAP_INPUT_ARENA:
          if (isCartesianInput(roger->button_reference[X],
                 roger->button_reference[Y], kEnvironment, &x, &y) == FALSE)
          break;

          //      int xbin, ybin; already defined above
          printf("Map editor input - x: %4.3f, y: %4.3f - button: %d\n",
           x, y, roger->button_event);

          if (kEnvironment == ARENA) {
            xbin = (x - MIN_X) / XDELTA;
            ybin = NBINS - (y - MIN_Y) / YDELTA;
          } else {
            xbin = (x - MIN_X_DEV) / XDELTA_DEV;
            ybin = NBINS - (y - MIN_Y_DEV) / YDELTA_DEV;
          }


          if ((xbin<0) || (xbin>(NBINS-1)) || (ybin<0) || (ybin>(NBINS-1))) {
          printf("Out of the boundary!!!\n");
              }
          else {
            if (roger->button_event==LEFT_BUTTON) {// obstacles in Cartesian space
              if (roger->world_map.occupancy_map[ybin][xbin] == OBSTACLE) {
                printf("deleting an obstacle xbin=%d  ybin=%d\n", xbin, ybin);
                fflush(stdout);
                roger->world_map.occupancy_map[ybin][xbin] = FREESPACE;
                //      delete_bin_bumper(xbin,ybin);
              }
              else if (roger->world_map.occupancy_map[ybin][xbin] == FREESPACE) {
                printf("inserting an obstacle xbin=%d  ybin=%d\n", xbin, ybin);
                fflush(stdout);
                roger->world_map.occupancy_map[ybin][xbin] = OBSTACLE;
                roger->world_map.potential_map[ybin][xbin] = 1.0;
                roger->world_map.color_map[ybin][xbin] = LIGHTBLUE;
              }
            }
            else if (roger->button_event == MIDDLE_BUTTON) { }
            else if (roger->button_event == RIGHT_BUTTON) {
              if (roger->world_map.occupancy_map[ybin][xbin] == GOAL) {
                printf("deleting an goal xbin=%d  ybin=%d\n", xbin, ybin);
                fflush(stdout);
                roger->world_map.occupancy_map[ybin][xbin] = FREESPACE;
              }
              else if (roger->world_map.occupancy_map[ybin][xbin] == FREESPACE) {
                printf("inserting an goal xbin=%d  ybin=%d\n", xbin, ybin);
                fflush(stdout);
                roger->world_map.occupancy_map[ybin][xbin] = GOAL;
                roger->world_map.potential_map[ybin][xbin] = 0.0;
                }
              }

          }
        
        break;
        default:
        break;
      }
    }
  }  
}

// // define geometrical and inertial parameters for the NBODY dynamical system
// void initialize_toybox()
// {
//   /****************************** 0: CIRCLE **********************************/
//   toybox[CIRCLE].N = 1;
//   toybox[CIRCLE].color[0] = world_colors[RED];
//   toybox[CIRCLE].mass = M_TOY;
//   toybox[CIRCLE].moi = I_TOY;
//   toybox[CIRCLE].mu = 0.5;
//   toybox[CIRCLE].pos[X] = -2.3;
//   toybox[CIRCLE].pos[Y] = -2.9;
//   toybox[CIRCLE].pos[THETA] = 0.0;

//   toybox[CIRCLE].vel[X] = mobile_base_1.x_dot;
//   toybox[CIRCLE].vel[Y] = mobile_base_1.y_dot;
//   toybox[CIRCLE].vel[THETA] = mobile_base_1.theta_dot;

//   toybox[CIRCLE].net_Fext[X] = toybox[CIRCLE].net_Fext[Y] =
//     toybox[CIRCLE].net_Fext[THETA] = 0.0;

//   /**************************** 1: TRIANGLE **********************************/
//   toybox[TRIANGLE].N = 3;
//   toybox[TRIANGLE].vertices[0][X] = 0.0;
//   toybox[TRIANGLE].vertices[0][Y] = 0.333;
//   toybox[TRIANGLE].vertices[1][X] = 0.289;
//   toybox[TRIANGLE].vertices[1][Y] = -0.167;
//   toybox[TRIANGLE].vertices[2][X] = -0.289;
//   toybox[TRIANGLE].vertices[2][Y] = -0.167;

//   toybox[TRIANGLE].color[0] = world_colors[RED];
//   toybox[TRIANGLE].color[1] = world_colors[RED];
//   toybox[TRIANGLE].color[2] = world_colors[RED];
//   toybox[TRIANGLE].mass = M_TOY;
//   toybox[TRIANGLE].moi = I_TOY;
//   toybox[TRIANGLE].mu = 0.5;
//   toybox[TRIANGLE].pos[X] = -1.15;
//   toybox[TRIANGLE].pos[Y] = -2.983;
//   toybox[TRIANGLE].pos[THETA] = 0.0;

//   toybox[TRIANGLE].vel[X] = mobile_base_1.x_dot;
//   toybox[TRIANGLE].vel[Y] = mobile_base_1.y_dot;
//   toybox[TRIANGLE].vel[THETA] = mobile_base_1.theta_dot;

//   toybox[TRIANGLE].net_Fext[X] = toybox[TRIANGLE].net_Fext[Y] =
//     toybox[TRIANGLE].net_Fext[THETA] = 0.0;

//   /**************************** 2: SQUARE ************************************/
//   toybox[SQUARE].N = 4;
//   toybox[SQUARE].vertices[0][X] = 0.25;
//   toybox[SQUARE].vertices[0][Y] = 0.25;
//   toybox[SQUARE].vertices[1][X] = 0.25;
//   toybox[SQUARE].vertices[1][Y] = -0.25;
//   toybox[SQUARE].vertices[2][X] = -0.25;
//   toybox[SQUARE].vertices[2][Y] = -0.25;
//   toybox[SQUARE].vertices[3][X] = -0.25;
//   toybox[SQUARE].vertices[3][Y] = 0.25;
//   toybox[SQUARE].color[0] = world_colors[RED];
//   toybox[SQUARE].color[1] = world_colors[RED];
//   toybox[SQUARE].color[2] = world_colors[RED];
//   toybox[SQUARE].color[3] = world_colors[RED];
//   toybox[SQUARE].mass = M_TOY;
//   toybox[SQUARE].moi = I_TOY;
//   toybox[SQUARE].mu = 0.5;
//   toybox[SQUARE].pos[X] = -0.0;
//   toybox[SQUARE].pos[Y] = -2.9;
//   toybox[SQUARE].pos[THETA] = 0.0;

//   toybox[SQUARE].vel[X] = mobile_base_1.x_dot;
//   toybox[SQUARE].vel[Y] = mobile_base_1.y_dot;
//   toybox[SQUARE].vel[THETA] = mobile_base_1.theta_dot;

//   toybox[SQUARE].net_Fext[X] = toybox[SQUARE].net_Fext[Y] =
//     toybox[SQUARE].net_Fext[THETA] = 0.0;

//   /************************* 3: TRIANGLE_RECEP *******************************/
//   toybox[TRIANGLE_RECEP].N = 11;
//   toybox[TRIANGLE_RECEP].vertices[0][X] = 0.500;
//   toybox[TRIANGLE_RECEP].vertices[0][Y] = 0.500;
//   toybox[TRIANGLE_RECEP].vertices[1][X] = 0.500;
//   toybox[TRIANGLE_RECEP].vertices[1][Y] = 0.375;
//   toybox[TRIANGLE_RECEP].vertices[2][X] = 0.338375;
//   toybox[TRIANGLE_RECEP].vertices[2][Y] = 0.375;

//   toybox[TRIANGLE_RECEP].vertices[3][X] = 0.088375;
//   toybox[TRIANGLE_RECEP].vertices[3][Y] = 0.000;
//   toybox[TRIANGLE_RECEP].vertices[4][X] = -0.088375;
//   toybox[TRIANGLE_RECEP].vertices[4][Y] = 0.000;

//   toybox[TRIANGLE_RECEP].vertices[5][X] = -0.338375;
//   toybox[TRIANGLE_RECEP].vertices[5][Y] = 0.375;
//   toybox[TRIANGLE_RECEP].vertices[6][X] = -0.500;
//   toybox[TRIANGLE_RECEP].vertices[6][Y] = 0.375;
//   toybox[TRIANGLE_RECEP].vertices[7][X] = -0.500;
//   toybox[TRIANGLE_RECEP].vertices[7][Y] = 0.500;
//   toybox[TRIANGLE_RECEP].vertices[8][X] = -0.250;
//   toybox[TRIANGLE_RECEP].vertices[8][Y] = 0.500;
//   toybox[TRIANGLE_RECEP].vertices[9][X] = 0.000;
//   toybox[TRIANGLE_RECEP].vertices[9][Y] = 0.125;
//   toybox[TRIANGLE_RECEP].vertices[10][X] = 0.250;
//   toybox[TRIANGLE_RECEP].vertices[10][Y] = 0.500;

//   toybox[TRIANGLE_RECEP].color[0] = world_colors[RED];
//   toybox[TRIANGLE_RECEP].color[1] = world_colors[RED];
//   toybox[TRIANGLE_RECEP].color[2] = world_colors[RED];
//   toybox[TRIANGLE_RECEP].color[3] = world_colors[RED];
//   toybox[TRIANGLE_RECEP].color[4] = world_colors[RED];
//   toybox[TRIANGLE_RECEP].color[5] = world_colors[RED];
//   toybox[TRIANGLE_RECEP].color[6] = world_colors[RED];
//   toybox[TRIANGLE_RECEP].color[7] = world_colors[RED];
//   toybox[TRIANGLE_RECEP].color[8] = world_colors[GREEN];
//   toybox[TRIANGLE_RECEP].color[9] = world_colors[GREEN];
//   toybox[TRIANGLE_RECEP].color[10] = world_colors[RED];

//   toybox[TRIANGLE_RECEP].mass = M_TOY;
//   toybox[TRIANGLE_RECEP].moi = I_TOY;
//   toybox[TRIANGLE_RECEP].mu = 0.5;
//   toybox[TRIANGLE_RECEP].pos[X] = 1.15;
//   toybox[TRIANGLE_RECEP].pos[Y] = -3.15;
//   toybox[TRIANGLE_RECEP].pos[THETA] = 0.0;

//   toybox[TRIANGLE_RECEP].vel[X] = mobile_base_1.x_dot;
//   toybox[TRIANGLE_RECEP].vel[Y] = mobile_base_1.y_dot;
//   toybox[TRIANGLE_RECEP].vel[THETA] = mobile_base_1.theta_dot;

//   toybox[TRIANGLE_RECEP].net_Fext[X] = toybox[TRIANGLE_RECEP].net_Fext[Y] =
//     toybox[TRIANGLE_RECEP].net_Fext[THETA] = 0.0;

//   /************************* 4: SQUARE_RECEP *********************************/
//   toybox[SQUARE_RECEP].N = 12;
//   toybox[SQUARE_RECEP].vertices[0][X] = 0.500;
//   toybox[SQUARE_RECEP].vertices[0][Y] = 0.500;
//   toybox[SQUARE_RECEP].vertices[1][X] = 0.500;
//   toybox[SQUARE_RECEP].vertices[1][Y] = 0.375;
//   toybox[SQUARE_RECEP].vertices[2][X] = 0.375;
//   toybox[SQUARE_RECEP].vertices[2][Y] = 0.375;
//   toybox[SQUARE_RECEP].vertices[3][X] = 0.375;
//   toybox[SQUARE_RECEP].vertices[3][Y] = 0.000;
//   toybox[SQUARE_RECEP].vertices[4][X] = -0.375;
//   toybox[SQUARE_RECEP].vertices[4][Y] = 0.000;
//   toybox[SQUARE_RECEP].vertices[5][X] = -0.375;
//   toybox[SQUARE_RECEP].vertices[5][Y] = 0.375;
//   toybox[SQUARE_RECEP].vertices[6][X] = -0.500;
//   toybox[SQUARE_RECEP].vertices[6][Y] = 0.375;
//   toybox[SQUARE_RECEP].vertices[7][X] = -0.500;
//   toybox[SQUARE_RECEP].vertices[7][Y] = 0.500;
//   toybox[SQUARE_RECEP].vertices[8][X] = -0.250;
//   toybox[SQUARE_RECEP].vertices[8][Y] = 0.500;
//   toybox[SQUARE_RECEP].vertices[9][X] = -0.250;
//   toybox[SQUARE_RECEP].vertices[9][Y] = 0.125;
//   toybox[SQUARE_RECEP].vertices[10][X] = 0.250;
//   toybox[SQUARE_RECEP].vertices[10][Y] = 0.125;
//   toybox[SQUARE_RECEP].vertices[11][X] = 0.250;
//   toybox[SQUARE_RECEP].vertices[11][Y] = 0.500;

//   toybox[SQUARE_RECEP].color[0] = world_colors[RED];
//   toybox[SQUARE_RECEP].color[1] = world_colors[RED];
//   toybox[SQUARE_RECEP].color[2] = world_colors[RED];
//   toybox[SQUARE_RECEP].color[3] = world_colors[RED];
//   toybox[SQUARE_RECEP].color[4] = world_colors[RED];
//   toybox[SQUARE_RECEP].color[5] = world_colors[RED];
//   toybox[SQUARE_RECEP].color[6] = world_colors[RED];
//   toybox[SQUARE_RECEP].color[7] = world_colors[RED];
//   toybox[SQUARE_RECEP].color[8] = world_colors[GREEN];
//   toybox[SQUARE_RECEP].color[9] = world_colors[GREEN];
//   toybox[SQUARE_RECEP].color[10] = world_colors[GREEN];
//   toybox[SQUARE_RECEP].color[11] = world_colors[RED];

//   toybox[SQUARE_RECEP].mass = M_TOY;
//   toybox[SQUARE_RECEP].moi = I_TOY;
//   toybox[SQUARE_RECEP].mu = 0.5;
//   toybox[SQUARE_RECEP].pos[X] = 2.3;
//   toybox[SQUARE_RECEP].pos[Y] = -3.15;
//   toybox[SQUARE_RECEP].pos[THETA] = 0.0;

//   toybox[SQUARE_RECEP].vel[X] = mobile_base_1.x_dot;
//   toybox[SQUARE_RECEP].vel[Y] = mobile_base_1.y_dot;
//   toybox[SQUARE_RECEP].vel[THETA] = mobile_base_1.theta_dot;

//   toybox[SQUARE_RECEP].net_Fext[X] = toybox[SQUARE_RECEP].net_Fext[Y] =
//     toybox[SQUARE_RECEP].net_Fext[THETA] = 0.0;
// }




void x_timer_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  int i, j, reset;
  static int render = RENDER_RATE, servo = SERVO_RATE, image = IMAGE_RATE;

  if (servo++ == SERVO_RATE) {

    reset = FALSE;
    write_interface(reset); // reset eliminates user goals and obstacles
                // from occupancy grids

    for (i = 0; i < numRoger; ++i) {
      if (socknew[i] > 0) {
        HandleGuiInteraction(&Rogers[i]);
        SocketCommunicate(&Rogers[i], socknew[i]);
      }
    }
      
    read_interface();
    servo = 1;
  }

  /* writes collision forces into respective data structures */
  /* obstacle data structure is global */
  compute_external_forces();
  simulate_object_polyball(&toy);

  for (i = 0; i < numRoger; ++i) {
    simulate_roger(&RogersBodys[i].mobile_base, RogersBodys[i].arms,
		   RogersBodys[i].eyes);
  }

  if (image++ == IMAGE_RATE) {
    for (i = 0; i < numRoger; ++i) {
      make_images(i);
    }
    image = 1;
  }

  if (render++ == RENDER_RATE) {
    for (i = 0; i < numRoger; ++i) {
      if ((HISTORY) && (history_ptr[i] < MAX_HISTORY)) {
        history[i][history_ptr[i]].arm_pos[LEFT][0] 
	  = RogersBodys[i].arms[LEFT][1].theta;
        history[i][history_ptr[i]].arm_pos[LEFT][1] 
	  = RogersBodys[i].arms[LEFT][2].theta;

        history[i][history_ptr[i]].arm_pos[RIGHT][0] 
	  = RogersBodys[i].arms[RIGHT][1].theta;
        history[i][history_ptr[i]].arm_pos[RIGHT][1] 
	  = RogersBodys[i].arms[RIGHT][2].theta;

        history[i][history_ptr[i]].base_pos[0] = RogersBodys[i].mobile_base.x;
        history[i][history_ptr[i]].base_pos[1] = RogersBodys[i].mobile_base.y;
        history[i][history_ptr[i]].base_pos[2] 
	  = RogersBodys[i].mobile_base.theta;

        ++history_ptr[i];
      }
    }

    if (kEnvironment == DEVELOPMENTAL) {
      draw_all_dev();
    } 
    else draw_all();
    
    render = 1;
  }

  simtime += DT;

  timer = XtAppAddTimeOut(app_con, TIMER_UPDATE, x_timer_proc,
			  (XtPointer)NULL);

  evalManager();
}


void x_start_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  void reset_pong_timers_and_score();

  if (!timer) {
    XkwSetWidgetLabel(start_w, "Stop");
    timer = XtAppAddTimeOut(app_con, TIMER_UPDATE, x_timer_proc,
      (XtPointer)NULL);
  
    if (evalType == PONG) {
      reset_pong_timers_and_score(&pong_evaluator);
      reset_pong();
    }

  }
  else {
    XkwSetWidgetLabel(start_w, "Start");
    XtRemoveTimeOut(timer);
    timer = 0;
  }

}


void x_control_mode_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  Rogers[0].control_mode = change_control_mode_dev();

  switch (Rogers[0].control_mode) {
     case PROJECT1:
       XkwSetWidgetLabel(control_mode_w, "1-MotorUnits"); break;
     case PROJECT2:
       XkwSetWidgetLabel(control_mode_w, "2-ArmKinematics"); break;
     case PROJECT3:
       XkwSetWidgetLabel(control_mode_w, "3-Vision"); break;
     case PROJECT4:
       XkwSetWidgetLabel(control_mode_w, "4-SearchTrack"); break;
     case PROJECT5:
       XkwSetWidgetLabel(control_mode_w, "5-StereoKinematics"); break;
     case PROJECT6:
       XkwSetWidgetLabel(control_mode_w, "6-Kalman"); break;
     case PROJECT7:
       XkwSetWidgetLabel(control_mode_w, "7-ChasePunch"); break;
     case PROJECT8:
       XkwSetWidgetLabel(control_mode_w, "8-PathPlanning"); break;
     case PROJECT9:
       XkwSetWidgetLabel(control_mode_w, "9-PONG"); break;
     case PROJECT10:
       XkwSetWidgetLabel(control_mode_w, "10-Model"); break;
     case PROJECT11:
       XkwSetWidgetLabel(control_mode_w, "11-Belief"); break;
     default: break;
  }
  //call init here makes it independent of timer running
  initialize_control_mode(&Rogers[0]);
}

int change_room_num()
{
  static int room_num;

  room_num = (room_num + 1) % N_ROOMS;
  return (room_num);
}

void x_room_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  void initialize_room();
  Rogers[0].room_num = change_room_num();

  switch (Rogers[0].room_num) {
     case R0:
       XkwSetWidgetLabel(room_w, "Room: 0");
       break;
     case R1:
       XkwSetWidgetLabel(room_w, "Room: 1");
       break;
     case R2:
       XkwSetWidgetLabel(room_w, "Room: 2");
       break;
     default:
       break;
  }
  initialize_room(&Rogers[0]);
}


void initialize_room(roger)
Robot * roger;
{
  int i, j;
  FILE *fp;
  char line[NBINS + 2];

  // read in appropriate Room file selected by user
  switch (roger->room_num) {
  case R0:
    fp = fopen("../RogerProjects/ROOMS/R0.txt", "r");
    break;
  case R1:
    fp = fopen("../RogerProjects/ROOMS/R1.txt", "r");
    break;
  case R2:
    fp = fopen("../RogerProjects/ROOMS/R2.txt", "r");
    break;
  default:
    break;
  }

  // Initialize world geometry according to Room file
  for (i = 0; i<NBINS; i++) {
    fgets(line, NBINS + 2, fp);
    for (j = 0; j<NBINS; j++) {
      roger->world_map.occupancy_map[i][j] = FREESPACE;
      roger->world_map.potential_map[i][j] = 1.0;
      roger->arm_map[LEFT].occupancy_map[i][j] = FREESPACE;
      roger->arm_map[LEFT].potential_map[i][j] = 1.0;
      roger->arm_map[RIGHT].occupancy_map[i][j] = FREESPACE;
      roger->arm_map[RIGHT].potential_map[i][j] = 1.0;

      switch (line[j]) {
      case 'B':
        roger->world_map.occupancy_map[i][j] = OBSTACLE;
        roger->world_map.potential_map[i][j] = 1.0;
        roger->world_map.color_map[i][j] = DARKBLUE;
        break;
      case 'G':
        roger->world_map.occupancy_map[i][j] = OBSTACLE;
        roger->world_map.potential_map[i][j] = 1.0;
        roger->world_map.color_map[i][j] = LIGHTGREEN;
        break;
      case 'K':
        roger->world_map.occupancy_map[i][j] = OBSTACLE;
        roger->world_map.potential_map[i][j] = 1.0;
        roger->world_map.color_map[i][j] = 0;
        break;
      default:
        break;
      }
    }
  }
  fclose(fp);
}


void InitializeArenaEvironment(argc, argv)
int* argc; char **argv;
{
  kEnvironment = ARENA;
  static String fallback_resources[] = {
    "*title:  Roger-the-Crab",
    "*Roger-the-Crab*x: 100",
    "*Roger-the-Crab*y: 100",
    NULL,
  };
  Widget toplevel, form, widget;
  void x_clear();

  toplevel = XtAppInitialize(&app_con, "Roger-the-Crab", NULL, ZERO, argc,
    argv, fallback_resources, NULL, ZERO);
  form = XkwMakeForm(toplevel);
  widget = NULL;
  start_w = widget = XkwMakeCommand(form, NULL, widget, x_start_proc,
    "Start", BOXW, BOXH);
  input_mode_1_w = widget = XkwMakeCommand(form, NULL, widget, x_input_mode_arena_proc,
          "Input: Ball Position", BOXW, BOXH);
  control_mode_1_w = widget = XkwMakeCommand(form, NULL, widget,
    x_control_mode_arena_proc,
    "Control: 1-Motor Units", BOXW, BOXH);
  //input_mode_2_w = widget = XkwMakeCommand(form, NULL, widget, x_input_mode_2_proc,
  //                     "Input 2: Joint angles", BOXW, BOXH);
  // control_mode_2_w = widget = XkwMakeCommand(form, NULL, widget,
  //   x_control_mode_2_proc,
  //   "Control 2: ChasePunch", BOXW, BOXH);
  // params_w = widget = XkwMakeCommand(form, NULL, widget, x_params_proc,
  //           "Enter Params", BOXW, BOXH);
  //stream_w = widget = XkwMakeCommand(form, NULL, widget, x_visualize_proc,
  //           "Visualize",  BOXW, BOXH);
  widget = XkwMakeCommand(form, NULL, widget, x_quit_proc,
    "Quit", BOXW, BOXH);
  canvas_w = widget = XkwMakeCanvas(form, widget, NULL,
    x_canvas_proc, width, height);
  XtRealizeWidget(toplevel);
  display = XtDisplay(canvas_w);
  window = XtWindow(canvas_w);
  screen = DefaultScreen(display);
  depth = DefaultDepth(display, screen);
  foreground = BlackPixel(display, screen);
  background = WhitePixel(display, screen);

  gc = XCreateGC(display, window, 0, NULL);
  XSetFunction(display, gc, GXcopy);
  XSetForeground(display, gc, foreground);
  XSetBackground(display, gc, background);

  pixmap = XCreatePixmap(display, window, width_pixmap, height_pixmap, depth);
  x_clear();

  x_init_colors();

  int i;
  for (i = 0; i < numRoger; ++i) {
    initialize_room(&Rogers[i]);
  }


  reset = TRUE;
  initialize_simulator(reset); // initializes world boundaries,
                 // mobile_base, eyes[2], arms[2], and
                 // Roger interface structure

  simulate_object_polyball(&toy);

  
  for (i = 0; i < numRoger; ++i) {
    // initialize_room(&Rogers[i]);
    simulate_roger(&RogersBodys[i].mobile_base, RogersBodys[i].arms, RogersBodys[i].eyes);
    initialize_control_mode(&Rogers[i]);
  }

  for (i = 0; i < numRoger; ++i) {
    make_images(i);
  }

  draw_all();
  XtAppMainLoop(app_con);
}

void InitializeDevelopmentalEvironment(argc, argv)
int* argc; char **argv;
{
  kEnvironment = DEVELOPMENTAL;
  width = (int)(1.0* WIDTH_DEV);
  height = (int)(1.0*HEIGHT_DEV);
  width_pixmap = (int)((float)(WIDTH)* PIX_MAP_SIZE_RATIO);
  height_pixmap = (int)((float)(HEIGHT)* PIX_MAP_SIZE_RATIO);

  static String fallback_resources[] = { "*title:  Roger-the-Crab",
    "*Roger-the-Crab*x: 100",
    "*Roger-the-Crab*y: 100",NULL, };
  Widget toplevel, form, widget;
  void x_clear();

  //  void intersect_line_circle();
  //  intersect_line_circle();

  toplevel = XtAppInitialize(&app_con, "Roger-the-Crab", NULL, ZERO,
    argc, argv, fallback_resources, NULL, ZERO);


  form = XkwMakeForm(toplevel);
  widget = NULL;
  start_w = widget =
    XkwMakeCommand(form, NULL, widget, x_start_proc, "Start", BOXW_DEV, BOXH);
  input_mode_w = widget =
    XkwMakeCommand(form, NULL, widget, x_input_mode_dev_proc,
      "Input: Joint angles", BOXW_DEV, BOXH);
  control_mode_w = widget =
    XkwMakeCommand(form, NULL, widget, x_control_mode_proc,
      "1-Motor Units", BOXW_DEV, BOXH);
  room_w = widget =
    XkwMakeCommand(form, NULL, widget, x_room_proc, "Room: 0", BOXW_DEV, BOXH);
  params_w = widget =
    XkwMakeCommand(form, NULL, widget, x_params_proc, "Enter Params", BOXW_DEV, BOXH);
  stream_w = widget =
    XkwMakeCommand(form, NULL, widget, x_visualize_proc, "Visualize", BOXW_DEV, BOXH);
  widget = XkwMakeCommand(form, NULL, widget, x_quit_proc, "Quit", BOXW_DEV, BOXH);
  canvas_w = widget =
    XkwMakeCanvas(form, widget, NULL, x_canvas_proc, width, height);
  XtRealizeWidget(toplevel);
  display = XtDisplay(canvas_w);
  window = XtWindow(canvas_w);
  screen = DefaultScreen(display);
  depth = DefaultDepth(display, screen);
  foreground = BlackPixel(display, screen);
  background = WhitePixel(display, screen);

  gc = XCreateGC(display, window, 0, NULL);
  XSetFunction(display, gc, GXcopy);
  XSetForeground(display, gc, foreground);
  XSetBackground(display, gc, background);

  pixmap = XCreatePixmap(display, window, width_pixmap, height_pixmap, depth);
  x_clear();

  x_init_colors();

  reset = TRUE;

  initialize_room(&Rogers[0]);
  initialize_simulator(reset);

  simulate_roger(&RogersBodys[0].mobile_base, RogersBodys[0].arms, RogersBodys[0].eyes);
  initialize_control_mode(&Rogers[0]);

  simulate_object_polyball(&toy);
  make_images(0);

  draw_all_dev();

  XtAppMainLoop(app_con);
}

int main(argc, argv)
int argc; char **argv;
{
  int i, chosen_env;

  if (argc != 3 && argc != 4) {
    printf("Usage: ./simulator EnvironmentNum RobotNum (optional)EvaluationType.\n");
    return 0;
  }

  if (argc == 4) {
    evalType = atoi(argv[3]);
  }
  numRoger = atoi(argv[2]);
  chosen_env = atoi(argv[1]);
  numObjects = 3 * numRoger + numToy;

  // Check the correctness of input arguments
  if (numRoger > MaxRobotNum) {
    printf("Violating maximum robot number (%d) \n", MaxRobotNum);
    return 0;
  }

  if ( (chosen_env != ARENA) && (chosen_env != DEVELOPMENTAL) ) {
    printf("Unknown environment!\n ARENA : 0, DEVELOPMENTAL: 1\n");
    return 0;
  }

  if ( (chosen_env == DEVELOPMENTAL) && (numRoger > 1) ) {
    printf("Violating maximum robot number supported in DEVELOPMENTAL environment (1)\n");
    return 0;
  }


  for (i = 0; i < numRoger; ++i) {
    SocketInit(ServerPorts[i], &socknew[i]);
  }


  if (chosen_env == ARENA) {
    InitializeArenaEvironment(&argc, argv);
  } else {
    InitializeDevelopmentalEvironment(&argc, argv);
  }  

}

