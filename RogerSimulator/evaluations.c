#include <stdio.h>
#include "include/evaluations.h"

int pong_game_over_detection(pong_evaluator) 
PongEval* pong_evaluator;
{
  if (pong_evaluator->goal_1 == MAX_SCORE || pong_evaluator->goal_2 == MAX_SCORE) {
    if (pong_evaluator->goal_1 == MAX_SCORE) {
      printf("Roger 1 WINS\n");
    }
    if (pong_evaluator->goal_2 == MAX_SCORE) {
      printf("Roger 2 WINS\n");
    }
    return 1;
  }
  return 0;
}

void reset_pong_timers(pong_evaluator)
PongEval* pong_evaluator;
{
  pong_evaluator->possession_time_1 = 0.0;
  pong_evaluator->possession_time_2 = 0.0;
  pong_evaluator->continuous_possession_time_1 = 0.0;
  pong_evaluator->continuous_possession_time_2 = 0.0;
}

void reset_pong_timers_and_score(pong_evaluator) 
PongEval* pong_evaluator;
{
  pong_evaluator->possession_time_1 = 0.0;
  pong_evaluator->possession_time_2 = 0.0;
  pong_evaluator->continuous_possession_time_1 = 0.0;
  pong_evaluator->continuous_possession_time_2 = 0.0;
  pong_evaluator->goal_1 = 0;
  pong_evaluator->goal_2 = 0;
}

int pong_goal_detection(min_x, max_x, ball_x, middle, r_goal, DT, pong_evaluator)
double min_x, max_x, ball_x, middle, r_goal, DT;
PongEval* pong_evaluator;
 {
  
  if (ball_x > middle) {
    pong_evaluator->possession_time_2 += DT;
    pong_evaluator->continuous_possession_time_2 += DT;
    pong_evaluator->continuous_possession_time_1 = 0;
  }
  else if (ball_x < middle) {
    pong_evaluator->possession_time_1 += DT;
    pong_evaluator->continuous_possession_time_1 += DT;
    pong_evaluator->continuous_possession_time_2 = 0;
  }

  if (pong_evaluator->continuous_possession_time_1 > MAX_POSSESSION_TIME ||
      pong_evaluator->continuous_possession_time_2 > MAX_POSSESSION_TIME) {
    if (pong_evaluator->continuous_possession_time_1 < 
        pong_evaluator->continuous_possession_time_2) {
      printf("Roger 2 exceeded maximum possession time\n");
      pong_evaluator->goal_1++;
    }
    else {
      printf("Roger 1 exceeded maximum possession time\n");
      pong_evaluator->goal_2++;
    }
    return 1;
  }

  if ((pong_evaluator->possession_time_1 + pong_evaluator->possession_time_2) > MAX_TIME) {
    if (pong_evaluator->possession_time_1 < pong_evaluator->possession_time_2) {
      printf("Total possession time with a goal exceeded. Roger 1 reacted most quickly\n");
      pong_evaluator->goal_1++;
    }
    else {
      printf("Total possession time with a goal exceeded. Roger 2 reacted most quickly\n");
      pong_evaluator->goal_2++;
    }
    return 1;
  }
  else {
    if ((ball_x + r_goal) >= max_x) {
      printf("Roger 1 SCORES\n");
      pong_evaluator->goal_1++;
      return 1;
    }
    else if ((ball_x - r_goal) <= min_x) {
      printf("Roger 2 SCORES\n");
      pong_evaluator->goal_2++;
      return 1;
    }
  }
  return 0;
}

int pong_fault_detection(min_x, max_x, min_y, max_y, middle, 
                         base_pos_x, base_pos_y, base_radius, 
                         arm1_pos_x, arm1_pos_y, arm2_pos_x, arm2_pos_y, hand_radius, 
                         roger_id, pong_evaluator) 
double min_x, max_x, min_y, max_y, middle, base_radius, hand_radius;
double base_pos_x, base_pos_y, arm1_pos_x, arm1_pos_y, arm2_pos_x, arm2_pos_y;
int roger_id;
PongEval* pong_evaluator;
{
  //printf("%lf\n", min_x);
  //printf("%lf\n", max_x);
  //printf("%lf\n", min_y);
  //printf("%lf\n", max_y);
  //printf("%lf\n", middle);
  //printf("%lf\n", base_radius);
  //printf("%lf\n", hand_radius);
  //printf("%lf\n", base_pos_x);
  //printf("%lf\n", base_pos_y);
  //printf("%lf\n", arm1_pos_x);
  //printf("%lf\n", arm1_pos_y);
  //printf("%lf\n", arm2_pos_x);
  //printf("%lf\n", arm2_pos_y);
  //printf("%d\n", roger_id);
  
  if ((base_pos_x + base_radius) > max_x || (base_pos_x - base_radius) < min_x ||
      (base_pos_y + base_radius) > max_y || (base_pos_y - base_radius) < min_y) {
    printf("Roger %d ran into wall\n", roger_id);
    if (roger_id == 1) {
      pong_evaluator->goal_2++;
    }
    else {
      pong_evaluator->goal_1++;
    }
    return 1;
  }
  
  if ((arm1_pos_x + hand_radius) > max_x || (arm1_pos_x - hand_radius) < min_x ||
      (arm1_pos_y + hand_radius) > max_y || (arm1_pos_y - hand_radius) < min_y ||
      (arm2_pos_x + hand_radius) > max_x || (arm2_pos_x - hand_radius) < min_x ||
      (arm2_pos_y + hand_radius) > max_y || (arm2_pos_y - hand_radius) < min_y) {
    printf("Roger %d touched wall\n", roger_id);
    if (roger_id == 1) {
      pong_evaluator->goal_2++;
    }
    else {
      pong_evaluator->goal_1++;
    }
    return 1;
  }

  if (roger_id == 1) {
    if ((base_pos_x + base_radius) > middle ||
        (arm1_pos_x + hand_radius) > middle ||
        (arm2_pos_x + hand_radius) > middle) {
      printf("Roger 1 crossed center \n");
      pong_evaluator->goal_2++;
      return 1;
    }
  }
  else {
    if ((base_pos_x - base_radius) < middle ||
        (arm1_pos_x - hand_radius) < middle ||
        (arm2_pos_x - hand_radius) < middle) {
      printf("Roger 2 crossed center\n");
      pong_evaluator->goal_1++;
      return 1;
    }
  }
  return 0;
}

