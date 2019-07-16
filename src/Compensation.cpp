#include "CYdLidar.h"
#include "common.h"
#include <map>
#include <regex>

using namespace std;
using namespace ydlidar;
using namespace impl;


void CYdLidar::retSetData() {
  for (int i = 0; i < MAXCHECKTIMES; i++) {
    check_queue_size[i] = 0;
    auto_check_sum_queue[i] = 0;
    auto_check_distance [i] = -1;
  }
}

void CYdLidar::handleScanData(double angle, double distance) {
  double ori_angle = angle;

  if (ori_angle > 360) {
    ori_angle -= 360;
  } else if (ori_angle < 0) {
    ori_angle += 360;
  }

  for (int j = 0; j < MAXCHECKTIMES; j++) {
    if ((m_angle_threshold[2 * j] + MAXCHECKIGNOREANGLE) < ori_angle &&
        ori_angle < (m_angle_threshold[2 * j + 1] - MAXCHECKIGNOREANGLE)) {
      if (distance > 10 && check_queue_size[j] < 10) {
        check_queue_size[j]++;
        auto_check_sum_queue[j] += (int)distance;
        auto_check_distance[j] = (int)distance;
      }
    }
  }


}



void CYdLidar::handleCheckData() {
  int distance_state = -1;
  has_check_state = false;

  for (int j = MAXCHECKTIMES - 1; j >= 0; j--) {
    if (check_queue_size[j] &&
        (auto_check_distance[j] - auto_check_sum_queue[j] / check_queue_size[j]) < 1) {
      distance_state = auto_check_distance[j * 2];

      switch (j * 2) {
        case CHECK_ANGLE_MIN:
          handleLidarDis(distance_state);
          break;

        case SIGMA_ANGLE_MIN:
          break;

        case SIGN_ANGLE_MIN:
          break;

        case SURE_ANGLE_MIN:
          handleLidarDis(distance_state);
          break;

        default:
          break;
      }
    }

  }

  if (m_action_startup && has_check_state) {
    has_check_flag = true;
  }

  handleCheckState();
  handleCheckStep();

}

void CYdLidar::handleCheckStep() {
  if (current_frequency < 20 && current_frequency > 1) {
    if (fabs(current_frequency - last_frequency) < 3.0) {

    } else {
      fprintf(stderr, "frequency jump.......\n");
    }

  } else {
    fprintf(stderr, "frequency error.......\n");
  }
}

void CYdLidar::handleCheckDis() {
  switch (auto_check_distance[CHECK_ANGLE_MIN]) {
    case CHECKINGDISTANCE:
      break;

    case FAILEDDISTANCE:
      break;

    case ADJUSTDISTANCE:
      break;

    case SUCCESSDISTANCE:
      break;

    case SAVESUCCESSDISTANCE:
      break;

    case MAXDISTANCE:

      break;

    default:
      break;
  }

}

void CYdLidar::handleLidarDis(int distance) {

  switch (distance) {
    case CHECKINGDISTANCE:

      break;

    case (CHECKINGDISTANCE+1000):
      has_check_state = true;

      break;

    case FAILEDDISTANCE:
      has_check_state = true;

      break;

    case (FAILEDDISTANCE + 1000):
      has_check_state = true;

      break;

    case ADJUSTDISTANCE:
      has_check_state = true;

      break;

    case (ADJUSTDISTANCE+1000):
      has_check_state = true;

      break;

    case SUCCESSDISTANCE:
      has_check_state = true;

      break;

    case (SUCCESSDISTANCE + 1000):
      has_check_state = true;

      break;

    case SAVESUCCESSDISTANCE:
      has_check_state = true;

      break;

    case (SAVESUCCESSDISTANCE + 1000):
      has_check_state = true;
      break;

    case SUREDISTANCE:
      has_check_state = true;
      break;

    case MAXDISTANCE:
      has_check_state = true;
      break;

    default:
      if (abs(distance - (int(distance / 1000)) * 1000) < 2) {
        has_check_state = true;
      } else {

      }

      break;
  }

}

void CYdLidar::handleCheckState() {
  switch (m_state) {
    case NORMAL:

      break;

    case CHECKING:

      break;

    case CHECK_FAILED:


      break;

    case CHECK_ADJUST:



      break;

    case CHECK_SUCCESS:


      break;

    case CHECK_FININSHED:

      break;

    default:
      break;
  }
}
