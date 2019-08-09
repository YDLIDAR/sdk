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

  for (int i = 0; i < MAXCALIBRATIONRANGE; i++) {
    m_distance_queue[i].clear();
    m_percentage[i] = 100.0;
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

  for (int i = 0; i < MAXCALIBRATIONRANGE; i++) {
    if (fabs(m_Calibration_angle[i] - ori_angle) < MAXCHECKIGNOREANGLE) {
      if (distance > 10 &&
          fabs(m_Calibration_distance[i] - distance) < MaxEntryDiff) {
        m_distance_queue[i].push_back(distance);
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
        case CHECK_ANGLE_MIN://通讯状态
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
  handleCalibrationState();
  handleCheckStep();

}

void CYdLidar::handleCheckStep() {
  if (current_frequency < 20 && current_frequency > 1) {
    if (fabs(current_frequency - last_frequency) < 3.0) {

    } else {
      fprintf(stderr, "frequency jump.......\n");
      m_check_state_error = JUMPFREQUENCY;
    }

  } else {
    fprintf(stderr, "frequency error.......\n");
    m_check_state_error = FREQUENCYOUT;
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

void CYdLidar::handleCalibrationState() {
  if (has_check_state && !m_action_startup) {
    bool pass = true;
    bool m_interference = false;

    for (int i = 0; i < MAXCALIBRATIONRANGE; i++) {
      if (m_distance_queue[i].size()) {
        double sum = std::accumulate(std::begin(m_distance_queue[i]),
                                     std::end(m_distance_queue[i]), 0.0);
        double mean =  sum / m_distance_queue[i].size(); //均值

        if (fabs(mean - m_last_check_distance[i]) > MaxStdDev) {
          pass &= false;
          m_interference = true;
        } else {
          if (pass) {
            start_check_count++;

            if (m_mean_distance_queue[i].size() > MAXSTARTCHECKCOUNT) {
              m_mean_distance_queue[i].erase(m_mean_distance_queue[i].begin());
            }

            m_mean_distance_queue[i].push_back(mean);
            m_percentage[i] = (mean - m_Calibration_distance[i]) /
                              m_Calibration_distance[i];
          } else {
            start_check_count = 0;
          }
        }

        m_last_check_distance[i] = mean;

      } else {
        pass &= false;
      }
    }



    if (pass) {
      if (start_check_count > MAXSTARTCHECKCOUNT) {

      } else {

      }

    } else {
      if (m_interference) {

      } else {

      }
    }

  }
}
