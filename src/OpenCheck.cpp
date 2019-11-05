#include "CYdLidar.h"
#include "common.h"
#include <map>
#include <regex>

using namespace std;
using namespace ydlidar;
using namespace impl;

void CYdLidar::startCorrectionMod() {
  m_action_startup = true;
  m_action_step = 0;
  m_action_state = 0;
  has_check_flag = false;
  action_check_time = getTime();
}

void CYdLidar::OnEnter(double frequency) {
  if (!m_action_startup) {
    return;
  }

  ActionStateUpdate(frequency);
}

void CYdLidar::ActionStateUpdate(double frequency) {
  bool retVal = false;

  switch (m_action_state) {
    case 0:
      setActionState(true);
      break;

    case 1:
      if (frequency < m_min_action_frequency) {
        action_check_time = getTime();
        m_action_state++;
        m_action_step++;
      }

      retVal = CheckStateTimeout(true);
      break;

    case 2:
      retVal = CheckStateTimeout(true);

      if (retVal && frequency < m_min_action_frequency) {
        action_check_time = getTime();

        if (m_action_step < max_action_step && has_check_flag) {
          printf("The correction model has been opened\n");
          m_action_startup = false;
          resetCheckState();
          setCurrentFrequencyStatus(true);
          setLastFrequencyStatus(true);
          setCheckFinished(false);
          return;
        }

        if (m_action_step <= max_action_step || !has_check_flag) {
          if (m_action_step == max_action_step + 2) {
            fprintf(stderr, "Failed to open correction mod\n");
            m_action_startup = false;
            return;
          }

          m_action_state++;
        } else {
          if (has_check_flag) {
            printf("Successful Opening of correction Mod\n");
            m_action_startup = false;
            resetCheckState();
            setCurrentFrequencyStatus(true);
            setLastFrequencyStatus(true);
            setCheckFinished(false);
          } else {

          }
        }
      }

      break;

    case 3:
      setActionState(false);
      break;

    case 4:
      if (frequency > m_max_action_frequency) {
        action_check_time = getTime();
        m_action_state++;
      }

      retVal = CheckStateTimeout(false);

      break;

    case 5:
      retVal = CheckStateTimeout(false);

      if (retVal && frequency > m_max_action_frequency) {
        action_check_time = getTime();
        m_action_state = 0;
      }

      break;

    default:
      break;
  }
}

void CYdLidar::setActionState(bool isLowerSpeed) {
  if (isLowerSpeed) {
    lowSpeed();
  } else {
    hightSpeed();
  }

  m_action_state++;
}

bool CYdLidar::CheckStateTimeout(bool isLowerSpeed) {
  uint64_t current_time = getTime();
  bool retVal = false;
  int64_t time_diff = current_time - action_check_time;

  if (time_diff > min_check_time) {
    retVal = true;
  }

  if (time_diff > max_check_time) {
    retVal = false;
    fprintf(stderr, isLowerSpeed ? " Minimum frequency error\n" :
            " Maximum frequency error\n");
    m_check_state_error = isLowerSpeed ? MINFREQERROR : MAXFREQERROR;
    m_action_step = max_action_step + 3;
    m_action_startup = false;
  }

  return retVal;

}
