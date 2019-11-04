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
      } else {//雷达没有放入工位或者精度不满足
        if (!getCheckFinished()) {
          fprintf(stderr, "Please put the robot in the correciton station...\n");
          fflush(stderr);
        }

      }
    }
  }

  //修正完成或者 在切换频率 清除计算百分比队列
  if (getChangeFrequency() || getCheckFinished()) {
    for (int i = 0; i < MAXCALIBRATIONRANGE; i++) {
      m_distance_queue[i].clear();
    }

    start_check_count = 0;
  }


}



void CYdLidar::handleCheckData() {
  int distance_state = -1;
  has_check_state = false;

  if (!getCheckFinished() || m_action_startup) {
    m_check_state_error = NOERROR;
  }

  for (int j = MAXCHECKTIMES - 1; j >= 0; j--) {
    if (check_queue_size[j] &&
        (auto_check_distance[j] - auto_check_sum_queue[j] / check_queue_size[j]) < 1) {
      distance_state = auto_check_distance[j * 2];

      switch (j * 2) {
        case CHECK_ANGLE_MIN://通讯状态
          handleLidarDis(distance_state);
          break;

        case SIGMA_ANGLE_MIN://修正值
          break;

        case SIGN_ANGLE_MIN://修正值符号
          break;

        case SURE_ANGLE_MIN://状态机
          handleLidarSureDis(distance_state);
          break;

        default:
          break;
      }
    } else {
      if (j * 2 == CHECK_ANGLE_MIN) {
        has_check_state = false;
        m_state = NORMAL;
      } else if (j * 2 == SURE_ANGLE_MIN) {

      }
    }

  }

  if (m_action_startup && has_check_state) {
    has_check_flag = true;//修正模式
  }

  handleCheckState();
  handleCalibrationState();
  handleCheckStep();

}

void CYdLidar::handleCheckStep() {
  if (current_frequency < 20 && current_frequency > 1) {
    if (fabs(current_frequency - last_frequency) < 3.0) {
      handleFrequencySM(current_frequency);
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
      m_state = NORMAL;
      break;

    case (CHECKINGDISTANCE+1000):
      has_check_state = true;
      m_state = CHECKING;

      break;

    case FAILEDDISTANCE:
      has_check_state = true;
      m_state = CHECK_FAILED;

      break;

    case (FAILEDDISTANCE + 1000):
      has_check_state = true;

      break;

    case ADJUSTDISTANCE:
      has_check_state = true;
      last_check_state = true;
      m_state = CHECK_ADJUST;
      break;

    case (ADJUSTDISTANCE+1000):
      has_check_state = true;

      break;

    case SUCCESSDISTANCE:
      has_check_state = true;
      last_check_state = true;
      m_state = CHECK_SUCCESS;

      break;

    case (SUCCESSDISTANCE + 1000):
      has_check_state = true;

      break;

    case SAVESUCCESSDISTANCE:
      has_check_state = true;
      last_check_state = true;
      m_state    = CHECK_FININSHED;
      break;

    case (SAVESUCCESSDISTANCE + 1000):
      has_check_state = true;
      break;

    case SUREDISTANCE:
      has_check_state = true;
      break;

    case MAXDISTANCE:
      has_check_state = true;
      last_check_state = false;
      m_state = NORMAL;
      break;

    default:
      last_check_state = false;

      if (abs(distance - (int(distance / 1000)) * 1000) < 2) {
        has_check_state = true;
      } else {
        has_check_state = false;
        m_state = NORMAL;
      }

      break;
  }

}

void CYdLidar::handleLidarSureDis(int distance) {
  state_distance = distance;

  switch (distance) {
    case CHECKINGDISTANCE:
      m_state = NORMAL;
      break;

    case (CHECKINGDISTANCE+1000):
      has_check_state = true;
      m_state = CHECKING;
      break;

    case FAILEDDISTANCE:
      has_check_state = true;
      m_state = CHECKING;
      break;

    case (FAILEDDISTANCE + 1000):
      has_check_state = true;
      m_state = CHECKING;

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
//      m_state    = CHECK_FININSHED;
      break;

    case MAXDISTANCE:
      has_check_state = true;
      m_state = NORMAL;
      break;

    default:
      if (abs(distance - (int(distance / 1000)) * 1000) < 2) {
        has_check_state = true;
      } else {
        has_check_state = false;
        m_state = NORMAL;
      }

      break;
  }

}

void CYdLidar::handleCheckState() {
  switch (m_state) {
    case NORMAL:

      break;

    case CHECKING:
      if (!getChangeFrequency() && !getCheckFinished()) {
        setChangeFrequency(true);
        m_check_freq_state = ENTER_STATE;
      }

      break;

    case CHECK_FAILED:

      if (!last_check_state && state_distance >= 5000 && state_distance <= 7000) {
        m_check_state_error = AUTOCHECKFAILED;//雷达自检错误, 不能修正
        setChangeFrequency(false);
        setCheckFinished(true);//修正完成
      }

      if (last_check_state) { //干扰信息出现
        m_check_state_error = NOISE;
      }

      break;

    case CHECK_ADJUST:
      if (m_check_state_error == AUTOCHECKFAILED) {
        m_check_state_error = NOERROR;
      }

      if (!getChangeFrequency() && state_distance == 4) {
        m_check_freq_state = SAVE_STATE;
        setChangeFrequency(true);
      }

      if (getCurrectTimeDiff() > 4.0 && (state_distance < 6000 ||
                                         state_distance == 7000) &&
          !getChangeFrequency()) {
        m_check_freq_state = SAVE_STATE;
        setChangeFrequency(true);
      }

      break;

    case CHECK_SUCCESS:
      if (getCurrectTimeDiff() > 4.0 && state_distance < 10000 &&
          !getChangeFrequency()) {
        setChangeFrequency(true);
        m_check_freq_state = SAVE_STATE;
      }

      break;

    case CHECK_FININSHED:
      setCheckFinished(true);
      setChangeFrequency(false);
      setResult(true);
      break;

    default:
      break;
  }

  updateCheckState();
}

void CYdLidar::updateCheckState() {
  if (m_last_state != m_state) {
    m_last_state = m_state;

    switch (m_state) {
      case NORMAL:
        printf("Step0: Ready......................\n\n");
        fflush(stdout);
        break;

      case CHECKING:
        printf("Step1: Checking......................\n\n");
        fflush(stdout);
        break;

      case CHECK_FAILED:
        printf("Step2: Exception......................\n\n");
        fflush(stdout);
        break;

      case CHECK_ADJUST:
        printf("Step3: Adjust......................\n\n");
        fflush(stdout);
        break;

      case CHECK_SUCCESS:
        printf("Step4: Save......................\n\n");
        fflush(stdout);
        break;

      case CHECK_FININSHED:
        printf("Step5: Finished......................\n\n");
        fflush(stdout);
        break;

      default:
        break;
    }
  }
}

float CYdLidar::getCurrectTimeDiff() const {
  uint64_t current_time = getTime();
  int64_t time_diff = current_time - last_change_time;
  float diff = static_cast<float>(time_diff / 1e9);
  return diff;
}

void CYdLidar::handleCalibrationState() {
  if (has_check_state && !m_action_startup && !getCheckFinished() &&
      !getChangeFrequency()) {//在修正模式中
    bool pass = true;
    bool m_interference = false;//干扰

    for (int i = 0; i < MAXCALIBRATIONRANGE; i++) {
      if (m_distance_queue[i].size()) {
        double sum = std::accumulate(std::begin(m_distance_queue[i]),
                                     std::end(m_distance_queue[i]), 0.0);
        double mean =  sum / m_distance_queue[i].size(); //均值

        if (fabs(mean - m_last_check_distance[i]) > MaxStdDev) {
          pass &= false;
          m_interference = true;
          m_check_state_error = MAXSTDEVERROR;
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
      //收集雷达数据正常, 开始计算精度
      if (start_check_count > MAXSTARTCHECKCOUNT) {
        bool flag = true;
        bool enter_model = false;

        if (!last_check_state && m_state == NORMAL) {
          enter_model = true;
        }

        for (int i = 0; i < MAXCALIBRATIONRANGE; i++) {
          if (m_percentage[i] < enter_model ? m_check_percentage[i] :
              m_pass_percentage[i]) {
            flag &= true;
          } else {
            flag &= false;
          }
        }

        //精度是否合格
        if (flag) {
          //执行修正
          if (enter_model) {
            m_check_freq_state = ENTER_STATE;
            start_check_count = 0;
          } else {//修正正确, 保证修正
            m_check_freq_state = SAVE_STATE;
          }

          setChangeFrequency(true);
          m_check_state_error = NOERROR;

        } else {

          if (m_state == CHECK_SUCCESS &&
              last_check_state) {//修正精度不合格, 取消重新修正
            if (state_distance >= SAVESUCCESSDISTANCE) {
              m_check_freq_state = CANCEL_STATE;
              setChangeFrequency(true);
              printf("Revoke the correction value............\n\n");
              fflush(stdout);
            }
          }

          if (enter_model) {//超出修正精度, 不修正
            m_check_state_error = OUTOFRANGE;
          }

        }

      } else {//等待达到检测次数

      }

    } else {
      //抖动太大, 雷达还没稳定到校准工装上
      if (m_interference) {
        m_check_state_error = MAXSTDEVERROR;
      } else {//距离超出指定精度
        m_check_state_error = OUTOFRANGE;
      }
    }

  }
}

//重置修正状态
void CYdLidar::resetCheckState() {
  change_frequency_times = 0;
  echo_frequency_times = 0;
  m_check_freq_state = ENTER_STATE;
  last_check_state = false;
  m_check_state_error = NOERROR;
  state_distance = 0;
  has_check_state = false;
  start_check_count = 0;
  m_state = NORMAL;
  m_last_state = IDEL;
  last_change_time = getTime();
  setMinFrequency(m_max_check_frequency);
  setMaxFrequency(m_min_check_frequency);
  setResult(false);

  for (int i = 0; i < MAXCALIBRATIONRANGE; i++) {
    m_distance_queue[i].clear();
    m_last_check_distance[i] = 0.0;
  }
}


//频率切换状态机
void CYdLidar::handleFrequencySM(double frequency) {

  setMaxLowFrequencyTimes(MaxPlusTimes);
  setMaxHightFrequencyTimes(MaxPlusTimes);
  setMaxEchoTimes(2);

  switch (m_check_freq_state) {
    case ENTER_STATE:
      setMaxEchoTimes(4);
      break;

    case SAVE_STATE:

      break;

    case CANCEL_STATE:
      setMaxHightFrequencyTimes(MaxCancelPlusTimes);
      break;

    case SENSOR_STATE:

      break;

    case ONETIMES_STATE:

      break;

    default:
      break;
  }

  autoChangeFequency(frequency);
}


//切换频率
void CYdLidar::autoChangeFequency(double frequency) {
  if (getChangeFrequency()) {
    change_frequency_times++;

    if (frequency < m_min_check_frequency) {
      if (getCurrentFrequencyStatus() && (getLastFrequencyStatus() ||
                                          change_frequency_times > getMaxLowFrequencyTimes())) {
        setCurrentFrequencyStatus(false);//高频
        echo_frequency_times++;

        if (echo_frequency_times > getMaxEchoTimes()) {
          echo_frequency_times = 0;
          setCurrentFrequencyStatus(true);//低频
          setChangeFrequency(false);

          if (m_state == CHECK_ADJUST && m_check_freq_state == CANCEL_STATE) {
            m_check_freq_state = SAVE_STATE;
            setChangeFrequency(true);
          }
        } else {
          hightSpeed();
        }

        setLastFrequencyStatus(getCurrentFrequencyStatus());
        change_frequency_times = 0;

      }
    }

    if (frequency > m_max_check_frequency) {
      if (!getCurrentFrequencyStatus() &&
          change_frequency_times > getMaxHightFrequencyTimes()) {
        setCurrentFrequencyStatus(true);
        lowSpeed();
        change_frequency_times = 0;
      }
    }

    checkFrequencyState(frequency);
    last_change_time = getTime();

  } else {
    change_frequency_times = 0;
    echo_frequency_times = 0;
    setMinFrequency(m_max_check_frequency);
    setMaxFrequency(m_min_check_frequency);

    if (frequency < m_min_check_frequency &&
        last_frequency < m_min_check_frequency && !getCurrentFrequencyStatus()) {
      setCurrentFrequencyStatus(true);
      setLastFrequencyStatus(true);
    } else if (frequency > m_max_check_frequency &&
               last_frequency > m_max_check_frequency && getCurrentFrequencyStatus()) {
      setCurrentFrequencyStatus(false);
      setLastFrequencyStatus(false);
    }
  }

  if (frequency < getMinFrequency()) {
    setMinFrequency(frequency);
  }

  if (frequency > getMaxFrequency()) {
    setMaxFrequency(frequency);
  }



}

//检测频率切换是否异常
void CYdLidar::checkFrequencyState(double frequency) {
  if (change_frequency_times > (getCurrentFrequencyStatus() ?
                                getMaxLowFrequencyTimes() :  getMaxHightFrequencyTimes()) * 4) {
    bool save_result = false;

    if (getMinFrequency() > m_min_check_frequency &&
        getMaxFrequency() < m_max_check_frequency) {
      m_check_state_error = MINMAXFREQERROR;
      save_result = true;
    } else if (getMinFrequency() > m_min_check_frequency) {
      m_check_state_error = MINFREQERROR;
      save_result = true;
    } else if (getMaxFrequency() < m_max_check_frequency) {
      m_check_state_error = MAXFREQERROR;
      save_result = true;
    } else {
      if (getCurrentFrequencyStatus()) {
        m_check_state_error = MINFREQERROR;
        save_result = true;
        setMinFrequency(frequency);
      } else {
        m_check_state_error = MAXFREQERROR;
        save_result = true;
        setMaxFrequency(frequency);
      }

    }

    if (save_result) {
      setCurrentFrequencyStatus(false);
      setChangeFrequency(false);
      resetCheckState();
    }
  } else if (change_frequency_times > 3 * (getCurrentFrequencyStatus() ?
             getMaxLowFrequencyTimes() :  getMaxHightFrequencyTimes())) {
    if (change_frequency_times % 4 == 0) {
      if (getCurrentFrequencyStatus()) {
        lowSpeed();
      } else {
        hightSpeed();
      }
    }
  }
}


bool CYdLidar::IsCheckingFinished() {
  return !(m_action_startup || getCheckFinished());
}

