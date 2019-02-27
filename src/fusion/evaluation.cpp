#ifndef EVALUATION_H
#define EVALUATION_H

#include <Eigen/Dense>

namespace kalman {

class Evaluation {
 public:

  void Update(Eigen::Array4d &predicted, Eigen::Array4d &truth) {
    sum_of_squared_erros += (predicted - truth).square();
    num_errors++;
  }

  Eigen::Array4d CalculateRMSE() {
    return (sum_of_squared_erros / num_errors).sqrt();
  }

 private:
  Eigen::Array4d sum_of_squared_erros = Eigen::Array4d::Zero();
  size_t num_errors = 0;
};

}

#endif //EVALUATION_H
