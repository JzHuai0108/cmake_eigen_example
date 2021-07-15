#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>

Eigen::Quaterniond simNoisyRotation(const Eigen::Quaterniond &q,
                                    double rotationNoiseMagnitude) {
  Eigen::Vector3d rotationVector = Eigen::Vector3d::Random();
  Eigen::Quaterniond dq(
      Eigen::AngleAxisd(rotationNoiseMagnitude, rotationVector.normalized()));
  return q * dq;
}

Eigen::Vector3d simNoisyTranslation(const Eigen::Vector3d &r,
                                    double translationNoiseMagnitude) {
  Eigen::Vector3d direction = Eigen::Vector3d::Random();
  return r + direction.normalized() * translationNoiseMagnitude;
}

int main(int argc, char **argv) {
  double rotError = 0.5 * M_PI / 180;
  double transError = 0.02;
  std::cout << "Given an Euclidean transformation, create a noisy one."
            << std::endl;
  std::cout << "Usage: " << argv[0]
            << " <rotationNoiseMagnitude> <translationNoiseMagnitude>"
            << std::endl;
  if (argc > 2) {
    rotError = std::stof(argv[1]);
    transError = std::stof(argv[2]);
  }

  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;

  // EuRoC dataset T_S_Cl
  // S IMU sensor
  // Cl left camera
  rotation << 0.0148655429818, -0.999880929698, 0.00414029679422,
      0.999557249008, 0.0149672133247, 0.025715529948,
      -0.0257744366974, 0.00375618835797, 0.999660727178;
  translation << -0.0216401454975, -0.064676986768, 0.00981073058949;

  // add error
  Eigen::Quaterniond quat(rotation);
  Eigen::Quaterniond noisyQuat = simNoisyRotation(quat, rotError);
  Eigen::Matrix3d noisyRotation = noisyQuat.toRotationMatrix();

  Eigen::Vector3d noisyTranslation =
      simNoisyTranslation(translation, transError);

  // check
  Eigen::AngleAxisd aa(quat.inverse() * Eigen::Quaterniond(noisyRotation));
  std::cout << aa.angle() << " should be " << rotError << ".\n";
  double delta = (noisyTranslation - translation).norm();
  std::cout << delta << " should be " << transError << ".\n";

  // print
  std::cout << "Noisy rotation and translation:\n";
  std::cout << "  data: [" << std::setprecision(12);
  int i = 0;
  for (; i < 3; ++i) {
    int j = 0;
    for (; j < 2; ++j) {
      std::cout << noisyRotation(i, j) << ", ";
    }
    if (i == 2)
      std::cout << noisyRotation(i, j) << "]\n\n";
    else
      std::cout << noisyRotation(i, j) << ",\n          ";
  }

  std::cout << "  data: [" << noisyTranslation[0] << ", " << noisyTranslation[1]
            << ", " << noisyTranslation[2] << "]\n\n";

  return 0;
}
