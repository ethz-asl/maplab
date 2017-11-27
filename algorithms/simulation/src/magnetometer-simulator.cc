#include "simulation/magnetometer-simulator.h"

#include <random>

#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/time.h>
#include <maplab-common/capture-std-output.h>
#include <maplab-common/parallel-process.h>

#include "simulation/external/geomagnetism.h"
#include "simulation/external/world-magnetic-model.h"

namespace simulation {

struct MagnetometerData {
  MAGtype_MagneticModel *MagneticModel, *TimedMagneticModel;
  MAGtype_Ellipsoid Ellip;
  MAGtype_Geoid Geoid;
};

MagnetometerSimulator::MagnetometerSimulator(const YearMonthDay& utc_date) {
  world_magnetic_model_.reset(new WorldMagneticModel(utc_date));
  add_noise_ = false;
}

MagnetometerSimulator::MagnetometerSimulator(
    const YearMonthDay& utc_date,
    const MagnetometerNoiseSettings& noise_settings)
    : noise_settings_(noise_settings) {
  world_magnetic_model_.reset(new WorldMagneticModel(utc_date));
  add_noise_ = true;
}

void MagnetometerSimulator::simulateMeasurements(
    const Eigen::VectorXd& timestamps_nanoseconds,
    const aslam::TransformationVector& T_G_Bs,
    Aligned<std::vector, Eigen::Vector3d>* magnetic_fields_tesla) {
  CHECK_EQ(timestamps_nanoseconds.rows(), static_cast<int>(T_G_Bs.size()));
  CHECK_NOTNULL(magnetic_fields_tesla);
  size_t num_samples = static_cast<size_t>(timestamps_nanoseconds.rows());

  auto createMagneticFieldForPose =
      [&](const std::vector<size_t>& sample_idx_range) {
        for (size_t sample_idx : sample_idx_range) {
          CHECK_LT(sample_idx, num_samples);
          aslam::Transformation T_B_G = T_G_Bs[sample_idx].inverse();
          Eigen::Matrix3d R_B_G = T_B_G.getRotationMatrix();
          // Calculate the reference field from world magnetic model.
          Eigen::Vector3d reference_field;
          world_magnetic_model_->calculateMagneticField(
              T_B_G.getPosition(), &reference_field);

          Eigen::Vector3d magnetic_field_tesla;
          if (add_noise_) {
            // Calculate difference in time from timestamps.
            double dt_nanoseconds;
            if (sample_idx > 0) {
              dt_nanoseconds = timestamps_nanoseconds[sample_idx] -
                               timestamps_nanoseconds[sample_idx - 1];
            } else {
              dt_nanoseconds = timestamps_nanoseconds[sample_idx + 1] -
                               timestamps_nanoseconds[sample_idx];
            }
            // Add noise.
            // TODO(hitimo): More sophisticated magnetometer noise model.
            std::default_random_engine generator;
            std::normal_distribution<double> distribution(0.0, 1.0);
            Eigen::Vector3d normal_samples;
            for (size_t i = 0u; i < 3; ++i) {
              normal_samples(i) = distribution(generator);
            }

            magnetic_field_tesla =
                noise_settings_.misalignment_matrix * R_B_G * reference_field +
                noise_settings_.bias_vector +
                sqrt(
                    noise_settings_.power_spectral_density_noise /
                    (dt_nanoseconds * 1.0e-9)) *
                    normal_samples;
          } else {
            magnetic_field_tesla = R_B_G * reference_field;
          }
          magnetic_fields_tesla->emplace_back(magnetic_field_tesla);
        }
      };

  // Iterate over all samples.
  LOG(INFO) << "Simulating Magnetometer... ";
  const bool kAlwaysParallelize = true;
  const size_t num_threads = common::getNumHardwareThreads();
  common::ParallelProcess(
      num_samples, createMagneticFieldForPose, kAlwaysParallelize, num_threads);
}

void MagnetometerSimulator::calculateMagneticField(
    const Eigen::Vector3d& llh, const Eigen::Matrix3d& R_B_G,
    Eigen::Vector3d* magnetic_field_tesla) {
  CHECK_NOTNULL(magnetic_field_tesla);
  // Calculate the reference field from world magnetic model.
  Eigen::Vector3d reference_field;
  world_magnetic_model_->calculateMagneticField(llh, &reference_field);
  *magnetic_field_tesla = R_B_G * reference_field;
}

WorldMagneticModel::WorldMagneticModel(const YearMonthDay& utc_date) {
  data_.reset(new MagnetometerData);
  loadWorldMagneticModel();
  utc_date_ = utc_date;
}

WorldMagneticModel::~WorldMagneticModel() {
  MAG_FreeMagneticModelMemory(data_->MagneticModel);
  MAG_FreeMagneticModelMemory(data_->TimedMagneticModel);
}

void WorldMagneticModel::loadWorldMagneticModel() {
  // Memory allocation: Storing the WMM Model parameters.
  data_->MagneticModel = MAG_AllocateModelMemory(kNumberTerms);

  // Memory allocation: Storing the time modified WMM Model parameters.
  data_->TimedMagneticModel = MAG_AllocateModelMemory(kNumberTerms);

  if (data_->MagneticModel == nullptr || data_->TimedMagneticModel == nullptr) {
    LOG(FATAL) << "Cannot allocate memory for magnetic model!";
  }

  data_->Geoid.UseGeoid = 0;
  MAG_SetDefaults(&data_->Ellip, &data_->Geoid);

  // Feed the magnetic model with the current official magnetic field
  // parameters.
  data_->MagneticModel->Main_Field_Coeff_G[0] = 0.0;
  data_->MagneticModel->Secular_Var_Coeff_G[0] = 0.0;
  data_->MagneticModel->Main_Field_Coeff_H[0] = 0.0;
  data_->MagneticModel->Secular_Var_Coeff_H[0] = 0.0;
  for (int i = 0; i < kNumberTerms - 1; i++) {
    int n = static_cast<int>(kMagneticCoefficients[i][0]);
    int m = static_cast<int>(kMagneticCoefficients[i][1]);
    if (m <= n) {
      int index = (n * (n + 1) / 2 + m);
      data_->MagneticModel->Main_Field_Coeff_G[index] =
          kMagneticCoefficients[i][2];
      data_->MagneticModel->Secular_Var_Coeff_G[index] =
          kMagneticCoefficients[i][4];
      data_->MagneticModel->Main_Field_Coeff_H[index] =
          kMagneticCoefficients[i][3];
      data_->MagneticModel->Secular_Var_Coeff_H[index] =
          kMagneticCoefficients[i][5];
    }
  }
  data_->MagneticModel->epoch = 2015.0;
  strcpy(data_->MagneticModel->ModelName, "WMM-2015");  // NOLINT

  data_->MagneticModel->nMax = kMaximumDegreeModel;
  data_->MagneticModel->nMaxSecVar = kMaximumDegreeModel;
  data_->TimedMagneticModel->nMax = kMaximumDegreeModel;
  data_->TimedMagneticModel->nMaxSecVar = kMaximumDegreeModel;
}

void WorldMagneticModel::calculateMagneticField(
    const Eigen::Vector3d& llh, Eigen::Vector3d* magnetic_field_tesla) {
  CHECK_NOTNULL(magnetic_field_tesla);
  // Set time of request.
  MAGtype_Date UserDate;
  UserDate.Year = utc_date_.year;
  UserDate.Month = utc_date_.month;
  UserDate.Day = utc_date_.day;

  // Catch unsupported user date.
  char error[255];
  if (!MAG_DateToYear(&UserDate, error)) {
    LOG(FATAL);
  }

  MAGtype_CoordGeodetic CoordGeodetic;
  // Geodetic latitude in degree.
  CoordGeodetic.phi = llh(0);
  // Geodetic longitude in degree.
  CoordGeodetic.lambda = llh(1);
  // Conversion from meter to kilometer.
  CoordGeodetic.HeightAboveEllipsoid = llh(2) / 1000.0;

  // Convert from geodetic to spherical representation: Equations: 17-18, WMM
  // Technical report:
  // Convert geodetic coordinates, (defined by the WGS-84 reference ellipsoid),
  // to Earth Centered Earth Fixed Cartesian coordinates, and then to spherical
  // coordinates.
  MAGtype_CoordSpherical CoordSpherical;
  MAG_GeodeticToSpherical(data_->Ellip, CoordGeodetic, &CoordSpherical);

  // Time adjust the coefficients, Equation 19, WMM Technical report.
  MAG_TimelyModifyMagneticModel(
      UserDate, data_->MagneticModel, data_->TimedMagneticModel);

  // Computes the geoMagnetic field elements and their time change.
  MAGtype_GeoMagneticElements GeoMagneticElements;
  MAG_Geomag(
      data_->Ellip, CoordSpherical, CoordGeodetic, data_->TimedMagneticModel,
      &GeoMagneticElements);

  if (VLOG_IS_ON(100)) {
    MAGtype_GeoMagneticElements magnetic_field_errors;
    MAG_WMMErrorCalc(GeoMagneticElements.H, &magnetic_field_errors);
    {
      common::RedirectStreamToGlog<> capture(std::cout, google::INFO, 100);
      MAG_PrintUserDataWithUncertainty(
          GeoMagneticElements, magnetic_field_errors, CoordGeodetic, UserDate,
          data_->TimedMagneticModel, &data_->Geoid);
    }
  }

  *magnetic_field_tesla =
      Eigen::Vector3d(
          GeoMagneticElements.X, GeoMagneticElements.Y, GeoMagneticElements.Z) *
      kNanoTeslaToTesla;
}

}  // namespace simulation
