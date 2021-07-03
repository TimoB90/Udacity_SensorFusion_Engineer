#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
#include <iostream>
using namespace std;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // set state dimension
  n_x_ = 5;
    
  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd::Identity(5, 5);
    
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.8;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
   is_initialized_ = false;

   // set augmented dimension
   n_aug_  = n_x_ + 2;
   lambda_ = 3 - n_aug_;
   aug_dim = 2 * n_aug_ + 1;
  
    // Setup augmented weights vector
    weights_ = VectorXd(aug_dim);
    double weight_0 = lambda_/(lambda_+n_aug_);
    double weight = 0.5/(lambda_+n_aug_);
    weights_.fill(weight);
    weights_(0) = weight_0;
  
  
     // define spreading parameter
     Xsig_pred_ = MatrixXd(n_x_, aug_dim);
  
 	 //.. Initialize measurement noise covairance matix
  	 R_radar_ = MatrixXd(3, 3);
 	 R_radar_ << std_radr_*std_radr_, 0, 0,
           	     0, std_radphi_*std_radphi_, 0,
                 0, 0, std_radrd_*std_radrd_;

 	 R_lidar_ = MatrixXd(2,2);
 	 R_lidar_ << std_laspx_*std_laspx_, 0,
             	 0, std_laspy_*std_laspy_;

    //.. the current NIS for radar
    NIS_radar_ = 0.0;

    //.. the current NIS for laser
    NIS_laser_ = 0.0;
}

UKF::~UKF() {}

// Initialize each UKF instance exactly once
void UKF::InitializeUKF(MeasurementPackage meas_package) {

  // Use the first measurement to set the value of the mean state
  x_.fill(0.0);
  x_.head(2) << meas_package.raw_measurements_;

  time_us_ = meas_package.timestamp_;

  is_initialized_ = true;
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
    if (!is_initialized_)
    {
      // For the initial measurement, skip the prediction step
      InitializeUKF(meas_package);
    }
    
   // On each subsequent measurement, trigger a full prediction/update cycle
   double delta_t = (meas_package.timestamp_ - time_us_) / 1e6;
   time_us_ = meas_package.timestamp_;
    
   Prediction(delta_t);

   if (MeasurementPackage::SensorType::LASER == meas_package.sensor_type_)
   {
     UpdateLidar(meas_package);
   }
   else if (MeasurementPackage::SensorType::RADAR == meas_package.sensor_type_)
   {
     UpdateRadar(meas_package);
   }
}



void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
    
   // 1. Step
       //.. Generate augmented sigma points
       VectorXd x_aug = VectorXd(n_aug_);                   // create augmented mean vector
       MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);           // create augmented state covariance
       MatrixXd Xsig_aug = MatrixXd(n_aug_, aug_dim);       // create sigma point matrix
       
       // create augmented mean state
       x_aug.head(n_x_) = x_;
       x_aug(n_x_) = 0;
       x_aug(n_x_ + 1) = 0;
        
       // create augmented covariance matrix
       P_aug.fill(0.0);
       P_aug.topLeftCorner(n_x_,n_x_) = P_;
       P_aug(n_x_,n_x_) = std_a_ * std_a_;
       P_aug(n_x_ + 1 , n_x_ + 1) = std_yawdd_ * std_yawdd_;
        
       // create square root matrix
       MatrixXd L = P_aug.llt().matrixL();
    
        // create augmented sigma points
        Xsig_aug.col(0)  = x_aug;
  
        for (int i = 0; i< n_aug_; i++)
        {
          Xsig_aug.col(i+1)       	 = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
          Xsig_aug.col(i+1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
        }
    
    // 2. Step
        // predict sigma points
         for (int i = 0; i < aug_dim; i++)
         {
           // extract values for better readability
           double p_x = Xsig_aug(0,i);
           double p_y = Xsig_aug(1,i);
           double v = Xsig_aug(2,i);
           double yaw = Xsig_aug(3,i);
           double yawd = Xsig_aug(4,i);
           double nu_a = Xsig_aug(5,i);
           double nu_yawdd = Xsig_aug(6,i);

           // predicted state values
           double px_p, py_p, v_p, yaw_p, yawd_p;

           // avoid division by zero
           if (fabs(yawd) > 0.001)
           {
               px_p = p_x + v / yawd * ( sin (yaw + yawd * delta_t) - sin(yaw));
               py_p = p_y + v / yawd * ( cos(yaw) - cos(yaw + yawd * delta_t) );
           }
           else
           {
               px_p = p_x + v * delta_t * cos(yaw);
               py_p = p_y + v * delta_t * sin(yaw);
           }

           v_p = v;
           yaw_p = yaw + yawd * delta_t;
           yawd_p = yawd;

           // add noise
           px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
           py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
           v_p = v_p + nu_a * delta_t;
             
           yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
           yawd_p = yawd_p + nu_yawdd * delta_t;

           // write predicted sigma point into right column
           Xsig_pred_(0,i) = px_p;
           Xsig_pred_(1,i) = py_p;
           Xsig_pred_(2,i) = v_p;
           Xsig_pred_(3,i) = yaw_p;
           Xsig_pred_(4,i) = yawd_p;
         }
    
    // 3. Step
        // predicted state mean
        // weights still defined in line 77
        x_.fill(0.0);
        for (int i = 0; i < aug_dim; i++)
        {
          x_ = x_ + weights_(i) * Xsig_pred_.col(i);
        }
    
        // predicted state covariance matrix
         P_.fill(0.0);
         for (int i = 0; i < aug_dim; ++i)
         {  
           // state difference
           VectorXd x_diff = Xsig_pred_.col(i) - x_;
             
           // angle normalization
           while (x_diff(3)> M_PI)
           {
               x_diff(3) -= 2.* M_PI;
           }
           while (x_diff(3)< -M_PI)
           {
               x_diff(3) += 2. * M_PI;
           }

           P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
         }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
    
    // Lidar: 4. Step
         //.. extract measurement as VectorXd
         VectorXd z_ = meas_package.raw_measurements_;

         int n_z_ = 2;                                       // lidar data contains position x and y of target
    
         MatrixXd Zsig = MatrixXd(n_z_, aug_dim);            // create matrix for sigma points
         VectorXd z_pred = VectorXd(n_z_);                   // mean predicted measurement
         MatrixXd S = MatrixXd(n_z_,n_z_);                   // measurement covariance matrix S
    
        // transform sigma points into measurement space
        // lidar: no transformation needed
        for (int i = 0; i < aug_dim; i++)
        {
          // measurement model
          Zsig(0,i) = Xsig_pred_(0,i);
          Zsig(1,i) = Xsig_pred_(1,i);
        }
    
        // mean predicted measurement
        z_pred.fill(0.0);
        for (int i=0; i < aug_dim; ++i)
        {
          z_pred = z_pred + weights_(i) * Zsig.col(i);
        }

        // innovation covariance matrix S
        S.fill(0.0);
        for (int i = 0; i < aug_dim; ++i)
        {
          // residual
          VectorXd z_diff = Zsig.col(i) - z_pred;
          S = S + weights_(i) * z_diff * z_diff.transpose();
        }
    
        // add measurement noise covariance matrix
        // R_lidar_ still defined in line 86
        S = S + R_lidar_;
    
    // Lidar: 5. Step
        MatrixXd Tc = MatrixXd(n_x_, n_z_);               // create matrix for cross correlation Tc
    
        // calculate cross correlation matrix
        Tc.fill(0.0);
        for (int i = 0; i < aug_dim; ++i)
        {
          // residual
          VectorXd z_diff = Zsig.col(i) - z_pred;
          // state difference
          VectorXd x_diff = Xsig_pred_.col(i) - x_;

          Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
        }
    
        // Kalman gain K;
        MatrixXd K = Tc * S.inverse();

        // residual
        VectorXd z_diff = z_ - z_pred;

        // update state mean and covariance matrix
        x_ = x_ + K * z_diff;
        P_ = P_ - K * S * K.transpose();
       
       
    // Lidar: 6.Step
        //.. calculate NIS
        NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
  		cout << "Laser NIS (2-df X^2, 95% < 5.991) = " << NIS_laser_ << endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
    
   // Radar: 4. Step
        //.. extract measurement as VectorXd
        VectorXd z_ = meas_package.raw_measurements_;

        int n_z_ = z_.size();  //Vector 3x1
    
        MatrixXd Zsig = MatrixXd(n_z_, aug_dim);            // create matrix for sigma points in measurement space
        VectorXd z_pred = VectorXd(n_z_);                   // mean predicted measurement
        MatrixXd S = MatrixXd(n_z_,n_z_);                   // measurement covariance matrix S
    
        // transform sigma points into measurement space
        for (int i = 0; i < aug_dim; ++i)
        {
          // extract values for better readability
          double p_x  = Xsig_pred_(0,i);
          double p_y  = Xsig_pred_(1,i);
          double v    = Xsig_pred_(2,i);
          double yaw  = Xsig_pred_(3,i);
          double yawd = Xsig_pred_(4,i);

          double rho  = sqrt(pow(p_x, 2) + pow(p_y, 2));
          double phi  = atan2(p_y, p_x);
          double rhod = (p_x * cos(yaw) * v + p_y * sin(yaw) * v) / rho;

          // measurement model
          Zsig(0,i) = rho;             // rho
          Zsig(1,i) = phi;             // phi
          Zsig(2,i) = rhod;			   // rho_dot
        }
    
        // mean predicted measurement
        z_pred.fill(0.0);
        for (int i=0; i < aug_dim; i++)
        {
          z_pred = z_pred + weights_(i) * Zsig.col(i);
        }
    
        // innovation covariance matrix S
        S.fill(0.0);
        for (int i = 0; i < aug_dim; ++i)
        {
          // residual
          VectorXd z_diff = Zsig.col(i) - z_pred;
          // angle normalization
          while (z_diff(1) > M_PI)
          {
              z_diff(1)-=2.*M_PI;
          }
          while (z_diff(1)<-M_PI)
          {
              z_diff(1)+=2.*M_PI;
          }
            
          S = S + weights_(i) * z_diff * z_diff.transpose();
        }
    
        // add measurement noise covariance matrix  
        S = S + R_radar_;
    
    
    // Radar:  5. Step
        MatrixXd Tc = MatrixXd(n_x_, n_z_);               // create matrix for cross correlation Tc
    
        // calculate cross correlation matrix
        Tc.fill(0.0);
        for (int i = 0; i < aug_dim; i++)
        {
          // residual
          VectorXd z_diff = Zsig.col(i) - z_pred;
          // angle normalization
          while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
          while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

          // state difference
          VectorXd x_diff = Xsig_pred_.col(i) - x_;
          // angle normalization
          while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
          while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

          Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
        }
    
        // Kalman gain K;
        MatrixXd K = Tc * S.inverse();

        // residual
        VectorXd z_diff = z_ - z_pred;

        // angle normalization
        while (z_diff(1) > M_PI)
        {
        	z_diff(1) -= 2. * M_PI;
        }
        while (z_diff(1) <- M_PI)
        {
       	    z_diff(1) += 2. * M_PI;
        }

        // update state mean and covariance matrix
        x_ = x_ + K * z_diff;
        P_ = P_ - K * S * K.transpose();
    
    
    // Radar: 6.Step
        //.. calculate NIS
        NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
		cout << "Radar NIS (3-df X^2, 95% < 7.815) = " << NIS_radar_ << endl;
}
