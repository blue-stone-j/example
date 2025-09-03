#include <gtsam/geometry/Rot3.h>
//(roll，pitch, yaw)，参数为float
gtsam::Rot3 temRot = gtsam::Rot3::RzRyRx(a, b, c);
/*************************************************/
#include <gtsam/geometry/Pose3.h>
gtsam::Pose3 pose;           // 6 degree of fredom
pose = pose1.between(pose2); // pose = pose2 - pose1
//转换为6D位姿
x     = pose.translation().x();
y     = pose.translation().y();
z     = pose.translation().z();
roll  = pose.rotation().roll();
pitch = pose.rotation().pitch();
yaw   = pose.rotation().yaw();

#include <gtsam/slam/PriorFactor.h>
//PriorFactor 先验密度，相当于g2o中的一元边，只连接一个变量
/*************************************************/
#include <gtsam/slam/BetweenFactor.h>
//BetweenFactor 相当于g2o中的二元边，连接两个变量
/*----回环帧，添加between因子----*/
//poseLoop = pre - cur
gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(curLoopInd, preLoopInd, poseLoop, noiseLoop));
/*------------添加完成----------*/
/*----相邻帧，添加between因子----*/
//lastInd 因子图的上一节点序号；currInd 当前节点序号；poseBetween = currInd - lastInd
gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(lastInd, currInd, poseBetween, noiseBetween));
initialEstimate.insert(currInd, state2GtsamPose(currKeyState));
/*------------添加完成----------*/
/*************************************************/
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
//FactorGraph相当于构建一个函数模型，与具体的变量的取值Values无关
gtsam::NonlinearFactorGraph gtSAMgraph;
/*************************************************/
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
/*************************************************/
#include <gtsam/nonlinear/Marginals.h> //???
//Marginal:边缘化，因子图计算出了联合后验密度,对于每个位姿x的边缘后验密度,使用均值和协方差对其进行近似。
/*************************************************/
#include <gtsam/nonlinear/Values.h>
// a map(映射) from keys to values，可以当作数组、向量使用；
gtsam::Values graphValues;                                       //在声明时，value值的类型没有确定，在定义时确定值的类型???
gtsam::Pose3 isamPose = isamCurrentEstimate.at<gtsam::Pose3>(i); //一种用法
/*************************************************/
#include <gtsam/nonlinear/ISAM2.h>
ISAM2Params parameters;
//差值大于0.1需要重新线性化
parameters.relinearizeThreshold = 0.01;
//每当有1个值需要重新线性化时，对贝叶斯树进行更新
parameters.relinearizeSkip = 1;
//是否可以重新线性化任意变量
parameters.enableRelinearization = true;
//是否计算线性化误差默认false
parameters.evaluateNonlinearError = false;
//default: true 是否保存保存线性化结果，可以优化性能，但是当线性化容易计算时会造成相反效果
parameters.cacheLinearizedFactors = false;
//默认为QR分解，还可以选用CHOESKY分解，但CHOESKY求解数值不稳定
parameters.factorization = ISAM2Params::Factorization::QR;
// debug时key值形式默认
parameters.keyFormatter = DefaultKeyFormatter;
//是否计算返回ISAM2Result::detailedResults，会增加计算量
parameters.enableDetailedResults = true;
//是否只进行部分更新功能
parameters.enablePartialRelinearizationCheck = false;
//当要移除许多因子时，比如ISAM2做固定平滑时，启用此选项第一值进行插入时避免出现NULL值，但是会造成每当有新值插入时必须查找
parameters.findUnusedFactorSlots = false;

// SILENT → no output (what you want)
// ERROR → errors only
// TERMINATION → only convergence summary
// VALUES → values at each iteration
// DELTA, LINEAR, TRYLAMBDA, LAMBDA → progressively more detailed debug output
parameters.verbosity = gtsam::NonlinearOptimizerParams::SILENT;
/*************************************************/

gtsam::Vector3 prevVel_; // 3 degree of fredom，和eigen的矩阵完全相同



/*************************************************/



// construct preintegration with some parameters(noise, direction of gravity)
imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
//reset integration with initial bias; initial IMU bias is nessary for preintegration
resetIntegrationAndSetBias(prevBias_);
// update preintegratrion and covariance
integrateMeasurement(gtsam::Vector3(acc.x, acc.y, acc.z), gtsam::Vector3(angular_vel.x, angular_vel.y, angular_vel.z), dt);
// time span from ith frame to jth frame
deltaTij
    // calculate relative state;reckon state of next key frame with last farme;(state,bias)
    gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);


gtsam::imuBias::ConstantBias prevBias_; // imu bias
// prior state, which estimated state should be close to;(estimated state, prior constraint, confidence/weight)
gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
// construct imufactor by preintegration; imu constraint;
gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), //??? constraint between states; state of 2 neighbor frame should be close to it
gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // noise model;(dimension, value)
