// Fixed-size FK/Jacobian helpers copied from the existing checked controller; no hardware calls.
#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <thread>
#include <vector>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <franka/robot.h>

using Pose=std::array<double,16>;
using Q=std::array<double,7>;
using M4=Eigen::Matrix4d;
using V3=Eigen::Vector3d;
using J67=Eigen::Matrix<double,6,7>;
volatile std::sig_atomic_t interrupted=0;
void stop(int){interrupted=1;}
double blend(double x){x=std::clamp(x,0.0,1.0);return x*x*x*(10+x*(-15+6*x));}
M4 matrix(const Pose& p){return Eigen::Map<const M4>(p.data());}
V3 xyz(const Pose& p){return {p[12],p[13],p[14]};}
double angle(const Pose& a,const Pose& b){return Eigen::Quaterniond(matrix(a).topLeftCorner<3,3>()).normalized().angularDistance(Eigen::Quaterniond(matrix(b).topLeftCorner<3,3>()).normalized());}
struct Metrics{double full,fixed,condition,q2,extension;};
struct Model{
  std::array<M4,7> origins;M4 ee;double length=.3;
  std::pair<M4,J67> evaluate(const Q& q)const{
    M4 t=M4::Identity();std::array<V3,7> axis,position;
    for(int i=0;i<7;++i){
      t=t*origins[i];axis[i]=t.block<3,1>(0,2);position[i]=t.block<3,1>(0,3);
      M4 rotation=M4::Identity();rotation.topLeftCorner<3,3>()=Eigen::AngleAxisd(q[i],V3::UnitZ()).toRotationMatrix();t=t*rotation;
    }
    t=t*ee;J67 j;
    for(int i=0;i<7;++i){j.block<3,1>(0,i)=axis[i].cross(t.block<3,1>(0,3)-position[i])/length;j.block<3,1>(3,i)=axis[i];}
    return {t,j};
  }
  Metrics metrics(const Q& q)const{
    const auto j=evaluate(q).second;Eigen::Matrix<double,6,6> fixed;
    constexpr int columns[6]={0,1,3,4,5,6};for(int i=0;i<6;++i)fixed.col(i)=j.col(columns[i]);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> a(j*j.transpose(),Eigen::EigenvaluesOnly),b(fixed*fixed.transpose(),Eigen::EigenvaluesOnly);
    if(a.info()!=Eigen::Success||b.info()!=Eigen::Success)throw std::runtime_error("Jacobian eigensolver failed");
    double full=std::sqrt(std::max(0.0,a.eigenvalues()[0])),f=std::sqrt(std::max(0.0,b.eigenvalues()[0]));
    double condition=std::max(std::sqrt(a.eigenvalues()[5])/std::max(full,1e-12),std::sqrt(b.eigenvalues()[5])/std::max(f,1e-12));
    return {full,f,condition,std::abs(q[1]),std::abs(q[3]+.467002423653011)};
  }
};
