/*
 * joint_manager.cpp
 *
 *  Created on: Sep 1, 2017
 *      Author: silence
 */

#include "repository/joint_manager.h"
#include "repository/motor.h"

#include "foundation/cfg_reader.h"

#ifndef NAN
#define NAN  (0.0/0.0)
#endif

namespace agile_robot {

SINGLETON_IMPL(JointManager)

JointManager::JointManager()
  : ResourceManager<Joint>(),
    jnt_mode_(JntCmdType::CMD_POS) {
  jnt_list_by_type_.resize(LegType::N_LEGS);
  for (auto& leg : jnt_list_by_type_) {
    leg.resize(JntType::N_JNTS);
    for (auto& jnt : leg)
      jnt = nullptr;
  }
}

JointManager::~JointManager() {
  // Nothing to do here
  for (auto& j : res_list_)
    j->stop();
}

void JointManager::add(Joint* cr) {
  auto _res = Label::getHardwareByName<Joint>(cr->getLabel());
  if (nullptr == _res) {
    LOG_ERROR << "Cannot found the joint labeled " << cr->getLabel();
    return;
  }

  jnt_list_by_name_.insert(std::make_pair(_res->joint_name(), _res));
  jnt_list_by_type_[_res->leg_type()][_res->joint_type()]   = _res;

  // LOG_DEBUG << "JointManager has received a joint -- " << _res->getLabel();
}

JointManager::iterator JointManager::find(const std::string& _n) {
  for (JointManager::iterator itr = JointManager::res_list_.begin();
      itr != JointManager::res_list_.end(); ++itr) {

    if (0 == _n.compare((*itr)->joint_name())) return itr;
  }

  return end();
}

void JointManager::addJointCommand(LegType owner, JntType type, double val) {
  if (nullptr != jnt_list_by_type_[owner][type])
    jnt_list_by_type_[owner][type]->updateJointCommand(val);
}

void JointManager::addJointCommand(const std::string& name, double val) {
  if (jnt_list_by_name_.end() != jnt_list_by_name_.find(name))
    jnt_list_by_name_[name]->updateJointCommand(val);
}

MiiPtr<Joint> JointManager::getJointHandle(LegType owner, JntType type) {
  return jnt_list_by_type_[owner][type];
}

MiiPtr<Joint> JointManager::getJointHandle(const std::string& _jn) {
  auto itr = jnt_list_by_name_.find(_jn);
  if (jnt_list_by_name_.end() == itr)
    return nullptr;

  return itr->second;
}

void JointManager::setJointCommandMode(JntCmdType type) {
  jnt_mode_ = type;
}

const JntCmdType& JointManager::getJointCommandMode() {
  return jnt_mode_;
}

double JointManager::operator()(LegType owner, JntType type, JntDataType data) {
  if (nullptr == jnt_list_by_type_[owner][type]) return NAN;

  switch (data) {
  case JntDataType::POS: return jnt_list_by_type_[owner][type]->joint_position();
  case JntDataType::VEL: return jnt_list_by_type_[owner][type]->joint_velocity();
  case JntDataType::TOR: return jnt_list_by_type_[owner][type]->joint_torque();
  default: return 0;
  }
}

const double* JointManager::joint_position_const_pointer(LegType _owner, JntType _type) {
  return jnt_list_by_type_[_owner][_type]->joint_position_const_pointer();
}
const double* JointManager::joint_velocity_const_pointer(LegType _owner, JntType _type) {
  return jnt_list_by_type_[_owner][_type]->joint_velocity_const_pointer();
}
const double* JointManager::joint_torque_const_pointer  (LegType _owner, JntType _type) {
  return jnt_list_by_type_[_owner][_type]->joint_torque_const_pointer();
}

void JointManager::joint_position_const_pointer(LegType _owner, JntType _type, const double* & _c_p) {
  _c_p = jnt_list_by_type_[_owner][_type]->joint_position_const_pointer();
}
void JointManager::joint_velocity_const_pointer(LegType _owner, JntType _type, const double* & _c_p) {
  _c_p = jnt_list_by_type_[_owner][_type]->joint_velocity_const_pointer();
}
void JointManager::joint_torque_const_pointer  (LegType _owner, JntType _type, const double* & _c_p) {
  _c_p = jnt_list_by_type_[_owner][_type]->joint_torque_const_pointer();
}
// override
void JointManager::joint_position_const_pointer(const std::string& _n, const double* & _c_p) {
  _c_p = jnt_list_by_name_[_n]->joint_position_const_pointer();
}
void JointManager::joint_velocity_const_pointer(const std::string& _n, const double* & _c_p) {
  _c_p = jnt_list_by_name_[_n]->joint_velocity_const_pointer();
}
void JointManager::joint_torque_const_pointer(const std::string& _n, const double* & _c_p) {
  _c_p = jnt_list_by_name_[_n]->joint_torque_const_pointer();
}

void JointManager::joint_names(std::vector<std::string>& names) {
  names.clear();
  for (auto jnt : res_list_) {
    names.push_back(jnt->joint_name());
  }
}
// override
void JointManager::joint_position_const_pointer(std::vector<const double*>& _c_ps) {
  _c_ps.clear();
  for (auto jnt : res_list_) {
    _c_ps.push_back(jnt->joint_position_const_pointer());
  }
}
void JointManager::joint_velocity_const_pointer(std::vector<const double*>& _c_ps) {
  _c_ps.clear();
  for (auto jnt : res_list_) {
    _c_ps.push_back(jnt->joint_velocity_const_pointer());
  }
}
void JointManager::joint_torque_const_pointer(std::vector<const double*>& _c_ps) {
  _c_ps.clear();
  for (auto jnt : res_list_) {
    _c_ps.push_back(jnt->joint_torque_const_pointer());
  }
}

//cog,lft-leg,rgt-leg
void JointManager::get_cog(std::function<void(MiiPtr<Joint>&)> cb){

}

void JointManager::get_lft_leg(std::function<void(MiiPtr<Joint>&)> cb){
 
}

void JointManager::get_rgt_leg(std::function<void(MiiPtr<Joint>&)> cb){
 
}


void JointManager::foreach(std::function<void(MiiPtr<Joint>&)> cb) {
  for (auto& j : jnt_list_by_name_) cb(j.second);
}

void JointManager::foreach(LegType leg, std::function<void(MiiPtr<Joint>&)> cb) {
  for (auto& j : jnt_list_by_type_[leg]) cb(j);
}

void JointManager::foreach(JntType jnt, std::function<void(MiiPtr<Joint>&)> cb) {
  for (auto& js : jnt_list_by_type_) cb(js[jnt]);
}

void JointManager::foreach(const std::vector<LegType>& legs, std::function<void(MiiPtr<Joint>&)> cb) {
  for (const auto& leg : legs)
    for (auto& j : jnt_list_by_type_[leg]) cb(j);
}

void JointManager::foreach(const std::vector<JntType>& js, std::function<void(MiiPtr<Joint>&)> cb) {
  for (auto& legs : jnt_list_by_type_)
    for (auto& j : js) cb(legs[j]);
}

void JointManager::foreach(const std::vector<std::string>& js, std::function<void(MiiPtr<Joint>&)> cb) {
  for (const auto& j : js) cb(jnt_list_by_name_[j]);
}

/*JointManager::iterator JointManager::find(const std::string& _n) {
  for (auto itr = res_list_.begin(); itr != res_list_.end(); ++itr) {
    if (itr->joint_name() == _n) return itr;
  }
  return end();
}*/

} /* namespace middleware */
