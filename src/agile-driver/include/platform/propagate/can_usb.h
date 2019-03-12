/*
 * can_usb.h
 *
 *  Created on: Apr 26, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_PLATFORM_PROPAGATE_CAN_USB_H_
#define INCLUDE_PLATFORM_PROPAGATE_CAN_USB_H_

#include "propagate.h"

#include "controlcan.h"
#include <boost/lockfree/queue.hpp>

class TimeControl;

namespace agile_robot {

class CanUsb: public Propagate {
public:
  CanUsb();
  virtual bool auto_init() override;

  virtual ~CanUsb();

public:

  virtual bool start() override;
  virtual void stop()  override;

//  virtual bool write(const Packet&) override;
//  virtual bool read(Packet&)        override;

protected:
  boost::lockfree::queue<VCI_CAN_OBJ>* recv_buffer_;//VCI_CAN_OBJ  CAN帧结构体
  boost::lockfree::queue<VCI_CAN_OBJ>* send_buffer_;
  bool         connected_;
  uint64_t     tick_r_interval_;
  uint64_t     tick_w_interval_;

  virtual void do_exchange_w();
  virtual void do_exchange_r();

private:
  int      recv_buf_size_;
  PVCI_CAN_OBJ recv_msgs_;//VCI_CAN_OBJ结构体是CAN帧结构体
  PVCI_CAN_OBJ send_msgs_;

  VCI_INIT_CONFIG config_;//CI_INIT_CONFIG结构体定义了初始化CAN的配置。结构体将在VCI_InitCan函数中被
  //填充,即初始化之前,要先填好这个结构体变量

  // TimeControl* __timer;
};

} /* namespace agile_robot */

#endif /* INCLUDE_PLATFORM_PROPAGATE_CAN_USB_H_ */
