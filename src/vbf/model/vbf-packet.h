/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 IITP RAS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Based on
 *      NS-2 vbf model developed by the CMU/MONARCH group and optimized and
 *      tuned by Samir Das and Mahesh Marina, University of Cincinnati;
 *
 *      vbf-UU implementation by Erik Nordström of Uppsala University
 *      http://core.it.uu.se/core/index.php/vbf-UU
 *
 * Authors: Elena Buchatskaia <borovkovaes@iitp.ru>
 *          Pavel Boyko <boyko@iitp.ru>
 */
#ifndef vbfPACKET_H
#define vbfPACKET_H

#include <iostream>
#include "ns3/header.h"
#include "ns3/enum.h"
#include "ns3/ipv4-address.h"
#include <map>
#include "ns3/nstime.h"

namespace ns3 {
namespace vbf {

/**
* \ingroup vbf
* \brief MessageType enumeration
*/
enum MessageType
{
  vbfTYPE_VBF  = 1,   //!< vbfTYPE
};


/**
* \ingroup vbf Header
* \brief Route vbfHeader Message Format
  \verbatim
  0                   1                   2                   3
  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |     Type      |            pakcetNum          |    hopCount   |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                       targetIpAddress                         |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                       sourceIpAddress                         |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-|
  |                      forwarderIpAddress                       |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                        m_timegenerate                         |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                        m_timeforward                          |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  \endverbatim
*/
class vbfHeader : public Header
{
public:
  /// constructor
  vbfHeader ();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId ();
  TypeId GetInstanceTypeId () const;
  uint32_t GetSerializedSize () const;
  void Serialize (Buffer::Iterator i) const;
  uint32_t Deserialize (Buffer::Iterator start);
  void Print (std::ostream &os) const;

  vbfHeader (MessageType messageType, uint16_t pkNum, uint8_t hopCount, 
                      Ipv4Address targetAddr, Ipv4Address sourceAddr, Ipv4Address forwardAddr,
                      Time timeGenerate, Time timeForward);

  //设置Header信息
  void SetMessType(uint8_t messageType)
  {
    m_messType = messageType;
  }
  void SetPkNum(uint16_t pkNum)
  {
    m_pkNum = pkNum;
  }
  void SetHopCount(uint8_t hopCount)
  {
    m_hopCount = hopCount;
  }
  void SetTargetAddr(Ipv4Address targetAddr)
  {
    m_targetAddr = targetAddr;
  }
  void SetSenderAddr(Ipv4Address senderAddr)
  {
    m_sourceAddr = senderAddr;
  }
  void SetForwardAddr(Ipv4Address forwardAddr)
  {
    m_forwardAddr = forwardAddr;
  }
  void SetTimeGenerate(Time timeGenerate)
  {
    m_timegenerate = uint32_t(timeGenerate.GetSeconds());
  }
  void SetTimeForward(Time timeForward)
  {
    m_timeforward = uint32_t(timeForward.GetSeconds());
  }

  //获取Header信息
  uint8_t GetMessType()
  {
    return m_messType;
  }
  uint16_t GetPkNum()
  {
    return m_pkNum;
  }
  uint8_t GetHopCount()
  {
    return m_hopCount;
  }
  Ipv4Address GetTargetAddr()
  {
    return m_targetAddr;
  }
  Ipv4Address GetSenderAddr()
  {
    return m_sourceAddr;
  }
  Ipv4Address GetForwardAddr()
  {
    return m_forwardAddr;
  }
  Time GetTimeGenerate()
  {
    return Time(m_timegenerate);
  }
  Time GetTimeForward()
  {
    return Time(m_timeforward);
  }


  /**
   * \brief Comparison operator
   * \param o VBF header to compare
   * \return true if the vbf headers are equal
   */
  bool operator== (vbfHeader const & o) const;

private:
    uint8_t m_messType;
    uint16_t m_pkNum;   //数据包的编号，每产生一个packet，m_pkNum+1.
    uint8_t m_hopCount; //包被转发的次数
    Ipv4Address   m_targetAddr;      //最终目标ip地址
    Ipv4Address   m_sourceAddr;   //源ip地址
    Ipv4Address   m_forwardAddr; //上一跳转发该packet的节点的ip地址
    uint32_t m_timegenerate;   //该packet被创建的时间，在路由过程保持不变
    uint32_t m_timeforward;      //上一跳节点转发该packet时的时间，每一跳会变化
};

/**
  * \brief Stream output operator
  * \param os output stream
  * \return updated stream
  */
std::ostream & operator<< (std::ostream & os, vbfHeader const &);

}  // namespace vbf
}  // namespace ns3

#endif /* vbfPACKET_H */
