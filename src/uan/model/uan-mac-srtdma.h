/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 University of Washington
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
 * Author: Leonard Tracy <lentracy@gmail.com>
 */

#ifndef UAN_MAC_SRTDMA_H
#define UAN_MAC_SRTDMA_H

#include "uan-mac.h"
#include "ns3/mac8-address.h"
#include "ns3/node-list.h"
#include <map>

//着色表，根据特定网络结构修改
uint32_t Color1[] = {3,10,17,24,31,38,45,52,59,66,73,80,87,94,101};
std::set<uint32_t> Color1Set(std::begin(Color1), std::end(Color1));
uint32_t Color2[] = {4,11,18,25,32,39,46,53,60,67,74,81,88,95,102};
std::set<uint32_t> Color2Set(std::begin(Color2), std::end(Color2));
uint32_t Color3[] = {1,8,15,22,29,36,43,50,57,64,71,78,85,92,99};
std::set<uint32_t> Color3Set(std::begin(Color3), std::end(Color3));
uint32_t Color4[] = {0,7,14,21,28,35,42,49,56,63,70,77,84,91,98};
std::set<uint32_t> Color4Set(std::begin(Color4), std::end(Color4));
uint32_t Color5[] = {2,9,16,23,30,37,44,51,58,65,72,79,86,93,100};
std::set<uint32_t> Color5Set(std::begin(Color5), std::end(Color5));
uint32_t Color6[] = {5,12,19,26,33,40,47,54,61,68,75,82,89,96};
std::set<uint32_t> Color6Set(std::begin(Color6), std::end(Color6));
uint32_t Color7[] = {6,13,20,27,34,41,48,55,62,69,76,83,90,97};
std::set<uint32_t> Color7Set(std::begin(Color7), std::end(Color7));

std::vector<std::set<uint32_t>*> ColorVector = {&Color1Set,&Color2Set,&Color3Set,&Color4Set,&Color5Set,&Color6Set,&Color7Set};
uint32_t myColorArray[7][7] = {
  {0,1,10,14,19,8,4},
  {1,0,5,20,17,13,7},
  {10,5,0,2,12,15,18},
  {14,20,2,0,6,11,16},
  {19,17,12,6,0,3,9},
  {8,13,15,11,3,0,21},
  {4,7,18,16,9,21,0}
};

namespace ns3
{

class UanPhy;
class UanTxMode;

/**
 * \ingroup uan
 *
 * ALOHA MAC Protocol, the simplest MAC protocol for wireless networks.
 *
 * Packets enqueued are immediately transmitted.  This MAC attaches
 * a UanHeaderCommon to outgoing packets for address information.
 * (The type field is not used)
 */
class UanMacSrTDMA : public UanMac
{
public:
  /** Default constructor */
  UanMacSrTDMA ();
  /** Dummy destructor, see DoDispose. */
  virtual ~UanMacSrTDMA ();
  /**
   * Register this type.
   * \return The TypeId.
   */
  static TypeId GetTypeId (void);


  // Inherited methods
  virtual bool Enqueue (Ptr<Packet> pkt, uint16_t protocolNumber, const Address &dest);
  virtual void SetForwardUpCb (Callback<void, Ptr<Packet>, uint16_t, const Mac8Address&> cb);
  virtual void AttachPhy (Ptr<UanPhy> phy);
  virtual void Clear (void);
  int64_t AssignStreams (int64_t stream);

private:
  /** The MAC address. */
  Mac8Address m_address;
  /** PHY layer attached to this MAC. */
  Ptr<UanPhy> m_phy;
  /** Forwarding up callback. */
  Callback<void, Ptr<Packet>, uint16_t, const Mac8Address& > m_forUpCb;
  /** Flag when we've been cleared. */
  bool m_cleared;

  //Node Color
  uint32_t m_color;
  //Node id
  uint32_t m_nodeid;
  //Address2Color Table
  static std::map<Address, uint32_t> m_Address2Color;
  //ColorQueue
  std::vector<std::list<Ptr<Packet>>>* m_ColorQueue;
  //Time transmission
  Time m_transTime;
  //Time propagation
  Time m_propTime;

  /**
   * Receive packet from lower layer (passed to PHY as callback).
   *
   * \param pkt Packet being received.
   * \param sinr SINR of received packet.
   * \param txMode Mode of received packet.
   */
  void RxPacketGood (Ptr<Packet> pkt, double sinr, UanTxMode txMode);

  /**
   * Packet received at lower layer in error.
   *
   * \param pkt Packet received in error.
   * \param sinr SINR of received packet.
   */
  void RxPacketError (Ptr<Packet> pkt, double sinr);

  /**
   * Get nodeId from the macAddress
   * \param macAddr the macAddress of a Netdevice
   * \return nodeid if find or -1 if not find
  */
  uint32_t
  GetNodeIdWithMacAddress(const Address &macAddr);

  /**
   * SendPacket
  */
  void
  SendPacket();

  /**
   * SetColor and Start to send packet
  */
  void SetColor();

  /**
   * True Send Packet From the ColorQueue
  */
  void
  SendPacketFromColorQueue(uint32_t destColor);

protected:
  virtual void DoDispose ();

};  // class UanMacSrTDMA

std::map<Address, uint32_t> UanMacSrTDMA::m_Address2Color;

} // namespace ns3

#endif /* UAN_MAC_ALOHA_H */
