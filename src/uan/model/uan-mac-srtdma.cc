/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Ljy <SCUT>
 */

#include "uan-mac-srtdma.h"
#include "uan-tx-mode.h"
#include "ns3/log.h"
#include "uan-phy.h"
#include "uan-header-common.h"

#include <iostream>

namespace ns3
{
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

// uint32_t Color1[] = {0};
// std::set<uint32_t> Color1Set(std::begin(Color1), std::end(Color1));
// uint32_t Color2[] = {1};
// std::set<uint32_t> Color2Set(std::begin(Color2), std::end(Color2));
// uint32_t Color3[] = {2};
// std::set<uint32_t> Color3Set(std::begin(Color3), std::end(Color3));
// uint32_t Color4[] = {3};
// std::set<uint32_t> Color4Set(std::begin(Color4), std::end(Color4));
// uint32_t Color5[] = {4};
// std::set<uint32_t> Color5Set(std::begin(Color5), std::end(Color5));
// uint32_t Color6[] = {5};
// std::set<uint32_t> Color6Set(std::begin(Color6), std::end(Color6));
// uint32_t Color7[] = {6};
// std::set<uint32_t> Color7Set(std::begin(Color7), std::end(Color7));

// std::vector<std::set<uint32_t>*> ColorVector = {&Color1Set,&Color2Set,&Color3Set,&Color4Set,&Color5Set,&Color6Set,&Color7Set};
// uint32_t myColorArray[7][7] = {
//   {0,1,10,14,19,8,4},
//   {1,0,5,20,17,13,7},
//   {10,5,0,2,12,15,18},
//   {14,20,2,0,6,11,16},
//   {19,17,12,6,0,3,9},
//   {8,13,15,11,3,0,21},
//   {4,7,18,16,9,21,0}
// };



std::map<Address, uint32_t> UanMacSrTDMA::m_Address2Color;
NS_LOG_COMPONENT_DEFINE ("UanMacSrTDMA");
  
NS_OBJECT_ENSURE_REGISTERED (UanMacSrTDMA);

UanMacSrTDMA::UanMacSrTDMA ()
  : UanMac (),
    m_cleared (false)
{
  m_ColorQueue = new std::vector<std::list<Ptr<Packet>>>(7);
}

UanMacSrTDMA::~UanMacSrTDMA ()
{
}

void
UanMacSrTDMA::Clear ()
{
  if (m_cleared)
    {
      return;
    }
  m_cleared = true;
  if (m_phy)
    {
      m_phy->Clear ();
      m_phy = 0;
    }
}

void
UanMacSrTDMA::DoDispose ()
{
  Clear ();
  UanMac::DoDispose ();
}

TypeId
UanMacSrTDMA::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanMacSrTDMA")
    .SetParent<UanMac> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanMacSrTDMA> ()
    .AddAttribute ("transmissionDelay",
                  "packet transmission delay",
                  TimeValue (Seconds (0.6)), //set phyRate = 6000bit/s --> txDelay = 0.58s --> 0.6s
                  MakeTimeAccessor (&UanMacSrTDMA::m_transTime),
                  MakeTimeChecker ()
                  )
   .AddAttribute ("propagationDelay",
                  "one hop propagation delay",
                  TimeValue (Seconds (14)),
                  MakeTimeAccessor (&UanMacSrTDMA::m_propTime),
                  MakeTimeChecker ()
                  )
  ;
  return tid;
}

bool
UanMacSrTDMA::Enqueue (Ptr<Packet> packet, uint16_t protocolNumber, const Address &dest)
{
  NS_LOG_DEBUG ("" << Simulator::Now ().GetSeconds () << " MAC " << Mac8Address::ConvertFrom (GetAddress ()) << " Queueing packet for " << Mac8Address::ConvertFrom (dest));
  //识别地址对应的Color
  uint32_t destColor = m_Address2Color[dest];

  //给packet添加mac头部
  Mac8Address src = Mac8Address::ConvertFrom (GetAddress ());
  Mac8Address udest = Mac8Address::ConvertFrom (dest);
  UanHeaderCommon header;
  header.SetSrc (src);
  header.SetDest (udest);
  header.SetType (0);
  header.SetProtocolNumber (protocolNumber);
  packet->AddHeader (header);

  NS_LOG_DEBUG("颜色为"<<m_color<<"节点储存一个发往颜色为"<<destColor<<"邻居的包.");
  //将packet添加到ColorQueue对应Color的链表中
  (*m_ColorQueue)[destColor].push_back(packet);

  return true;
}

void
UanMacSrTDMA::SetForwardUpCb (Callback<void, Ptr<Packet>, uint16_t, const Mac8Address&> cb)
{
  m_forUpCb = cb;
}

uint32_t
UanMacSrTDMA::GetNodeIdWithMacAddress(const Address &macAddr)
{
  NS_LOG_FUNCTION(this << macAddr);
  uint32_t nNode = NodeList::GetNNodes();
  for(uint32_t i=0; i<nNode; i++)
  {
    Ptr<Node> node = NodeList::GetNode(i);
    //水声网卡是第0个网卡
    if(macAddr == node->GetDevice(0)->GetAddress())
    {
      return node->GetId();
    }
  }

  NS_LOG_ERROR("can not find the macAddr.");
  return -1;
}


void
UanMacSrTDMA::SetColor()
{
    //通过mac地址找到节点id
  m_address = Mac8Address::ConvertFrom (GetAddress ());
  m_nodeid = GetNodeIdWithMacAddress(m_address);
  NS_LOG_DEBUG("my nodeId is " << m_nodeid);

  //在这里进行节点颜色的设置 颜色=0.1.2.3.4.5.6
  for (int i = 0; i < int(ColorVector.size()); ++i) 
  {
      if (ColorVector[i]->find(m_nodeid) != ColorVector[i]->end())
      {
          m_color = i;
          NS_LOG_DEBUG("my node color is " << m_color);
          break; 
      }
  }

  //记录节点颜色到节点表中
  m_Address2Color[m_address] = m_color;

  //在这里进行时隙事件的循环启动
  SendPacket();
}

void
UanMacSrTDMA::AttachPhy (Ptr<UanPhy> phy)
{
  NS_LOG_FUNCTION(this);
  m_phy = phy;
  m_phy->SetReceiveOkCallback (MakeCallback (&UanMacSrTDMA::RxPacketGood, this));
  m_phy->SetReceiveErrorCallback (MakeCallback (&UanMacSrTDMA::RxPacketError, this));

  Simulator::Schedule (Seconds(10), &UanMacSrTDMA::SetColor,this);
}

void
UanMacSrTDMA::SendPacket()
{
  //查询7x7的邻居调度表，根据调度表去安排节点在哪个时隙发送数据包
  for(uint32_t i=0; i<ColorVector.size(); i++)
  {
    if(i != m_color)
    {
      if((*m_ColorQueue)[i].empty())
      {
        // NS_LOG_DEBUG("颜色为"<<m_color<<"节点关于颜色为"<<i<<"邻居的发送数据包队列为空.");
      }
      else
      {
        uint32_t sendSlot = myColorArray[m_color][i];
        Time sendTime = (sendSlot-1)*m_transTime;
        NS_LOG_DEBUG("颜色为"<<m_color<<"的节点关于颜色为"<<i<<"的邻居的发送数据包队列非空,在" << sendTime.GetSeconds() << "s后发送数据.");
        Simulator::Schedule(sendTime,&UanMacSrTDMA::SendPacketFromColorQueue,this,i);
      }
    }
  }

  //下一轮发送
  Time nextSendPacket = m_propTime + 21*m_transTime;
  Simulator::Schedule (nextSendPacket, &UanMacSrTDMA::SendPacket,this);
}


void
UanMacSrTDMA::SendPacketFromColorQueue(uint32_t destColor)
{
  if(!m_phy->IsStateTx ())
  {
    Ptr<Packet> colorPacket = (*m_ColorQueue)[destColor].front();
    NS_LOG_DEBUG ("The Phy not in Tx.  can send the packet to phy.");
    m_phy->SendPacket (colorPacket, GetTxModeIndex ());
    (*m_ColorQueue)[destColor].pop_front();
  }
  else
  {
    NS_LOG_DEBUG ("The Phy in Tx.  drop the packet.");
  }
}

void
UanMacSrTDMA::RxPacketGood (Ptr<Packet> pkt, double sinr, UanTxMode txMode)
{
  NS_UNUSED (sinr);
  UanHeaderCommon header;
  pkt->RemoveHeader (header);

  if (header.GetDest () == GetAddress () || header.GetDest () == Mac8Address::GetBroadcast ())
  {
    m_forUpCb (pkt, header.GetProtocolNumber (), header.GetSrc ());
    NS_LOG_DEBUG ("Receiving packet from " << header.GetSrc () << " For " << header.GetDest ());
  }
  else
  {
    // NS_LOG_DEBUG ("The packet is not sent for me");
  }

}

void
UanMacSrTDMA::RxPacketError (Ptr<Packet> pkt, double sinr)
{
  NS_LOG_DEBUG ("" << Simulator::Now ().GetSeconds() << " MAC " << Mac8Address::ConvertFrom (GetAddress ()) << " Received packet in error with sinr " << sinr);
}

int64_t
UanMacSrTDMA::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  return 0;
}

}
