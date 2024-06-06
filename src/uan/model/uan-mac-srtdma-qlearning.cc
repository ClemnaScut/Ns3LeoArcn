/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Ljy <SCUT>
 */

#include "uan-mac-srtdma-qlearning.h"
#include "uan-tx-mode.h"
#include "ns3/log.h"
#include "uan-phy.h"
#include "uan-header-common.h"

#include <iostream>

namespace ns3
{
//着色表，根据特定网络结构修改
extern uint32_t Color1[];
extern std::set<uint32_t> Color1Set;
extern uint32_t Color2[];
extern std::set<uint32_t> Color2Set;
extern uint32_t Color3[];
extern std::set<uint32_t> Color3Set;
extern uint32_t Color4[];
extern std::set<uint32_t> Color4Set;
extern uint32_t Color5[];
extern std::set<uint32_t> Color5Set;
extern uint32_t Color6[];
extern std::set<uint32_t> Color6Set;
extern uint32_t Color7[];
extern std::set<uint32_t> Color7Set;

extern std::vector<std::set<uint32_t>*> ColorVector;
extern uint32_t myColorArray[7][7];

std::map<Address, uint32_t> UanMacSrTDMAQ::m_Address2Color;
double UanMacSrTDMAQ::SendPeriod[uanNum] = {0.0};

NS_LOG_COMPONENT_DEFINE ("UanMacSrTDMAQ");
  
NS_OBJECT_ENSURE_REGISTERED (UanMacSrTDMAQ);

UanMacSrTDMAQ::UanMacSrTDMAQ ()
  : UanMac (),
    m_cleared (false)
{
  m_ColorQueue = new std::vector<std::list<Ptr<Packet>>>(7);
}

UanMacSrTDMAQ::~UanMacSrTDMAQ ()
{
}

void
UanMacSrTDMAQ::Clear ()
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
UanMacSrTDMAQ::DoDispose ()
{
  Clear ();
  UanMac::DoDispose ();
}

TypeId
UanMacSrTDMAQ::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanMacSrTDMAQ")
    .SetParent<UanMac> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanMacSrTDMAQ> ()
    .AddAttribute ("transmissionDelay",
                  "packet transmission delay",
                  TimeValue (Seconds(0.32)),
                  MakeTimeAccessor (&UanMacSrTDMAQ::m_transTime),
                  MakeTimeChecker ()
                  )
   .AddAttribute ("propagationDelay",
                  "one hop propagation delay",
                  TimeValue (Seconds (13.33)),
                  MakeTimeAccessor (&UanMacSrTDMAQ::m_propTime),
                  MakeTimeChecker ()
                  )
   .AddAttribute ("timeInOneSlot",
                  "one slot time",
                  TimeValue (Seconds (0.64)),
                  MakeTimeAccessor (&UanMacSrTDMAQ::m_slotTime),
                  MakeTimeChecker ()
                  )
  ;
  return tid;
}

bool
UanMacSrTDMAQ::Enqueue (Ptr<Packet> packet, uint16_t protocolNumber, const Address &dest)
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
UanMacSrTDMAQ::SetForwardUpCb (Callback<void, Ptr<Packet>, uint16_t, const Mac8Address&> cb)
{
  m_forUpCb = cb;
}

uint32_t
UanMacSrTDMAQ::GetNodeIdWithMacAddress(const Address &macAddr)
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

double 
UanMacSrTDMAQ::calculateDiffOnNeighbor(uint32_t nodeid)
{
  uint32_t neibors[6] = {nodeid-12,nodeid-11,nodeid-1,nodeid+1,nodeid+11,nodeid+12};
  uint32_t neibor;
  double sumdiff = 0;
  uint32_t num = 0;
  // NS_LOG_UNCOND("my Node ID: " << nodeid << " "<< SendPeriod[nodeid]);

  for(uint32_t i=0;i<6;i++)
  {
    neibor=neibors[i];
    if(!checkUanNode(nodeid,neibor))
    {
      continue; //不是邻居
    }
    else
    {
      double diff = abs((SendPeriod[nodeid]-SendPeriod[neibor]));
      sumdiff += diff;
      // NS_LOG_UNCOND("Myneibor ID: " << neibor << " " << SendPeriod[neibor]);
      num++;
    }
  }

  // NS_LOG_UNCOND("sumdiff: " << sumdiff);
  return sumdiff/num;
}

bool 
UanMacSrTDMAQ::checkUanNode(uint32_t id1,uint32_t id2)
{
  if(id2<0 || id2>102)
  {
    return false;
  }

  if(distanceof2uan(id1,id2) >= 30000)
  {
    return false;
  }

  return true;
}

double 
UanMacSrTDMAQ::distanceof2uan(uint32_t id1,uint32_t id2)
{
  Ptr<Node> node1 = NodeList::GetNode(id1);
  Vector p1 = node1->GetObject<MobilityModel>()->GetPosition();
  Ptr<Node> node2 = NodeList::GetNode(id2);
  Vector p2 = node2->GetObject<MobilityModel>()->GetPosition();

  double ret = sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
  return ret;

}


void
UanMacSrTDMAQ::PrintSynRate()
{
    // double sum = 0.0;
    // for(int i = 0; i < 103; ++i) {
    //     sum += SendPeriod[i];
    // }
    // double average = sum / 103;

    for(int i = 0; i < 103; ++i) {
        NS_LOG_UNCOND(SendPeriod[i]);
    }

  double sumDiff=0;
  for(int i = 0; i < 103; ++i) {
      double diffOnNeibor = calculateDiffOnNeighbor(i);
      sumDiff += diffOnNeibor;
  }
  
  NS_LOG_UNCOND("-----------sumDiff " << sumDiff/103);

    // // 打印平均值
    // // NS_LOG_UNCOND("Average: " << average);

    // 打印每个元素与平均值的差值的平方，即计算方差
    // double sumDiff = 0.0;
    // for(int i = 0; i < 103; ++i) {
    //     double diff = std::pow(SendPeriod[i] - average,2);
    //     sumDiff += diff;
    // }
    // NS_LOG_UNCOND("-----------sumDiff " << sumDiff);

  
    Simulator::Schedule(Seconds(300), &UanMacSrTDMAQ::PrintSynRate);
}

void
UanMacSrTDMAQ::SetColor()
{
    //通过mac地址找到节点id
  m_address = Mac8Address::ConvertFrom (GetAddress ());
  m_nodeid = GetNodeIdWithMacAddress(m_address);
  if(m_nodeid == 0) //选择一个节点作为计算同步率的节点，调用计算同步率的静态函数即可
  {
    Simulator::Schedule(Seconds(100), &UanMacSrTDMAQ::PrintSynRate);
  }
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
  Ptr<UniformRandomVariable> m_uniformRandomVariable = CreateObject<UniformRandomVariable>();
  m_sendTime =  Seconds(m_uniformRandomVariable->GetValue(0,100));
  NS_LOG_DEBUG("Set sendTime: " << m_sendTime.GetSeconds() << ". Now the Q value is " << m_Qposition << ".");
  Simulator::Schedule(m_sendTime, &UanMacSrTDMAQ::SetSendEvent,this);
}

void
UanMacSrTDMAQ::AttachPhy (Ptr<UanPhy> phy)
{
  NS_LOG_FUNCTION(this);
  m_phy = phy;
  m_phy->SetReceiveOkCallback (MakeCallback (&UanMacSrTDMAQ::RxPacketGood, this));
  m_phy->SetReceiveErrorCallback (MakeCallback (&UanMacSrTDMAQ::RxPacketError, this));

  m_qSlot = m_slotTime/Qdiv;
  Simulator::Schedule(Seconds(10), &UanMacSrTDMAQ::SetColor,this);
}

void
UanMacSrTDMAQ::SetSendEvent()
{
  // NS_LOG_FUNCTION(this);
    //找到Ｑ值表中最大的值对应的那个索引，有多个值则返回多个索引
    uint32_t max_value = Qtable[0];
    std::vector<uint32_t> max_indices = {0};

    for (int32_t i = 0; i < Qdiv*m_slotNum; ++i) 
    {
      if (Qtable[i] > max_value) 
      {
        max_value = Qtable[i];
        max_indices.clear();
        max_indices.push_back(i);
      } else if (Qtable[i] == max_value) 
      {
        max_indices.push_back(i);
      }
    }

  if(max_value == 0)//最开始的时候
  {
    SendPacket();
  }
  else
  {
    int32_t newQposition;
    if(max_indices.size() == 1) //最大Ｑ值只有一个
    {
      newQposition = max_indices.front(); //直接改为最大Ｑ值
      // newQposition = std::trunc((max_indices.front() + m_Qposition)/2); //取折中
    }
    else //有多个相同的最大Ｑ值，求出索引的中位数
    {
      std::sort(max_indices.begin(), max_indices.end());
      size_t size = max_indices.size();
      if (size % 2 == 0) {
          // 偶数个元素，取中间两个元素的平均值
          newQposition = std::trunc((max_indices[size / 2 - 1] + max_indices[size / 2]) / 2.0);
      } else {
          // 奇数个元素，取中间的元素
          newQposition = max_indices[size / 2];
      }
    }

    // NS_LOG_DEBUG("Q值最大的位置在: " << newQposition << " 原本位置为: " << m_Qposition);
    Time nextSendTime;
    if(newQposition < m_Qposition)
    {
      nextSendTime = (newQposition-m_Qposition)*m_qSlot + m_slotNum*m_slotTime;
    }
    else if(newQposition > m_Qposition)
    {
      nextSendTime = (newQposition-m_Qposition)*m_qSlot;
    }
    else if(newQposition == m_Qposition)
    {
      nextSendTime = Seconds(0);
    }
    Simulator::Schedule (nextSendTime, &UanMacSrTDMAQ::SendPacket,this);
    //更新自己的Ｑ值位置
    m_Qposition = newQposition;
    //记录节点下一轮发送的时间点，用于计算同步率
  }

}

void
UanMacSrTDMAQ::SendPacket()
{
  // NS_LOG_FUNCTION(this);
  m_sendTime = Simulator::Now();
  SendPeriod[m_nodeid] = m_sendTime.GetSeconds();
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
        Time sendTime = (sendSlot-1)*m_slotTime;
        NS_LOG_DEBUG("颜色为"<<m_color<<"的节点关于颜色为"<<i<<"的邻居的发送数据包队列非空,在" << sendTime.GetSeconds() << "s后发送数据.");
        Simulator::Schedule(sendTime,&UanMacSrTDMAQ::SendPacketFromColorQueue,this,i);
      }
    }
  }

  //下一轮发送
  Time nextSendPacket = m_propTime + 21*m_slotTime;
  Simulator::Schedule (nextSendPacket, &UanMacSrTDMAQ::SetSendEvent,this);
  // Simulator::Schedule (nextSendPacket, &UanMacSrTDMAQ::SendPacket,this);
}


void
UanMacSrTDMAQ::SendPacketFromColorQueue(uint32_t destColor)
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
UanMacSrTDMAQ::RxPacketGood (Ptr<Packet> pkt, double sinr, UanTxMode txMode)
{
  NS_UNUSED (sinr);
  UanHeaderCommon header;
  pkt->RemoveHeader (header);

  Time recvTime = Simulator::Now ();
  NS_LOG_DEBUG ("Receiving Packet from " << header.GetSrc() << " for " << header.GetDest());
  uint32_t neighborColor = m_Address2Color[header.GetSrc()];
  uint32_t destColor = m_Address2Color[header.GetDest()];
  uint32_t sendSlot = myColorArray[destColor][neighborColor];
  Time neighborSendTime = recvTime-(sendSlot-1)*m_slotTime-m_propTime-m_transTime; //计算这个邻居发包起始时间是没问题的

  NS_LOG_DEBUG ("recvTime: " << recvTime.GetSeconds());
  NS_LOG_DEBUG ("Neighbor SendTime: " << neighborSendTime.GetSeconds() << " My SendTime:  " << m_sendTime.GetSeconds());

  double interval = (neighborSendTime.GetSeconds() - m_sendTime.GetSeconds())/m_qSlot.GetSeconds();
  int32_t intervalNum = std::trunc(interval);
  NS_LOG_DEBUG("The interval is " << interval << " -> " << intervalNum);

  int32_t QvalueIncrease = m_Qposition+intervalNum;
  while(QvalueIncrease >= m_slotNum*Qdiv)
  {
    QvalueIncrease -=m_slotNum*Qdiv;
  }
  while(QvalueIncrease < 0)
  {
    QvalueIncrease +=m_slotNum*Qdiv;
  }

  Qtable[QvalueIncrease]++;

  if (header.GetDest () == GetAddress () || header.GetDest () == Mac8Address::GetBroadcast ())
  {
    m_forUpCb (pkt, header.GetProtocolNumber (), header.GetSrc ());
    NS_LOG_DEBUG ("Receiving Packet Successfully. from " << header.GetSrc () << " for me.");
  }
  else
  {
    // NS_LOG_DEBUG ("The packet is not sent for me");
  }

}

void
UanMacSrTDMAQ::RxPacketError (Ptr<Packet> pkt, double sinr)
{
  NS_LOG_DEBUG ("" << Simulator::Now ().GetSeconds() << " MAC " << Mac8Address::ConvertFrom (GetAddress ()) << " Received packet in error with sinr " << sinr);
}

int64_t
UanMacSrTDMAQ::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  return 0;
}

}
