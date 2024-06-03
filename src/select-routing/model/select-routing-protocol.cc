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
 *      http://core.it.uu.se/core/number.php/vbf-UU
 *
 * Authors: Elena Buchatskaia <borovkovaes@iitp.ru>
 *          Pavel Boyko <boyko@iitp.ru>
 */


#include "select-routing-protocol.h"
#include "select-routing-packet.h"
#include "ns3/log.h"
#include "ns3/boolean.h"
#include "ns3/random-variable-stream.h"
#include "ns3/inet-socket-address.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/udp-l4-protocol.h"
#include "ns3/udp-header.h"
#include "ns3/wifi-net-device.h"
#include "ns3/adhoc-wifi-mac.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/node-list.h"
#include <algorithm>
#include <math.h>
#include <limits>

namespace ns3 {
NS_LOG_COMPONENT_DEFINE ("SelectRoutingProtocol");

namespace select_route {

#define MAX_UAN_NODE 102 //水下节点的最大nodeid
#define SatelliteId 103
uint32_t buoyId[] = {25,28,31,71,74,77};
std::set<uint32_t> buoyIdSet(std::begin(buoyId), std::end(buoyId)) ; //储存buoy节点的NodeId
uint32_t buoyNbhId[] = 
{13,14,24,26,36,37,
 16,17,27,29,39,40,
 19,20,30,32,42,43,
 59,60,70,72,82,83,
 62,63,73,75,85,86,
 65,66,76,78,88,89
};
std::set<uint32_t> buoyNeighborIdSet(std::begin(buoyNbhId), std::end(buoyNbhId));
 //储存buoy节点的邻居NodeId

std::map<uint32_t, NeighborInfoTable*>  RoutingProtocol::nodeId2Table;

// ------------------------------------------Neighbor Table---------------------------------------------------
NeighborInfoTable::NeighborInfoTable()
{
  NS_LOG_FUNCTION(this);
  m_isInit = false;
  m_length = 0;
}
NeighborInfoTable::~NeighborInfoTable()
{
  NS_LOG_FUNCTION(this);
}

bool
NeighborInfoTable::isInit()
{
  NS_LOG_FUNCTION(this);
  return m_isInit;
}

uint32_t
NeighborInfoTable::findNeighborBuoy()
{
  NS_LOG_FUNCTION(this);
	for (std::vector<NeighborInfo>::iterator iter = m_table.begin(); iter != m_table.end(); iter++)
	{
    if(iter->n_nodeType== NodeType::BUOY)
    {
      return iter->n_nodeid;
    }
  }
  return -1;
}

bool 
NeighborInfoTable::findNodeInTable(uint32_t id)
{
	for (std::vector<NeighborInfo>::iterator iter = m_table.begin(); iter != m_table.end(); iter++)
	{
    if(id == iter->n_nodeid)
    {
      return true;
    }
  }
  return false;
}

bool 
NeighborInfoTable::findIpv4AddressInTable(const Ipv4Address& ipv4)
{
  for (std::vector<NeighborInfo>::iterator iter = m_table.begin(); iter != m_table.end(); iter++)
	{
    if(ipv4 == iter->n_ip)
    {
      return true;
    }
  }
  return false;
}


// 辅助函数：将枚举值转换为字符串
std::string to_string(NodeType nodeType)
{
    switch (nodeType)
    {
    case NodeType::UAN:
        return "UAN";
    case NodeType::BUOY:
        return "BUOY";
    case NodeType::SATELLITE:
        return "SATELLITE";
    default:
        return "Unknown";
    }
}

void
NeighborInfoTable::PrintTable()
{
  NS_LOG_FUNCTION(this);
	for (std::vector<NeighborInfo>::iterator iter = m_table.begin(); iter != m_table.end(); iter++)
	{

    NS_LOG_DEBUG(iter->n_nodeid << " " << iter->n_ip << " " << iter->n_position << " "
                  << to_string(iter->n_nodeType) <<  ".");
  }
}

//------------------------------------Hash Table--------------------------------------

AquaSimPktHashTable::AquaSimPktHashTable()
{
  NS_LOG_FUNCTION(this);
  m_windowSize=WINDOW_SIZE; //初始化windowSize
}

AquaSimPktHashTable::~AquaSimPktHashTable()
{
  NS_LOG_FUNCTION(this);
  for (std::map<hash_entry,vbf_neighborhood*>::iterator it=m_htable.begin(); it!=m_htable.end(); ++it) {
    delete it->second; //避免内存泄露
  }
  m_htable.clear();
}

void
AquaSimPktHashTable::Reset()
{
  m_htable.clear();
}

vbf_neighborhood*
AquaSimPktHashTable::GetHash(Ipv4Address senderAddr, unsigned int pk_num)
{
  hash_entry entry = std::make_pair (senderAddr,pk_num);
  std::map<hash_entry,vbf_neighborhood*>::iterator it;

  it = m_htable.find(entry); 
  if (it == m_htable.end())
    return NULL;

  return it->second;
}

void
AquaSimPktHashTable::PutInHash(Ipv4Address sAddr, unsigned int pkNum, Vector p, double factor)
{
  NS_LOG_DEBUG("PutinHash begin:" << sAddr << "," << pkNum << ",(" << p.x << "," << p.y << "," << p.z << "), factor: "
                  << factor);
	
  vbf_neighborhood* hashPtr;
  hash_entry entry = std::make_pair (sAddr,pkNum);
  std::map<hash_entry,vbf_neighborhood*>::iterator it;
	int k=pkNum-m_windowSize;
	if(k>0)    //1.表已满的处理：表示表已经存放不下新的entry，此时要erase前面的
	{
		for (int i=0; i<k; i++)
		{
      entry.second = i;
      it = m_htable.find(entry);
      if(it != m_htable.end())
      {
        hashPtr = it->second;
        delete hashPtr;
        m_htable.erase(it);
      }
		}
	}

  entry.second = pkNum;
  hashPtr = GetHash(sAddr,pkNum);
	if (hashPtr != NULL) //2.曾经收到过该packet的处理：更新neighbor结构体信息，添加关于此packet对应的neighbor
	{
		int m=hashPtr->number;
		// printf("hash_table: this is not old item, there are %d item inside\n",m);
		if (m<MAX_NEIGHBOR) {
			hashPtr->number++;
			hashPtr->neighbor[m].x=p.x;
			hashPtr->neighbor[m].y=p.y;
			hashPtr->neighbor[m].z=p.z;
      hashPtr->neighborFactor[m] = factor;
		}
		return;
	}
  //3.第一次收到此packet的处理，在此packet对应的neighbor结构体的第一个元素位置添加信息
	hashPtr=new vbf_neighborhood[1];
	hashPtr[0].number=1;
	hashPtr[0].neighbor[0].x=p.x;
	hashPtr[0].neighbor[0].y=p.y;
	hashPtr[0].neighbor[0].z=p.z;
  hashPtr[0].neighborFactor[0]=factor;

  std::pair<hash_entry,vbf_neighborhood*> newPair;
  newPair.first=entry; newPair.second=hashPtr;
  if (m_htable.insert(newPair).second == false)
  {
    delete newPair.second;
  }
}




//------------------------------------Routing Protocol--------------------------------------
NS_OBJECT_ENSURE_REGISTERED (RoutingProtocol);

/// UDP Port for vbf control traffic
const uint32_t RoutingProtocol::vbf_PORT = 654;

RoutingProtocol::RoutingProtocol ():
  m_pkNum(0),
  m_priority(1)
{
  m_targetPos = Vector();
  m_uniformRandomVariable = CreateObject<UniformRandomVariable>();
}

TypeId
RoutingProtocol::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::select_route::RoutingProtocol")
    .SetParent<Ipv4RoutingProtocol> ()
    .SetGroupName ("vbf_hybird")
    .AddConstructor<RoutingProtocol> ()
    .AddAttribute ("HopByHopModel", "Hop by hop VBF setting. Default 0 is false.",
      IntegerValue(0),
      MakeIntegerAccessor(&RoutingProtocol::m_hopByHop),
      MakeIntegerChecker<int>())
    .AddAttribute ("EnableRouting", "Enable routing VBF setting. Default 1 is true.",
      IntegerValue(1),
      MakeIntegerAccessor(&RoutingProtocol::m_enableRouting),
      MakeIntegerChecker<int>())
    .AddAttribute ("Width", "Width of VBF. Default is 30000.",
      DoubleValue(40000),
      MakeDoubleAccessor(&RoutingProtocol::m_width),
      MakeDoubleChecker<double>())
    .AddAttribute ("TransRange", "TransRange of the UnderWaterNode. Default is 25000.",
      DoubleValue(25000),
      MakeDoubleAccessor(&RoutingProtocol::m_TransRange),
      MakeDoubleChecker<double>())
    .AddAttribute ("SoundSpeed", "Sound Speed. Default is 1500m/s",
      DoubleValue(1500),
      MakeDoubleAccessor(&RoutingProtocol::m_SoundSpeed),
      MakeDoubleChecker<double>())
    .AddAttribute ("TargetPos", "Position of target sink (x,y,z).",
      Vector3DValue(),
      MakeVector3DAccessor(&RoutingProtocol::m_targetPos),
      MakeVector3DChecker())
  ;
  return tid;
}

int64_t
RoutingProtocol::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uniformRandomVariable->SetStream(stream);
  return 1;
}

RoutingProtocol::~RoutingProtocol (){}

void
RoutingProtocol::NotifyAddAddress (uint32_t i, Ipv4InterfaceAddress address){}

void
RoutingProtocol::NotifyRemoveAddress (uint32_t i, Ipv4InterfaceAddress address){}

void
RoutingProtocol::NotifyInterfaceUp (uint32_t i){}

void
RoutingProtocol::NotifyInterfaceDown (uint32_t i){}

void
RoutingProtocol::DoDispose (){}


void
RoutingProtocol::SetIpv4 (Ptr<Ipv4> ipv4)
{
  NS_ASSERT (ipv4 != 0);
  NS_ASSERT (m_ipv4 == 0);
  m_ipv4 = ipv4;

  // Create loop route. It is asserted that the only one interface up for now is loopback
  NS_ASSERT (m_ipv4->GetNInterfaces () == 1 && m_ipv4->GetAddress (0, 0).GetLocal () == Ipv4Address ("127.0.0.1"));
  m_loopNet = m_ipv4->GetNetDevice (0);
  NS_ASSERT (m_loopNet != 0);
}

void
RoutingProtocol::PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit) const
{ 
  NS_LOG_FUNCTION(this);
}

void
RoutingProtocol::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  //这里进行初始化邻居信息
  uint32_t myNodeId = this->m_ipv4->GetObject<Node>()->GetId();
  auto it = buoyIdSet.find(myNodeId);
  if (it != buoyIdSet.end() && !this->m_nbhTable.m_isInit) 
  {
    // 元素找到，是浮标节点
      buoyTableInit(myNodeId);
  }
  else 
  {
    // 元素未找到，是水下或者卫星节点
    if(myNodeId != 103) //是其他水下节点
    {
      if(!this->m_nbhTable.m_isInit){
      Simulator::Schedule(Seconds(1),&RoutingProtocol::uanTableInit,this,myNodeId);}
    }
    else  //是卫星
    {
      if(!this->m_nbhTable.m_isInit){
      Simulator::Schedule(Seconds(2),&RoutingProtocol::satelliteTableInit,this,myNodeId);}
    }
  
  }
  Ipv4RoutingProtocol::DoInitialize ();
}




bool 
RoutingProtocol::buoyTableInit(uint32_t nodeId)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT(this->m_nbhTable.m_isInit == false);
  // auto methodFunc = bind(&HttpRequest::setMethod,this,placeholders::_1);
  auto uanFunc = bind(&RoutingProtocol::GetUanIpv4WithNode,this,std::placeholders::_1);
  auto satelliteFunc = bind(&RoutingProtocol::GetSatelliteIpv4WithNode,this,std::placeholders::_1);
  //浮标节点有六个水下邻居和一个卫星邻居
  AddNeighborToTable(nodeId-12,NodeType::UAN,uanFunc);
  AddNeighborToTable(nodeId-11,NodeType::UAN,uanFunc);
  AddNeighborToTable(nodeId-1,NodeType::UAN,uanFunc);
  AddNeighborToTable(nodeId+1,NodeType::UAN,uanFunc);
  AddNeighborToTable(nodeId+11,NodeType::UAN,uanFunc);
  AddNeighborToTable(nodeId+12,NodeType::UAN,uanFunc);
  AddNeighborToTable(SatelliteId,NodeType::SATELLITE,satelliteFunc);

  this->m_nbhTable.m_isInit = true;
  nodeId2Table.insert(std::make_pair(nodeId,&this->m_nbhTable));
  NS_LOG_DEBUG ("The Buoy Node "  << this->m_ipv4->GetObject<Node>()->GetId() << " Neighbor table Init Over.");
  this->m_nbhTable.PrintTable();
  return true;
}

bool 
RoutingProtocol::uanTableInit(uint32_t nodeId)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT(this->m_nbhTable.m_isInit == false);
  auto uanFunc = bind(&RoutingProtocol::GetUanIpv4WithNode,this,std::placeholders::_1);

#if 1
  uint32_t buoyId = findBuoyNbhNode(nodeId);
  if(buoyId != 0)
  {
    AddNeighborToTable(buoyId,NodeType::BUOY,uanFunc);
  }
  //其他水下节点都有六个水下邻居
   AddNeighborToTable(nodeId-12,NodeType::UAN,uanFunc);
   AddNeighborToTable(nodeId-11,NodeType::UAN,uanFunc);
   AddNeighborToTable(nodeId-1,NodeType::UAN,uanFunc);
   AddNeighborToTable(nodeId+1,NodeType::UAN,uanFunc);
   AddNeighborToTable(nodeId+11,NodeType::UAN,uanFunc);
   AddNeighborToTable(nodeId+12,NodeType::UAN,uanFunc);
#else //纯水下测试时的代码
   AddNeighborToTable(nodeId-3,NodeType::UAN,uanFunc);
   AddNeighborToTable(nodeId-2,NodeType::UAN,uanFunc);
   AddNeighborToTable(nodeId-1,NodeType::UAN,uanFunc);
   AddNeighborToTable(nodeId+1,NodeType::UAN,uanFunc);
   AddNeighborToTable(nodeId+2,NodeType::UAN,uanFunc);
   AddNeighborToTable(nodeId+3,NodeType::UAN,uanFunc);
#endif

  this->m_nbhTable.m_isInit = true;
  nodeId2Table.insert(std::make_pair(nodeId,&this->m_nbhTable));
  NS_LOG_DEBUG ("The Uan Node "  << this->m_ipv4->GetObject<Node>()->GetId() << " Neighbor table Init Over.");
  this->m_nbhTable.PrintTable();
  return true;

}

bool 
RoutingProtocol::satelliteTableInit(uint32_t nodeId)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT(this->m_nbhTable.m_isInit == false);
  auto satelliteFunc = bind(&RoutingProtocol::GetSatelliteIpv4WithNode,this,std::placeholders::_1);
  //卫星节点有六个浮标邻居
  for (auto it = buoyIdSet.begin(); it != buoyIdSet.end(); ++it)
  {
    AddNeighborToTable(*it,NodeType::BUOY,satelliteFunc);
  }

  this->m_nbhTable.m_isInit = true;
  nodeId2Table.insert(std::make_pair(nodeId,&this->m_nbhTable));
  NS_LOG_DEBUG ("The Satellite Node "  << this->m_ipv4->GetObject<Node>()->GetId() << " Neighbor table Init Over.");
  this->m_nbhTable.PrintTable();
  return true;

}


uint32_t 
RoutingProtocol::findBuoyNbhNode(uint32_t id)
{
  auto it = buoyIdSet.begin();
  for (; it != buoyIdSet.end(); ++it)
  {
    //查找哪个浮标节点是这个id节点的邻居
    if(nodeId2Table[*it]->m_isInit && nodeId2Table[*it]->findNodeInTable(id))
    {
      return *it;
    }
    else
    {}
  }

  return 0;
}


bool 
RoutingProtocol::checkUanNode(uint32_t id)
{
  if(id<0 || id>MAX_UAN_NODE)
  {
    return false;
  }

  if(distanceof2uan(id) >= 30000)
  {
    return false;
  }

  return true;
}

double 
RoutingProtocol::distanceof2uan(uint32_t id)
{
  Vector p1 = m_ipv4->GetObject<MobilityModel>()->GetPosition();
  Ptr<Node> node = NodeList::GetNode(id);
  Vector p2 = node->GetObject<MobilityModel>()->GetPosition();

  double ret = sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
  return ret;

}



void 
RoutingProtocol::AddNeighborToTable(uint32_t id, NodeType nodeType, InitFunc func)
{
  NS_LOG_FUNCTION (this);
  if(!this->m_nbhTable.findNodeInTable(id))
  {
    if(nodeType == NodeType::UAN && !checkUanNode(id)){ return;} //普通水下节点但不符合邻居

    Ptr<Node> node = NodeList::GetNode(id);
    Ipv4Address ipv4Addr = func(node);
    Vector position = node->GetObject<MobilityModel>()->GetPosition();
    NeighborInfo newNbhInfo{id,ipv4Addr,position,nodeType};
    this->m_nbhTable.m_table.push_back(newNbhInfo);
    this->m_nbhTable.m_length++;
  }
}

Ipv4Address 
RoutingProtocol::GetUanIpv4WithNode(Ptr<Node> node)
{
  NS_LOG_FUNCTION(this << node->GetId());
  Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
  //水下节点： 0-本地网卡  1-uan
  //浮标节点： 0-本地网卡  1-uan  2-wifi  3-satellite
  Ipv4InterfaceAddress ipv4Addr = ipv4->GetAddress(1,0);
  return ipv4Addr.GetLocal();
}

// Ipv4Address 
// RoutingProtocol::GetWifiIpv4WithNode(Ptr<Node> node)
// {
//   NS_LOG_FUNCTION(this << node->GetId());
//   Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
//   Ipv4InterfaceAddress ipv4Addr; 
//   //浮标节点： 0-本地网卡  1-uan  2-wifi  3-satellite
//   //只有浮标节点有wifi网卡，但是浮标节点的wifi暂时不用和其他节点通信
//   ipv4Addr = ipv4->GetAddress(2,0);
//   return ipv4Addr.GetLocal();
// }


Ipv4Address 
RoutingProtocol::GetSatelliteIpv4WithNode(Ptr<Node> node)
{
  NS_LOG_FUNCTION(this << node->GetId());
  Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
  Ipv4InterfaceAddress ipv4Addr; 
  if(node->GetId() == SatelliteId)
  {
    //卫星节点： 0-本地网卡  1-satellite
    ipv4Addr = ipv4->GetAddress(1,0);
  }
  else
  {
    //浮标节点： 0-本地网卡  1-uan  2-wifi  3-satellite
    ipv4Addr = ipv4->GetAddress(3,0);
  }
  return ipv4Addr.GetLocal();
} 


double
RoutingProtocol::Distance(const Ipv4Address& dest, const Ipv4Address& neighbor)
{
  Ptr<Node> destNode = GetNodeWithIpv4Address(dest);
  Ptr<Node> neighborNode = GetNodeWithIpv4Address(neighbor);
  Vector p1 = destNode->GetObject<MobilityModel>()->GetPosition();
  Vector p2 = neighborNode->GetObject<MobilityModel>()->GetPosition();
  double ret = sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
  return ret;
}

Ipv4Address 
RoutingProtocol::LookupNextHopSatellite(const Ipv4Header &header, const Ipv4Address& forwardAddr)
{
  NS_LOG_FUNCTION(this << header);
  Ipv4Address destination = header.GetDestination();

  std::map<Ipv4Address,double> neighborDistance;
  //卫星的邻居只有浮标，因此这里卫星将找一个距离目的节点最近的浮标作为网关
  for (std::vector<NeighborInfo>::iterator iter = m_nbhTable.m_table.begin(); iter != m_nbhTable.m_table.end(); iter++)
  {
    if(iter->n_nodeType==NodeType::BUOY)
    {
      //计算邻居节点中距离路由准线最近的节点(可以直接找距离目的节点最近的节点，因为这两种方式是等价的，由于网络拓扑的特殊性)
      double dis = Distance(destination,iter->n_ip);
      neighborDistance.insert(std::make_pair(iter->n_ip,dis));
      NS_LOG_DEBUG(iter->n_nodeid << " " << iter->n_ip << " node -- distance " << dis << ".");
    }
  }

  // 使用 std::max_element 找到值最小的元素
  auto maxElementIt = std::min_element(neighborDistance.begin(), neighborDistance.end(),
      [](const std::pair<Ipv4Address,double>& a, const std::pair<Ipv4Address,double>& b) {
          return a.second < b.second; // 比较 map 中的值
      });

  if (maxElementIt != neighborDistance.end()) 
  {
    NS_LOG_DEBUG("The node with the min distance- " << maxElementIt->second << " is: " << maxElementIt->first << ", "<< "set it as gateway.");
    return maxElementIt->first;
  } 
  else 
  {
    NS_LOG_DEBUG("The map is empty.");
    return Ipv4Address();
  }

}


Ipv4Address 
RoutingProtocol::LookupNextHop(const Ipv4Header &header, const Ipv4Address& forwardAddr)
{
  NS_LOG_FUNCTION(this << header);
  Ipv4Address destination = header.GetDestination();

  if(m_nbhTable.findIpv4AddressInTable(destination))
  {
    //目的节点就是自己的邻居
    return destination;
  }
  else
  {
    //如果邻居有卫星节点且该节点不是上一跳转发节点，则把该卫星节点作为下一跳ip
    for (std::vector<NeighborInfo>::iterator iter = m_nbhTable.m_table.begin(); iter != m_nbhTable.m_table.end(); iter++)
    {
      if(iter->n_nodeType==NodeType::SATELLITE && iter->n_ip!=forwardAddr)
      {
        NS_LOG_DEBUG(" node has satellite neighbor, set it as gateway.");
        return iter->n_ip;
      }
    }

    //邻居没有卫星节点，则进入这里进行下一步判断，如果有浮标节点且该节点不是上一跳转发节点，则把该浮标节点作为下一跳ip
    for (std::vector<NeighborInfo>::iterator iter = m_nbhTable.m_table.begin(); iter != m_nbhTable.m_table.end(); iter++)
    {
      if(iter->n_nodeType==NodeType::BUOY && iter->n_ip!=forwardAddr)
      {
        NS_LOG_DEBUG(" node has buoy neighbor " << iter->n_ip << ", set it as gateway.");
        //这里的判断是为了确保路径是源节点->目的节点，而浮标的作用只是在路径恰好经过的时候把包发给浮标
        if(Distance(destination,iter->n_ip) < Distance(destination,m_ipv4->GetAddress(1,0).GetLocal()))
        {
          return iter->n_ip;
        }
        else
        {
          break;
        }

      }
    }

    std::map<Ipv4Address,double> neighborDistance;
    //邻居没有卫星节点，且没有浮标节点，则进入这里进行下一步判断，即此时只需要考虑表中的UAN节点即可，从所有UAN节点中选择出距离目的节点最近的节点
    for (std::vector<NeighborInfo>::iterator iter = m_nbhTable.m_table.begin(); iter != m_nbhTable.m_table.end(); iter++)
    {
      if(iter->n_nodeType==NodeType::UAN)
      {
        //计算邻居节点中距离路由准线最近的节点(可以直接找距离目的节点最近的节点，因为这两种方式是等价的，由于网络拓扑的特殊性)
        double dis = Distance(destination,iter->n_ip);
        neighborDistance.insert(std::make_pair(iter->n_ip,dis));
        NS_LOG_DEBUG(iter->n_nodeid << " " << iter->n_ip << " node -- distance " << dis << ".");
      }
    }

        // 使用 std::max_element 找到值最小的元素
    auto maxElementIt = std::min_element(neighborDistance.begin(), neighborDistance.end(),
        [](const std::pair<Ipv4Address,double>& a, const std::pair<Ipv4Address,double>& b) {
            return a.second < b.second; // 比较 map 中的值
        });

    if (maxElementIt != neighborDistance.end()) 
    {
      NS_LOG_DEBUG("The node with the min distance- " << maxElementIt->second << " is: " << maxElementIt->first << ", "<< "set it as gateway.");
      return maxElementIt->first;
    } 
    else 
    {
      std::cout << "The map is empty." << std::endl;
      return Ipv4Address();
    }
  }


}


uint32_t
RoutingProtocol::LookupInterface(const Ipv4Address& gateway)
{
  NS_LOG_FUNCTION(this << gateway);
  uint32_t infNum = m_ipv4->GetNInterfaces();

  uint32_t i=0;
  for(; i<infNum; i++)
  {
    if(m_ipv4->GetAddress(i,0).GetLocal().GetSubnetDirectedBroadcast(Ipv4Mask("255.255.255.0")) 
    == gateway.GetSubnetDirectedBroadcast(Ipv4Mask("255.255.255.0")))
    {
      break;
    }
  }
  NS_LOG_DEBUG("node id:" << m_ipv4->GetObject<Node>()->GetId() << " has " << infNum << " interfaces.");
  NS_LOG_DEBUG("the output interface to " << gateway << " is " << m_ipv4->GetAddress(i,0) << ".");
  return i;

}

/// @brief  ipv4l3protocol在需要找路由时调用
/// @param p ipv4l3protocol传入的packet，含有传输层header
/// @param header ipv4l3protocol传入的ipv4层header
/// @param oif ipv4l3protocol传入的netdevice（指定是否需要从找到对应netdevice为出口的路由），一般会传入0
/// @param sockerr 传出参数，路由寻找出错时被使用（没什么用）
/// @return 
Ptr<Ipv4Route>
RoutingProtocol::RouteOutput (Ptr<Packet> p, const Ipv4Header &header,
                              Ptr<NetDevice> oif, Socket::SocketErrno &sockerr)
{
  NS_LOG_FUNCTION(this << header << (oif ? oif->GetIfIndex(): 0));



  //因为这里的RouteOutput是只有自身要发包的时候才会被ipv4l3调用的，所以这里的forwarderIp就肯定是节点本身+水下网卡的ip
  Ipv4Address myUanAddress = m_ipv4->GetAddress(1,0).GetLocal();
  forwardHeader fHeader(myUanAddress);
  p->AddHeader(fHeader);
  NS_LOG_DEBUG ("fHeader: " << fHeader);


  Ipv4Address destination = header.GetDestination();
  Ipv4Address source = myUanAddress;
  Ipv4Address gateway = LookupNextHop(header,fHeader.GetForwardAddr());
  uint32_t interfaceIdx = LookupInterface(gateway);

  Ptr<Ipv4Route> rtentry = 0;
  rtentry = Create<Ipv4Route> ();
  rtentry->SetDestination(destination);
  rtentry->SetSource(source);
  rtentry->SetGateway(gateway);
  rtentry->SetOutputDevice(m_ipv4->GetNetDevice(interfaceIdx));
  return rtentry;

}


// void
// RoutingProtocol::vbfSend(Ptr<Packet> p, Ipv4Address source, Ipv4Address dst)
// {
//   NS_LOG_FUNCTION(this << "TrueSend: from " << source << " to " << dst);
//   NS_ASSERT (p != 0);
//   Ptr<Packet> packet = p->Copy();


//   for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
//           m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
//   {
//     Ptr<Socket> socket = j->first;
//     Ipv4InterfaceAddress iface = j->second;

//     //vbf协议最终发送的都是ip层的广播包
//     Ipv4Address destination;
//     if (iface.GetMask () == Ipv4Mask::GetOnes ())
//     {
//       destination = Ipv4Address ("255.255.255.255");
//     }
//     else
//     {
//       destination = iface.GetBroadcast ();
//     }

//   //以des地址为广播ip地址，将这个vbfpacket从socket处发出
//   // Simulator::Schedule (Time (MilliSeconds (m_uniformRandomVariable->GetInteger (0, 10))), &RoutingProtocol::SendTo, this, socket, packet, destination);
//   Simulator::Schedule (Seconds(1), &RoutingProtocol::SendTo, this, socket, packet, destination);
//   }
// }


// void
// RoutingProtocol::SendTo (Ptr<Socket> socket, Ptr<Packet> packet, Ipv4Address destination)
// {
//   NS_LOG_FUNCTION(this);
//   socket->SendTo (packet, 0, InetSocketAddress (destination, vbf_PORT));

// }


bool
RoutingProtocol::RouteInput (Ptr<const Packet> p, const Ipv4Header &header, Ptr<const NetDevice> idev,
                  UnicastForwardCallback ucb, MulticastForwardCallback mcb,
                  LocalDeliverCallback lcb, ErrorCallback ecb)
{ 
  NS_LOG_FUNCTION(this << header);
  Ipv4Address recvIp =m_ipv4->GetAddress(m_ipv4->GetInterfaceForDevice (idev), 0).GetLocal();
  NS_LOG_DEBUG (this << " pkt size: "<< p->GetSerializedSize() << " dest: " << header.GetDestination () << 
                  " receive in: " << recvIp);
  NS_ASSERT (m_ipv4 != 0);
  NS_ASSERT (p != 0);
  // Check if input device supports IP
  NS_ASSERT (m_ipv4->GetInterfaceForDevice (idev) >= 0);
  int32_t iif = m_ipv4->GetInterfaceForDevice (idev);
  Ipv4Address dst = header.GetDestination ();
  Ipv4Address source = header.GetSource ();

  //取消forwarder头部
  Ptr<Packet> packet = p->Copy ();
  forwardHeader fHeader;
  packet->RemoveHeader(fHeader);
  NS_LOG_DEBUG ("fHeader: " << fHeader);

  if (IsMyOwnAddress (source))
  {
    return true;
  }

  //1.当des地址为自身时，这种情况当前节点最终接受packet时出现
  if (m_ipv4->IsDestinationAddress (dst, iif))
  {
    if (lcb.IsNull () == false)
      {
        NS_LOG_LOGIC ("Unicast local delivery to " << dst);
        lcb (packet, header, iif);
      }
    else
      {
        NS_LOG_ERROR ("Unable to deliver packet locally due to null callback " << packet->GetUid () << " from " << source);
        ecb (packet, header, Socket::ERROR_NOROUTETOHOST);
      }
    return true;
  }

  //2.单播转发
  uint32_t mynodeId = m_ipv4->GetObject<Node>()->GetId();
  Ipv4Address gateway;
  if(mynodeId == SatelliteId)
  {
    gateway = LookupNextHopSatellite(header,fHeader.GetForwardAddr());
  }
  else
  {
    gateway = LookupNextHop(header,fHeader.GetForwardAddr());
  }
 
  uint32_t interfaceIdx = LookupInterface(gateway);
  fHeader.SetForwardAddr(m_ipv4->GetAddress(interfaceIdx,0).GetLocal());

  packet->AddHeader(fHeader);

  Ptr<Ipv4Route> rtentry;
  rtentry = Create<Ipv4Route> ();
  rtentry->SetDestination(dst);
  rtentry->SetSource(source);
  rtentry->SetGateway(gateway);
  rtentry->SetOutputDevice(m_ipv4->GetNetDevice(interfaceIdx));

  if (ucb.IsNull () == false)
  {
    NS_LOG_LOGIC ("Unicast local delivery to " << dst << " the gateway is " << gateway << ".");
    ucb (rtentry, packet, header);
  }

  return true;


  // NS_LOG_FUNCTION (this << p->GetUid () << header.GetDestination () << idev->GetAddress ());
  // if (m_socketAddresses.empty ())
  // {
  //   NS_LOG_LOGIC ("No aodv interfaces");
  //   return false;
  // }
  // NS_ASSERT (m_ipv4 != 0);
  // NS_ASSERT (p != 0);
  // // Check if input device supports IP
  // NS_ASSERT (m_ipv4->GetInterfaceForDevice (idev) >= 0);
  // int32_t iif = m_ipv4->GetInterfaceForDevice (idev);
  // Ipv4Address dst = header.GetDestination ();
  // Ipv4Address origin = header.GetSource ();
  // Ipv4InterfaceAddress iface = m_ipv4->GetAddress (iif, 0); 

  // if (IsMyOwnAddress (origin))
  // {
  //   return true;
  // }

  // //拿到当前Ipv4Interface的广播地址(子网)
  // Ipv4Address BroadDest;
  // if (iface.GetMask () == Ipv4Mask::GetOnes ())
  // {
  //   BroadDest = Ipv4Address ("255.255.255.255");
  // }
  // else
  // {
  //   BroadDest = iface.GetBroadcast ();
  // }

  // /* 1.当前dst地址为广播地址，在VBF中每个节点接收到vbfpacket都会是这种情况，即收到广播packet，
  //      然后往上交给udp，udp再把vbfpacket交给vbf的socket，触发RecvVBF的callback函数，实际的
  //      处理由RecvVBF去处理
  // */
  
  // // Broadcast local delivery/forwarding
  // for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
  //       m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
  // {
  //   Ipv4InterfaceAddress iface = j->second;
  //   if (m_ipv4->GetInterfaceForAddress (iface.GetLocal ()) == iif)
  //   {
  //     if (dst == BroadDest)
  //       {
  //         Ptr<Packet> packet = p->Copy ();
  //         if (lcb.IsNull () == false)
  //         {
  //           NS_LOG_LOGIC ("Broadcast local delivery to " << iface.GetLocal ());
  //           lcb (p, header, iif);
  //           // Fall through to additional processing
  //         }
  //         else
  //         {
  //           NS_LOG_ERROR ("Unable to deliver packet locally due to null callback " << p->GetUid () << " from " << origin);
  //           ecb (p, header, Socket::ERROR_NOROUTETOHOST);
  //         }

  //         return true;
  //       }
  //   }
//   }

//   //2.当des地址为自身时，这种情况会在target接受packet时出现
//   if (m_ipv4->IsDestinationAddress (dst, iif))
//   {
//     if (lcb.IsNull () == false)
//       {
//         NS_LOG_LOGIC ("Unicast local delivery to " << dst);
//         lcb (p, header, iif);
//       }
//     else
//       {
//         NS_LOG_ERROR ("Unable to deliver packet locally due to null callback " << p->GetUid () << " from " << origin);
//         ecb (p, header, Socket::ERROR_NOROUTETOHOST);
//       }
//     return true;
//   }
//   return false;
  // return true;
}


Ptr<Node>
RoutingProtocol::GetNodeWithIpv4Address(Ipv4Address addr)
{
  NS_LOG_FUNCTION(this << addr);
  uint32_t nNode = NodeList::GetNNodes();
  for(uint32_t i=0; i<nNode; i++)
  {
    Ptr<Node> node = NodeList::GetNode(i);
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    int32_t iif = ipv4->GetInterfaceForAddress(addr);
    if(iif != -1)
    {
      return node;
    }
  }

  return 0;
}




Ptr<Node>
RoutingProtocol::GetNodeWithPosition(Vector pos)
{
  NS_LOG_FUNCTION(this << pos);
  uint32_t nNode = NodeList::GetNNodes();
  for(uint32_t i=0; i<nNode; i++)
  {
    Ptr<Node> node = NodeList::GetNode(i);
    Vector NodePos = node->GetObject<MobilityModel>()->GetPosition();
    if(NodePos.x == pos.x && NodePos.y == pos.y && NodePos.z == pos.z)
    {
      return node;
    }
  }
  return 0;
}


// void
// RoutingProtocol::RecvVBF(Ptr<Socket> socket)
// {
//   NS_LOG_FUNCTION (this << socket);
//   Address sourceAddress;
//   Ptr<Packet> packet = socket->RecvFrom (sourceAddress);
//   InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom (sourceAddress);
//   /*  这里的socket是外层vbf所使用的socket，也就是说假设我们的路径是A->B->C，那么在里层的socket对应的packet的路径是A->C
//       而在外层用于vbf的socket中，其vbfpacket过程是：
//       1.第一步A->B(ip头部：源地址A->目标地址广播)，而B成功接受了这个vbfpacket，
//       于是一次通信过程就完成了，也就是说B的socket此时拿到的源地址是A的地址，也可以认为是上一跳地址
//       2.第二步，假如B判断出自己是转发节点，则再把这个vbfpacket进行头部修改，然后再发到ip层再发出去
//       （此时同样有ip头部：源地址B->目标地址广播），C处接收到这个vbfpacket时，socket就会读到源地址变成了B，即上一跳地址变成了B

//       3.所以可以看出，在vbfpacket的每一跳转发中，这个vbfpacket上的ipheader中的目的地址一直都是255，而源地址则不停变为上一跳地址
//       4.真正的packet源地址存放在vbfheader中
//   */
  
//   Ipv4Address sender = inetSourceAddr.GetIpv4 (); //socket接收到的源地址，即上一跳地址
//   Ipv4Address receiver; //receiver是当前节点ip地址

//   if (m_socketAddresses.find (socket) != m_socketAddresses.end ())
//   {
//     receiver = m_socketAddresses[socket].GetLocal ();
//   }
//   else if (m_socketSubnetBroadcastAddresses.find (socket) != m_socketSubnetBroadcastAddresses.end ())
//   {
//     receiver = m_socketSubnetBroadcastAddresses[socket].GetLocal ();
//   }
//   else
//   {
//     NS_ASSERT_MSG (false, "Received a packet from an unknown socket");
//   }
//   NS_LOG_DEBUG ("vbf node " << this->m_ipv4->GetObject<Node>()->GetId() << " received a vbf packet from " << sender << " to " << receiver);

//   vbfHeader vbf;
//   packet->PeekHeader(vbf);

//   //vbfpacket在此进行处理
//   Ipv4Address forwarderAddr = sender; //上一跳节点ip
//   Vector forwarderPos = GetNodeWithIpv4Address(forwarderAddr)->GetObject<MobilityModel> ()->GetPosition();//上一跳节点地址，用于下面的HashTable
//   Ipv4Address myAddr = receiver; //当前节点ip


//   //判断是自己的哪个网卡拿到了这个packet，以此来得到对应传播范围
//   uint32_t iif = m_ipv4->GetInterfaceForAddress(myAddr);
//   Ipv4InterfaceAddress iface = m_ipv4->GetAddress (iif, 0); 
//   double Range;
//   double Speed;
//   if(iface.GetBroadcast() == Ipv4Address("10.1.1.255"))
//   {
//     Range = 25000;
//     Speed = 1500;
//   }
//   else if(iface.GetBroadcast() == Ipv4Address("10.1.2.255"))
//   {
//     Range = 50000;
//     Speed = 300000000;
//   }
//   else if(iface.GetBroadcast() == Ipv4Address("10.1.3.255"))
//   {
//     Range = 2500000;
//     Speed = 300000000;
//   }
//   else
//   {
//     NS_ASSERT("not the vaild address");
//   }

//   vbf_neighborhood *hashPtr= PktTable.GetHash(vbf.GetSenderAddr(), vbf.GetPkNum());
// 	//一个节点可能会从不同的邻居处收到这个packet，所以检查是否曾经收到过这个packet，因为sourceIp和pkNum会唯一标识一个packet
//   if (hashPtr != NULL) 
//   {
//     double factor = CalculateNeighborFactor(packet,forwarderPos,Range);
//     PktTable.PutInHash(vbf.GetSenderAddr(), vbf.GetPkNum(),forwarderPos, factor);
//     packet=0;
//     return;
// 	}
// 	else {
// 		// Never receive it before ? Put in hash table.
// 		//printf("vectrobasedforward: this is new packet\n");
//     double factor = CalculateNeighborFactor(packet,forwarderPos,Range);
// 		PktTable.PutInHash(vbf.GetSenderAddr(), vbf.GetPkNum(),forwarderPos, factor);

//     ConsiderNew(packet,myAddr,Range,Speed);
//   }


// }


// void
// RoutingProtocol::ConsiderNew(Ptr<Packet> pkt, Ipv4Address receiver, double range, double speed)
// {
//   NS_LOG_FUNCTION (this << receiver << pkt);
//   NS_LOG_DEBUG("the range of receive node: " << range);
//   vbfHeader vbf;
//   Ptr<Packet> packet = pkt->Copy();
//   packet->PeekHeader(vbf);

//   Ipv4Address targetAddr = vbf.GetTargetAddr();
//   Ipv4Address myAddr = receiver;
//   Ptr<Node> myNode = GetNodeWithIpv4Address(myAddr);

//   //1,本节点是target节点
//   if(targetAddr == myAddr)
//   {
//     NS_LOG_DEBUG ("Node " << myNode->GetId() << " is the destination, receive it!");
//     TargetReceivePacket(packet);
//     return;
//   }

//   //2.不是target节点
//   if(myAddr != targetAddr)
//   {
//     if(IsCloseEnough(packet)) //节点位于路由管道内部
//     {
//       //计算最终的Tadaptation Delay
//       double Delay = CalculateDelay(pkt, range,speed); 
//       SetDelayTimer(packet, Delay);
//     }
//     else if(IsSatellite(myNode)) //卫星节点，测试代码
//     {
//       SetDelayTimer(packet, 0);
//     }
//     else //节点不在路由管道，直接抛弃packet
//     {
//       pkt=0;
//     }e
//   }

//   return;
// }

// bool
// RoutingProtocol::IsSatellite(Ptr<Node> node)
// {
//   if(node->GetObject<MobilityModel>()->GetPosition().z > 1000000)
//   return true;
//   else return false;
// }

// bool
// RoutingProtocol::IsCloseEnough(Ptr<Packet> pkt)
// {
//   NS_LOG_FUNCTION (this); 
//   // if ((Projection(pkt) <= m_width))   //正常管道
//   // {
//   //   NS_LOG_DEBUG("the Node is Close Enough");
//   //   return true;
//   // }
//   if(Distance(pkt) <= m_width)  //适应卫星的特殊化管道
//   {
//     NS_LOG_DEBUG("the Node is Close Enough");
//     return true;
//   }
//   else return false;
// }


// double
// RoutingProtocol::Projection(Ptr<Packet> pkt)
// {
//   NS_LOG_FUNCTION (this);
//   vbfHeader vbf;
//   Ptr<Packet> packet = pkt->Copy();
//   packet->PeekHeader(vbf);

//   Ipv4Address senderAddr = vbf.GetSenderAddr();
//   Ipv4Address forwarderAddr = vbf.GetForwardAddr();
//   Ipv4Address targetAddr = vbf.GetTargetAddr();

//   Ptr<Node> senderNode = GetNodeWithIpv4Address(senderAddr);
//   Ptr<Node> forwarderNode = GetNodeWithIpv4Address(forwarderAddr);
//   Ptr<Node> targetNode = GetNodeWithIpv4Address(targetAddr);
//   Ptr<Node> myNode =  m_ipv4->GetObject<Node>();

//   Vector senderPos = senderNode->GetObject<MobilityModel>()->GetPosition();
//   Vector forwarderPos = forwarderNode->GetObject<MobilityModel>()->GetPosition();
//   Vector targetPos = targetNode->GetObject<MobilityModel>()->GetPosition();
//   Vector myPos = myNode->GetObject<MobilityModel>()->GetPosition();

//   if(!m_hopByHop)
//   {
//     //the vector from sender to target
//     Vector s2t = targetPos - senderPos;
//     //the vector from sender to myNode
//     Vector s2m = myPos - senderPos;

//     double cross_x = s2m.y*s2t.z - s2m.z*s2t.y;
//     double cross_y = s2m.z*s2t.x - s2m.x*s2t.z;
//     double cross_z = s2m.x*s2t.y - s2m.y*s2t.x;

// 	//求s2t和s2m构成的平行四边形面积
//   double area = sqrt(cross_x*cross_x + cross_y*cross_y + cross_z*cross_z);
// 	//求底边长度
// 	double length=sqrt(s2t.x*s2t.x + s2t.y*s2t.y + s2t.z*s2t.z);
//   if(length == 0) return 0;
//   double projection = area/length;

//   NS_LOG_DEBUG("Node " << myNode->GetId() << " No Hopbyhop Projection Calculate: area is " << area 
//                 << " length is " << length << " projection is " << projection);

//   return projection;
//   }
//   else
//   {
//     //the vector from forwarder to target
//     Vector f2t = targetPos - forwarderPos;
//     //the vector from forwarder to myNode
//     Vector f2m = myPos - forwarderPos;

//     double cross_x = f2m.y*f2t.z - f2m.z*f2t.y;
//     double cross_y = f2m.z*f2t.x - f2m.x*f2t.z;
//     double cross_z = f2m.x*f2t.y - f2m.y*f2t.x;

//     double area = sqrt(cross_x*cross_x + cross_y*cross_y + cross_z*cross_z);
//     double length=sqrt(f2t.x*f2t.x + f2t.y*f2t.y + f2t.z*f2t.z);
//     if(length == 0) return 0;
//     double projection = area/length;

//     NS_LOG_DEBUG("Node " << myNode->GetId() << " Use Hopbyhop Projection Calculate: area is " << area 
//                  << " length is " << length << " projection is " << projection);

//     return projection;

//   }

// }

// double
// RoutingProtocol::Distance(Ptr<Packet> pkt)
// {
//   NS_LOG_FUNCTION (this);
//   vbfHeader vbf;
//   Ptr<Packet> packet = pkt->Copy();
//   packet->PeekHeader(vbf);

//   Ipv4Address senderAddr = vbf.GetSenderAddr();
//   Ipv4Address forwarderAddr = vbf.GetForwardAddr();
//   Ipv4Address targetAddr = vbf.GetTargetAddr();

//   Ptr<Node> senderNode = GetNodeWithIpv4Address(senderAddr);
//   Ptr<Node> forwarderNode = GetNodeWithIpv4Address(forwarderAddr);
//   Ptr<Node> targetNode = GetNodeWithIpv4Address(targetAddr);
//   Ptr<Node> myNode =  m_ipv4->GetObject<Node>();

//   Vector senderPos = senderNode->GetObject<MobilityModel>()->GetPosition();
//   // Vector forwarderPos = forwarderNode->GetObject<MobilityModel>()->GetPosition();
//   Vector targetPos = targetNode->GetObject<MobilityModel>()->GetPosition();
//   Vector myPos = myNode->GetObject<MobilityModel>()->GetPosition();


//   //the vector from sender to target
//   Vector s2t = targetPos - senderPos;

//   Vector n = {s2t.y, -s2t.x, 0};
//   double lengthN = n.GetLength();
//   n = {s2t.y/lengthN, -s2t.x/lengthN, 0}; //归一化
//   double distance = fabs(n.x*(myPos.x-senderPos.x)+n.y*(myPos.y-senderPos.y)+0)/n.GetLength();

//   NS_LOG_DEBUG("Node " << myNode->GetId() << " Distance is " << distance);

//   return distance;
  
// }



// double
// RoutingProtocol::CalculateDelay(Ptr<Packet> pkt, double range, double speed)
// {
//   NS_LOG_FUNCTION (this);
//   vbfHeader vbf;
//   Ptr<Packet> packet = pkt->Copy();
//   packet->PeekHeader(vbf);

//   Ipv4Address forwarderAddr = vbf.GetForwardAddr();
//   Ipv4Address targetAddr = vbf.GetTargetAddr();

//   Ptr<Node> forwarderNode = GetNodeWithIpv4Address(forwarderAddr);
//   Ptr<Node> targetNode = GetNodeWithIpv4Address(targetAddr);
//   Ptr<Node> myNode =  m_ipv4->GetObject<Node>();

//   Vector forwarderPos = forwarderNode->GetObject<MobilityModel>()->GetPosition();
//   Vector targetPos = targetNode->GetObject<MobilityModel>()->GetPosition();
//   Vector myPos = myNode->GetObject<MobilityModel>()->GetPosition();

//   //the vector from forwarder to target
//   Vector f2t = targetPos - forwarderPos;
//   //the vector from forwarder to myNode
//   Vector f2m = myPos - forwarderPos;

//   //利用向量点积的关系求f2t和f2m之间的夹角
//   double f2mf2t = f2t.x*f2m.x + f2t.y*f2m.y + f2t.z*f2m.z;
//   double length_f2m = sqrt(f2m.x*f2m.x + f2m.y*f2m.y + f2m.z*f2m.z);
//   double length_f2t = sqrt(f2t.x*f2t.x + f2t.y*f2t.y + f2t.z*f2t.z);
//   double cos_theta;
//   if(length_f2m==0 || length_f2t==0){cos_theta = 0;}
//   else{cos_theta = f2mf2t/(length_f2m*length_f2t);}

//   //利用cos_theta和projection去计算factor
//   m_TransRange = range;
//   m_SoundSpeed = speed;
//   double p = Projection(pkt);
//   double factor = (p/m_width)+(m_TransRange-length_f2m*cos_theta)/m_TransRange;
//   NS_ASSERT(factor>=0);

//   NS_LOG_DEBUG("Node " << myNode->GetId() << " Calculate Fator: " <<  "cos_theta is: " << cos_theta
//                 << " length_f2m is " << length_f2m << " fator is " << factor);

//   //计算最终的Tadaptation Delay
//   double Delay = sqrt(factor)*DELAY_PRE + (m_TransRange-length_f2m)/m_SoundSpeed; //wifi和uan的m_TransRange不一样，所以会导致delay=负数
//   NS_LOG_DEBUG("Calculate Delay = " << Delay);
//   NS_ASSERT(Delay>=0);
//   return Delay;
// }

// double
// RoutingProtocol::CalculateNeighborFactor(Ptr<Packet> pkt, Vector neighborPos, double range)
// {
//   NS_LOG_FUNCTION (this);
//   vbfHeader vbf;
//   Ptr<Packet> packet = pkt->Copy();
//   packet->PeekHeader(vbf);

//   Ipv4Address targetAddr = vbf.GetTargetAddr();

//   Ptr<Node> targetNode = GetNodeWithIpv4Address(targetAddr);
//   Ptr<Node> myNode =  m_ipv4->GetObject<Node>();

//   Vector forwarderPos = neighborPos;
//   Vector targetPos = targetNode->GetObject<MobilityModel>()->GetPosition();
//   Vector myPos = myNode->GetObject<MobilityModel>()->GetPosition();

//   //the vector from forwarder to target
//   Vector f2t = targetPos - forwarderPos;
//   //the vector from forwarder to myNode
//   Vector f2m = myPos - forwarderPos;

//   //利用向量点积的关系求f2t和f2m之间的夹角
//   double f2mf2t = f2t.x*f2m.x + f2t.y*f2m.y + f2t.z*f2m.z;
//   double length_f2m = sqrt(f2m.x*f2m.x + f2m.y*f2m.y + f2m.z*f2m.z);
//   double length_f2t = sqrt(f2t.x*f2t.x + f2t.y*f2t.y + f2t.z*f2t.z);
//   double cos_theta;
//   if(length_f2m==0 || length_f2t==0){cos_theta = 0;}
//   else{cos_theta = f2mf2t/(length_f2m*length_f2t);}

//   //根据邻居的位置去判断是哪个接受网卡，以此得到范围

//   //利用cos_theta和projection去计算factor
  
//   m_TransRange = range;
//   double p = Projection(pkt);
//   double factor = (p/m_width)+(m_TransRange-length_f2m*cos_theta)/m_TransRange;
//   NS_LOG_DEBUG( " Calculate Neighbor fator is " << factor);
//   NS_ASSERT(factor>=0);
//   return factor;
// }


// void
// RoutingProtocol::SetDelayTimer(Ptr<Packet> pkt, double delay)
// {
//   NS_LOG_FUNCTION(this << delay);
//   if(delay<0)delay=0;
//   Simulator::Schedule(Seconds(delay),&RoutingProtocol::Timeout,this,pkt);
// }


// void
// RoutingProtocol::Timeout(Ptr<Packet> pkt)
// {
//   NS_LOG_FUNCTION(this);
//   vbfHeader vbf;
//   Ptr<Packet> packet = pkt->Copy();
//   packet->PeekHeader(vbf);

//   vbf_neighborhood* hashPtr= PktTable.GetHash(vbf.GetSenderAddr(), vbf.GetPkNum());
//   if (hashPtr != NULL) //存在邻居，这里似乎是必然出现这种情况的，毕竟至少此时的packet的forwarder肯定是一个邻居
//   {
//     int num_neighbor = hashPtr->number;
//     if(num_neighbor!=1)
//     {
//       if(num_neighbor == MAX_NEIGHBOR) //有太多邻居，自己不需要再转发这个packet了
//       {
//         pkt=0;
//         return;
//       }
//       else //计算min(factor0,factor1,...,factorN)，若小于m_priority/(2^N) 则转发
//       {
//         double minFactor;
//         // Vector neighborPos;
//         // Ptr<Node> neighborNode;
//         for(int i=0; i<num_neighbor; i++)
//         {
//           // neighborPos = hashPtr->neighbor[i];
//           double neighborFactor = hashPtr->neighborFactor[i];
//           if(i == 0)
//           {
//             minFactor = neighborFactor;
//           }
//           else
//           {
//             if(neighborFactor < minFactor)
//             {
//               minFactor = neighborFactor;
//             }
//           }
//         }
//         NS_LOG_DEBUG( num_neighbor << " Neighbor Factor and the min Factor is: " << minFactor);

//         if(minFactor < (m_priority/pow(2,num_neighbor-1)))//进行转发
//         {
//           NS_LOG_DEBUG("the Node can be a Forwarding Node");
//           //可能会从多个网卡处转发出去，所以vbfpacket中存放的forwarderIp会不一样
//           //这里可以想办法改进，不是每个节点都平均化，所以如果浮标节点接到，可以让浮标节点只用卫星网卡去发packet，避免再把packet发回去水下网络中
//           for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
//                m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
//             {
//               vbfHeader vbfh;
//               Ptr<Packet> newPacket = pkt->Copy();
//               newPacket->RemoveHeader(vbfh);
//               Ptr<Socket> socket = j->first;
//               Ipv4InterfaceAddress iface = j->second;
//               Ipv4Address forwarderIp = iface.GetLocal();
//               uint8_t hops = vbfh.GetHopCount();

//               vbfh.SetForwardAddr(forwarderIp);
//               vbfh.SetHopCount(hops+1);
//               newPacket->AddHeader(vbfh);

//               Ipv4Address destination;
//               if (iface.GetMask () == Ipv4Mask::GetOnes ())
//                 {
//                   destination = Ipv4Address ("255.255.255.255");
//                 }
//               else
//                 {
//                   destination = iface.GetBroadcast ();
//                 }
//             //需要设定一个随机延时，避免碰撞
//               uint32_t sendTime= m_uniformRandomVariable->GetInteger (0, 5);
//               NS_LOG_DEBUG("subnet number:" << destination << ". Set send delay: " << sendTime);
//               Simulator::Schedule (Seconds(sendTime), &RoutingProtocol::SendTo, this, socket, newPacket, destination);

//             }
//         }
//         else  //丢弃
//         {
//         pkt=0;
//         return;
//         }
//       }
//     }
//     else //num_neighbor=1，即该forwarder是唯一邻居
//     {
//       NS_LOG_DEBUG("the Node can be a Forwarding Node(as the onlyNeighbor)");
//       for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
//           m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
//       {
//         vbfHeader vbfh;
//         Ptr<Packet> newPacket = pkt->Copy();
//         newPacket->RemoveHeader(vbfh);
//         Ptr<Socket> socket = j->first;
//         Ipv4InterfaceAddress iface = j->second;
//         Ipv4Address forwarderIp = iface.GetLocal();
//         uint8_t hops = vbfh.GetHopCount();

//         vbfh.SetForwardAddr(forwarderIp);
//         vbfh.SetHopCount(hops+1);
//         newPacket->AddHeader(vbfh);

//         Ipv4Address destination;
//         if (iface.GetMask () == Ipv4Mask::GetOnes ())
//         {
//           destination = Ipv4Address ("255.255.255.255");
//         }
//         else
//         {
//           destination = iface.GetBroadcast ();
//         }
//         uint32_t sendTime= m_uniformRandomVariable->GetInteger (0, 5);
//         NS_LOG_DEBUG("subnet number:" << destination << ". Set send delay: " << sendTime);
//         Simulator::Schedule (Seconds(sendTime), &RoutingProtocol::SendTo, this, socket, newPacket, destination);
//       }
//     }

//   }

// }



// void
// RoutingProtocol::TargetReceivePacket(Ptr<Packet> pkt)
// {
//   NS_LOG_FUNCTION(this);
//   Ptr<Packet> TruePacket = pkt->Copy();
//   vbfHeader vbf;
//   TruePacket->RemoveHeader(vbf); //此时得到一个带有真正ip头部的ip packet，进行本地Delivery

//   Ipv4Header ipv4h;
//   TruePacket->PeekHeader(ipv4h);

//   Ipv4Address TrueSource = ipv4h.GetSource();
//   Ipv4Address TrueDst = ipv4h.GetDestination();
//   Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol>();
//   int32_t Recviif = l3->GetInterfaceForAddress(TrueDst);
//   Ptr<NetDevice> Recvdevice =  l3->GetNetDevice(Recviif);

//   l3->Receive(Recvdevice, TruePacket, Ipv4L3Protocol::PROT_NUMBER, TrueSource, TrueDst, NetDevice::PacketType (0));
//   NS_LOG_DEBUG("True Local Delivery");

// }


// Ptr<Socket>
// RoutingProtocol::FindSocketWithInterfaceAddress (Ipv4InterfaceAddress addr ) const
// {
//   NS_LOG_FUNCTION (this << addr);
//   for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
//          m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
//     {
//       Ptr<Socket> socket = j->first;
//       Ipv4InterfaceAddress iface = j->second;
//       if (iface == addr)
//         {
//           return socket;
//         }
//     }
//   Ptr<Socket> socket;
//   return socket;
// }

// Ptr<Socket>
// RoutingProtocol::FindSubnetBroadcastSocketWithInterfaceAddress (Ipv4InterfaceAddress addr ) const
// {
//   NS_LOG_FUNCTION (this << addr);
//   for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
//          m_socketSubnetBroadcastAddresses.begin (); j != m_socketSubnetBroadcastAddresses.end (); ++j)
//     {
//       Ptr<Socket> socket = j->first;
//       Ipv4InterfaceAddress iface = j->second;
//       if (iface == addr)
//         {
//           return socket;
//         }
//     }
//   Ptr<Socket> socket;
//   return socket;
// }


bool
RoutingProtocol::IsMyOwnAddress (Ipv4Address src)
{
  NS_LOG_FUNCTION (this << src);
  uint32_t infNum =  m_ipv4->GetNInterfaces();
  for(uint32_t i=0; i<infNum; i++)
  {
    if(src == m_ipv4->GetAddress(i,0).GetLocal())
    {
      return true;
    }
  }

  return false;
}





}
}
//------------------------------------------Over----------------------------------------

