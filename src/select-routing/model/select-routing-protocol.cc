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
// #define buoyandsatellite
// #define MAX_UAN_NODE 38 //水下节点的最大nodeid
// #define SatelliteId 103
// uint32_t buoyId[] = {25,28,31,71,74,77};
// std::set<uint32_t> buoyIdSet(std::begin(buoyId), std::end(buoyId)) ; //储存buoy节点的NodeId
// uint32_t buoyNbhId[] = 
// {13,14,24,26,36,37,
//  16,17,27,29,39,40,
//  19,20,30,32,42,43,
//  59,60,70,72,82,83,
//  62,63,73,75,85,86,
//  65,66,76,78,88,89
// };
// std::set<uint32_t> buoyNeighborIdSet(std::begin(buoyNbhId), std::end(buoyNbhId));
//  储存buoy节点的邻居NodeId

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


//------------------------------------Routing Protocol--------------------------------------
NS_OBJECT_ENSURE_REGISTERED (RoutingProtocol);

/// UDP Port for vbf control traffic
const uint32_t RoutingProtocol::vbf_PORT = 654;

RoutingProtocol::RoutingProtocol ()
{
  m_uniformRandomVariable = CreateObject<UniformRandomVariable>();
}

TypeId
RoutingProtocol::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::select_route::RoutingProtocol")
    .SetParent<Ipv4RoutingProtocol> ()
    .SetGroupName ("select_route")
    .AddConstructor<RoutingProtocol> ()
    // .AddAttribute ("satelliteId", "array of satellite id",
    //   PointerValue(),
    //   MakeDoubleAccessor(&RoutingProtocol::m_satelliteId),
    //   MakeDoubleChecker<double>())
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
RoutingProtocol::SetSatelliteId(std::vector<uint32_t> v)
{
  m_satelliteId=v;
}

void 
RoutingProtocol::SetBuoyId(std::vector<uint32_t> v)
{
  m_buoyId=v;
}

void
RoutingProtocol::SetBuoyNbhId(std::vector<uint32_t> v)
{
  m_buoynhbId=v;
}

void
RoutingProtocol::SetUanMaxId(uint32_t id)
{
  m_uanMaxNodeId=id;
}

void
RoutingProtocol::SetBuoySatelliteMode(bool flag)
{
  m_bsmode=flag;
}

void
RoutingProtocol::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  //这里进行初始化邻居信息
  uint32_t myNodeId = this->m_ipv4->GetObject<Node>()->GetId();

  if(m_bsmode)
  {
    //此时是卫星、浮标、水下节点
    auto it = std::find(m_buoyId.begin(),m_buoyId.end(),myNodeId);
    if (it != m_buoyId.end() && !this->m_nbhTable.m_isInit) 
    {
      // 元素找到，是浮标节点
        buoyTableInit(myNodeId);
    }
    else 
    {
      // 元素未找到，是水下或者卫星节点
      auto it = std::find(m_satelliteId.begin(),m_satelliteId.end(),myNodeId);
      if (it != m_satelliteId.end() && !this->m_nbhTable.m_isInit) 
      {
      //是卫星
        if(!this->m_nbhTable.m_isInit){
        Simulator::Schedule(Seconds(2),&RoutingProtocol::satelliteTableInit,this,myNodeId);} 
      }
      else  
      {
        //是其他水下节点
        if(!this->m_nbhTable.m_isInit){
        Simulator::Schedule(Seconds(1),&RoutingProtocol::uanTableInit,this,myNodeId);}
      }
    }

  }
  else
  {
    if(!this->m_nbhTable.m_isInit){
    Simulator::Schedule(Seconds(1),&RoutingProtocol::uanTableInit,this,myNodeId);}
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
  auto it = m_satelliteId.begin();
  for(;it!=m_satelliteId.end();it++)
  {
    AddNeighborToTable(*it,NodeType::SATELLITE,satelliteFunc);
  }


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

  if(m_bsmode)
  {
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
  }
  else //纯水下测试时的代码
  {
   AddNeighborToTable(nodeId-3,NodeType::UAN,uanFunc);
   AddNeighborToTable(nodeId-2,NodeType::UAN,uanFunc);
   AddNeighborToTable(nodeId-1,NodeType::UAN,uanFunc);
   AddNeighborToTable(nodeId+1,NodeType::UAN,uanFunc);
   AddNeighborToTable(nodeId+2,NodeType::UAN,uanFunc);
   AddNeighborToTable(nodeId+3,NodeType::UAN,uanFunc);
  }

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
  for (auto it = m_buoyId.begin(); it != m_buoyId.end(); ++it)
  {
    AddNeighborToTable(*it,NodeType::BUOY,satelliteFunc);
  }

  this->m_nbhTable.m_isInit = true;
  nodeId2Table.insert(std::make_pair(nodeId,&this->m_nbhTable));
  NS_LOG_DEBUG ("The Satellite Node "  << this->m_ipv4->GetObject<Node>()->GetId() << " Neighbor table Init Over.");
  this->m_nbhTable.PrintTable();
  return true;

}

//查找哪个浮标节点是这个id节点的邻居,没有则返回０
uint32_t 
RoutingProtocol::findBuoyNbhNode(uint32_t id)
{
  auto it = m_buoyId.begin();
  for (; it != m_buoyId.end(); ++it)
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
  if(id<0 || id>m_uanMaxNodeId)
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
  auto it=m_satelliteId.begin();
  for(;it!=m_satelliteId.end();it++)
  {
    if(node->GetId() == *it)
    {
      //卫星节点： 0-本地网卡  1-satellite
      ipv4Addr = ipv4->GetAddress(1,0);
      return ipv4Addr.GetLocal();
    }
  }
  //浮标节点： 0-本地网卡  1-uan  2-wifi  3-satellite
  ipv4Addr = ipv4->GetAddress(3,0);
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
  auto it=m_satelliteId.begin();
  for(;it!=m_satelliteId.end();it++)
  {
    if(mynodeId==*it) break;
  }

  if(it != m_satelliteId.end()) //代表该节点是卫星节点
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

