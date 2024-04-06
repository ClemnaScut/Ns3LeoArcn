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


#include "vbf-routing-protocol.h"
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
NS_LOG_COMPONENT_DEFINE ("vbfRoutingProtocol");

namespace vbf {

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

// /// @brief 参考另一个PutInHash函数
// /// @param sAddr source ip address of vbfpacket
// /// @param pkNum pk number of vbfpacket
// void
// AquaSimPktHashTable::PutInHash(Ipv4Address sAddr, unsigned int pkNum, double factor)
// {

// 	vbf_neighborhood* hashPtr;

//   hash_entry entry = std::make_pair (sAddr,pkNum);
//   std::map<hash_entry,vbf_neighborhood*>::iterator it;

// 	int k=pkNum-m_windowSize;
// 	if(k>0)
// 	{
// 		for (int i=0; i<k; i++)
// 		{
//       entry.second = i;
//       it = m_htable.find(entry);
//       if(it != m_htable.end())
//       {
//         hashPtr = it->second;
//         delete hashPtr;
//         m_htable.erase(it);
//       }
// 		}
// 	}

//   entry.second = pkNum;
//   hashPtr = GetHash(sAddr,pkNum);
// 	if (hashPtr != NULL) { 
// 		int m=hashPtr->number;
// 		if (m<MAX_NEIGHBOR) {
// 			hashPtr->number++;
// 			hashPtr->neighbor[m].x=0;
// 			hashPtr->neighbor[m].y=0;
// 			hashPtr->neighbor[m].z=0;
// 		}
// 		return;
// 	}
// 	hashPtr=new vbf_neighborhood[1];
// 	hashPtr[0].number=1;
// 	hashPtr[0].neighbor[0].x=0;
// 	hashPtr[0].neighbor[0].y=0;
// 	hashPtr[0].neighbor[0].z=0;
//   std::pair<hash_entry,vbf_neighborhood*> newPair;
//   newPair.first=entry; newPair.second=hashPtr;
//   if (m_htable.insert(newPair).second == false)
//   {
//     delete newPair.second;
//   }
// }

/// @brief 将一个packet-neighborhood信息放入HashTable中
/// @param sAddr source ip address of vbfpacket
/// @param pkNum pk number of vbfpacket
/// @param p forwarder position(neighborhood position)
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
  static TypeId tid = TypeId ("ns3::vbf::RoutingProtocol")
    .SetParent<Ipv4RoutingProtocol> ()
    .SetGroupName ("vbf")
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

RoutingProtocol::~RoutingProtocol ()
{
}


void
RoutingProtocol::NotifyAddAddress (uint32_t i, Ipv4InterfaceAddress address)
{
  NS_LOG_FUNCTION (this << " interface " << i << " address " << address);
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
  if (!l3->IsUp (i))
    {
      return;
    }
  if (l3->GetNAddresses (i) == 1)
    {
      Ipv4InterfaceAddress iface = l3->GetAddress (i, 0);
      Ptr<Socket> socket = FindSocketWithInterfaceAddress (iface);
      if (!socket)
        {
          if (iface.GetLocal () == Ipv4Address ("127.0.0.1"))
            {
              return;
            }
          // Create a socket to listen only on this interface
          Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (),
                                                     UdpSocketFactory::GetTypeId ());
          NS_ASSERT (socket != 0);
          socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvVBF,this));
          socket->BindToNetDevice (l3->GetNetDevice (i));
          socket->Bind (InetSocketAddress (iface.GetLocal (), vbf_PORT));
          socket->SetAllowBroadcast (true);
          m_socketAddresses.insert (std::make_pair (socket, iface));

          // create also a subnet directed broadcast socket
          socket = Socket::CreateSocket (GetObject<Node> (),
                                         UdpSocketFactory::GetTypeId ());
          NS_ASSERT (socket != 0);
          socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvVBF, this));
          socket->BindToNetDevice (l3->GetNetDevice (i));
          socket->Bind (InetSocketAddress (iface.GetBroadcast (), vbf_PORT));
          socket->SetAllowBroadcast (true);
          socket->SetIpRecvTtl (true);
          m_socketSubnetBroadcastAddresses.insert (std::make_pair (socket, iface));
        }
    }
  else
    {
      NS_LOG_LOGIC ("vbf does not work with more then one address per each interface. Ignore added address");
    }
}



void
RoutingProtocol::NotifyRemoveAddress (uint32_t i, Ipv4InterfaceAddress address)
{
  NS_LOG_FUNCTION (this);
  Ptr<Socket> socket = FindSocketWithInterfaceAddress (address);
  if (socket)
    {
      socket->Close ();
      m_socketAddresses.erase (socket);

      Ptr<Socket> unicastSocket = FindSubnetBroadcastSocketWithInterfaceAddress (address);
      if (unicastSocket)
        {
          unicastSocket->Close ();
          m_socketAddresses.erase (unicastSocket);
        }

      Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
      if (l3->GetNAddresses (i))
        {
          Ipv4InterfaceAddress iface = l3->GetAddress (i, 0);
          // Create a socket to listen only on this interface
          Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (),
                                                     UdpSocketFactory::GetTypeId ());
          NS_ASSERT (socket != 0);
          socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvVBF, this));
          // Bind to any IP address so that broadcasts can be received
          socket->BindToNetDevice (l3->GetNetDevice (i));
          socket->Bind (InetSocketAddress (iface.GetLocal (), vbf_PORT));
          socket->SetAllowBroadcast (true);
          socket->SetIpRecvTtl (true);
          m_socketAddresses.insert (std::make_pair (socket, iface));

          // create also a unicast socket
          socket = Socket::CreateSocket (GetObject<Node> (),
                                         UdpSocketFactory::GetTypeId ());
          NS_ASSERT (socket != 0);
          socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvVBF, this));
          socket->BindToNetDevice (l3->GetNetDevice (i));
          socket->Bind (InetSocketAddress (iface.GetBroadcast (), vbf_PORT));
          socket->SetAllowBroadcast (true);
          socket->SetIpRecvTtl (true);
          m_socketSubnetBroadcastAddresses.insert (std::make_pair (socket, iface));

        }
      if (m_socketAddresses.empty ())
        {
          NS_LOG_LOGIC ("No vbf interfaces");
          return;
        }
    }
  else
    {
      NS_LOG_LOGIC ("Remove address not participating in vbf operation");
    }
}


void
RoutingProtocol::NotifyInterfaceUp (uint32_t i)
{
  NS_LOG_FUNCTION (this << m_ipv4->GetAddress (i, 0).GetLocal ());
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
  if (l3->GetNAddresses (i) > 1)
    {
      NS_LOG_WARN ("vbf does not work with more then one address per each interface.");
    }
  Ipv4InterfaceAddress iface = l3->GetAddress (i, 0);
  if (iface.GetLocal () == Ipv4Address ("127.0.0.1"))
    {
      return;
    }

  // Create a socket to listen only on this interface
  Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (),
                                             UdpSocketFactory::GetTypeId ());
  NS_ASSERT (socket != 0);
  socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvVBF, this));
  socket->BindToNetDevice (l3->GetNetDevice (i));
  socket->Bind (InetSocketAddress (iface.GetLocal (), vbf_PORT));
  socket->SetAllowBroadcast (true);
  socket->SetIpRecvTtl (true);
  m_socketAddresses.insert (std::make_pair (socket, iface));

  // create also a subnet broadcast socket
  socket = Socket::CreateSocket (GetObject<Node> (),
                                 UdpSocketFactory::GetTypeId ());
  NS_ASSERT (socket != 0);
  socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvVBF, this));
  socket->BindToNetDevice (l3->GetNetDevice (i));
  socket->Bind (InetSocketAddress (iface.GetBroadcast (), vbf_PORT));
  socket->SetAllowBroadcast (true);
  socket->SetIpRecvTtl (true);
  m_socketSubnetBroadcastAddresses.insert (std::make_pair (socket, iface));

  // Add local broadcast record to the routing table
  Ptr<NetDevice> dev = m_ipv4->GetNetDevice (m_ipv4->GetInterfaceForAddress (iface.GetLocal ()));

  // Allow neighbor manager use this interface for layer 2 feedback if possible
  Ptr<WifiNetDevice> wifi = dev->GetObject<WifiNetDevice> ();
  if (wifi == 0)
    {
      return;
    }
  Ptr<WifiMac> mac = wifi->GetMac ();
  if (mac == 0)
    {
      return;
    }
}


void
RoutingProtocol::NotifyInterfaceDown (uint32_t i)
{
  NS_LOG_FUNCTION (this << m_ipv4->GetAddress (i, 0).GetLocal ());

  // Disable layer 2 link state monitoring (if possible)
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> ();
  Ptr<NetDevice> dev = l3->GetNetDevice (i);
  Ptr<WifiNetDevice> wifi = dev->GetObject<WifiNetDevice> ();

  // Close socket
  Ptr<Socket> socket = FindSocketWithInterfaceAddress (m_ipv4->GetAddress (i, 0));
  NS_ASSERT (socket);
  socket->Close ();
  m_socketAddresses.erase (socket);

  // Close socket
  socket = FindSubnetBroadcastSocketWithInterfaceAddress (m_ipv4->GetAddress (i, 0));
  NS_ASSERT (socket);
  socket->Close ();
  m_socketSubnetBroadcastAddresses.erase (socket);

  if (m_socketAddresses.empty ())
    {
      NS_LOG_LOGIC ("No vbf interfaces");
      return;
    }
}

void
RoutingProtocol::DoDispose ()
{
  m_ipv4 = 0;
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::iterator iter =
         m_socketAddresses.begin (); iter != m_socketAddresses.end (); iter++)
    {
      iter->first->Close ();
    }
  m_socketAddresses.clear ();
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::iterator iter =
         m_socketSubnetBroadcastAddresses.begin (); iter != m_socketSubnetBroadcastAddresses.end (); iter++)
    {
      iter->first->Close ();
    }
  m_socketSubnetBroadcastAddresses.clear ();
  Ipv4RoutingProtocol::DoDispose ();
}


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
  if (m_socketAddresses.empty ())
    {
      sockerr = Socket::ERROR_NOROUTETOHOST;
      NS_LOG_LOGIC ("No vbf interfaces");
      Ptr<Ipv4Route> route;
      return route;
    }
  sockerr = Socket::ERROR_NOTERROR;
  Ptr<Ipv4Route> route;
  Ipv4Address dst = header.GetDestination ();

  //获取本节点的各种信息
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol> (); //ipv4l3protocol
  int32_t iif = l3->GetInterfaceForAddress(header.GetSource()); //当前ip地址对应的Ipv4interface索引值
  Ipv4InterfaceAddress iface = l3->GetAddress (iif, 0);  //当前ip地址对应的Ipv4interface
  Ipv4Address myaddr = m_ipv4->GetAddress (iif, 0).GetLocal (); //该Ipv4interface对应的ip地址，其实就是Header中的source_Ip地址

  //拿到当前Ipv4Interface的广播地址(子网)
  Ipv4Address BroadDest;
  if (iface.GetMask () == Ipv4Mask::GetOnes ())
    {
	    BroadDest = Ipv4Address ("255.255.255.255");
    }
  else
    {
	    BroadDest = iface.GetBroadcast ();
    }


  //这里只需要处理单播且目标地址不是自己的情况，因为Ip层广播的情况其实是在ipv4l3protocol中处理的，只有单播情况才会调用RouteOutput找路由
  if(dst!=BroadDest && dst!=myaddr)
  {
    vbfHeader vbf(vbfTYPE_VBF,m_pkNum,1,dst,myaddr,myaddr,Simulator::Now(),Simulator::Now());
    m_pkNum++;
    Ptr<Packet> vbfpacket = p->Copy();
    vbfpacket->AddHeader(header);//!!这里有待考虑，是为了后面的TargetReceivePacket
    vbfpacket->AddHeader(vbf);
    NS_LOG_DEBUG("In VBFheader, destination is:" << vbf.GetTargetAddr() << ", source is:" << vbf.GetSenderAddr()
                  << ",forwarder address is:" << vbf.GetForwardAddr() << ",hop count is:" << vbf.GetHopCount()
                  << ",packet number is:" << vbf.GetPkNum() << ",packet genarate time is:"<< vbf.GetTimeGenerate().GetSeconds()
                  << ",packet forwarded time is:" << vbf.GetTimeForward().GetSeconds());
    
    Simulator::Schedule(Seconds(0),&RoutingProtocol::vbfSend, this, vbfpacket, myaddr, dst);
    return Ptr<Ipv4Route> ();
  }
  else
  {
      NS_LOG_DEBUG ("Output device doesn't match. Dropped.");
      sockerr = Socket::ERROR_NOROUTETOHOST;
      return Ptr<Ipv4Route> ();
  }

}


void
RoutingProtocol::vbfSend(Ptr<Packet> p, Ipv4Address source, Ipv4Address dst)
{
  NS_LOG_FUNCTION(this << "TrueSend: from " << source << " to " << dst);
  NS_ASSERT (p != 0);
  Ptr<Packet> packet = p->Copy();


  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
          m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
  {
    Ptr<Socket> socket = j->first;
    Ipv4InterfaceAddress iface = j->second;

    //vbf协议最终发送的都是ip层的广播包
    Ipv4Address destination;
    if (iface.GetMask () == Ipv4Mask::GetOnes ())
    {
      destination = Ipv4Address ("255.255.255.255");
    }
    else
    {
      destination = iface.GetBroadcast ();
    }

  //以des地址为广播ip地址，将这个vbfpacket从socket处发出
  // Simulator::Schedule (Time (MilliSeconds (m_uniformRandomVariable->GetInteger (0, 10))), &RoutingProtocol::SendTo, this, socket, packet, destination);
  Simulator::Schedule (Seconds(1), &RoutingProtocol::SendTo, this, socket, packet, destination);
  }
}


void
RoutingProtocol::SendTo (Ptr<Socket> socket, Ptr<Packet> packet, Ipv4Address destination)
{
  NS_LOG_FUNCTION(this);
  socket->SendTo (packet, 0, InetSocketAddress (destination, vbf_PORT));

}


bool
RoutingProtocol::RouteInput (Ptr<const Packet> p, const Ipv4Header &header, Ptr<const NetDevice> idev,
                  UnicastForwardCallback ucb, MulticastForwardCallback mcb,
                  LocalDeliverCallback lcb, ErrorCallback ecb)
{
  NS_LOG_FUNCTION (this << p->GetUid () << header.GetDestination () << idev->GetAddress ());
  if (m_socketAddresses.empty ())
  {
    NS_LOG_LOGIC ("No aodv interfaces");
    return false;
  }
  NS_ASSERT (m_ipv4 != 0);
  NS_ASSERT (p != 0);
  // Check if input device supports IP
  NS_ASSERT (m_ipv4->GetInterfaceForDevice (idev) >= 0);
  int32_t iif = m_ipv4->GetInterfaceForDevice (idev);
  Ipv4Address dst = header.GetDestination ();
  Ipv4Address origin = header.GetSource ();
  Ipv4InterfaceAddress iface = m_ipv4->GetAddress (iif, 0); 

  if (IsMyOwnAddress (origin))
  {
    return true;
  }

  //拿到当前Ipv4Interface的广播地址(子网)
  Ipv4Address BroadDest;
  if (iface.GetMask () == Ipv4Mask::GetOnes ())
  {
    BroadDest = Ipv4Address ("255.255.255.255");
  }
  else
  {
    BroadDest = iface.GetBroadcast ();
  }

  /* 1.当前dst地址为广播地址，在VBF中每个节点接收到vbfpacket都会是这种情况，即收到广播packet，
       然后往上交给udp，udp再把vbfpacket交给vbf的socket，触发RecvVBF的callback函数，实际的
       处理由RecvVBF去处理
  */
  
  // Broadcast local delivery/forwarding
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
        m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
  {
    Ipv4InterfaceAddress iface = j->second;
    if (m_ipv4->GetInterfaceForAddress (iface.GetLocal ()) == iif)
    {
      if (dst == BroadDest)
        {
          Ptr<Packet> packet = p->Copy ();
          if (lcb.IsNull () == false)
          {
            NS_LOG_LOGIC ("Broadcast local delivery to " << iface.GetLocal ());
            lcb (p, header, iif);
            // Fall through to additional processing
          }
          else
          {
            NS_LOG_ERROR ("Unable to deliver packet locally due to null callback " << p->GetUid () << " from " << origin);
            ecb (p, header, Socket::ERROR_NOROUTETOHOST);
          }

          return true;
        }
    }
  }


  //2.当des地址为自身时，这种情况会在target接受packet时出现
  if (m_ipv4->IsDestinationAddress (dst, iif))
  {
    if (lcb.IsNull () == false)
      {
        NS_LOG_LOGIC ("Unicast local delivery to " << dst);
        lcb (p, header, iif);
      }
    else
      {
        NS_LOG_ERROR ("Unable to deliver packet locally due to null callback " << p->GetUid () << " from " << origin);
        ecb (p, header, Socket::ERROR_NOROUTETOHOST);
      }
    return true;
  }
  return false;
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


void
RoutingProtocol::RecvVBF(Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  Address sourceAddress;
  Ptr<Packet> packet = socket->RecvFrom (sourceAddress);
  InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom (sourceAddress);
  /*  这里的socket是外层vbf所使用的socket，也就是说假设我们的路径是A->B->C，那么在里层的socket对应的packet的路径是A->C
      而在外层用于vbf的socket中，其vbfpacket过程是：
      1.第一步A->B(ip头部：源地址A->目标地址广播)，而B成功接受了这个vbfpacket，
      于是一次通信过程就完成了，也就是说B的socket此时拿到的源地址是A的地址，也可以认为是上一跳地址
      2.第二步，假如B判断出自己是转发节点，则再把这个vbfpacket进行头部修改，然后再发到ip层再发出去
      （此时同样有ip头部：源地址B->目标地址广播），C处接收到这个vbfpacket时，socket就会读到源地址变成了B，即上一跳地址变成了B

      3.所以可以看出，在vbfpacket的每一跳转发中，这个vbfpacket上的ipheader中的目的地址一直都是255，而源地址则不停变为上一跳地址
      4.真正的packet源地址存放在vbfheader中
  */
  
  Ipv4Address sender = inetSourceAddr.GetIpv4 (); //socket接收到的源地址，即上一跳地址
  Ipv4Address receiver; //receiver是当前节点ip地址

  if (m_socketAddresses.find (socket) != m_socketAddresses.end ())
  {
    receiver = m_socketAddresses[socket].GetLocal ();
  }
  else if (m_socketSubnetBroadcastAddresses.find (socket) != m_socketSubnetBroadcastAddresses.end ())
  {
    receiver = m_socketSubnetBroadcastAddresses[socket].GetLocal ();
  }
  else
  {
    NS_ASSERT_MSG (false, "Received a packet from an unknown socket");
  }
  NS_LOG_DEBUG ("vbf node " << this->m_ipv4->GetObject<Node>()->GetId() << " received a vbf packet from " << sender << " to " << receiver);

  vbfHeader vbf;
  packet->PeekHeader(vbf);

  //vbfpacket在此进行处理
  Ipv4Address forwarderAddr = sender; //上一跳节点ip
  Vector forwarderPos = GetNodeWithIpv4Address(forwarderAddr)->GetObject<MobilityModel> ()->GetPosition();//上一跳节点地址，用于下面的HashTable
  Ipv4Address myAddr = receiver; //当前节点ip


  //判断是自己的哪个网卡拿到了这个packet，以此来得到对应传播范围
  uint32_t iif = m_ipv4->GetInterfaceForAddress(myAddr);
  Ipv4InterfaceAddress iface = m_ipv4->GetAddress (iif, 0); 
  double Range;
  double Speed;
  if(iface.GetBroadcast() == Ipv4Address("10.1.1.255"))
  {
    Range = 25000;
    Speed = 1500;
  }
  else if(iface.GetBroadcast() == Ipv4Address("10.1.2.255"))
  {
    Range = 50000;
    Speed = 300000000;
  }
  else if(iface.GetBroadcast() == Ipv4Address("10.1.3.255"))
  {
    Range = 2500000;
    Speed = 300000000;
  }
  else
  {
    NS_ASSERT("not the vaild address");
  }

  vbf_neighborhood *hashPtr= PktTable.GetHash(vbf.GetSenderAddr(), vbf.GetPkNum());
	//一个节点可能会从不同的邻居处收到这个packet，所以检查是否曾经收到过这个packet，因为sourceIp和pkNum会唯一标识一个packet
  if (hashPtr != NULL) 
  {
    double factor = CalculateNeighborFactor(packet,forwarderPos,Range);
    PktTable.PutInHash(vbf.GetSenderAddr(), vbf.GetPkNum(),forwarderPos, factor);
    packet=0;
    return;
	}
	else {
		// Never receive it before ? Put in hash table.
		//printf("vectrobasedforward: this is new packet\n");
    double factor = CalculateNeighborFactor(packet,forwarderPos,Range);
		PktTable.PutInHash(vbf.GetSenderAddr(), vbf.GetPkNum(),forwarderPos, factor);

    ConsiderNew(packet,myAddr,Range,Speed);
  }


}


void
RoutingProtocol::ConsiderNew(Ptr<Packet> pkt, Ipv4Address receiver, double range, double speed)
{
  NS_LOG_FUNCTION (this << receiver << pkt);
  NS_LOG_DEBUG("the range of receive node: " << range);
  vbfHeader vbf;
  Ptr<Packet> packet = pkt->Copy();
  packet->PeekHeader(vbf);

  Ipv4Address targetAddr = vbf.GetTargetAddr();
  Ipv4Address myAddr = receiver;
  Ptr<Node> myNode = GetNodeWithIpv4Address(myAddr);

  //1,本节点是target节点
  if(targetAddr == myAddr)
  {
    NS_LOG_DEBUG ("Node " << myNode->GetId() << " is the destination, receive it!");
    TargetReceivePacket(packet);
    return;
  }

  //2.不是target节点
  if(myAddr != targetAddr)
  {
    if(IsCloseEnough(packet)) //节点位于路由管道内部
    {
      //计算最终的Tadaptation Delay
      double Delay = CalculateDelay(pkt, range,speed); 
      SetDelayTimer(packet, Delay);
    }
    else if(IsSatellite(myNode)) //卫星节点，测试代码
    {
      SetDelayTimer(packet, 0);
    }
    else //节点不在路由管道，直接抛弃packet
    {
      pkt=0;
    }
  }

  return;
}

bool
RoutingProtocol::IsSatellite(Ptr<Node> node)
{
  if(node->GetObject<MobilityModel>()->GetPosition().z > 1000000)
  return true;
  else return false;
}

bool
RoutingProtocol::IsCloseEnough(Ptr<Packet> pkt)
{
  NS_LOG_FUNCTION (this); 
  // if ((Projection(pkt) <= m_width))   //正常管道
  // {
  //   NS_LOG_DEBUG("the Node is Close Enough");
  //   return true;
  // }
  if(Distance(pkt) <= m_width)  //适应卫星的特殊化管道
  {
    NS_LOG_DEBUG("the Node is Close Enough");
    return true;
  }
  else return false;
}


double
RoutingProtocol::Projection(Ptr<Packet> pkt)
{
  NS_LOG_FUNCTION (this);
  vbfHeader vbf;
  Ptr<Packet> packet = pkt->Copy();
  packet->PeekHeader(vbf);

  Ipv4Address senderAddr = vbf.GetSenderAddr();
  Ipv4Address forwarderAddr = vbf.GetForwardAddr();
  Ipv4Address targetAddr = vbf.GetTargetAddr();

  Ptr<Node> senderNode = GetNodeWithIpv4Address(senderAddr);
  Ptr<Node> forwarderNode = GetNodeWithIpv4Address(forwarderAddr);
  Ptr<Node> targetNode = GetNodeWithIpv4Address(targetAddr);
  Ptr<Node> myNode =  m_ipv4->GetObject<Node>();

  Vector senderPos = senderNode->GetObject<MobilityModel>()->GetPosition();
  Vector forwarderPos = forwarderNode->GetObject<MobilityModel>()->GetPosition();
  Vector targetPos = targetNode->GetObject<MobilityModel>()->GetPosition();
  Vector myPos = myNode->GetObject<MobilityModel>()->GetPosition();

  if(!m_hopByHop)
  {
    //the vector from sender to target
    Vector s2t = targetPos - senderPos;
    //the vector from sender to myNode
    Vector s2m = myPos - senderPos;

    double cross_x = s2m.y*s2t.z - s2m.z*s2t.y;
    double cross_y = s2m.z*s2t.x - s2m.x*s2t.z;
    double cross_z = s2m.x*s2t.y - s2m.y*s2t.x;

	//求s2t和s2m构成的平行四边形面积
  double area = sqrt(cross_x*cross_x + cross_y*cross_y + cross_z*cross_z);
	//求底边长度
	double length=sqrt(s2t.x*s2t.x + s2t.y*s2t.y + s2t.z*s2t.z);
  if(length == 0) return 0;
  double projection = area/length;

  NS_LOG_DEBUG("Node " << myNode->GetId() << " No Hopbyhop Projection Calculate: area is " << area 
                << " length is " << length << " projection is " << projection);

  return projection;
  }
  else
  {
    //the vector from forwarder to target
    Vector f2t = targetPos - forwarderPos;
    //the vector from forwarder to myNode
    Vector f2m = myPos - forwarderPos;

    double cross_x = f2m.y*f2t.z - f2m.z*f2t.y;
    double cross_y = f2m.z*f2t.x - f2m.x*f2t.z;
    double cross_z = f2m.x*f2t.y - f2m.y*f2t.x;

    double area = sqrt(cross_x*cross_x + cross_y*cross_y + cross_z*cross_z);
    double length=sqrt(f2t.x*f2t.x + f2t.y*f2t.y + f2t.z*f2t.z);
    if(length == 0) return 0;
    double projection = area/length;

    NS_LOG_DEBUG("Node " << myNode->GetId() << " Use Hopbyhop Projection Calculate: area is " << area 
                 << " length is " << length << " projection is " << projection);

    return projection;

  }

}

double
RoutingProtocol::Distance(Ptr<Packet> pkt)
{
  NS_LOG_FUNCTION (this);
  vbfHeader vbf;
  Ptr<Packet> packet = pkt->Copy();
  packet->PeekHeader(vbf);

  Ipv4Address senderAddr = vbf.GetSenderAddr();
  Ipv4Address forwarderAddr = vbf.GetForwardAddr();
  Ipv4Address targetAddr = vbf.GetTargetAddr();

  Ptr<Node> senderNode = GetNodeWithIpv4Address(senderAddr);
  Ptr<Node> forwarderNode = GetNodeWithIpv4Address(forwarderAddr);
  Ptr<Node> targetNode = GetNodeWithIpv4Address(targetAddr);
  Ptr<Node> myNode =  m_ipv4->GetObject<Node>();

  Vector senderPos = senderNode->GetObject<MobilityModel>()->GetPosition();
  // Vector forwarderPos = forwarderNode->GetObject<MobilityModel>()->GetPosition();
  Vector targetPos = targetNode->GetObject<MobilityModel>()->GetPosition();
  Vector myPos = myNode->GetObject<MobilityModel>()->GetPosition();


  //the vector from sender to target
  Vector s2t = targetPos - senderPos;

  Vector n = {s2t.y, -s2t.x, 0};
  double lengthN = n.GetLength();
  n = {s2t.y/lengthN, -s2t.x/lengthN, 0}; //归一化
  double distance = fabs(n.x*(myPos.x-senderPos.x)+n.y*(myPos.y-senderPos.y)+0)/n.GetLength();

  NS_LOG_DEBUG("Node " << myNode->GetId() << " Distance is " << distance);

  return distance;
  
}



double
RoutingProtocol::CalculateDelay(Ptr<Packet> pkt, double range, double speed)
{
  NS_LOG_FUNCTION (this);
  vbfHeader vbf;
  Ptr<Packet> packet = pkt->Copy();
  packet->PeekHeader(vbf);

  Ipv4Address forwarderAddr = vbf.GetForwardAddr();
  Ipv4Address targetAddr = vbf.GetTargetAddr();

  Ptr<Node> forwarderNode = GetNodeWithIpv4Address(forwarderAddr);
  Ptr<Node> targetNode = GetNodeWithIpv4Address(targetAddr);
  Ptr<Node> myNode =  m_ipv4->GetObject<Node>();

  Vector forwarderPos = forwarderNode->GetObject<MobilityModel>()->GetPosition();
  Vector targetPos = targetNode->GetObject<MobilityModel>()->GetPosition();
  Vector myPos = myNode->GetObject<MobilityModel>()->GetPosition();

  //the vector from forwarder to target
  Vector f2t = targetPos - forwarderPos;
  //the vector from forwarder to myNode
  Vector f2m = myPos - forwarderPos;

  //利用向量点积的关系求f2t和f2m之间的夹角
  double f2mf2t = f2t.x*f2m.x + f2t.y*f2m.y + f2t.z*f2m.z;
  double length_f2m = sqrt(f2m.x*f2m.x + f2m.y*f2m.y + f2m.z*f2m.z);
  double length_f2t = sqrt(f2t.x*f2t.x + f2t.y*f2t.y + f2t.z*f2t.z);
  double cos_theta;
  if(length_f2m==0 || length_f2t==0){cos_theta = 0;}
  else{cos_theta = f2mf2t/(length_f2m*length_f2t);}

  //利用cos_theta和projection去计算factor
  m_TransRange = range;
  m_SoundSpeed = speed;
  double p = Projection(pkt);
  double factor = (p/m_width)+(m_TransRange-length_f2m*cos_theta)/m_TransRange;
  NS_ASSERT(factor>=0);

  NS_LOG_DEBUG("Node " << myNode->GetId() << " Calculate Fator: " <<  "cos_theta is: " << cos_theta
                << " length_f2m is " << length_f2m << " fator is " << factor);

  //计算最终的Tadaptation Delay
  double Delay = sqrt(factor)*DELAY_PRE + (m_TransRange-length_f2m)/m_SoundSpeed; //wifi和uan的m_TransRange不一样，所以会导致delay=负数
  NS_LOG_DEBUG("Calculate Delay = " << Delay);
  NS_ASSERT(Delay>=0);
  return Delay;
}

double
RoutingProtocol::CalculateNeighborFactor(Ptr<Packet> pkt, Vector neighborPos, double range)
{
  NS_LOG_FUNCTION (this);
  vbfHeader vbf;
  Ptr<Packet> packet = pkt->Copy();
  packet->PeekHeader(vbf);

  Ipv4Address targetAddr = vbf.GetTargetAddr();

  Ptr<Node> targetNode = GetNodeWithIpv4Address(targetAddr);
  Ptr<Node> myNode =  m_ipv4->GetObject<Node>();

  Vector forwarderPos = neighborPos;
  Vector targetPos = targetNode->GetObject<MobilityModel>()->GetPosition();
  Vector myPos = myNode->GetObject<MobilityModel>()->GetPosition();

  //the vector from forwarder to target
  Vector f2t = targetPos - forwarderPos;
  //the vector from forwarder to myNode
  Vector f2m = myPos - forwarderPos;

  //利用向量点积的关系求f2t和f2m之间的夹角
  double f2mf2t = f2t.x*f2m.x + f2t.y*f2m.y + f2t.z*f2m.z;
  double length_f2m = sqrt(f2m.x*f2m.x + f2m.y*f2m.y + f2m.z*f2m.z);
  double length_f2t = sqrt(f2t.x*f2t.x + f2t.y*f2t.y + f2t.z*f2t.z);
  double cos_theta;
  if(length_f2m==0 || length_f2t==0){cos_theta = 0;}
  else{cos_theta = f2mf2t/(length_f2m*length_f2t);}

  //根据邻居的位置去判断是哪个接受网卡，以此得到范围

  //利用cos_theta和projection去计算factor
  
  m_TransRange = range;
  double p = Projection(pkt);
  double factor = (p/m_width)+(m_TransRange-length_f2m*cos_theta)/m_TransRange;
  NS_LOG_DEBUG( " Calculate Neighbor fator is " << factor);
  NS_ASSERT(factor>=0);
  return factor;
}


void
RoutingProtocol::SetDelayTimer(Ptr<Packet> pkt, double delay)
{
  NS_LOG_FUNCTION(this << delay);
  if(delay<0)delay=0;
  Simulator::Schedule(Seconds(delay),&RoutingProtocol::Timeout,this,pkt);
}


void
RoutingProtocol::Timeout(Ptr<Packet> pkt)
{
  NS_LOG_FUNCTION(this);
  vbfHeader vbf;
  Ptr<Packet> packet = pkt->Copy();
  packet->PeekHeader(vbf);

  vbf_neighborhood* hashPtr= PktTable.GetHash(vbf.GetSenderAddr(), vbf.GetPkNum());
  if (hashPtr != NULL) //存在邻居，这里似乎是必然出现这种情况的，毕竟至少此时的packet的forwarder肯定是一个邻居
  {
    int num_neighbor = hashPtr->number;
    if(num_neighbor!=1)
    {
      if(num_neighbor == MAX_NEIGHBOR) //有太多邻居，自己不需要再转发这个packet了
      {
        pkt=0;
        return;
      }
      else //计算min(factor0,factor1,...,factorN)，若小于m_priority/(2^N) 则转发
      {
        double minFactor;
        // Vector neighborPos;
        // Ptr<Node> neighborNode;
        for(int i=0; i<num_neighbor; i++)
        {
          // neighborPos = hashPtr->neighbor[i];
          double neighborFactor = hashPtr->neighborFactor[i];
          if(i == 0)
          {
            minFactor = neighborFactor;
          }
          else
          {
            if(neighborFactor < minFactor)
            {
              minFactor = neighborFactor;
            }
          }
        }
        NS_LOG_DEBUG( num_neighbor << " Neighbor Factor and the min Factor is: " << minFactor);

        if(minFactor < (m_priority/pow(2,num_neighbor-1)))//进行转发
        {
          NS_LOG_DEBUG("the Node can be a Forwarding Node");
          //可能会从多个网卡处转发出去，所以vbfpacket中存放的forwarderIp会不一样
          for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
               m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
            {
              vbfHeader vbfh;
              Ptr<Packet> newPacket = pkt->Copy();
              newPacket->RemoveHeader(vbfh);
              Ptr<Socket> socket = j->first;
              Ipv4InterfaceAddress iface = j->second;
              Ipv4Address forwarderIp = iface.GetLocal();
              uint8_t hops = vbfh.GetHopCount();

              vbfh.SetForwardAddr(forwarderIp);
              vbfh.SetHopCount(hops+1);
              newPacket->AddHeader(vbfh);

              Ipv4Address destination;
              if (iface.GetMask () == Ipv4Mask::GetOnes ())
                {
                  destination = Ipv4Address ("255.255.255.255");
                }
              else
                {
                  destination = iface.GetBroadcast ();
                }
            //需要设定一个随机延时，避免碰撞
              uint32_t sendTime= m_uniformRandomVariable->GetInteger (0, 5);
              NS_LOG_DEBUG("subnet number:" << destination << ". Set send delay: " << sendTime);
              Simulator::Schedule (Seconds(sendTime), &RoutingProtocol::SendTo, this, socket, newPacket, destination);

            }
        }
        else  //丢弃
        {
        pkt=0;
        return;
        }
      }
    }
    else //num_neighbor=1，即该forwarder是唯一邻居
    {
      NS_LOG_DEBUG("the Node can be a Forwarding Node(as the onlyNeighbor)");
      for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
          m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
      {
        vbfHeader vbfh;
        Ptr<Packet> newPacket = pkt->Copy();
        newPacket->RemoveHeader(vbfh);
        Ptr<Socket> socket = j->first;
        Ipv4InterfaceAddress iface = j->second;
        Ipv4Address forwarderIp = iface.GetLocal();
        uint8_t hops = vbfh.GetHopCount();

        vbfh.SetForwardAddr(forwarderIp);
        vbfh.SetHopCount(hops+1);
        newPacket->AddHeader(vbfh);

        Ipv4Address destination;
        if (iface.GetMask () == Ipv4Mask::GetOnes ())
        {
          destination = Ipv4Address ("255.255.255.255");
        }
        else
        {
          destination = iface.GetBroadcast ();
        }
        uint32_t sendTime= m_uniformRandomVariable->GetInteger (0, 5);
        NS_LOG_DEBUG("subnet number:" << destination << ". Set send delay: " << sendTime);
        Simulator::Schedule (Seconds(sendTime), &RoutingProtocol::SendTo, this, socket, newPacket, destination);
      }
    }

  }

}



void
RoutingProtocol::TargetReceivePacket(Ptr<Packet> pkt)
{
  NS_LOG_FUNCTION(this);
  Ptr<Packet> TruePacket = pkt->Copy();
  vbfHeader vbf;
  TruePacket->RemoveHeader(vbf); //此时得到一个带有真正ip头部的ip packet，进行本地Delivery

  Ipv4Header ipv4h;
  TruePacket->PeekHeader(ipv4h);

  Ipv4Address TrueSource = ipv4h.GetSource();
  Ipv4Address TrueDst = ipv4h.GetDestination();
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol>();
  int32_t Recviif = l3->GetInterfaceForAddress(TrueDst);
  Ptr<NetDevice> Recvdevice =  l3->GetNetDevice(Recviif);

  l3->Receive(Recvdevice, TruePacket, Ipv4L3Protocol::PROT_NUMBER, TrueSource, TrueDst, NetDevice::PacketType (0));
  NS_LOG_DEBUG("True Local Delivery");

}


Ptr<Socket>
RoutingProtocol::FindSocketWithInterfaceAddress (Ipv4InterfaceAddress addr ) const
{
  NS_LOG_FUNCTION (this << addr);
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
         m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
    {
      Ptr<Socket> socket = j->first;
      Ipv4InterfaceAddress iface = j->second;
      if (iface == addr)
        {
          return socket;
        }
    }
  Ptr<Socket> socket;
  return socket;
}

Ptr<Socket>
RoutingProtocol::FindSubnetBroadcastSocketWithInterfaceAddress (Ipv4InterfaceAddress addr ) const
{
  NS_LOG_FUNCTION (this << addr);
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
         m_socketSubnetBroadcastAddresses.begin (); j != m_socketSubnetBroadcastAddresses.end (); ++j)
    {
      Ptr<Socket> socket = j->first;
      Ipv4InterfaceAddress iface = j->second;
      if (iface == addr)
        {
          return socket;
        }
    }
  Ptr<Socket> socket;
  return socket;
}


bool
RoutingProtocol::IsMyOwnAddress (Ipv4Address src)
{
  NS_LOG_FUNCTION (this << src);
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
         m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
    {
      Ipv4InterfaceAddress iface = j->second;
      if (src == iface.GetLocal ())
        {
          return true;
        }
    }
  return false;
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
  Ipv4RoutingProtocol::DoInitialize ();
}





}
}
//------------------------------------------Over----------------------------------------

