/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 * Author: Tim Schubert <ns-3-leo@timschubert.net>
 */

#include <fstream>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/leo-module.h"
#include "math.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("LeoCircularOrbitTracingExample1");

std::map<Ptr<Node>, Ptr<Socket> > m_Uansockets; //!< send and receive sockets
std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_UanSktAddr;



void
SendTo (Ptr<Socket> socket, Ptr<Packet> packet, Ipv4Address destination)
{
  socket->SendTo (packet, 0, InetSocketAddress (destination, 12345));
}


void
NodeSendPacket(NodeContainer nodec, Ipv4InterfaceContainer ipv4c)
{
    Ptr<Packet> packet = Create<Packet>(128);
    // NS_LOG_DEBUG("Source Send Packet sequence number is "<< m_seqnum);


    // Ptr<Node> srcsend = UanSrcNode.Get(m_uniformRandomVariable->GetInteger (0, 15));
    // Ptr<Socket> srcsocket =  m_Uansockets[srcsend];
    // Ipv4Address destAddr = uan_inter.GetAddress(m_uniformRandomVariable->GetInteger (0, 102),0);
    // SendTo(srcsocket, packet, destAddr);

    Ptr<Node> srcsend = nodec.Get(0);
    Ptr<Socket> srcsocket =  m_Uansockets[srcsend];
    Ipv4Address destAddr = ipv4c.GetAddress(0,0);
    NS_LOG_DEBUG("Packet "<<packet->GetUid()<<" has been sent from "<< m_UanSktAddr[srcsocket].GetLocal() << 
              " to " << destAddr);
    SendTo(srcsocket, packet, destAddr); 



//	erv = CreateObjectWithAttributes<ExponentialRandomVariable>("Mean",DoubleValue(1/m_lambda));
//	Simulator::Schedule (Seconds(erv->GetValue()), &AecnExample::NodeSendPacket, this);
    Time nextSend = Seconds(10);
    // Time nextSend = Seconds(1000);
    Simulator::Schedule(nextSend, &NodeSendPacket,nodec,ipv4c);
    NS_LOG_DEBUG("next Packet will be sent after " << nextSend.GetSeconds() << "s.");

}

void
ReceivePacket (Ptr<Socket> socket)
{
    Ptr<Packet> packet = socket->Recv ();
    NS_LOG_DEBUG("Packet "<<packet->GetUid()<<" has been received in " << Simulator::Now().GetMilliSeconds());
}

void CourseChange (std::string context, Ptr<const MobilityModel> position)
{
  // if(int(Simulator::Now().GetSeconds())%100 ==0)
  // {
    Vector pos = position->GetPosition ();
    Ptr<const Node> node = position->GetObject<Node> ();
    Vector Psource = pos;
    // std::cout << Simulator::Now () << " " << node->GetId () << " " << pos.x << " " << pos.y << " " << pos.z << " " << position->GetVelocity ().GetLength() << std::endl;
    double LatSource = atan(Psource.z/(sqrt(Psource.x*Psource.x + Psource.y*Psource.y)))/(M_PI / 180);
    double LonSource = atan(Psource.y / Psource.x)/(M_PI / 180);
    if(Psource.x<0 && Psource.y>0) LonSource+=180;
    if(Psource.x<0 && Psource.y<0) LonSource-=180;
    NS_LOG_DEBUG("time:" << Simulator::Now().GetSeconds() << " Id: " << node->GetId() << " y: " << position->GetPosition().y << " Lat: " << LatSource << " Lon " << LonSource );
  // }



}

int main(int argc, char *argv[])
{
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_satelliteSktAddr;
	std::map<Ptr<Node>, Ptr<Socket> > m_satellitesockets; //!< send and receive sockets

  CommandLine cmd;
  std::string orbitFile;
  std::string traceFile;
  std::string duration = "6557s"; //一个周期=6557s--1200km
  LogComponentEnable("LeoCircularOrbitTracingExample1",LOG_ALL);
  // LogComponentEnable( "MockChannel",LOG_ALL);
  // LogComponentEnable( "LeoPropagationLossModel",LOG_ALL);

  LogComponentEnableAll(LOG_PREFIX_TIME);
	LogComponentEnableAll(LOG_PREFIX_NODE);
	// LogComponentEnableAll(LOG_PREFIX_FUNC);
	// LogComponentEnableAll(LOG_PREFIX_LEVEL);

  cmd.AddValue("orbitFile", "CSV file with orbit parameters", orbitFile);
  cmd.AddValue("traceFile", "CSV file to store mobility trace in", traceFile);
  cmd.AddValue("precision", "ns3::LeoCircularOrbitMobilityModel::Precision");
  cmd.AddValue("duration", "Duration of the simulation in seconds", duration);
  cmd.Parse (argc, argv);


  NS_LOG_INFO("\nThe Gnd Node");
  double lat_o = 20;
  double lon_o = 119;

  NodeContainer uanNode;
  LeoGndNodeHelper ground;
  LeoLatLong station(lat_o,lon_o);
  ground.Install(uanNode,station,0);
  for(uint32_t i=0; i<uanNode.GetN(); i++)
  {
    Vector Psource = uanNode.Get(i)->GetObject<MobilityModel>()->GetPosition();
    double LatSource = atan(Psource.z/(sqrt(Psource.x*Psource.x + Psource.y*Psource.y)))/(M_PI / 180);
    double LonSource = atan(Psource.y / Psource.x)/(M_PI / 180);
    if(Psource.x<0 && Psource.y>0) LonSource+=180;
    if(Psource.x<0 && Psource.y<0) LonSource-=180;
    NS_LOG_DEBUG("uanNode Id: " << i);
    NS_LOG_DEBUG("the altitude: " << Psource.GetLength() - LEO_EARTH_RAD);
    NS_LOG_DEBUG("LatSource: " << LatSource << " LonSource " << LonSource );
    NS_LOG_DEBUG("Position: " << Psource);
    // std::cout << satellites.Get(i)->GetObject<MobilityModel>()->GetPosition() << std::endl;
  }

  // LeoLatLong source1 (23, 123);
  // LeoLatLong source2 (0, 0);
  // LeoGndNodeHelper ground;
  // NodeContainer users = ground.Install (source1,source2);
  // std::cout << users.Get(0)->GetObject<MobilityModel>()->GetPosition() << std::endl;
  // std::cout << users.Get(1)->GetObject<MobilityModel>()->GetPosition() << std::endl;

  // NS_LOG_INFO("calculate the lon and lat");
  // Vector Psource1 = users.Get(0)->GetObject<MobilityModel>()->GetPosition();
  // Vector Psource2 = users.Get(1)->GetObject<MobilityModel>()->GetPosition();
  // double LatSource1 = atan(Psource1.z/(sqrt(Psource1.x*Psource1.x + Psource1.y*Psource1.y)))/(M_PI / 90);
  // double LatSource2 = atan(Psource2.z/(sqrt(Psource2.x*Psource2.x + Psource2.y*Psource2.y)))/(M_PI / 90);
  // double LonSource1 = atan(Psource1.y / Psource1.x)/(M_PI / 180);
  // double LonSource2 = atan(Psource2.y / Psource2.x)/(M_PI / 180);
  // if(Psource1.x<0 && Psource1.y>0) LonSource1+=180;
  // if(Psource1.x<0 && Psource1.y<0) LonSource1-=180;
  // NS_LOG_DEBUG("LatSource1: " << LatSource1 << " LonSource1 " << LonSource1 );
  // NS_LOG_DEBUG("LatSource2: " << LatSource2 << " LonSource2 " << LonSource2 );


  NS_LOG_INFO("\nThe Satellite Node");
  NodeContainer satellites;
  LeoOrbitNodeHelper orbit;
  //代码已搞定，此处可以均匀生成三颗卫星，然后卫星按一个周期=6557s的规律进行旋转，默认地面站是不移动的
  satellites = orbit.Install ( LeoOrbit (1200, 20, 1, 3));
  NS_LOG_DEBUG("初始位置安装完成");
  
  //高度1200km
  //北纬10
  //它是在install函数里面给每个卫星分配经度的
  for(uint32_t i=0; i<satellites.GetN(); i++)
  {
    Vector Psource = satellites.Get(i)->GetObject<MobilityModel>()->GetPosition();
    double LatSource = atan(Psource.z/(sqrt(Psource.x*Psource.x + Psource.y*Psource.y)))/(M_PI / 180);
    double LonSource = atan(Psource.y / Psource.x)/(M_PI / 180);
    if(Psource.x<0 && Psource.y>0) LonSource+=180;
    if(Psource.x<0 && Psource.y<0) LonSource-=180;
    NS_LOG_DEBUG("Satellite Node Id: " << i);
    NS_LOG_DEBUG("the altitude: " << Psource.GetLength() - LEO_EARTH_RAD);
    NS_LOG_DEBUG("LatSource: " << LatSource << " LonSource " << LonSource );
    NS_LOG_DEBUG("Position: " << satellites.Get(i)->GetObject<MobilityModel>()->GetPosition());
    // std::cout << satellites.Get(i)->GetObject<MobilityModel>()->GetPosition() << std::endl;
  }




  NS_LOG_DEBUG("-----------Initializing Saltellite Device-----------");
  LeoChannelHelper utCh;
  NetDeviceContainer satellite_device = utCh.Install (satellites, uanNode);

  InternetStackHelper stack;
	stack.Install (uanNode);
	stack.Install (satellites);


	Ipv4AddressHelper address;
	address.SetBase("10.1.1.0", "255.255.255.0");
	NS_LOG_DEBUG ("Assign Uan IP Addresses.");
	Ipv4InterfaceContainer sateInf = address.Assign(satellite_device);

  Config::Connect ("/NodeList/1/$ns3::MobilityModel/CourseChange",
                   MakeCallback (&CourseChange));
  std::cout << "Time,Satellite,x,y,z,Speed" << std::endl;

  // Vector Psource = satellites.Get(0)->GetObject<MobilityModel>()->GetPosition();
  // double LatSource = atan(Psource.z/(sqrt(Psource.x*Psource.x + Psource.y*Psource.y)))/(M_PI / 90);
  // double LonSource = atan(Psource.y / Psource.x)/(M_PI / 180);
  // if(Psource.x<0 && Psource.y>0) LonSource+=180;
  // if(Psource.x<0 && Psource.y<0) LonSource-=180;
  // NS_LOG_DEBUG("time:" << Simulator::Now().GetSeconds() << " LatSource: " << LatSource << " LonSource " << LonSource );

	NS_LOG_DEBUG ("Assign Socket.");
  for(uint32_t i=0; i<satellite_device.GetN(); i++)
  {
      Ptr<NetDevice> dev = satellite_device.Get (i);
      Ptr<Node> uannode = dev->GetNode ();
      Ptr<Ipv4> ipv4 = uannode->GetObject<Ipv4> ();
      int32_t interfaceId = ipv4->GetInterfaceForDevice (dev);
      if (interfaceId == -1)
      {
      interfaceId = ipv4->AddInterface (dev);
      }
      Ipv4InterfaceAddress iface = ipv4->GetAddress(interfaceId,0);
      Ipv4Address ip_addr = iface.GetLocal();
      if (iface.GetLocal () == Ipv4Address ("127.0.0.1"))
      {
          return -1;
      }
      Ptr<Socket> cur_socket = Socket::CreateSocket(uannode,
                              UdpSocketFactory::GetTypeId ());
      NS_ASSERT (cur_socket != 0);

      cur_socket->SetRecvCallback (MakeCallback (&ReceivePacket));
      cur_socket->BindToNetDevice (dev);
      cur_socket->Bind (InetSocketAddress (ip_addr, 12345));
      cur_socket->SetAllowBroadcast (true);
      //cur_socket->SetIpRecvTtl (true);
      m_UanSktAddr.insert(std::make_pair(cur_socket,iface));
      m_Uansockets.insert(std::make_pair(uannode,cur_socket));
      NS_LOG_DEBUG("Uan/Satellite Node Uan ID =" << uannode->GetId() << " Socket =" <<cur_socket << " Ipv4Address " << ip_addr);
  }


	NS_LOG_DEBUG ("Start Send.");
  NodeSendPacket(uanNode,sateInf);
  

  Simulator::Stop (Time (duration));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}


