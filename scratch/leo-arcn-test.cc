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

#include <iostream>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/leo-module.h"
#include "ns3/network-module.h"
#include "ns3/aodv-module.h"
#include "ns3/udp-server.h"
#include "ns3/wifi-helper.h"
#include "ns3/yans-wifi-helper.h"
// #include "ns3/epidemic-routing-module.h"

using namespace ns3;

// static void
// EchoTxRx (std::string context, const Ptr< const Packet > packet, const TcpHeader &header, const Ptr< const TcpSocketBase > socket)
// {
//   std::cout << Simulator::Now () << ":" << context << ":" << packet->GetUid() << ":" << socket->GetNode () << ":" << header.GetSequenceNumber () << std::endl;
// }

NS_LOG_COMPONENT_DEFINE ("LeoArcn");

int main (int argc, char *argv[])
{

  CommandLine cmd;
  // LogComponentEnable("MockNetDevice",LOG_LEVEL_ALL);
  // LogComponentEnable("MockChannel",LOG_LEVEL_ALL);
  LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
  LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);
	LogComponentEnableAll(LOG_PREFIX_TIME);
	LogComponentEnableAll(LOG_PREFIX_NODE);
	LogComponentEnableAll(LOG_PREFIX_FUNC);
	LogComponentEnableAll(LOG_PREFIX_LEVEL);

  cmd.Parse (argc, argv);


  //---------------------------Create Nodes--------------------------------------
  NodeContainer BuoyNodes;
  BuoyNodes.Create(2);
  NodeContainer Satellites;
  Satellites.Create(1);


	MobilityHelper mobility;
	Ptr<ListPositionAllocator> nodesPositionAlloc = CreateObject<ListPositionAllocator> ();

  nodesPositionAlloc->Add (Vector(-50000, 0, 0));
  nodesPositionAlloc->Add (Vector(50000, 0, 0));
  // nodesPositionAlloc->Add (Vector(0, -50000, 0));
  // nodesPositionAlloc->Add (Vector(0, 50000, 0));
  nodesPositionAlloc->Add (Vector(0, 0, 2000000));

  mobility.SetPositionAllocator (nodesPositionAlloc);
  mobility.Install (BuoyNodes);
  mobility.Install (Satellites);


  //Install Buoy_wifi_Devices
  NS_LOG_DEBUG("-----------Initializing Wireless-----------");
  WifiHelper wifiHelper;
  wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211a);
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper wifiPhy;
  wifiPhy = YansWifiPhyHelper::Default ();
  wifiPhy.Set ("RxGain", DoubleValue (0) );
  wifiPhy.Set ("TxGain", DoubleValue (1)); //TxGain = 1dB
  wifiPhy.Set("TxPowerStart", DoubleValue(82));
  wifiPhy.Set("TxPowerEnd", DoubleValue(82));  //TxPower = m_txPowerRadio = 82dbm
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);

  wifiPhy.SetChannel (wifiChannel.Create());
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");
  wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                              "DataMode",StringValue ("OfdmRate6Mbps"),
                              "ControlMode",StringValue ("OfdmRate6Mbps"));

  NetDeviceContainer Buoy_wifiDevices = wifiHelper.Install (wifiPhy, wifiMac, BuoyNodes);
  // std::cout << Buoy_wifiDevices.Get(0)->GetAddress() << std::endl;


  //Install SatellitesDevices
  LeoChannelHelper utCh;
  NetDeviceContainer SatellitesDevices = utCh.Install (Satellites, BuoyNodes);
  // std::cout << Satellites.Get(0)->GetDevice(0)->GetAddress() << std::endl;
  // std::cout << BuoyNodes.Get(0)->GetDevice(0)->GetAddress()<< std::endl;
  // std::cout << BuoyNodes.Get(0)->GetDevice(1)->GetAddress()<< std::endl;


  InternetStackHelper stack;

  // AodvHelper aodv;
  // aodv.Set ("EnableHello", BooleanValue (false));
  // //aodv.Set ("HelloInterval", TimeValue (Seconds (10)));
  // if (ttlThresh != 0)
  //   {
  //   aodv.Set ("TtlThreshold", UintegerValue (ttlThresh));
  //   aodv.Set ("NetDiameter", UintegerValue (2*ttlThresh));
  //   }
  // stack.SetRoutingHelper (aodv);

  
  stack.Install (Satellites);
  stack.Install (BuoyNodes);
  Ipv4AddressHelper ipv4;

  ipv4.SetBase ("10.1.2.0", "255.255.255.0");
  ipv4.Assign(Buoy_wifiDevices);

  ipv4.SetBase ("10.1.3.0", "255.255.255.0");
  ipv4.Assign (SatellitesDevices);
  //sate ---10.1.3.1    //BuoyNodes----10.1.3.2~10.1.3.5

//
//   if (islEnabled)
//     {
//       std::cerr << "ISL enabled" << std::endl;
//       IslHelper islCh;
//       NetDeviceContainer islNet = islCh.Install (satellites);
//       ipv4.SetBase ("10.2.0.0", "255.255.0.0");
//       ipv4.Assign (islNet);
//     }


  UdpEchoServerHelper echoServer (9);

  ApplicationContainer serverApps = echoServer.Install (BuoyNodes.Get(1));

  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (100.0));

  UdpEchoClientHelper echoClient (Ipv4Address("10.1.3.255"), 9);
  echoClient.SetAttribute ("MaxPackets", UintegerValue (1));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (10)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (1024));

  ApplicationContainer clientApps = 
    echoClient.Install (BuoyNodes.Get(0));
  clientApps.Start (Seconds (2.0));
  clientApps.Stop (Seconds (10.0));

  // std::cout << Satellites.Get (0)->GetObject<Ipv4L3Protocol>()->GetAddress(1,0) << std::endl;
  NS_LOG_UNCOND ("Run Simulation.");
  Simulator::Stop (Seconds (20));
  Simulator::Run ();
  Simulator::Destroy ();
  NS_LOG_UNCOND ("Done.");

  // Ptr<PacketSink> sink1 = DynamicCast<PacketSink> (sinkApps.Get (0));
  // std::cout << users.Get (0)->GetId () << ":" << users.Get (1)->GetId () << ": " << sink1->GetTotalRx () << std::endl;

  // out.close ();
  // std::cout.rdbuf(coutbuf);

  return 0;
}
