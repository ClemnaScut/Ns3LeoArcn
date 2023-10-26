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
 * This is an example script for AODV manet routing protocol.
 *
 * Authors: Pavel Boyko <boyko@iitp.ru>
 */

//#include "ns3/aodv-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mobility-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/v4ping-helper.h"
#include "ns3/energy-source-container.h"
#include "ns3/energy-module.h"
#include "ns3/uan-helper.h"
#include "ns3/uan-channel.h"
#include "ns3/uan-module.h"
#include "ns3/netanim-module.h"
#include "ns3/acoustic-modem-energy-model-helper.h"
#include "ns3/node-container.h"

#include "ns3/stats-module.h"
#include "ns3/applications-module.h"
#include "ns3/aodv-module.h"
#include "ns3/tag.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/netanim-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/ptr.h"
#include "ns3/timer.h"
#include "ns3/traffic-control-module.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/acoustic-modem-energy-model-helper.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/header.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/mac8-address.h"
#include "ns3/olsr-helper.h"
#include "ns3/olsr-routing-protocol.h"


#include <iostream>
#include <cmath>

#include <math.h>
#include <fstream>
#include <string>
#include <cassert>
#include <vector>
#include <sstream>
#include <iomanip>
#include <numeric>
#include <algorithm>
#include <map>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("uantest");
/**
 * \ingroup aodv-examples
 * \ingroup examples
 * \brief Test script.
 *
 * This script creates 1-dimensional grid topology and then ping last node from the first one:
 *
 * [10.0.0.1] <-- step --> [10.0.0.2] <-- step --> [10.0.0.3] <-- step --> [10.0.0.4]
 *
 * ping 10.0.0.4
 */
class AodvExample
{
public:
  AodvExample ();
  /**
   * \brief Configure script parameters
   * \param argc is the command line argument count
   * \param argv is the command line arguments
   * \return true on successful configuration
  */
  bool Configure (int argc, char **argv);
  /// Run simulation
  void Run (AodvExample test);
  /**
   * Report results
   * \param os the output stream
   */
  void Report (std::ostream & os);

  void Teardown ();

  struct BuoyParam
{
	NodeContainer Nodes;
	NetDeviceContainer WifiDevice;
	NetDeviceContainer UanDevice;
	DeviceEnergyModelContainer WifiEnergy;
	DeviceEnergyModelContainer UanEnergy;
	Ipv4InterfaceContainer WifiInterfaces;
	// Ipv4InterfaceContainer UanInterfaces;
};
//private:
	// parameters
	uint32_t numUansrc;
	uint32_t numUansink;
	uint32_t numUan;
	uint32_t numsrcgtwbuoy;
	uint32_t numsinkgtwbuoy;

	uint32_t numgtbuoy;
//	uint32_t numBuoy;
	//topology setting
	double m_depth;            //<Src depth.
	double m_boundary;         //<size of bundary in meters.

//	double step;  /// Distance between nodes, meters
	double totalTime;  /// Simulation time, seconds
	bool pcap;  /// Write per-device PCAP traces if true
	bool printRoutes;  /// Print routes if true
	uint32_t m_moduleDataRate;    //UanTxMode
	double m_txPowerAcoustic;    //Phy
	std::string m_uanMacType;        //UanMac
	uint32_t m_cwMin;                //CWmac attributes
	Time m_slotTime;                    //!< Slot time duration.
	uint16_t m_port;                    //socket port
	double m_startTime;


	//result
	double EnergyConsumSrc;
	double EnergyConsumSink;
    double EnergyConsum;
	double EnergyConsumBuoyWifi;
    double EnergyConsumBuoyUan;


	DeviceEnergyModelContainer uan_energy;
	DeviceEnergyModelContainer uan_src_energy;
	DeviceEnergyModelContainer uan_sink_energy;
	DeviceEnergyModelContainer buoy_wifi_energy;
	DeviceEnergyModelContainer buoy_uan_energy;


	/// srcnode used in the example
	NodeContainer uansrcNode;
	/// sinknode used in the example
	NodeContainer uansinkNode;
	/// uannode used in the example
	NodeContainer uanNode;
	/// buoy src gateway node used in the example
	NodeContainer srcgatewaybuoynode;
	/// buoy sink gateway node used in the example
	NodeContainer sinkgatewaybuoynode;
	/// gateway node used in the example
	NodeContainer gatewaybuoynode;

	NetDeviceContainer uansrc_device;
	NetDeviceContainer uansink_device;
	NetDeviceContainer uan_device;
	NetDeviceContainer srcgateway_buoywireless_device;
	NetDeviceContainer srcgateway_buoyuan_device;
	NetDeviceContainer sinkgateway_buoywireless_device;
	NetDeviceContainer sinkgateway_buoyuan_device;

	NetDeviceContainer gateway_buoywireless_device;
	NetDeviceContainer gateway_buoyuan_device;


	Ipv4InterfaceContainer uansrc_inter;
	Ipv4InterfaceContainer uansink_inter;
	Ipv4InterfaceContainer uan_inter;
	Ipv4InterfaceContainer srcgateway_buoywireless_inter;
	Ipv4InterfaceContainer srcgateway_buoyuan_inter;
	Ipv4InterfaceContainer sinkgateway_buoywireless_inter;
	Ipv4InterfaceContainer sinkgateway_buoyuan_inter;
	Ipv4InterfaceContainer buoy_wireless_inter;

	Ipv4InterfaceContainer gateway_buoywireless_inter;
	Ipv4InterfaceContainer gateway_buoyuan_inter;

	Ptr<Node> src;
	NetDeviceContainer src_device;
	uint32_t srcId;
	Ptr<Node> sink;
	NetDeviceContainer sink_device;
	uint32_t sinkId;
	Ptr<Node> dstnode;
	//packet
	uint32_t m_pktSize;        // The size of packets transmitted.
	uint32_t m_pktCount;      // Total number of packets to send
	Time m_pktInterval; // Delay between transmissions
	std::string m_asciitracefile; //!< Name for ascii trace file, default uan-cw-example.asc.
	double m_SinkRecvNum;
	Ptr<Socket> m_socket;  /// The socket we send packets from
	Ipv4Address m_remote;  /// Remote address,destination address

	double m_offset;
	double m_Prss;
	double m_txPowerRadio;
	double m_step;
	std::string m_wifiPhyMode;

	Ptr<ListPositionAllocator> pos_uan = CreateObject<ListPositionAllocator> ();
	Ptr<ListPositionAllocator> pos_buoy = CreateObject<ListPositionAllocator> ();

	std::map<Ptr<Node>, Ptr<Socket> > m_sockets; //!< send and receive sockets
	std::map< Ptr<Socket>, Ipv4Address > m_uanSktAddr;
	std::map< Ptr<Socket>, Ptr<NetDevice> > m_uanSockets;
	std::map< Ptr<NetDevice>, Ptr<Socket> > m_uanDevSkt;
	std::map< uint32_t, Ipv4Address > m_uanDevID;
	std::map< Ptr<NetDevice>, std::vector <uint64_t> > m_uanPktID;

	std::map< Ptr<Socket>, Ipv4Address > m_buoysrcWifiSktAddr;
	std::map< Ptr<Socket>, Ipv4Address > m_buoysrcUanSktAddr;
	std::map< Ptr<Socket>, Ipv4Address > m_buoysinkWifiSktAddr;
	std::map< Ptr<Socket>, Ipv4Address > m_buoysinkUanSktAddr;
	std::map< Ptr<Socket>, Ipv4Address > m_buoywirelessSktAddr;

	std::map< Ptr<Socket>, Ptr<NetDevice> > m_buoysrcWifiSockets;
	std::map< Ptr<Socket>, Ptr<NetDevice> > m_buoysrcUanSockets;
	std::map< Ptr<Socket>, Ptr<NetDevice> > m_buoysinkWifiSockets;
	std::map< Ptr<Socket>, Ptr<NetDevice> > m_buoysinkUanSockets;
	std::map< Ptr<Socket>, Ptr<NetDevice> > m_buoywirelessSockets;

	std::map< Ptr<NetDevice>, Ptr<Socket> > m_buoysrcWifiDevSkt;
	std::map< Ptr<NetDevice>, Ptr<Socket> > m_buoysrcUanDevSkt;
	std::map< Ptr<NetDevice>, Ptr<Socket> > m_buoysinkWifiDevSkt;
	std::map< Ptr<NetDevice>, Ptr<Socket> > m_buoysinkUanDevSkt;
	std::map< Ptr<NetDevice>, Ptr<Socket> > m_buoywirelessDevSkt;

	std::map< uint32_t, Ipv4Address > m_buoysrcWifiID;
	std::map< uint32_t, Ipv4Address > m_buoysrcUanID;
	std::map< uint32_t, Ipv4Address > m_buoysinkWifiID;
	std::map< uint32_t, Ipv4Address > m_buoysinkUanID;
	std::map< uint32_t, Ipv4Address > m_buoywirelessID;

	std::map< Ptr<NetDevice>, std::vector <uint64_t> > m_buoysrcWifiPktID;
	std::map< Ptr<NetDevice>, std::vector <uint64_t> > m_buoysrcUanPktID;
	std::map< Ptr<NetDevice>, std::vector <uint64_t> > m_buoysinkWifiPktID;
	std::map< Ptr<NetDevice>, std::vector <uint64_t> > m_buoysinkUanPktID;
	std::map< Ptr<NetDevice>, std::vector <uint64_t> > m_buoywirelessPktID;

private:


  Ptr<Socket> SetupSocket(Ipv4Address addr, Ptr<Node> node, Ptr<NetDevice> dev);
  /// Create the nodes
  void CreateNodes ();
  /// Create the devices
  void CreateDevices (UanHelper uanHelper,AodvExample test,YansWifiPhyHelper wifiPhy);
  /// Create the network
  void InstallInternetStack ();
  /// Create the simulation applications
  void InstallApplications ();

  void UanNodeSendPacket();

  void SendSinglePacket (Ptr<Node> node, Ptr<Packet> pkt, Ipv4Address dst);

  void ReceivePacket (Ptr<Socket> socket);
  /**
   * Print the received packet
   * \param socket The receiving socket
   */
  void PrintReceivedPacket (Ptr<Socket> socket);

  void SinkRecvCB ();
//  std::map<Ptr<Node>, Ptr<Socket> > m_sockets; //!< send and receive sockets

  BuoyParam InstallDevices(NodeContainer nodes, UanHelper &uan, Ptr<UanChannel> uanChannel, WifiHelper &wifi, YansWifiPhyHelper &wifiPhy, WifiMacHelper &wifiMac, YansWifiChannelHelper &wifiChannel);

  void SetPosition();

  void Ipv4L3Tx0 (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface);

  void Ipv4L3Rx0 (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface);

  void Ipv4L3SinkGTTx (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface);

  void Ipv4L3SinkRx (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface);

  void Ipv4L3Tx1 (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface);

  void Ipv4L3Rx1 (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface);

  void Ipv4L3Tx2 (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface);

  void Ipv4L3Rx2 (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface);

  void Ipv4L3Tx3 (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface);

  void Ipv4L3Rx3 (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface);

  void Calculate();
//  void UanDevTx (Ptr<const Packet> packet);
//
//  void UanDevRx (Ptr<const Packet> packet);
};

int main (int argc, char **argv)
{
  AodvExample test;
//  LogComponentEnableAll(LOG_PREFIX_ALL);
	LogComponentEnable("uantest", LOG_LEVEL_INFO);
	LogComponentEnableAll(LOG_PREFIX_TIME);
	LogComponentEnableAll(LOG_PREFIX_NODE);
	LogComponentEnableAll(LOG_PREFIX_FUNC);
	LogComponentEnableAll(LOG_PREFIX_LEVEL);

  if (!test.Configure (argc, argv))
    NS_FATAL_ERROR ("Configuration failed. Aborted.");
//	 LogComponentEnable("UanMacCw", LOG_LEVEL_ALL);
//	 LogComponentEnable("UanPhyGen", LOG_LEVEL_ALL);
//	 LogComponentEnable("UanNetDevice", LOG_LEVEL_ALL);
//	 LogComponentEnable("UanChannel", LOG_LEVEL_ALL);
	 LogComponentEnable("AodvRoutingProtocol", LOG_LEVEL_ALL);
//	 LogComponentEnable("V4Ping", LOG_LEVEL_ALL);
//	 LogComponentEnable("WifiPhy", LOG_LEVEL_ALL);
//	 LogComponentEnable("YansWifiChannel", LOG_LEVEL_ALL);
//
//	 LogComponentEnable("ArpL3Protocol", LOG_LEVEL_ALL);
//	 LogComponentEnable("TrafficControlLayer", LOG_LEVEL_ALL);

////     LogComponentEnable("AodvRoutingProtocol", LOG_LEVEL_ALL);
//	LogComponentEnable("UanNetDevice", LOG_LEVEL_ALL);
////	LogComponentEnable("UanMacCw", LOG_LEVEL_ALL);
//	LogComponentEnable("UanTxMode", LOG_LEVEL_ALL);
////	LogComponentEnable("UanPhyGen", LOG_LEVEL_ALL);
//	LogComponentEnable("UanChannel", LOG_LEVEL_ALL);
////	LogComponentEnable("UanMacAloha", LOG_LEVEL_ALL);
//
//	// LogComponentEnable("WifiHelper", LOG_LEVEL_ALL);
//	 LogComponentEnable("AdhocWifiMac", LOG_LEVEL_ALL);
//	// LogComponentEnable("YansWifiPhy", LOG_LEVEL_ALL);
//	LogComponentEnable("WifiPhy", LOG_LEVEL_ALL);
//	// LogComponentEnable("WifiNetDevice", LOG_LEVEL_ALL);
//	LogComponentEnable("YansWifiChannel", LOG_LEVEL_ALL);

//	LogComponentEnable("AodvRoutingProtocol", LOG_LEVEL_ALL);
//  LogComponentEnable("AodvNeighbors", LOG_LEVEL_ALL);
//  	LogComponentEnable("OlsrRoutingProtocol", LOG_LEVEL_ALL);

  test.Run (test);
  test.Report (std::cout);
  test.Teardown ();
  return 0;
}

//-----------------------------------------------------------------------------
AodvExample::AodvExample () :
  numUansrc (1),
  numUansink(1),
  numUan(13),
  numgtbuoy(4),
//  numsrcgtwbuoy(2),
//  numsinkgtwbuoy(2),
//  numBuoy(1),
  m_depth(-500),
  m_boundary(1000),
//  step (100),
  totalTime (6000),
  pcap (true),
  printRoutes (true),
  m_moduleDataRate(4800),
  m_txPowerAcoustic(137),
//  m_txPowerAcoustic(135),
  m_uanMacType("ns3::UanMacAloha"),
  m_cwMin(4),
  m_slotTime(Seconds (0.1)),
//  m_port(654),
  m_port(9),
  m_startTime(1),
  m_pktSize(100),
  m_pktCount(300),
  m_pktInterval(10),
  m_asciitracefile ("uanbuoytest1.asc"),
  m_SinkRecvNum(0),
  m_socket(0),
  m_remote("10.1.2.2"),
  m_offset(81),
  m_Prss(-80),
  m_txPowerRadio(100),
  m_step(1),
  m_wifiPhyMode("DsssRate1Mbps")
{
}

bool
AodvExample::Configure (int argc, char **argv)
{
  // Enable AODV logs by default. Comment this if too noisy
//	LogComponentEnable("AodvRoutingProtocol", LOG_LEVEL_ALL);

	LogComponentEnableAll(LOG_PREFIX_TIME);
	LogComponentEnableAll(LOG_PREFIX_NODE);
	LogComponentEnableAll(LOG_PREFIX_FUNC);
	LogComponentEnableAll(LOG_PREFIX_LEVEL);
	SeedManager::SetSeed(12345);
	CommandLine cmd;

	cmd.AddValue("pcap", "Write PCAP traces.", pcap);
	cmd.AddValue("printRoutes", "Print routing table dumps.", printRoutes);
//	cmd.AddValue("uansrcNode", "Number of nodes.", uansrcNode);
//	cmd.AddValue("numBuoy", "Number of nodes.", numBuoy);
	cmd.AddValue("time", "Simulation time, s.", totalTime);
//	cmd.AddValue("step", "Grid step, m", step);

	cmd.Parse(argc, argv);
	return true;
}

void
AodvExample::Run (AodvExample test)
{
  //Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (1)); // enable rts cts all the time.
  CreateNodes ();
  UanHelper uanHelper;
  YansWifiPhyHelper wifiPhy;
  CreateDevices (uanHelper,test,wifiPhy);
  InstallInternetStack ();
  InstallApplications ();

  std::string ascii_name ("uanbuoy-aodv.asc");
  std::ofstream ascii (ascii_name.c_str());
  uanHelper.EnableAsciiAll(ascii);

  AsciiTraceHelper asciiwifi;
  Ptr<OutputStreamWrapper> stream = asciiwifi.CreateFileStream ("uanbuoy-aodv.tr");
  wifiPhy.EnableAsciiAll (stream);

  //  UanNodeSendPacket();

  Config::ConnectWithoutContext(
  		"/NodeList/0/$ns3::Ipv4L3Protocol/Tx",
  		MakeCallback(&AodvExample::Ipv4L3Tx0, this));
//
  Config::ConnectWithoutContext(
  		"/NodeList/15/$ns3::Ipv4L3Protocol/Rx",
  		MakeCallback(&AodvExample::Ipv4L3Rx0, this));
//
//  Config::ConnectWithoutContext(
//  		"/NodeList/1/$ns3::Ipv4L3Protocol/Tx",
//  		MakeCallback(&AodvExample::Ipv4L3Tx1, this));
//
//  Config::ConnectWithoutContext(
//  		"/NodeList/1/$ns3::Ipv4L3Protocol/Rx",
//  		MakeCallback(&AodvExample::Ipv4L3Rx1, this));
//
//  Config::ConnectWithoutContext(
//  		"/NodeList/2/$ns3::Ipv4L3Protocol/Tx",
//  		MakeCallback(&AodvExample::Ipv4L3Tx2, this));
//
//  Config::ConnectWithoutContext(
//  		"/NodeList/2/$ns3::Ipv4L3Protocol/Rx",
//  		MakeCallback(&AodvExample::Ipv4L3Rx2, this));
//
//  Config::ConnectWithoutContext(
//  		"/NodeList/3/$ns3::Ipv4L3Protocol/Tx",
//  		MakeCallback(&AodvExample::Ipv4L3Tx3, this));
//
//  Config::ConnectWithoutContext(
//  		"/NodeList/3/$ns3::Ipv4L3Protocol/Rx",
//  		MakeCallback(&AodvExample::Ipv4L3Rx3, this));


  NodeContainer::Iterator m_srcnode = uansrcNode.Begin ();
  (*m_srcnode)->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetArpCache ()->SetWaitReplyTimeout (Seconds (10));

  NodeContainer::Iterator m_gatewaynode = gatewaybuoynode.Begin ();
  (*m_gatewaynode)->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetArpCache ()->SetWaitReplyTimeout (Seconds (10));

//  NodeContainer::Iterator m_srcgatewaynode = srcgatewaybuoynode.Begin ();
//  (*m_srcgatewaynode)->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetArpCache ()->SetWaitReplyTimeout (Seconds (10));
//
//  NodeContainer::Iterator m_sinkgatewaynode = sinkgatewaybuoynode.Begin ();
//  (*m_sinkgatewaynode)->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetArpCache ()->SetWaitReplyTimeout (Seconds (10));

   NodeContainer::Iterator m_sinkNode = uansinkNode.Begin ();
  (*m_sinkNode)->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetArpCache ()->SetWaitReplyTimeout (Seconds (10));


	 AnimationInterface anim ("uanbuoyaodv-animation.xml"); // Mandatory
	 for (uint32_t i = 0; i < uansrcNode.GetN (); ++i)
	 {
	 	anim.UpdateNodeDescription (uansrcNode.Get (i), "UAN"); // Optional
	 	anim.UpdateNodeColor (uansrcNode.Get (i), 0, 255, 0); // Optional
	 }
	 for (uint32_t i = 0; i < uansinkNode.GetN (); ++i)
	 {
	 	anim.UpdateNodeDescription (uansinkNode.Get (i), "UAN"); // Optional
	 	anim.UpdateNodeColor (uansinkNode.Get (i), 0, 255, 0); // Optional
	 }
	 for (uint32_t i = 0; i < uanNode.GetN (); ++i)
	 {
	 	anim.UpdateNodeDescription (uanNode.Get (i), "UAN"); // Optional
	 	anim.UpdateNodeColor (uanNode.Get (i), 0, 255, 0); // Optional
	 }
	 for (uint32_t i = 0; i < gatewaybuoynode.GetN (); ++i)
	 {
	 	anim.UpdateNodeDescription (gatewaybuoynode.Get (i), "UAN"); // Optional
	 	anim.UpdateNodeColor (gatewaybuoynode.Get (i), 0, 255, 0); // Optional
	 }


  std::cout << "Starting simulation for " << totalTime << " s ...\n";
  NS_LOG_UNCOND("Simulation run "<<totalTime<<" seconds....\n");
  Simulator::Schedule(Seconds(totalTime + 1), &AodvExample::Calculate, this);

  Simulator::Stop (Seconds (totalTime+1));
  Simulator::Run ();
  Simulator::Destroy ();
  Teardown();
}

void
AodvExample::Report (std::ostream &)
{
}

void
AodvExample::ReceivePacket (Ptr<Socket> socket)
{
  NS_LOG_INFO ("Received one packet!");
  Ptr<Packet> packet = socket->Recv ();
  SocketIpTosTag tosTag;
  if (packet->RemovePacketTag (tosTag))
    {
      NS_LOG_INFO (" TOS = " << (uint32_t)tosTag.GetTos ());
    }
  SocketIpTtlTag ttlTag;
  if (packet->RemovePacketTag (ttlTag))
    {
      NS_LOG_INFO (" TTL = " << (uint32_t)ttlTag.GetTtl ());
    }
}

void
AodvExample::CreateNodes ()
{

	uansrcNode.Create(numUansrc);
//	srcgatewaybuoynode.Create(numsrcgtwbuoy);
	gatewaybuoynode.Create(numgtbuoy);
//	sinkgatewaybuoynode.Create(numsinkgtwbuoy);
	uansinkNode.Create(numUansink);
	uanNode.Create(numUan);

	// Name uan src nodes
	for (uint32_t i = 0; i < numUansrc; ++i)
	{
		std::ostringstream os;
		os << "uansrcNode-" << i;
		Names::Add(os.str(), uansrcNode.Get(i));
	}
//	// Name uan src Gateway nodes
//	for (uint32_t i = 0; i < numsrcgtwbuoy; ++i)
//	{
//		std::ostringstream os;
//		os << "srcgatewaybuoynode-" << i;
//		Names::Add(os.str(), srcgatewaybuoynode.Get(i));
//	}
//	 Name buoy gateway nodes
	for (uint32_t i = 0; i < numgtbuoy; ++i)
	{
		std::ostringstream os;
		os << "BuoygatewayNode-" << i;
		Names::Add (os.str (), gatewaybuoynode.Get (i));
	}

	// Name uan sink nodes
	for (uint32_t i = 0; i < numUansink; ++i)
	{
		std::ostringstream os;
		os << "uansinkNode-" << i;
		Names::Add(os.str(), uansinkNode.Get(i));
	}

	MobilityHelper mobility;
	Ptr<ListPositionAllocator> nodesPositionAlloc = CreateObject<ListPositionAllocator> ();
	double interval = 1000;

	  nodesPositionAlloc->Add (Vector (0.0, 0.0, m_depth));
	  nodesPositionAlloc->Add (Vector (interval, 0.0, m_depth));
	  nodesPositionAlloc->Add (Vector (interval*2, 0.0, m_depth));
	  nodesPositionAlloc->Add (Vector (interval*3, 0.0, m_depth));
	  nodesPositionAlloc->Add (Vector (interval*4, 0.0, m_depth));

	  nodesPositionAlloc->Add (Vector (0.0, interval, m_depth));
	  nodesPositionAlloc->Add (Vector (interval, interval, m_depth));
	  nodesPositionAlloc->Add (Vector (interval*2, interval, m_depth));
	  nodesPositionAlloc->Add (Vector (interval*3, interval, m_depth));
	  nodesPositionAlloc->Add (Vector (interval*4, interval, m_depth));

	  nodesPositionAlloc->Add (Vector (0.0, interval*2, m_depth));
	  nodesPositionAlloc->Add (Vector (interval, interval*2, m_depth));
	  nodesPositionAlloc->Add (Vector (interval*2, interval*2, m_depth));
	  nodesPositionAlloc->Add (Vector (interval*3, interval*2, m_depth));
	  nodesPositionAlloc->Add (Vector (interval*4, interval*2, m_depth));

//	  nodesPositionAlloc->Add (Vector (0.0, 0.0, 0.0));
//	  nodesPositionAlloc->Add (Vector (0.0, 1000.0, 0.0));
//	  nodesPositionAlloc->Add (Vector (1500.0, 0.0, 0.0));
//	  nodesPositionAlloc->Add (Vector (1500.0, 1000.0, 0.0));

	  nodesPositionAlloc->Add (Vector (500.0, 400.0, 0.0));
	  nodesPositionAlloc->Add (Vector (500.0, 1600.0, 0.0));
	  nodesPositionAlloc->Add (Vector (3500.0, 400.0, 0.0));
	  nodesPositionAlloc->Add (Vector (3500.0, 1600.0, 0.0));

//	  nodesPositionAlloc->Add (Vector (250.0, 250.0, 0.0));
//	  nodesPositionAlloc->Add (Vector (250.0, 750.0, 0.0));
//	  nodesPositionAlloc->Add (Vector (1250.0, 250.0, 0.0));
//	  nodesPositionAlloc->Add (Vector (1250.0, 750.0, 0.0));


	  mobility.SetPositionAllocator (nodesPositionAlloc);
	  mobility.Install (uansrcNode);
	  mobility.Install (uanNode);
	  mobility.Install (uansinkNode);
	  mobility.Install (gatewaybuoynode);

//	  mobility.Install (srcgatewaybuoynode);
//	  mobility.Install (sinkgatewaybuoynode);


}

void
AodvExample::CreateDevices (UanHelper uanHelper,AodvExample test,YansWifiPhyHelper wifiPhy)
{
	NS_LOG_DEBUG("-----------Initializing Uan-----------");
	std::string perModel = "ns3::UanPhyPerGenDefault";
	std::string sinrModel = "ns3::UanPhyCalcSinrDefault";
	ObjectFactory obf;
	obf.SetTypeId(perModel);
	Ptr<UanPhyPer> phyPer = obf.Create<UanPhyPer>();
	obf.SetTypeId(sinrModel);
	Ptr<UanPhyCalcSinr> phySinr = obf.Create<UanPhyCalcSinr>();
	UanTxMode uanMode;
//	uanMode = UanTxModeFactory::CreateMode(UanTxMode::FSK, 4800,
//			50000, 25000, 20000, 2, "Default mode");
	uanMode = UanTxModeFactory::CreateMode(UanTxMode::FSK, 4800,
			10000, 25000, 20000, 2, "Default mode");
	UanModesList myModes;
	myModes.AppendMode(uanMode);

	uanHelper.SetPhy("ns3::UanPhyGen", "PerModel", PointerValue(phyPer),
			"SinrModel", PointerValue(phySinr), "SupportedModes",
			UanModesListValue(myModes), "TxPower",
			DoubleValue(m_txPowerAcoustic));
	uanHelper.SetMac(m_uanMacType);

	Ptr<UanPropModelThorp> prop = CreateObject<UanPropModelThorp>();
	Ptr<UanNoiseModelDefault> noise = CreateObject<UanNoiseModelDefault> ();
	Ptr<UanChannel> uanChannel = CreateObject<UanChannel>();
	uanChannel->SetPropagationModel(prop);
	uanChannel->SetNoiseModel (noise);

	uansrc_device = uanHelper.Install(uansrcNode, uanChannel);
	uan_device = uanHelper.Install(uanNode, uanChannel);
	uansink_device = uanHelper.Install(uansinkNode, uanChannel);
	gateway_buoyuan_device = uanHelper.Install(gatewaybuoynode, uanChannel);

//	srcgateway_buoyuan_device = uanHelper.Install(srcgatewaybuoynode, uanChannel);
//	sinkgateway_buoyuan_device = uanHelper.Install(sinkgatewaybuoynode, uanChannel);
	NS_LOG_DEBUG("-----------Initializing Wireless-----------");
	WifiHelper wifiHelper;
	wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211b);
	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
//	wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
//	wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel");
//	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
	wifiPhy = YansWifiPhyHelper::Default ();
	wifiPhy.Set ("RxGain", DoubleValue (-10) );
	wifiPhy.Set ("TxGain", DoubleValue (m_offset + m_Prss));
	wifiPhy.Set("TxPowerStart", DoubleValue(m_txPowerRadio));
	wifiPhy.Set("TxPowerEnd", DoubleValue(m_txPowerRadio));
	wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);

	wifiPhy.SetChannel (wifiChannel.Create());
	WifiMacHelper wifiMac;
	wifiMac.SetType ("ns3::AdhocWifiMac");
  	wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (m_wifiPhyMode),
                                "ControlMode",StringValue (m_wifiPhyMode));

  	NS_LOG_DEBUG("-----------Creat buoy wireless device-----------");
  	gateway_buoywireless_device = wifiHelper.Install (wifiPhy, wifiMac, gatewaybuoynode);
//	srcgateway_buoywireless_device = wifiHelper.Install (wifiPhy, wifiMac, srcgatewaybuoynode);
//	sinkgateway_buoywireless_device = wifiHelper.Install (wifiPhy, wifiMac, sinkgatewaybuoynode);
	NS_LOG_DEBUG("-----Finish Buoy wireless device setting.------");

	NS_LOG_DEBUG("-----------Initializing Energy-----------");
	BasicEnergySourceHelper energySourceHelper;
	energySourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (900000));
	EnergySourceContainer uanSources = energySourceHelper.Install (uanNode);
	for (NodeContainer::Iterator n = uanNode.Begin (); n != uanNode.End (); ++n)
	{
		uanSources.Add ((*n)->GetObject<EnergySourceContainer> ()->Get (0));
	}
	AcousticModemEnergyModelHelper acousticEnergyHelper;
	uan_energy = acousticEnergyHelper.Install (uan_device, uanSources);

	EnergySourceContainer uansrcSources = energySourceHelper.Install (uansrcNode);
	for (NodeContainer::Iterator n = uansrcNode.Begin (); n != uansrcNode.End (); ++n)
	{
		uansrcSources.Add ((*n)->GetObject<EnergySourceContainer> ()->Get (0));
	}
	uan_src_energy = acousticEnergyHelper.Install (uansrc_device, uansrcSources);

	EnergySourceContainer uansinkSources = energySourceHelper.Install (uansinkNode);
	for (NodeContainer::Iterator n = uansinkNode.Begin (); n != uansinkNode.End (); ++n)
	{
		uansinkSources.Add ((*n)->GetObject<EnergySourceContainer> ()->Get (0));
	}
	uan_sink_energy = acousticEnergyHelper.Install (uansink_device, uansinkSources);


	EnergySourceContainer buoywifiSources = energySourceHelper.Install (gatewaybuoynode);
	NS_LOG_DEBUG("Set Buoy Wifi Energy.");
	WifiRadioEnergyModelHelper radioEnergyHelper;
	// configure radio energy model
	radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));
	// install device model
	buoy_wifi_energy = radioEnergyHelper.Install (gateway_buoywireless_device, buoywifiSources);

	NS_LOG_DEBUG("Set Buoy Uan Energy.");
	EnergySourceContainer buoyuanSources;
	for(NodeContainer::Iterator node = gatewaybuoynode.Begin (); node != gatewaybuoynode.End (); node++)
	{
		buoyuanSources.Add ((*node)->GetObject<EnergySourceContainer> ()->Get (0));
	}
	buoy_uan_energy = acousticEnergyHelper.Install (gateway_buoyuan_device, buoyuanSources);



//	EnergySourceContainer buoywifiSources = energySourceHelper.Install (gatewaybuoynode);
//	EnergySourceContainer buoyuanSources = energySourceHelper.Install (gatewaybuoynode);
//	for (NodeContainer::Iterator n = gatewaybuoynode.Begin (); n != gatewaybuoynode.End (); ++n)
//	{
//		buoywifiSources.Add ((*n)->GetObject<EnergySourceContainer> ()->Get (0));
//		buoyuanSources.Add ((*n)->GetObject<EnergySourceContainer> ()->Get (0));
//	}
//	buoy_wifi_energy = acousticEnergyHelper.Install (gateway_buoywireless_device, buoywifiSources);
//	buoy_uan_energy = acousticEnergyHelper.Install (gateway_buoyuan_device, buoyuanSources);
	NS_LOG_DEBUG("Assign IP Addresses.");
}
Ptr<Socket>
AodvExample::SetupSocket(Ipv4Address addr, Ptr<Node> node, Ptr<NetDevice> dev)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> socket = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, m_port);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address::GetBroadcast (), m_port);
  socket->Bind (local);
  socket->BindToNetDevice (dev);
  socket->SetAllowBroadcast (true);
  socket->Connect (remote);

  return socket;
}
void
AodvExample::InstallInternetStack ()
{
	//--------------------------------------------aodv routing--------------------------------------------
	AodvHelper aodv;
	aodv.Set("HelloInterval", TimeValue(Seconds(500)));
//	aodv.Set("TimeoutBuffer", UintegerValue(2));
	aodv.Set("NodeTraversalTime", TimeValue(Seconds(2.5)));
	aodv.Set("NextHopWait", TimeValue(Seconds(2.51)));
	aodv.Set("ActiveRouteTimeout", TimeValue(Seconds(2000)));
	aodv.Set("MyRouteTimeout", TimeValue(Seconds(2000)));
//	aodv.Set("BlackListTimeout", TimeValue(Seconds(5.6)));
	aodv.Set("DeletePeriod", TimeValue(Seconds(500)));
//	aodv.Set("NetDiameter", UintegerValue(35));
	aodv.Set("NetTraversalTime", TimeValue(Seconds(100)));
	aodv.Set("PathDiscoveryTime", TimeValue(Seconds(100)));
//	aodv.Set("MaxQueueLen", UintegerValue(64));
//	aodv.Set("PathDiscoveryTime", TimeValue(Seconds(5.6)));
//	aodv.Set("MaxQueueTime", TimeValue(Seconds(300)));
//	aodv.Set("AllowedHelloLoss", UintegerValue(4));
//	aodv.Set("EnableHello", BooleanValue(0));
//	aodv.Set("TtlStart", UintegerValue(7));
//	aodv.Set("RerrRateLimit", UintegerValue(0));

//	OlsrHelper olsr;
//	olsr.Set("HelloInterval", TimeValue(Seconds(20)));
//	olsr.Set("TcInterval", TimeValue(Seconds(50)));
//	olsr.Set("MidInterval", TimeValue(Seconds(50)));
//	olsr.Set("HnaInterval", TimeValue(Seconds(50)));

	InternetStackHelper stack;
	stack.SetRoutingHelper (aodv);
//	stack.SetRoutingHelper (olsr);
	stack.Install(uansrcNode);
	stack.Install(uanNode);
	stack.Install (uansinkNode);
	stack.Install (gatewaybuoynode);
//	stack.Install (srcgatewaybuoynode);
//	stack.Install (sinkgatewaybuoynode);
//	stack.Install (buoyNodes);

	Ipv4AddressHelper address;
	address.SetBase("10.1.1.0", "255.255.255.0");
	NS_LOG_DEBUG ("Assign Buoy uan IP Addresses.");
	uansrc_inter = address.Assign(uansrc_device);
	uan_inter = address.Assign(uan_device);
	uansink_inter = address.Assign(uansink_device);
//	srcgateway_buoyuan_inter = address.Assign(srcgateway_buoyuan_device);
//	sinkgateway_buoyuan_inter = address.Assign(sinkgateway_buoyuan_device);
	gateway_buoyuan_inter = address.Assign(gateway_buoyuan_device);

	NS_LOG_DEBUG ("Assign Buoy Buoywifi IP Addresses.");
	address.SetBase("10.1.2.0", "255.255.255.0");
//	srcgateway_buoywireless_inter = address.Assign(srcgateway_buoywireless_device);
//	sinkgateway_buoywireless_inter = address.Assign(sinkgateway_buoywireless_device);
	gateway_buoywireless_inter = address.Assign(gateway_buoywireless_device);

	//change arp cache wait time.
	NodeContainer::Iterator m_uansrcNode = uansrcNode.Begin ();
	m_uansrcNode = uansrcNode.Begin();
	while (m_uansrcNode != uansrcNode.End()) {
		(*m_uansrcNode)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->SetWaitReplyTimeout(
				Seconds(20));
		m_uansrcNode++;
	}

	NodeContainer::Iterator m_uanNode = uanNode.Begin ();
	m_uansrcNode = uanNode.Begin();
	while (m_uanNode != uanNode.End()) {
		(*m_uanNode)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->SetWaitReplyTimeout(
				Seconds(20));
		m_uanNode++;
	}

	NodeContainer::Iterator m_uansinkNode = uansinkNode.Begin();
	m_uansinkNode = uansinkNode.Begin();
	while (m_uansinkNode != uansinkNode.End()) {
		(*m_uansinkNode)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->SetWaitReplyTimeout(
				Seconds(20));
		m_uansinkNode++;
	}

	NodeContainer::Iterator m_gatewaybuoynode = gatewaybuoynode.Begin();
	m_gatewaybuoynode = gatewaybuoynode.Begin();
	while (m_gatewaybuoynode != gatewaybuoynode.End())
	{
		(*m_gatewaybuoynode)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->SetWaitReplyTimeout(
				Seconds(20));
		m_gatewaybuoynode++;
	}

//	NodeContainer::Iterator m_srcgatewaybuoynode = srcgatewaybuoynode.Begin();
//	m_srcgatewaybuoynode = srcgatewaybuoynode.Begin();
//	while (m_srcgatewaybuoynode != srcgatewaybuoynode.End()) {
//		(*m_srcgatewaybuoynode)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->SetWaitReplyTimeout(
//				Seconds(10));
//		m_srcgatewaybuoynode++;
//	}
//
//	NodeContainer::Iterator m_sinkgatewaybuoynode = sinkgatewaybuoynode.Begin();
//	m_sinkgatewaybuoynode = sinkgatewaybuoynode.Begin();
//	while (m_sinkgatewaybuoynode != sinkgatewaybuoynode.End()) {
//		(*m_sinkgatewaybuoynode)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->SetWaitReplyTimeout(
//				Seconds(10));
//		m_sinkgatewaybuoynode++;
//	}

	if (printRoutes)
	{
		Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(
				"uanbuoy-aodv.routes", std::ios::out);
		aodv.PrintRoutingTableAllAt(Seconds(500), routingStream);
	}
}

void
AodvExample::InstallApplications ()
{
//  V4PingHelper ping (uansrc_inter.GetAddress (0));
  V4PingHelper ping ("10.1.1.15");
  ping.SetAttribute ("Verbose", BooleanValue (true));
//  ping.SetAttribute ("Interval", TimeValue (Seconds(totalTime/2)));
  ping.SetAttribute ("Interval", TimeValue (Seconds(5)));
  ping.SetAttribute ("Size", UintegerValue (50));
  ApplicationContainer p = ping.Install (uansrcNode.Get (0));
  p.Start (Seconds (0));
  p.Stop (Seconds (totalTime) - Seconds (0.001));
}

void
AodvExample::UanNodeSendPacket()
{
//	NodeContainer::Iterator node = uanNodes.Begin ();
//	Ptr<UniformRandomVariable> uniformRandomVariable = CreateObject<UniformRandomVariable>();

//	  std::map< Ptr<Node>, Ptr<Socket> >::iterator j = m_sockets.find (2);
//	sink = uansinkNode.Get(0);
	src = uansrcNode.Get(0);
	Ptr<NetDevice> srcuanDevice = src->GetDevice(0);
	Ptr<Socket> srcsocket = m_uanDevSkt[srcuanDevice];
	InetSocketAddress remote = InetSocketAddress ("10.1.3.1", m_port);
	srcsocket->Connect (remote);
//	Ipv4Address dst = sink->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetAddress(0).GetLocal();
	Ipv4Address dst = Ipv4Address ("10.1.3.1");
	NS_LOG_DEBUG("destination ip address is ="<<dst);
    Ptr<Packet> pkt = Create<Packet>(m_pktSize);
//	double time = uniformRandomVariable->GetValue(0, 1);
    SendSinglePacket(src, pkt, dst);
//    Simulator::Schedule(Seconds(0), &AodvExample::SendSinglePacket, this,src, pkt, dst);
//	node++;
	sink = uansinkNode.Get(0);
	Ptr<NetDevice> sinkuanDevice = sink->GetDevice(0);
	Ptr<Socket> sinksocket = m_uanDevSkt[sinkuanDevice];
//	sinksocket->SetRecvCallback(MakeCallback(&AodvExample::PrintReceivedPacket, this));
//	sinksocket->SetRecvCallback (MakeCallback (&AodvExample::ReceivePacket, this));
	Simulator::Schedule(Seconds(m_pktInterval), &AodvExample::UanNodeSendPacket, this);
}

void
AodvExample::SendSinglePacket (Ptr<Node> node, Ptr<Packet> packet, Ipv4Address dst)
{
  NS_LOG_UNCOND ( Simulator::Now ().GetSeconds () << "s" << " packet sent to " << dst );
  InetSocketAddress ipv4_destination = InetSocketAddress (dst, m_port);

  Ptr<NetDevice> uanDevice = node->GetDevice(0);

  m_uanDevSkt[uanDevice]->SendTo (packet, 0, ipv4_destination);
//  m_sockets[node]->SendTo (packet, 0, ipv4_destination);
}

void
AodvExample::Teardown ()
{
  std::map<Ptr<Node>, Ptr<Socket> >::iterator socket;

  for (socket = m_sockets.begin (); socket != m_sockets.end (); socket++)
    {
      socket->second->Close ();
    }
}

void
AodvExample::PrintReceivedPacket (Ptr<Socket> socket)
{
  Address srcAddress;
  while (socket->GetRxAvailable () > 0)
    {
      Ptr<Packet> packet = socket->RecvFrom (srcAddress);
      uint8_t energyReading;
      packet->CopyData (&energyReading, 1);

      if(InetSocketAddress::IsMatchingType (srcAddress))
        {
          NS_LOG_UNCOND ( "Time: " << Simulator::Now ().GetSeconds () << "s" << " | Node: " <<
                          InetSocketAddress::ConvertFrom (srcAddress).GetIpv4 ());
        }
    }
}

AodvExample::BuoyParam
AodvExample::InstallDevices(NodeContainer nodes, UanHelper &uan, Ptr<UanChannel> uanChannel, WifiHelper &wifi, YansWifiPhyHelper &wifiPhy, WifiMacHelper &wifiMac, YansWifiChannelHelper &wifiChannel)
{
	NS_LOG_DEBUG("Create Buoy Nodes");
	YansWifiPhyHelper wifi_phy = wifiPhy;
	wifi_phy.SetChannel (wifiChannel.Create ());
	WifiMacHelper wifi_mac = wifiMac;
	NS_LOG_DEBUG("Set Buoy Uan Device");
	NetDeviceContainer uanDevices = uan.Install (nodes, uanChannel);
	NS_LOG_DEBUG("Set Buoy Wifi Device");
	NetDeviceContainer wifiDevices = wifi.Install (wifi_phy, wifi_mac, nodes);

// 	AodvHelper aodv;
//	InternetStackHelper stack;
//	stack.SetRoutingHelper (aodv); // has effect on the next Install ()
//	stack.Install (nodes);
//	Ipv4AddressHelper address;
//	NS_LOG_DEBUG ("Assign Buoy Wifi IP Addresses.");
//	address.SetBase ("10.1.2.100", "255.255.255.0");
//	Ipv4InterfaceContainer wifiInterfaces;
//	wifiInterfaces= address.Assign (wifiDevices);

	// NS_LOG_DEBUG ("Assign Buoy Uan IP Addresses.");
	// address.SetBase ("10.1.2.0", "255.255.255.0");
	// Ipv4InterfaceContainer uanInterfaces;
	// uanInterfaces= address.Assign (uanDevices);

	/* energy source */
	BasicEnergySourceHelper basicSourceHelper;
	// configure energy source
	basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ",  DoubleValue (100000.0));
	// install source
	EnergySourceContainer wifiSources = basicSourceHelper.Install (nodes);
	/* device energy model */
	NS_LOG_DEBUG("Set Buoy Wifi Energy.");
	WifiRadioEnergyModelHelper radioEnergyHelper;
	// configure radio energy model
	radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));
	// install device model
	DeviceEnergyModelContainer wifiModels = radioEnergyHelper.Install (wifiDevices, wifiSources);

	NS_LOG_DEBUG("Set Buoy Uan Energy.");
	EnergySourceContainer uanSources;
	for(NodeContainer::Iterator node = nodes.Begin (); node != nodes.End (); node++)
	{
		uanSources.Add ((*node)->GetObject<EnergySourceContainer> ()->Get (0));
	}
	AcousticModemEnergyModelHelper acousticEnergyHelper;
	DeviceEnergyModelContainer uanModels = acousticEnergyHelper.Install (uanDevices, uanSources);

    struct BuoyParam tmp;
	tmp.Nodes = nodes;
	tmp.WifiDevice = wifiDevices;
	tmp.UanDevice = uanDevices;
	tmp.WifiEnergy = wifiModels;
	tmp.UanEnergy = uanModels;
//	tmp.WifiInterfaces = wifiInterfaces;
//  tmp.UanInterfaces = uanInterfaces;
	return tmp;
}

void
AodvExample::SetPosition()
{
	Ptr<UniformRandomVariable> urv = CreateObject<UniformRandomVariable> ();
	Ptr<UniformRandomVariable> utheta = CreateObject<UniformRandomVariable> ();

//	double m_maxRange = m_boundary/2;
    double interval = m_boundary/4;


     pos_uan->Add(Vector(interval/2,interval/2,m_depth));
	 pos_uan->Add(Vector(interval * 3.5,interval * 3.5,m_depth));
    //4个
	 pos_buoy->Add(Vector(interval,interval,0));
	 pos_buoy->Add(Vector(interval,interval * 3,0));
	 pos_buoy->Add(Vector(interval * 2,interval * 2,0));
	 pos_buoy->Add(Vector(interval * 3,interval,0));
	 pos_buoy->Add(Vector(interval * 3,interval * 3,0));
}

void
AodvExample::Ipv4L3Tx0 (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
	NS_LOG_UNCOND(
			"Time(Second)= "<<
			Simulator::Now().GetSeconds()<<
			"Node 0 IpL3 Tx Packet Byte Count = "<<
			packet->GetSize()<<" Packet UId = "<<packet->GetUid()
	);
}

void
AodvExample::Ipv4L3Rx0 (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
	NS_LOG_UNCOND(
			"Time(Second)= "<<
			Simulator::Now().GetSeconds()<<
			"Node 0 IpL3 Rx Packet Byte Count = "<<
			packet->GetSize()<<" Packet UId = "<<packet->GetUid()
	);
}

void
AodvExample::Ipv4L3Tx1 (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
	NS_LOG_UNCOND(
			"Time(Second)= "<<
			Simulator::Now().GetSeconds()<<
			"Node 1 IpL3 Tx Packet Byte Count = "<<
			packet->GetSize()<<" Packet UId = "<<packet->GetUid()
	);
}

void
AodvExample::Ipv4L3Rx1 (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
	NS_LOG_UNCOND(
			"Time(Second)= "<<
			Simulator::Now().GetSeconds()<<
			"Node 1 IpL3 Rx Packet Byte Count = "<<
			packet->GetSize()<<" Packet UId = "<<packet->GetUid()
	);
}

void
AodvExample::Ipv4L3Tx2 (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
	NS_LOG_UNCOND(
			"Time(Second)= "<<
			Simulator::Now().GetSeconds()<<
			"Node 2 IpL3 Tx Packet Byte Count = "<<
			packet->GetSize()<<" Packet UId = "<<packet->GetUid()
	);
}

void
AodvExample::Ipv4L3Rx2 (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
	NS_LOG_UNCOND(
			"Time(Second)= "<<
			Simulator::Now().GetSeconds()<<
			"Node 2 IpL3 Rx Packet Byte Count = "<<
			packet->GetSize()<<" Packet UId = "<<packet->GetUid()
	);
}

void
AodvExample::Ipv4L3Tx3 (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
	NS_LOG_UNCOND(
			"Time(Second)= "<<
			Simulator::Now().GetSeconds()<<
			"Node 3 IpL3 Tx Packet Byte Count = "<<
			packet->GetSize()<<" Packet UId = "<<packet->GetUid()
	);
}

void
AodvExample::Ipv4L3Rx3 (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
	NS_LOG_UNCOND(
			"Time(Second)= "<<
			Simulator::Now().GetSeconds()<<
			"Node 3 IpL3 Rx Packet Byte Count = "<<
			packet->GetSize()<<" Packet UId = "<<packet->GetUid()
	);
}

//void
//AodvExample::UanDevTx (Ptr<const Packet> packet)
//{
//	NS_LOG_UNCOND(
//			"Time(Second)= "<<
//			Simulator::Now().GetSeconds()<<
//			"Src uandevice Tx Packet Byte Count = "<<
//			packet->GetSize()<<" Packet UId = "<<packet->GetUid()
//	);
//}
//
//void
//AodvExample::UanDevRx (Ptr<const Packet> packet)
//{
//	NS_LOG_UNCOND(
//			"Time(Second)= "<<
//			Simulator::Now().GetSeconds()<<
//			"Sink uandevice Rx Packet Byte Count = "<<
//			packet->GetSize()<<" Packet UId = "<<packet->GetUid()
//	);
//}
void
AodvExample::Calculate()
{
	EnergyConsumSrc = 0;  //源节点能量消耗
	EnergyConsumSink = 0;  //目的节点能量消耗
	EnergyConsum = 0;      //节点能量消耗
	EnergyConsumBuoyWifi = 0;//浮标节点无线能量消耗
    EnergyConsumBuoyUan = 0;//浮标节点水声能量消耗

	double uan_consumed;
	double uan_sink_consumed;
	double uan_src_consumed;
	double buoy_wifi_consumed;
	double buoy_uan_consumed;

	NS_LOG_INFO("-----------Collect the uan node energy consumption.-----------");
	for(NodeContainer::Iterator it = uanNode.Begin(); it != uanNode.End(); ++it)
	{
		Ptr<EnergySource> energySrc = (*it)->GetObject<EnergySourceContainer>()->Get(0);
		uan_consumed += energySrc->GetInitialEnergy() - energySrc->GetRemainingEnergy();
	}

//	for (DeviceEnergyModelContainer::Iterator iter = uan_energy.Begin (); iter != uan_energy.End (); ++iter)
//   {
//		uan_consumed += (*iter)->GetTotalEnergyConsumption ();
//	}
	EnergyConsum += uan_consumed;

	NS_LOG_INFO("-----------Collect the node energy consumption.-----------");

	for (DeviceEnergyModelContainer::Iterator iter = uan_src_energy.Begin (); iter != uan_src_energy.End (); ++iter)
   {
		uan_src_consumed += (*iter)->GetTotalEnergyConsumption ();
	}
	for (DeviceEnergyModelContainer::Iterator iter = uan_sink_energy.Begin (); iter != uan_sink_energy.End (); ++iter)
	{
		uan_sink_consumed += (*iter)->GetTotalEnergyConsumption ();
	}
	EnergyConsumSrc += uan_src_consumed;
	EnergyConsumSink += uan_sink_consumed;

	NS_LOG_INFO("-----------Collect the bouy node energy consumption.-----------");
	for (DeviceEnergyModelContainer::Iterator iter = buoy_wifi_energy.Begin (); iter != buoy_wifi_energy.End (); ++iter)
	{
		buoy_wifi_consumed += (*iter)->GetTotalEnergyConsumption ();
	}
	for (DeviceEnergyModelContainer::Iterator iter = buoy_uan_energy.Begin (); iter != buoy_uan_energy.End (); ++iter)
	{
		buoy_uan_consumed += (*iter)->GetTotalEnergyConsumption ();
	}
	EnergyConsumBuoyWifi += buoy_wifi_consumed;
	EnergyConsumBuoyUan += buoy_uan_consumed;

	NS_LOG_UNCOND("Below list the energy consumption." );
	NS_LOG_UNCOND("Node totally consumede energy is"/*<<std::setprecision(4)*/<<EnergyConsum<<"." );
	NS_LOG_UNCOND("Uan node consumed energy is"<<std::setprecision(4)<<uan_consumed<<"." );
	NS_LOG_UNCOND("Src node uan netdeivces consumed energy is"<<std::setprecision(4)<<uan_src_consumed<<"." );
	NS_LOG_UNCOND("Sink node uan netdeivces consumed energy is"<<std::setprecision(4)<<uan_sink_consumed<<"." );
	NS_LOG_UNCOND("Buoy node uan netdeivces consumed energy is"<<std::setprecision(4)<<buoy_uan_consumed<<"." );
	NS_LOG_UNCOND("Buoy node wifi netdeivces consumed energy is"<<std::setprecision(4)<<buoy_wifi_consumed<<"." );
}

