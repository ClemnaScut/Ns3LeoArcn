//#include "ns3/aodv-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mobility-helper.h"
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
#include "ns3/node.h"

#include "ns3/stats-module.h"
#include "ns3/applications-module.h"
#include "ns3/aodv-module.h"
#include "ns3/tag.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
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
#include <cstdlib>
#include <stdlib.h>
#include <random>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("aecntest");

class AecnExample
{
public:
  AecnExample ();

  bool Configure (int argc, char **argv);
  /// Run simulation
  void Run (AecnExample test);

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
};
//private:
	// parameters
	uint32_t numUansrc;//number of uan src node
	uint32_t numUansink;//number of uan sink node
	uint32_t numgtbuoy;//number of buoynode
	double m_depth;            // depth of uan node.
	double totalTime;  /// Simulation time, seconds
	bool pcap;  /// Write per-device PCAP traces if true
	bool printRoutes;  /// Print routes if true
	double m_txPowerAcoustic;    //Tx power of uan device
	std::string m_uanMacType;        //UanMac
	uint16_t m_port;                    //socket port

	//result
	double EnergyConsumSrc;
	double EnergyConsumSink;
    double EnergyConsum;
	double EnergyConsumBuoyWifi;
    double EnergyConsumBuoyUan;

	//result
	double DeliveryRatio;
    double End2EndDelay;
    double Throughput;
    double recvbitnum;
    double pbconsumenergy;//per bit/J

	DeviceEnergyModelContainer uan_src_energy;
	DeviceEnergyModelContainer uan_sink_energy;
	DeviceEnergyModelContainer buoy_wifi_energy;
	DeviceEnergyModelContainer buoy_uan_energy;


	/// srcnode used in the example
	NodeContainer uansrcNode;
	/// sinknode used in the example
	NodeContainer uansinkNode;
	/// gateway node used in the example
	NodeContainer gatewaybuoynode;

	NetDeviceContainer uansrc_device;
	NetDeviceContainer uansink_device;
	NetDeviceContainer gateway_buoywireless_device;
	NetDeviceContainer gateway_buoyuan_device;


	Ipv4InterfaceContainer uansrc_inter;
	Ipv4InterfaceContainer uansink_inter;
	Ipv4InterfaceContainer gateway_buoywireless_inter;
	Ipv4InterfaceContainer gateway_buoyuan_inter;

	//packet
	uint32_t m_pktSize;        // The size of packets transmitted.
	Time m_pktInterval; // Delay between transmissions

	double m_SinkRecvNum;//total packet of received.

	double m_offset;
	double m_Prss;
	double m_txPowerRadio;//Tx power of radio device.
	double m_step;
	uint32_t m_seqnum;//total packet of sended.
	double RunTime;
	double m_totaldelay;
	double m_lambda;
	double calculatecount;
	std::string m_wifiPhyMode;

	std::map<Ptr<Node>, Ptr<Socket> > m_srcsockets; //!< send and receive sockets
	std::map<Ptr<Node>, Ptr<Socket> > m_sinksockets; //!< send and receive sockets
	std::map<Ptr<Node>, Ptr<Socket> > m_buoyuansockets; //!< send and receive sockets
	std::map<Ptr<Node>, Ptr<Socket> > m_buoywifisockets; //!< send and receive sockets
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_srcSktAddr;
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_sinkSktAddr;
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_buoyWifiSktAddr;
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_buoyUanSktAddr;

	//sub net
	std::map<Ptr<Node>, Ptr<Socket> > m_subsrcsockets; //!< send and receive sockets
	std::map<Ptr<Node>, Ptr<Socket> > m_subsinksockets; //!< send and receive sockets
	std::map<Ptr<Node>, Ptr<Socket> > m_subbuoyuansockets; //!< send and receive sockets
	std::map<Ptr<Node>, Ptr<Socket> > m_subbuoywifisockets; //!< send and receive sockets
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_subsrcSktAddr;
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_subsinkSktAddr;
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_subbuoyWifiSktAddr;
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_subbuoyUanSktAddr;

    std::map<uint32_t, double> SinkMap_IdRecvTime;
    std::map<uint32_t, double> SrcMap_IdSendTime;

	Ptr<UniformRandomVariable> m_uniformRandomVariable;
	Ptr<ExponentialRandomVariable> erv;
private:

	// Create socket, used to send or receive packet.
  void SetupSocket();
  /// Create the nodes
  void CreateNodes ();
  /// Create the devices
  void CreateDevices (UanHelper uanHelper,AecnExample test,YansWifiPhyHelper wifiPhy);
  /// Create the network. (bind to Routing protocol)
  void InstallInternetStack ();
  /**
   * Print the received packet
   * \param socket The receiving socket
   */
  void ReceivePacket (Ptr<Socket> socket);

  BuoyParam InstallDevices(NodeContainer nodes, UanHelper &uan, Ptr<UanChannel> uanChannel, WifiHelper &wifi, YansWifiPhyHelper &wifiPhy, WifiMacHelper &wifiMac, YansWifiChannelHelper &wifiChannel);
  // Set position and mobility model of node.
  void SetPosition();
  // calculate the result!
  void Calculate();

  void SendTo (Ptr<Socket> socket, Ptr<Packet> packet, Ipv4Address destination);

  void NodeSendPacket ();

};

int main (int argc, char **argv)
{
  AecnExample test;
 LogComponentEnableAll(LOG_PREFIX_ALL);
	LogComponentEnableAll(LOG_PREFIX_TIME);
	LogComponentEnableAll(LOG_PREFIX_NODE);
	LogComponentEnableAll(LOG_PREFIX_FUNC);
	LogComponentEnableAll(LOG_PREFIX_LEVEL);

  if (!test.Configure (argc, argv))
    NS_FATAL_ERROR ("Configuration failed. Aborted.");
////	 LogComponentEnable("UanMacCw", LOG_LEVEL_ALL);
//	 LogComponentEnable("UanPhyGen", LOG_LEVEL_ALL);
//	 LogComponentEnable("UanNetDevice", LOG_LEVEL_ALL);
////	 LogComponentEnable("UanChannel", LOG_LEVEL_ALL);
	//  LogComponentEnable("AodvRoutingProtocol", LOG_LEVEL_ALL);
////	 LogComponentEnable("aecnRoutingProtocol", LOG_LEVEL_ALL);
	 LogComponentEnable("UdpL4Protocol", LOG_LEVEL_ALL);

////	 LogComponentEnable("V4Ping", LOG_LEVEL_ALL);
//	 LogComponentEnable("WifiPhy", LOG_LEVEL_ALL);
////	 LogComponentEnable("YansWifiChannel", LOG_LEVEL_ALL);
//	 LogComponentEnable("Ipv4L3Protocol", LOG_LEVEL_ALL);
//
//	 LogComponentEnable("ArpL3Protocol", LOG_LEVEL_ALL);
//	 LogComponentEnable("TrafficControlLayer", LOG_LEVEL_ALL);

////     LogComponentEnable("AodvRoutingProtocol", LOG_LEVEL_ALL);
//	LogComponentEnable("UanNetDevice", LOG_LEVEL_ALL);
////	LogComponentEnable("UanMacCw", LOG_LEVEL_ALL);
//	LogComponentEnable("UanTxMode", LOG_LEVEL_ALL);
////	LogComponentEnable("UanPhyGen", LOG_LEVEL_ALL);
//	LogComponentEnable("UanChannel", LOG_LEVEL_ALL);
//	LogComponentEnable("UanMacAloha", LOG_LEVEL_ALL);
////
////	 LogComponentEnable("WifiHelper", LOG_LEVEL_ALL);
//	 LogComponentEnable("AdhocWifiMac", LOG_LEVEL_ALL);
////	// LogComponentEnable("YansWifiPhy", LOG_LEVEL_ALL);
////	LogComponentEnable("WifiPhy", LOG_LEVEL_ALL);
////	 LogComponentEnable("WifiNetDevice", LOG_LEVEL_ALL);
////	LogComponentEnable("YansWifiChannel", LOG_LEVEL_ALL);
//
////	LogComponentEnable("AodvRoutingProtocol", LOG_LEVEL_ALL);
//  LogComponentEnable("AodvNeighbors", LOG_LEVEL_ALL);
//  	LogComponentEnable("OlsrRoutingProtocol", LOG_LEVEL_ALL);

  test.Run (test);
  test.Report (std::cout);
  test.Teardown ();
  return 0;
}

//-----------------------------------------------------------------------------
AecnExample::AecnExample () :
  numUansrc (4),
  numUansink(16),
  numgtbuoy(8),
  m_depth(-500),
  totalTime (2000),
  pcap (true),
  printRoutes (true),
  m_txPowerAcoustic(145),
  m_uanMacType("ns3::UanMacAloha"),
  m_port(26),
  m_pktSize(400),
  m_pktInterval(5),
  m_SinkRecvNum(0),
  m_offset(81),
  m_Prss(-80),
  m_txPowerRadio(82),
  m_step(1),
  m_seqnum(0),
  RunTime(0),
  m_totaldelay(0),
  m_lambda(0.2),
  calculatecount(0),
m_wifiPhyMode("OfdmRate6Mbps")

{
}

bool
AecnExample::Configure (int argc, char **argv)
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
AecnExample::Run (AecnExample test)
{
  //Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (1)); // enable rts cts all the time.
  CreateNodes ();
  SetPosition ();
  UanHelper uanHelper;
  YansWifiPhyHelper wifiPhy;
  CreateDevices (uanHelper,test,wifiPhy);
  InstallInternetStack ();
  SetupSocket();
  Simulator::Schedule(Seconds(3), &AecnExample::NodeSendPacket, this);

  std::string ascii_name ("AECN-aodv.asc");// Name for ascii trace file, default uan-cw-example.asc.
  std::ofstream ascii (ascii_name.c_str());
  uanHelper.EnableAsciiAll(ascii);

  AsciiTraceHelper asciiwifi;
  Ptr<OutputStreamWrapper> stream = asciiwifi.CreateFileStream ("AECN-aodv.tr");
  wifiPhy.EnableAsciiAll (stream);
  wifiPhy.EnablePcapAll(std::string ("AECN-aodv"));

  NodeContainer::Iterator m_srcnode = uansrcNode.Begin ();
  (*m_srcnode)->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetArpCache ()->SetWaitReplyTimeout (Seconds (10));

  NodeContainer::Iterator m_gatewaynode = gatewaybuoynode.Begin ();
  (*m_gatewaynode)->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetArpCache ()->SetWaitReplyTimeout (Seconds (10));

   NodeContainer::Iterator m_sinkNode = uansinkNode.Begin ();
  (*m_sinkNode)->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetArpCache ()->SetWaitReplyTimeout (Seconds (10));


	 AnimationInterface anim ("40uan8buoy-aodv-animation.xml"); // Mandatory
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
	 for (uint32_t i = 0; i < gatewaybuoynode.GetN (); ++i)
	 {
	 	anim.UpdateNodeDescription (gatewaybuoynode.Get (i), "UAN"); // Optional
	 	anim.UpdateNodeColor (gatewaybuoynode.Get (i), 0, 255, 0); // Optional
	 }


  std::cout << "Starting simulation for " << totalTime << " s ...\n";
  NS_LOG_UNCOND("Simulation run "<<totalTime<<" seconds....\n");
  Simulator::Schedule(Seconds(200), &AecnExample::Calculate, this);

  Simulator::Stop (Seconds (totalTime+1));
  Simulator::Run ();
  Simulator::Destroy ();
  Teardown();
}

void
AecnExample::Report (std::ostream &)
{
}

void
AecnExample::ReceivePacket (Ptr<Socket> socket)
{
  NS_LOG_INFO ("Received one packet!");
  m_SinkRecvNum++;
  Ptr<Packet> packet = socket->Recv ();
//  SocketIpTosTag tosTag;
////  double sendtime(0);
//  NS_LOG_DEBUG("Packet Uid ="<<packet->GetUid());
//  if (packet->RemovePacketTag (tosTag))
//    {
//      NS_LOG_INFO (" TOS = " << (uint32_t)tosTag.GetTos ());
//    }
//  SocketIpTtlTag ttlTag;
//  if (packet->RemovePacketTag (ttlTag))
//    {
//      NS_LOG_INFO (" TTL = " << (uint32_t)ttlTag.GetTtl ());
//    }
//	Time recvtime = Simulator::Now().GetSeconds();
//	SinkMap_IdRecvTime[packet->GetUid()] = recvtime;

//		sendtime == SrcMap_IdSendTime[packet->GetUid()];
		double delta = Simulator::Now ().GetSeconds() - SrcMap_IdSendTime[packet->GetUid()];
//		NS_LOG_UNCOND("Packet"<<packet->GetUid()<<"transmitted delay is  "<<std::setprecision(4)<<delta<<"s." );
		m_totaldelay =m_totaldelay+delta;
}

void
AecnExample::CreateNodes ()
{
	uansrcNode.Create(numUansrc);
	uansinkNode.Create(numUansink);
	gatewaybuoynode.Create(numgtbuoy);

	// Name uan src nodes
	for (uint32_t i = 0; i < numUansrc; ++i)
	{
		std::ostringstream os;
		os << "uansrcNode-" << i;
		Names::Add(os.str(), uansrcNode.Get(i));
	}

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
}

void
AecnExample::CreateDevices (UanHelper uanHelper,AecnExample test,YansWifiPhyHelper wifiPhy)
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
//	uanMode = UanTxModeFactory::CreateMode(UanTxMode::FSK, 9600,
//			10000, 25000, 20000, 2, "Default mode");
	uanMode = UanTxModeFactory::CreateMode(UanTxMode::PSK, 9600,
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
	uansink_device = uanHelper.Install(uansinkNode, uanChannel);
	gateway_buoyuan_device = uanHelper.Install(gatewaybuoynode, uanChannel);

	NS_LOG_DEBUG("-----------Initializing Wireless-----------");
	WifiHelper wifiHelper;
	wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211a);
	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
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
	NS_LOG_DEBUG("-----Finish Buoy wireless device setting.------");

	NS_LOG_DEBUG("-----------Initializing Energy-----------");
	BasicEnergySourceHelper energySourceHelper;
	energySourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (900000));
	AcousticModemEnergyModelHelper acousticEnergyHelper;

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
}

void
AecnExample::NodeSendPacket ()
{

          Ptr<Packet> packet = Create<Packet> (m_pktSize);
          m_seqnum++;
          NS_LOG_DEBUG("Source Send Packet sequence number is "<<m_seqnum);
          NS_LOG_DEBUG("Source Send Packet Uid ="<<packet->GetUid());

          double sendtime = Simulator::Now().GetSeconds();
		  SrcMap_IdSendTime[packet->GetUid()] = sendtime;


//	      Ptr<Node> srcsend = uansrcNode.Get(0);
//	      Ptr<Socket> socket =  m_srcsockets[srcsend];
//	      SendTo(socket, packet, Ipv4Address ("10.1.1.8"));
////			      erv = CreateObjectWithAttributes<ExponentialRandomVariable>("Mean",DoubleValue(1/m_lambda));
////			      Simulator::Schedule (Seconds(erv->GetValue()), &AecnExample::NodeSendPacket, this);
//	      Simulator::Schedule (Seconds(m_pktInterval), &AecnExample::NodeSendPacket, this);

//
          double original = rand()%4;
			if(original == 0)
			{
			      Ptr<Node> srcsend = uansrcNode.Get(0);
			      Ptr<Socket> socket =  m_srcsockets[srcsend];
			      SendTo(socket, packet, Ipv4Address ("10.1.1.8"));
		//			      erv = CreateObjectWithAttributes<ExponentialRandomVariable>("Mean",DoubleValue(1/m_lambda));
		//			      Simulator::Schedule (Seconds(erv->GetValue()), &AecnExample::NodeSendPacket, this);
			      Simulator::Schedule (Seconds(m_pktInterval), &AecnExample::NodeSendPacket, this);
			}
			else if(original == 1)
			{
			      Ptr<Node> buoywifisend = uansrcNode.Get(1);
			      Ptr<Socket> socket =  m_srcsockets[buoywifisend];
			      SendTo(socket, packet, Ipv4Address ("10.1.1.6"));
		//			      erv = CreateObjectWithAttributes<ExponentialRandomVariable>("Mean",DoubleValue(1/m_lambda));
		//			      Simulator::Schedule (Seconds(erv->GetValue()), &AecnExample::NodeSendPacket, this);
			      Simulator::Schedule (Seconds(m_pktInterval), &AecnExample::NodeSendPacket, this);
			}
			else if(original == 2)
			{
			      Ptr<Node> buoywifisend = uansrcNode.Get(2);
			      Ptr<Socket> socket =  m_srcsockets[buoywifisend];
			      SendTo(socket, packet, Ipv4Address ("10.1.1.5"));
		//			      erv = CreateObjectWithAttributes<ExponentialRandomVariable>("Mean",DoubleValue(1/m_lambda));
		//			      Simulator::Schedule (Seconds(erv->GetValue()), &AecnExample::NodeSendPacket, this);
			      Simulator::Schedule (Seconds(m_pktInterval), &AecnExample::NodeSendPacket, this);
			}
			else if(original == 3)
			{
			      Ptr<Node> buoywifisend = uansrcNode.Get(3);
			      Ptr<Socket> socket =  m_srcsockets[buoywifisend];
			      SendTo(socket, packet, Ipv4Address ("10.1.1.7"));
		//			      erv = CreateObjectWithAttributes<ExponentialRandomVariable>("Mean",DoubleValue(1/m_lambda));
		//			      Simulator::Schedule (Seconds(erv->GetValue()), &AecnExample::NodeSendPacket, this);
			      Simulator::Schedule (Seconds(m_pktInterval), &AecnExample::NodeSendPacket, this);
			}

}

void
AecnExample::SendTo (Ptr<Socket> socket, Ptr<Packet> packet, Ipv4Address destination)
{
  socket->SendTo (packet, 0, InetSocketAddress (destination, m_port));
}


void
AecnExample::Teardown ()
{
  std::map<Ptr<Node>, Ptr<Socket> >::iterator socket;

  for (socket = m_srcsockets.begin (); socket != m_srcsockets.end (); socket++)
    {
      socket->second->Close ();
    }

  for (socket = m_sinksockets.begin (); socket != m_sinksockets.end (); socket++)
    {
      socket->second->Close ();
    }

  for (socket = m_buoyuansockets.begin (); socket != m_buoyuansockets.end (); socket++)
    {
      socket->second->Close ();
    }

  for (socket = m_buoywifisockets.begin (); socket != m_buoywifisockets.end (); socket++)
    {
      socket->second->Close ();
    }
}

AecnExample::BuoyParam
AecnExample::InstallDevices(NodeContainer nodes, UanHelper &uan, Ptr<UanChannel> uanChannel, WifiHelper &wifi, YansWifiPhyHelper &wifiPhy, WifiMacHelper &wifiMac, YansWifiChannelHelper &wifiChannel)
{
	NS_LOG_DEBUG("Create Buoy Nodes");
	YansWifiPhyHelper wifi_phy = wifiPhy;
	wifi_phy.SetChannel (wifiChannel.Create ());
	WifiMacHelper wifi_mac = wifiMac;
	NS_LOG_DEBUG("Set Buoy Uan Device");
	NetDeviceContainer uanDevices = uan.Install (nodes, uanChannel);
	NS_LOG_DEBUG("Set Buoy Wifi Device");
	NetDeviceContainer wifiDevices = wifi.Install (wifi_phy, wifi_mac, nodes);
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
AecnExample::InstallInternetStack ()
{
	//--------------------------------------------aodv routing--------------------------------------------
	AodvHelper aodv;
	aodv.Set("HelloInterval", TimeValue(Seconds(1000)));
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
	aodv.Set("TtlStart", UintegerValue(20));

	InternetStackHelper stack;
	stack.SetRoutingHelper (aodv);
	stack.Install(uansrcNode);
	stack.Install (uansinkNode);
	stack.Install (gatewaybuoynode);

	Ipv4AddressHelper address;
	address.SetBase("10.1.1.0", "255.255.255.0");
	NS_LOG_DEBUG ("Assign Buoy uan IP Addresses.");
	uansrc_inter = address.Assign(uansrc_device);
	uansink_inter = address.Assign(uansink_device);
	gateway_buoyuan_inter = address.Assign(gateway_buoyuan_device);

	NS_LOG_DEBUG ("Assign Buoy Buoywifi IP Addresses.");
	address.SetBase("10.1.2.0", "255.255.255.0");
	gateway_buoywireless_inter = address.Assign(gateway_buoywireless_device);

	//change arp cache wait time.
	NodeContainer::Iterator m_uansrcNode = uansrcNode.Begin ();
	m_uansrcNode = uansrcNode.Begin();
	while (m_uansrcNode != uansrcNode.End()) {
		(*m_uansrcNode)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->SetWaitReplyTimeout(
				Seconds(20));
		m_uansrcNode++;
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


	if (printRoutes)
	{
		Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(
				"AECN-aodv.routes", std::ios::out);
		aodv.PrintRoutingTableAllAt(Seconds(180), routingStream);
	}
}

void
AecnExample::SetupSocket()
{
	  for (uint32_t i = 0; i < uansrc_device.GetN (); ++i)
	  {
            Ptr<NetDevice> dev = uansrc_device.Get (i);

            Ptr<Node> srcnode = dev->GetNode ();
            Ptr<Ipv4> ipv4 = srcnode->GetObject<Ipv4> ();
            int32_t interfaceId = ipv4->GetInterfaceForDevice (dev);
            if (interfaceId == -1)
              {
            	interfaceId = ipv4->AddInterface (dev);
              }
			uint32_t node_id = srcnode->GetId ();
			Ipv4InterfaceAddress iface = ipv4->GetAddress(interfaceId, 0);
			Ipv4Address ip_addr = iface.GetLocal();
			  if (iface.GetLocal () == Ipv4Address ("127.0.0.1"))
			    {

			      return;
			    }

		  Ptr<Socket> cur_socket = Socket::CreateSocket (srcnode,
		                                             UdpSocketFactory::GetTypeId ());
		  NS_ASSERT (cur_socket != 0);
		  cur_socket->SetRecvCallback (MakeCallback (&AecnExample::ReceivePacket, this));
		  cur_socket->BindToNetDevice (dev);
		  cur_socket->Bind (InetSocketAddress (ip_addr, m_port));
		  cur_socket->SetAllowBroadcast (true);
		  cur_socket->SetIpRecvTtl (true);
		  m_srcSktAddr.insert (std::make_pair (cur_socket, iface));
		  m_srcsockets.insert (std::make_pair (srcnode, cur_socket));

		  // create also a subnet broadcast socket
		  Ptr<Socket> cur_subsocket = Socket::CreateSocket (srcnode,
		                                 UdpSocketFactory::GetTypeId ());
		  NS_ASSERT (cur_subsocket != 0);
		  cur_subsocket->SetRecvCallback (MakeCallback (&AecnExample::ReceivePacket, this));
		  cur_subsocket->BindToNetDevice (dev);
		  cur_subsocket->Bind (InetSocketAddress (ip_addr, m_port));
		  cur_subsocket->SetAllowBroadcast (true);
		  cur_subsocket->SetIpRecvTtl (true);
		  m_subsrcSktAddr.insert (std::make_pair (cur_subsocket, iface));
		  m_subsrcsockets.insert (std::make_pair (srcnode, cur_subsocket));
		  NS_LOG_DEBUG("Src Node Uan ID =" << node_id << " Socket =" <<cur_socket << " Ipv4Address " << ip_addr);
	    }


	  for (uint32_t i = 0; i < uansink_device.GetN (); ++i)
	    {
          Ptr<NetDevice> dev = uansink_device.Get (i);

          Ptr<Node> sinknode = dev->GetNode ();
          Ptr<Ipv4> ipv4 = sinknode->GetObject<Ipv4> ();
          int32_t interfaceId = ipv4->GetInterfaceForDevice (dev);
          if (interfaceId == -1)
            {
          	interfaceId = ipv4->AddInterface (dev);
            }
			uint32_t node_id = sinknode->GetId ();
			Ipv4InterfaceAddress iface = ipv4->GetAddress(interfaceId, 0);
			Ipv4Address ip_addr = iface.GetLocal();
			  if (iface.GetLocal () == Ipv4Address ("127.0.0.1"))
			    {
			      return;
			    }

		  // Create a socket to listen only on this interface
		  Ptr<Socket> cur_socket = Socket::CreateSocket (sinknode,
		                                             UdpSocketFactory::GetTypeId ());
		  NS_ASSERT (cur_socket != 0);
		  cur_socket->SetRecvCallback (MakeCallback (&AecnExample::ReceivePacket, this));
		  cur_socket->BindToNetDevice (dev);
		  cur_socket->Bind (InetSocketAddress (ip_addr, m_port));
		  cur_socket->SetAllowBroadcast (true);
		  cur_socket->SetIpRecvTtl (true);
		  m_sinkSktAddr.insert (std::make_pair (cur_socket, iface));
		  m_sinksockets.insert (std::make_pair (sinknode, cur_socket));

		  // create also a subnet broadcast socket
		  Ptr<Socket> cur_subsocket = Socket::CreateSocket (sinknode,
		                                 UdpSocketFactory::GetTypeId ());
		  NS_ASSERT (cur_subsocket != 0);
		  cur_subsocket->SetRecvCallback (MakeCallback (&AecnExample::ReceivePacket, this));
		  cur_subsocket->BindToNetDevice (dev);
		  cur_subsocket->Bind (InetSocketAddress (ip_addr, m_port));
		  cur_subsocket->SetAllowBroadcast (true);
		  cur_subsocket->SetIpRecvTtl (true);
		  m_subsinkSktAddr.insert (std::make_pair (cur_subsocket, iface));
		  m_subsinksockets.insert (std::make_pair (sinknode, cur_subsocket));
		  NS_LOG_DEBUG("Sink Node Uan ID =" << node_id << " Socket =" <<cur_socket << " Ipv4Address " << ip_addr);
	    }

	  for (uint32_t i = 0; i < gateway_buoywireless_device.GetN (); ++i)
	    {
          Ptr<NetDevice> dev = gateway_buoywireless_device.Get (i);

          Ptr<Node> buoynode = dev->GetNode ();
          Ptr<Ipv4> ipv4 = buoynode->GetObject<Ipv4> ();
          int32_t interfaceId = ipv4->GetInterfaceForDevice (dev);
          if (interfaceId == -1)
            {
          	interfaceId = ipv4->AddInterface (dev);
            }
			uint32_t node_id = buoynode->GetId ();
			Ipv4InterfaceAddress iface = ipv4->GetAddress(interfaceId, 0);
			Ipv4Address ip_addr = iface.GetLocal();
			  if (iface.GetLocal () == Ipv4Address ("127.0.0.1"))
			    {
			      return;
			    }


	      //wifi device
		  // Create a socket to listen only on this interface
		  Ptr<Socket> bw_socket = Socket::CreateSocket (buoynode,
		                                             UdpSocketFactory::GetTypeId ());
		  NS_ASSERT (bw_socket != 0);
		  bw_socket->SetRecvCallback (MakeCallback (&AecnExample::ReceivePacket, this));
		  bw_socket->BindToNetDevice (dev);
		  bw_socket->Bind (InetSocketAddress (ip_addr, m_port));
		  bw_socket->SetAllowBroadcast (true);
		  bw_socket->SetIpRecvTtl (true);
		  m_buoyWifiSktAddr.insert (std::make_pair (bw_socket, iface));
		  m_buoywifisockets.insert (std::make_pair (buoynode, bw_socket));

		  // create a subnet broadcast socket
		  Ptr<Socket> bw_subsocket = Socket::CreateSocket (buoynode,
		                                 UdpSocketFactory::GetTypeId ());
		  NS_ASSERT (bw_subsocket != 0);
		  bw_subsocket->SetRecvCallback (MakeCallback (&AecnExample::ReceivePacket, this));
		  bw_subsocket->BindToNetDevice (dev);
		  bw_subsocket->Bind (InetSocketAddress (ip_addr, m_port));
		  bw_subsocket->SetAllowBroadcast (true);
		  bw_subsocket->SetIpRecvTtl (true);
		  m_subbuoyWifiSktAddr.insert (std::make_pair (bw_subsocket, iface));
		  m_subbuoywifisockets.insert (std::make_pair (buoynode, bw_subsocket));
		  NS_LOG_DEBUG("Buoy Node Wifi ID =" << node_id << " Socket =" <<bw_socket << " Ipv4Address " << ip_addr);
	    }
	  for (uint32_t i = 0; i < gateway_buoyuan_device.GetN (); ++i)
  	{

	      //uan device
          Ptr<NetDevice> dev = gateway_buoyuan_device.Get (i);

          Ptr<Node> buoynode = dev->GetNode ();
          Ptr<Ipv4> ipv4 = buoynode->GetObject<Ipv4> ();
          int32_t interfaceId = ipv4->GetInterfaceForDevice (dev);
          if (interfaceId == -1)
            {
          	interfaceId = ipv4->AddInterface (dev);
            }
			uint32_t node_id = buoynode->GetId ();
			Ipv4InterfaceAddress iface = ipv4->GetAddress(interfaceId, 0);
			Ipv4Address ip_addr = iface.GetLocal();
			  if (iface.GetLocal () == Ipv4Address ("127.0.0.1"))
			    {
			      return;
			    }

		  // Create a socket to listen only on this interface
		  Ptr<Socket> bu_socket = Socket::CreateSocket (buoynode,
		                                             UdpSocketFactory::GetTypeId ());
		  NS_ASSERT (bu_socket != 0);
		  bu_socket->SetRecvCallback (MakeCallback (&AecnExample::ReceivePacket, this));
		  bu_socket->BindToNetDevice (dev);
		  bu_socket->Bind (InetSocketAddress (ip_addr, m_port));
		  bu_socket->SetAllowBroadcast (true);
		  bu_socket->SetIpRecvTtl (true);
		  m_buoyUanSktAddr.insert (std::make_pair (bu_socket, iface));
		  m_buoyuansockets.insert (std::make_pair (buoynode, bu_socket));

		  // create also a subnet broadcast socket
		  Ptr<Socket> bu_subsocket = Socket::CreateSocket (buoynode,
		                                 UdpSocketFactory::GetTypeId ());
		  NS_ASSERT (bu_subsocket != 0);
		  bu_subsocket->SetRecvCallback (MakeCallback (&AecnExample::ReceivePacket, this));
		  bu_subsocket->BindToNetDevice (dev);
		  bu_subsocket->Bind (InetSocketAddress (ip_addr, m_port));
		  bu_subsocket->SetAllowBroadcast (true);
		  bu_subsocket->SetIpRecvTtl (true);
		  m_subbuoyUanSktAddr.insert (std::make_pair (bu_subsocket, iface));
		  m_subbuoyuansockets.insert (std::make_pair (buoynode, bu_subsocket));
		  NS_LOG_DEBUG("Buoy Node Uan ID =" << node_id << " Socket =" <<bu_socket << " Ipv4Address " << ip_addr);
  	}
}

void
AecnExample::SetPosition()
{
	MobilityHelper mobility;
	Ptr<ListPositionAllocator> nodesPositionAlloc = CreateObject<ListPositionAllocator> ();

//set sink uan node position
	  SeedManager::SetRun (SeedManager::GetRun () + 10);
	  Ptr<UniformRandomVariable> urv = CreateObject<UniformRandomVariable> ();
	  for (uint32_t i = 0; i < numUansrc; i++)
	  {
	    //uan node position
			double x = urv->GetValue(0, 10000);
			double y = urv->GetValue(0, 5000);
			nodesPositionAlloc->Add (Vector (x, y, m_depth));
//	     NS_LOG_INFO("Underwater node "<<i<<"'s x coordinate is " <<x);
//	     NS_LOG_INFO("Underwater node "<<i<<"'s y coordinate is " <<y);
//	     NS_LOG_INFO("Underwater node "<<i<<"'s depth coordinate is " <<m_depth);
	  }
	  for (uint32_t i = 0; i < numUansink; i++)
	  {
	    //uan node position
			double x = urv->GetValue(0, 10000);
			double y = urv->GetValue(0, 5000);
			nodesPositionAlloc->Add (Vector (x, y, m_depth));
	    // NS_LOG_INFO("Underwater node "<<i<<"'s x coordinate is " <<x);
	    // NS_LOG_INFO("Underwater node "<<i<<"'s y coordinate is " <<y);
	    // NS_LOG_INFO("Underwater node "<<i<<"'s depth coordinate is " <<depth);
	  }

	  //Set Buoy node's position
	  nodesPositionAlloc->Add (Vector (1200, 1200, 0));
	  nodesPositionAlloc->Add (Vector (5000, 1200, 0));
	  nodesPositionAlloc->Add (Vector (8800, 1200, 0));
	  nodesPositionAlloc->Add (Vector (1200, 3800, 0));
	  nodesPositionAlloc->Add (Vector (5000, 3800, 0));
	  nodesPositionAlloc->Add (Vector (8800, 3800, 0));
	  nodesPositionAlloc->Add (Vector (3100, 2500, 0));
	  nodesPositionAlloc->Add (Vector (6900, 2500, 0));

	  mobility.SetPositionAllocator (nodesPositionAlloc);
	  mobility.Install (uansrcNode);
	  mobility.Install (uansinkNode);
	  mobility.Install (gatewaybuoynode);
}

void
AecnExample::Calculate()
{
	EnergyConsumSrc = 0;  //Src energy consumption.
	EnergyConsumSink = 0;  //Sink energy consumption.
	calculatecount++;

	double uan_sink_consumed;
	double uan_src_consumed;

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

	End2EndDelay = m_totaldelay/m_SinkRecvNum;

	NS_LOG_INFO("-----------Collect the node delivery ratio.-----------");
	DeliveryRatio = (double(m_SinkRecvNum)) / double(m_seqnum);   //to bind

	NS_LOG_INFO("-----------Collect the network throughput-----------");
	recvbitnum = double(m_SinkRecvNum * m_pktSize * 8);
//	Throughput = double(m_SinkRecvNum * m_pktSize * 8)/ totalTime;
	Throughput = double(m_SinkRecvNum * m_pktSize * 8)/ (calculatecount*200);
	pbconsumenergy = recvbitnum/double(uan_src_consumed+uan_sink_consumed);
	NS_LOG_UNCOND("R:"<<std::setprecision(4)<<DeliveryRatio<<"t:"<<std::setprecision(4)<<Throughput<<"D:"<<std::setprecision(4)<<End2EndDelay<<"J:"<<std::setprecision(4)<<pbconsumenergy<<"." );
//
//	NS_LOG_UNCOND("Sink has totally received "<<m_SinkRecvNum<<"packets and src has totally sent "<<m_seqnum<<" packets.");
//	NS_LOG_UNCOND("The calculate count is  "<<std::setprecision(4)<<calculatecount<<"." );
//	NS_LOG_UNCOND("The delivery ratio is "<<std::setprecision(4)<<DeliveryRatio<<"." );
//	NS_LOG_UNCOND("The received bit number is  "<<std::setprecision(4)<<recvbitnum<<"." );
//	NS_LOG_UNCOND("The network throughput is  "<<std::setprecision(4)<<Throughput<<"." );
//	NS_LOG_UNCOND("The end to end delay is  "<<std::setprecision(4)<<End2EndDelay<<"." );
//	NS_LOG_UNCOND("bit/J  is"<<std::setprecision(4)<<pbconsumenergy<<"." );
//	NS_LOG_UNCOND("Below list the energy consumption." );
//	NS_LOG_UNCOND("Node totally consumede energy is"/*<<std::setprecision(4)*/<<EnergyConsum<<"." );
//	NS_LOG_UNCOND("Src node uan netdeivces consumed energy is"<<std::setprecision(4)<<uan_src_consumed<<"." );
//	NS_LOG_UNCOND("Sink node uan netdeivces consumed energy is"<<std::setprecision(4)<<uan_sink_consumed<<"." );
//	NS_LOG_UNCOND("Received per bit consumed underwater energy is"<<std::setprecision(4)<<pbconsumenergy<<"." );

	Simulator::Schedule (Seconds (200), &AecnExample::Calculate, this);//print parameter per 200s
}
