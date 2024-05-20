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
#include "ns3/vbf-helper.h"
#include "ns3/vbf-module.h"
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

NS_LOG_COMPONENT_DEFINE("arcn-protocol-test");

class ARCN
{
//private:
    /* Simulator */
	double totalTime;  /// Simulation time, seconds
	bool pcap;  /// Write per-device PCAP traces if true
	bool printRoutes;  /// Print routes if true
	Ptr<UniformRandomVariable> m_uniformRandomVariable;
	//Ptr<ExponentialRandomVariable> erv;
    double m_lambda;


    /* Node */
    uint32_t numUansrc;//number of uan src node
	uint32_t numUansink;//number of uan sink node
	uint32_t numgtbuoy;//number of buoynode
	NodeContainer uansrcNode;
	NodeContainer uansinkNode;
	NodeContainer gatewaybuoynode;
	NetDeviceContainer uansrc_device;
	NetDeviceContainer uansink_device;
	NetDeviceContainer gateway_buoywireless_device;
	NetDeviceContainer gateway_buoyuan_device;
	Ipv4InterfaceContainer uansrc_inter;
	Ipv4InterfaceContainer uansink_inter;
	Ipv4InterfaceContainer gateway_buoywireless_inter;
	Ipv4InterfaceContainer gateway_buoyuan_inter;

    /* Parameters */
	std::string m_uanMacType;        //UanMac
	std::string m_wifiPhyMode;         //WifiPhy
	double m_depth;            // depth of uan node.
    double m_interval;   //interval of uan node
	double m_txPowerAcoustic;    //Tx power of uan device
	uint16_t m_port;                    //socket port
	uint32_t m_pktSize;        // The size of packets transmitted.
	Time m_pktInterval; // Delay between transmissions
	double m_offset;
	double m_Prss;
	double m_txPowerRadio;//Tx power of radio device.
	double m_step;


    /* EnergyConsum */
	DeviceEnergyModelContainer uan_src_energy;
	DeviceEnergyModelContainer uan_sink_energy;
	DeviceEnergyModelContainer buoy_wifi_energy;
	DeviceEnergyModelContainer buoy_uan_energy;
	double EnergyConsumSrc;
	double EnergyConsumSink;
    double EnergyConsum;
	double EnergyConsumBuoyWifi;
    double EnergyConsumBuoyUan;

	/* Result */
    std::map<uint32_t, double> SinkMap_IdRecvTime;
    std::map<uint32_t, double> SrcMap_IdSendTime; //in order to collect the end2end delay
	double m_totaldelay;
    double recvbitnum;
	double m_SinkRecvNum; //total packet of received.
	double calculatecount;
	uint32_t m_seqnum;//total packet of sended.

	double DeliveryRatio;
    double End2EndDelay;
    double Throughput;
    double pbconsumenergy; // bit num per J

    /* map */
    std::map<Ptr<Node>, Ptr<Socket> > m_srcsockets; //!< send and receive sockets
	std::map<Ptr<Node>, Ptr<Socket> > m_sinksockets; //!< send and receive sockets
	std::map<Ptr<Node>, Ptr<Socket> > m_buoyuansockets; //!< send and receive sockets
	std::map<Ptr<Node>, Ptr<Socket> > m_buoywifisockets; //!< send and receive sockets
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_srcSktAddr;
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_sinkSktAddr;
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_buoyWifiSktAddr;
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_buoyUanSktAddr;

	/* map of sub net */
	// std::map<Ptr<Node>, Ptr<Socket> > m_subsrcsockets; //!< send and receive sockets
	// std::map<Ptr<Node>, Ptr<Socket> > m_subsinksockets; //!< send and receive sockets
	// std::map<Ptr<Node>, Ptr<Socket> > m_subbuoyuansockets; //!< send and receive sockets
	// std::map<Ptr<Node>, Ptr<Socket> > m_subbuoywifisockets; //!< send and receive sockets
	// std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_subsrcSktAddr;
	// std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_subsinkSktAddr;
	// std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_subbuoyWifiSktAddr;
	// std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_subbuoyUanSktAddr;


public:
    ARCN();
    ~ARCN();
    /// @brief Configure simulation
    bool Configure (int argc, char **argv);

    /// @brief Run the Simulator
    void Run ();

private:
    /// @brief Create the nodes
    void CreateNodes ();

    /// @brief Set position and mobility model of node.
    void SetPosition();

    /// @brief Create the devices
    void CreateDevices(UanHelper uanHelper, YansWifiPhyHelper wifiPhy);

    /// @brief Create the network. (bind to Routing protocol)
    void InstallInternetStack ();

    /// @brief Create socket, used to send or receive packet.
    void SetupSocket();

    /// @brief Arrange the node sending process
    void NodeSendPacket ();

    void NodeSendPacket2 ();


    /// @brief The socket receive callback
    void ReceivePacket (Ptr<Socket> socket);

    /// @brief Send packet to destination by socket 
    void SendTo (Ptr<Socket> socket, Ptr<Packet> packet, Ipv4Address destination);

    /// @brief Calaulate the Network performance: Throughput End2endDelay DeliveryRatio EnergyUsage
    void Calculate();



};

//-----------------------------------------------------------------------------
ARCN::ARCN():
    totalTime(800),
    pcap (true),
    printRoutes (true),

    numUansrc(1),
    numUansink(2),
    numgtbuoy(1),
    m_uanMacType("ns3::UanMacAloha"),
    m_wifiPhyMode("OfdmRate6Mbps"),
    m_depth(-500),
    m_interval(20000),
    m_txPowerAcoustic(160),
    m_port(1234),
    m_pktSize(400),
    m_pktInterval(200),
    m_offset(81),
    m_Prss(-80),
    m_txPowerRadio(92), //radio--30km
    //m_step(1),
    m_totaldelay(0),
    m_SinkRecvNum(0),
    calculatecount(0),
    m_seqnum(0)
    //m_lambda(0.2),


{
}

ARCN::~ARCN()
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

bool
ARCN::Configure(int argc, char* argv[])
{

    //LogComponentEnable("AodvRoutingProtocol", LOG_LEVEL_ALL);

    // LogComponentEnable("vbfRoutingProtocol", LOG_LEVEL_ALL);
    // LogComponentEnable("Ipv4L3Protocol",LOG_LEVEL_ALL);
    // LogComponentEnable("arcn-protocol-test", LOG_LEVEL_ALL);
    LogComponentEnable("Ipv4L3Protocol", LOG_LEVEL_LOGIC);
    // LogComponentEnable("Ipv4Interface", LOG_LEVEL_ALL);
    LogComponentEnable("Ipv4StaticRouting", LOG_LEVEL_ALL);
	// LogComponentEnable("UanPhyGen", LOG_LEVEL_ALL);
	LogComponentEnableAll(LOG_PREFIX_TIME);
	LogComponentEnableAll(LOG_PREFIX_NODE);
	LogComponentEnableAll(LOG_PREFIX_FUNC);
	LogComponentEnableAll(LOG_PREFIX_LEVEL);
    //LogComponentEnable("UanMacAloha", LOG_LEVEL_ALL);
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
ARCN::Run() 
{
    CreateNodes ();
    SetPosition ();
    UanHelper uanHelper;
    YansWifiPhyHelper wifiPhy;
    CreateDevices (uanHelper,wifiPhy);
    InstallInternetStack ();
    SetupSocket();
    Simulator::Schedule(Seconds(50), &ARCN::NodeSendPacket, this);
    //关于uan模型默认是半双工的测试，如果phy层处在TX阶段，就没办法Rx；同理如果处于Rx阶段，就没办法Tx？
    // Simulator::Schedule (Seconds(100), &ARCN::NodeSendPacket2, this); //包会在210.33s到达处在Tx状态的节点


    // Simulator::Schedule(Seconds(300), &ARCN::NodeSendPacket, this);
    // //关于uan模型默认是半双工的测试，如果phy层处在TX阶段，就没办法Rx；同理如果处于Rx阶段，就没办法Tx？
    // Simulator::Schedule (Seconds(300-14), &ARCN::NodeSendPacket2, this); //包会在299.33s到达处在Tx状态的节点





    NS_LOG_UNCOND("Starting simulation for " << totalTime << "...");
    Simulator::Schedule(Seconds(200), &ARCN::Calculate, this);
    Simulator::Stop (Seconds (totalTime+1));
    Simulator::Run ();
    Simulator::Destroy ();
    NS_LOG_UNCOND("Simulation run "<<totalTime<<" seconds....\n");
}

void
ARCN::CreateNodes()
{
    NS_LOG_DEBUG("-----------Initializing Nodes-----------");
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

    // Name buoy gateway nodes
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
ARCN::SetPosition()
{
    NS_LOG_DEBUG("-----------Initializing Position-----------");
	MobilityHelper mobility;
	Ptr<ListPositionAllocator> nodesPositionAlloc = CreateObject<ListPositionAllocator> ();

    //set sink uan node position
    double x = 20000;

    //Set UanSrc node's position
    nodesPositionAlloc->Add (Vector(x,0, m_depth));
    nodesPositionAlloc->Add (Vector(2*x,0, m_depth));
    nodesPositionAlloc->Add (Vector(3*x,0, m_depth));


    //Set Buoy node's position
    nodesPositionAlloc->Add (Vector(200*x, 0, 0));

    mobility.SetPositionAllocator (nodesPositionAlloc);
    mobility.Install (uansrcNode);
    mobility.Install (uansinkNode);
    mobility.Install (gatewaybuoynode);
   
}



void
ARCN::CreateDevices(UanHelper uanHelper, YansWifiPhyHelper wifiPhy)
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
    
    //1.set Mac and Phy
    uanMode = UanTxModeFactory::CreateMode(UanTxMode::FSK, 2000,
            2000, 2500, 2000, 2, "Default mode");
    UanModesList myModes;
    myModes.AppendMode(uanMode);

    uanHelper.SetPhy("ns3::UanPhyGen", 
            "PerModel", PointerValue(phyPer),
            "SinrModel", PointerValue(phySinr), 
            "SupportedModes",UanModesListValue(myModes), 
            "TxPower",DoubleValue(m_txPowerAcoustic));
    uanHelper.SetMac(m_uanMacType);

    //2.set Channel
    Ptr<UanPropModelThorp> prop = CreateObject<UanPropModelThorp>();
    Ptr<UanNoiseModelDefault> noise = CreateObject<UanNoiseModelDefault> ();
    Ptr<UanChannel> uanChannel = CreateObject<UanChannel>();
    uanChannel->SetPropagationModel(prop);
    uanChannel->SetNoiseModel (noise);

    //3.Install
    uansrc_device = uanHelper.Install(uansrcNode, uanChannel);
    uansink_device = uanHelper.Install(uansinkNode, uanChannel);
    gateway_buoyuan_device = uanHelper.Install(gatewaybuoynode, uanChannel);

    NS_LOG_DEBUG("-----------Initializing Wireless-----------");
    WifiHelper wifiHelper;
    wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211a);
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
    wifiPhy = YansWifiPhyHelper::Default ();
    wifiPhy.Set ("RxGain", DoubleValue (0) );
    wifiPhy.Set ("TxGain", DoubleValue (m_offset + m_Prss)); //TxGain = 1dB
    wifiPhy.Set("TxPowerStart", DoubleValue(m_txPowerRadio));
    wifiPhy.Set("TxPowerEnd", DoubleValue(m_txPowerRadio));  //TxPower = m_txPowerRadio = 82dbm
    wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);

    wifiPhy.SetChannel (wifiChannel.Create());
    WifiMacHelper wifiMac;
    wifiMac.SetType ("ns3::AdhocWifiMac");
    wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (m_wifiPhyMode),
                                "ControlMode",StringValue (m_wifiPhyMode));

    gateway_buoywireless_device = wifiHelper.Install (wifiPhy, wifiMac, gatewaybuoynode);


    NS_LOG_DEBUG("-----------Initializing Energy-----------");
    BasicEnergySourceHelper energySourceHelper;
    energySourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (900000));
    AcousticModemEnergyModelHelper acousticEnergyHelper;

    NS_LOG_DEBUG("Set Uan Src Energy.");
    EnergySourceContainer uansrcSources = energySourceHelper.Install (uansrcNode);
    for (NodeContainer::Iterator n = uansrcNode.Begin (); n != uansrcNode.End (); ++n)
    {
        uansrcSources.Add ((*n)->GetObject<EnergySourceContainer> ()->Get (0));
    }
    uan_src_energy = acousticEnergyHelper.Install (uansrc_device, uansrcSources);

    NS_LOG_DEBUG("Set Uan Sink Energy.");
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
ARCN::InstallInternetStack()
{
    //--------------------------------------------aodv routing--------------------------------------------
    NS_LOG_DEBUG("-----------Initializing Internet-----------");
//     AodvHelper aodv;
// 	aodv.Set("HelloInterval", TimeValue(Seconds(1000)));
// //	aodv.Set("TimeoutBuffer", UintegerValue(2));
// 	aodv.Set("NodeTraversalTime", TimeValue(Seconds(2.5)));
// 	aodv.Set("NextHopWait", TimeValue(Seconds(2.51)));
// 	aodv.Set("ActiveRouteTimeout", TimeValue(Seconds(2000)));
// 	aodv.Set("MyRouteTimeout", TimeValue(Seconds(2000)));
// //	aodv.Set("BlackListTimeout", TimeValue(Seconds(5.6)));
// 	aodv.Set("DeletePeriod", TimeValue(Seconds(500)));
// //	aodv.Set("NetDiameter", UintegerValue(35));
// 	aodv.Set("NetTraversalTime", TimeValue(Seconds(100)));
// 	aodv.Set("PathDiscoveryTime", TimeValue(Seconds(100)));
// 	aodv.Set("TtlStart", UintegerValue(20));

    // vbfHelper vbf;

	InternetStackHelper stack;
	// stack.SetRoutingHelper (vbf);
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
				Seconds(40));
        (*m_uansrcNode)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->SetAliveTimeout(
            Seconds(1000));
                
		m_uansrcNode++;
	}

	NodeContainer::Iterator m_uansinkNode = uansinkNode.Begin();
	m_uansinkNode = uansinkNode.Begin();
	while (m_uansinkNode != uansinkNode.End()) {
		(*m_uansinkNode)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->SetWaitReplyTimeout(
				Seconds(40));
        (*m_uansinkNode)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->SetAliveTimeout(
                Seconds(1000));
		m_uansinkNode++;
	}

	NodeContainer::Iterator m_gatewaybuoynode = gatewaybuoynode.Begin();
	m_gatewaybuoynode = gatewaybuoynode.Begin();
	while (m_gatewaybuoynode != gatewaybuoynode.End())
	{
		(*m_gatewaybuoynode)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->SetWaitReplyTimeout(
				Seconds(40));
		m_gatewaybuoynode++;
	}


    Ipv4StaticRoutingHelper RoutingHelper;
    Ptr<OutputStreamWrapper> routingStream1 = Create<OutputStreamWrapper>(std::string("static10s.tr"),std::ios::out);
    RoutingHelper.PrintRoutingTableAt(Seconds(10.0),uansrcNode.Get(0),routingStream1);

    Ptr<OutputStreamWrapper> routingStream2 = Create<OutputStreamWrapper>(std::string("static100s.tr"),std::ios::out);
    RoutingHelper.PrintRoutingTableAt(Seconds(100.0),uansrcNode.Get(0),routingStream2);

	// if (printRoutes)
	// {
	// 	Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(
	// 			"arcn-aodv.routes", std::ios::out);
	// 	aodv.PrintRoutingTableAllAt(Seconds(1800), routingStream);
	// }
}


void
ARCN::SetupSocket()
{
    for(uint32_t i=0; i<uansrc_device.GetN(); i++)
    {
        Ptr<NetDevice> dev = uansrc_device.Get (i);
        Ptr<Node> srcnode = dev->GetNode ();
        Ptr<Ipv4> ipv4 = srcnode->GetObject<Ipv4> ();
        int32_t interfaceId = ipv4->GetInterfaceForDevice (dev);
        if (interfaceId == -1)
        {
        interfaceId = ipv4->AddInterface (dev);
        }
        Ipv4InterfaceAddress iface = ipv4->GetAddress(interfaceId,0);
        Ipv4Address ip_addr = iface.GetLocal();
        if (iface.GetLocal () == Ipv4Address ("127.0.0.1"))
        {
            return;
        }
        Ptr<Socket> cur_socket = Socket::CreateSocket(srcnode,
                                UdpSocketFactory::GetTypeId ());
        NS_ASSERT (cur_socket != 0);

        cur_socket->SetRecvCallback (MakeCallback (&ARCN::ReceivePacket, this));
        cur_socket->BindToNetDevice (dev);
        cur_socket->Bind (InetSocketAddress (ip_addr, m_port));
        cur_socket->SetAllowBroadcast (true);
        //cur_socket->SetIpRecvTtl (true);
        m_srcSktAddr.insert(std::make_pair(cur_socket,iface));
        m_srcsockets.insert(std::make_pair(srcnode,cur_socket));
        NS_LOG_DEBUG("Src Node Uan ID =" << srcnode->GetId() << " Socket =" <<cur_socket << " Ipv4Address " << ip_addr);
    }

    for(uint32_t i=0; i<uansink_device.GetN(); i++)
    {
        Ptr<NetDevice> dev = uansink_device.Get (i);
        Ptr<Node> sinknode = dev->GetNode ();
        Ptr<Ipv4> ipv4 = sinknode->GetObject<Ipv4> ();
        int32_t interfaceId = ipv4->GetInterfaceForDevice (dev);
        if (interfaceId == -1)
        {
        interfaceId = ipv4->AddInterface (dev);
        }
        Ipv4InterfaceAddress iface = ipv4->GetAddress(interfaceId,0);
        Ipv4Address ip_addr = iface.GetLocal();
        if (iface.GetLocal () == Ipv4Address ("127.0.0.1"))
        {
            return;
        }
        Ptr<Socket> cur_socket = Socket::CreateSocket(sinknode,
                                UdpSocketFactory::GetTypeId ());
        NS_ASSERT (cur_socket != 0);

        cur_socket->SetRecvCallback (MakeCallback (&ARCN::ReceivePacket, this));
        cur_socket->BindToNetDevice (dev);
        cur_socket->Bind (InetSocketAddress (ip_addr, m_port));
        cur_socket->SetAllowBroadcast (true);
        //cur_socket->SetIpRecvTtl (true);
        m_srcSktAddr.insert(std::make_pair(cur_socket,iface));
        m_srcsockets.insert(std::make_pair(sinknode,cur_socket));
        NS_LOG_DEBUG("Sink Node Uan ID =" << sinknode->GetId() << " Socket =" <<cur_socket << " Ipv4Address " << ip_addr);
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
        bw_socket->SetRecvCallback (MakeCallback (&ARCN::ReceivePacket, this));
        bw_socket->BindToNetDevice (dev);
        bw_socket->Bind (InetSocketAddress (ip_addr, m_port));
        bw_socket->SetAllowBroadcast (true);
        bw_socket->SetIpRecvTtl (true);
        m_buoyWifiSktAddr.insert (std::make_pair (bw_socket, iface));
        m_buoywifisockets.insert (std::make_pair (buoynode, bw_socket));
        NS_LOG_DEBUG("gt Node Wire ID =" << buoynode->GetId () << " Socket =" <<bw_socket << " Ipv4Address " << ip_addr);
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
            bu_socket->SetRecvCallback (MakeCallback (&ARCN::ReceivePacket, this));
            bu_socket->BindToNetDevice (dev);
            bu_socket->Bind (InetSocketAddress (ip_addr, m_port));
            bu_socket->SetAllowBroadcast (true);
            bu_socket->SetIpRecvTtl (true);
            m_buoyUanSktAddr.insert (std::make_pair (bu_socket, iface));
            m_buoyuansockets.insert (std::make_pair (buoynode, bu_socket));
            NS_LOG_DEBUG("gt Node Uan ID =" << buoynode->GetId () << " Socket =" <<bu_socket << " Ipv4Address " << ip_addr);

    }

}


void
ARCN::ReceivePacket (Ptr<Socket> socket)
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

    //	sendtime == SrcMap_IdSendTime[packet->GetUid()];
    double delta = Simulator::Now ().GetSeconds() - SrcMap_IdSendTime[packet->GetUid()];
    NS_LOG_DEBUG("Packet "<<packet->GetUid()<<" has been received, transmitted delay is  "<<std::setprecision(4)<<delta<<"s." );
    m_totaldelay =m_totaldelay+delta;
}


void
ARCN::NodeSendPacket()
{
    Ptr<Packet> packet = Create<Packet>(m_pktSize); 
    m_seqnum++;
    NS_LOG_DEBUG("Source Send Packet sequence number is "<<m_seqnum);
    NS_LOG_DEBUG("Packet "<<packet->GetUid()<<" has been sent");

    double sendtime = Simulator::Now().GetSeconds();
    SrcMap_IdSendTime[packet->GetUid()] = sendtime; 

    Ptr<Node> srcsend = uansrcNode.Get(0);
    Ptr<Socket> socket =  m_srcsockets[srcsend];
    SendTo(socket, packet, Ipv4Address ("10.1.1.2"));
//	erv = CreateObjectWithAttributes<ExponentialRandomVariable>("Mean",DoubleValue(1/m_lambda));
//	Simulator::Schedule (Seconds(erv->GetValue()), &AecnExample::NodeSendPacket, this);

    //1.关于aloha不能同时发两个包的测试，如果第一个包还在传播延时中，那么第二个包会被aloha直接抛弃
    // Simulator::Schedule (Seconds(m_pktInterval), &ARCN::NodeSendPacket, this);
    // Simulator::Schedule (Seconds(m_pktInterval+0.1), &ARCN::NodeSendPacket, this);



}

void
ARCN::NodeSendPacket2()
{
    Ptr<Packet> packet = Create<Packet>(m_pktSize);
    m_seqnum++;
    NS_LOG_DEBUG("Source Send Packet sequence number is "<<m_seqnum);
    NS_LOG_DEBUG("Packet "<<packet->GetUid()<<" has been sent");

    double sendtime = Simulator::Now().GetSeconds();
    SrcMap_IdSendTime[packet->GetUid()] = sendtime; 

    Ptr<Node> srcsend = uansinkNode.Get(0);
    Ptr<Socket> socket =  m_srcsockets[srcsend];
    SendTo(socket, packet, Ipv4Address ("10.1.1.1"));
}


void
ARCN::SendTo (Ptr<Socket> socket, Ptr<Packet> packet, Ipv4Address destination)
{
  socket->SendTo (packet, 0, InetSocketAddress (destination, m_port));
}


void
ARCN::Calculate()
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


	NS_LOG_INFO("-----------Collect the End to End Delay.-----------");
	End2EndDelay = m_totaldelay/m_SinkRecvNum;

	NS_LOG_INFO("-----------Collect the node delivery ratio.-----------");
	DeliveryRatio = (double(m_SinkRecvNum)) / double(m_seqnum);   //to bind

	NS_LOG_INFO("-----------Collect the network throughput-----------");
	recvbitnum = double(m_SinkRecvNum * m_pktSize * 8);
//	Throughput = double(m_SinkRecvNum * m_pktSize * 8)/ totalTime;
	Throughput = double(m_SinkRecvNum * m_pktSize * 8)/ (calculatecount*200);
	pbconsumenergy = recvbitnum/double(uan_src_consumed+uan_sink_consumed);

	NS_LOG_INFO("-----------Print the Network statistics-----------");
	NS_LOG_UNCOND("DeliveryRatio:"<<std::setprecision(4)<<DeliveryRatio
	<<" Throughput:"<<std::setprecision(4)<<Throughput
	<<" End2EndDelay:"<<std::setprecision(4)<<End2EndDelay
	<<" BitsPerJenergy:"<<std::setprecision(4)<<pbconsumenergy<<"." );


    //cauculate the network per 200s
	Simulator::Schedule (Seconds (200), &ARCN::Calculate, this);//print parameter per 200s

}



int main (int argc, char **argv)
{
  ARCN test;
  if (!test.Configure (argc, argv))
    NS_FATAL_ERROR ("Configuration failed. Aborted.");
  test.Run ();

  return 0;
}