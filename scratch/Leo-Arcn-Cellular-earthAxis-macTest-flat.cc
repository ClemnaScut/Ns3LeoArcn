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
#include "ns3/select-routing-helper.h"
#include "ns3/select-routing-module.h"
#include "ns3/node.h"
#include "ns3/leo-channel-helper.h"
#include "ns3/leo-module.h"

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
#include "ns3/uan-mac-srtdma.h"
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

NS_LOG_COMPONENT_DEFINE("Leo-Arcn");

class ARCN
{
//private:
    /* Simulator */
	double totalTime;  /// Simulation time, seconds
	bool pcap;  /// Write per-device PCAP traces if true
	bool printRoutes;  /// Print routes if true
	Ptr<UniformRandomVariable> m_uniformRandomVariable;
	Ptr<ExponentialRandomVariable> m_exponentiaflRandomVariable;
    double m_lambda;


    /* Node */
	uint32_t numUan;//number of uan node
	uint32_t numgtbuoy;//number of buoynode
    uint32_t numLeo; //number of leo

	NodeContainer uanNode;
	NodeContainer gatewaybuoyNode;
    NodeContainer leoNode;

	NetDeviceContainer uan_device;
	NetDeviceContainer gateway_buoywireless_device;
    NetDeviceContainer satellite_device;

	Ipv4InterfaceContainer uan_inter;
	Ipv4InterfaceContainer gateway_buoywireless_inter;
    Ipv4InterfaceContainer satellite_inter;

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
    uint32_t rng;



    /* EnergyConsum */
	DeviceEnergyModelContainer uan_energy;
	DeviceEnergyModelContainer buoy_wifi_energy;
	DeviceEnergyModelContainer buoy_uan_energy;
	double EnergyConsumUan;
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
    Time m_calculateInterval;

	double DeliveryRatio;
    double End2EndDelay;
    double Throughput;
    double pbconsumenergy; // bit num per J

    /* map */
    std::map<Ptr<Node>, Ptr<Socket> > m_Uansockets; //!< send and receive sockets
	std::map<Ptr<Node>, Ptr<Socket> > m_buoywifisockets; //!< send and receive sockets
	std::map<Ptr<Node>, Ptr<Socket> > m_satellitesockets; //!< send and receive sockets
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_UanSktAddr;
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_buoyWifiSktAddr;
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_satelliteSktAddr;

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

    /// @brief The socket receive callback
    void ReceivePacket (Ptr<Socket> socket);

    /// @brief Send packet to destination by socket 
    void SendTo (Ptr<Socket> socket, Ptr<Packet> packet, Ipv4Address destination);

    /// @brief Calaulate the Network performance: Throughput End2endDelay DeliveryRatio EnergyUsage
    void Calculate();



};

//-----------------------------------------------------------------------------
ARCN::ARCN():
    totalTime(120000),
    pcap (true),
    printRoutes (true),

    numUan(103),
    numgtbuoy(6),
    numLeo(1),
    // m_uanMacType("ns3::UanMacAloha"),
    m_uanMacType("ns3::UanMacSrTDMAQ"),
    // m_uanMacType("ns3::UanMacSrTDMA"),
    m_wifiPhyMode("OfdmRate6Mbps"),
    m_depth(-500),
    m_interval(20000),
    m_txPowerAcoustic(160),
    m_port(1234),
    m_pktSize(400),
    m_pktInterval(50),
    m_offset(81),
    m_Prss(-80),
    m_txPowerRadio(92), //radio--30km
    //m_step(1),
    rng(1),
    m_totaldelay(0),
    m_SinkRecvNum(0),
    calculatecount(0),
    m_seqnum(0),
    m_calculateInterval(Seconds(2000))
    // m_lambda(0.2)
{

}

ARCN::~ARCN()
{

  std::map<Ptr<Node>, Ptr<Socket> >::iterator socket;

  for (socket = m_Uansockets.begin (); socket != m_Uansockets.end (); socket++)
    {
      socket->second->Close ();
    }

  for (socket = m_buoywifisockets.begin (); socket != m_buoywifisockets.end (); socket++)
    {
      socket->second->Close ();
    }

  for (socket = m_satellitesockets.begin (); socket != m_satellitesockets.end (); socket++)
    {
      socket->second->Close ();
    }

}

bool
ARCN::Configure(int argc, char* argv[])
{

    // LogComponentEnable("AodvRoutingProtocol", LOG_LEVEL_ALL);

    // LogComponentEnable("SelectRoutingProtocol", LOG_LEVEL_DEBUG);
    // LogComponentEnable("UdpL4Protocol", LOG_LEVEL_ALL);
    // LogComponentEnable("UanMacAloha", LOG_LEVEL_ALL);
    // LogComponentEnable("UanMacSrTDMAQ", LOG_LEVEL_ALL);
    // LogComponentEnable("Ipv4L3Protocol",LOG_LEVEL_ALL);
    // LogComponentEnable("Leo-Arcn", LOG_LEVEL_DEBUG);
    // LogComponentEnable("LeoGndNodeHelper",LOG_LEVEL_INFO);
    // LogComponentEnable("LeoPropagationLossModel",LOG_LEVEL_INFO);
    // LogComponentEnable("MockNetDevice", LOG_LEVEL_DEBUG);
    // LogComponentEnable("Ipv4L3Protocol", LOG_LEVEL_INFO);


	// LogComponentEnable("UanPhyGen", LOG_LEVEL_ALL);
    // LogComponentEnable("UanChannel", LOG_LEVEL_ALL);
	LogComponentEnableAll(LOG_PREFIX_TIME);
	LogComponentEnableAll(LOG_PREFIX_NODE);
	LogComponentEnableAll(LOG_PREFIX_FUNC);
	LogComponentEnableAll(LOG_PREFIX_LEVEL);
    // LogComponentEnable("UanMacAloha", LOG_LEVEL_ALL);
	// SeedManager::SetSeed(12345);
	CommandLine cmd;

    cmd.AddValue("rng","Number of rng",rng);   
	cmd.AddValue("pcap", "Write PCAP traces.", pcap);
	cmd.AddValue("printRoutes", "Print routing table dumps.", printRoutes);
//	cmd.AddValue("uansrcNode", "Number of nodes.", uansrcNode);
//	cmd.AddValue("numBuoy", "Number of nodes.", numBuoy);
	cmd.AddValue("time", "Simulation time, s.", totalTime);
//	cmd.AddValue("step", "Grid step, m", step);
    RngSeedManager::SetSeed (12345); // Changes seed from default of 1 to 3
    RngSeedManager::SetRun (rng);
	cmd.Parse(argc, argv);

    m_uniformRandomVariable = CreateObject<UniformRandomVariable>();
    m_exponentiaflRandomVariable = CreateObjectWithAttributes<ExponentialRandomVariable>("Mean",DoubleValue(1)); 
	return true;
}

void
ARCN::Run() 
{

    std::cout << "Starting simulation for " << totalTime << " s ...\n";
    CreateNodes ();
    SetPosition ();
    UanHelper uanHelper;
    YansWifiPhyHelper wifiPhy;
    CreateDevices (uanHelper,wifiPhy);
    InstallInternetStack ();
    SetupSocket();
    Simulator::Schedule(Seconds(200), &ARCN::NodeSendPacket, this);
    // Simulator::Schedule(Seconds(300), &ARCN::NodeSendPacket, this);


    NS_LOG_UNCOND("Simulation run "<<totalTime<<" seconds....\n");
    Simulator::Schedule(m_calculateInterval, &ARCN::Calculate, this);

    Simulator::Stop (Seconds (totalTime));
    Simulator::Run ();
    Simulator::Destroy ();
}

void
ARCN::CreateNodes()
{
    NS_LOG_DEBUG("-----------Initializing Nodes-----------");
    NS_LOG_INFO("Create Uan and Buoy Node");
	uanNode.Create(numUan);
    gatewaybuoyNode.Add(uanNode.Get(25));
    gatewaybuoyNode.Add(uanNode.Get(28));
    gatewaybuoyNode.Add(uanNode.Get(31));
    gatewaybuoyNode.Add(uanNode.Get(71));
    gatewaybuoyNode.Add(uanNode.Get(74));
    gatewaybuoyNode.Add(uanNode.Get(77));

    NS_LOG_INFO("Create Leo Node");
    leoNode.Create(numLeo);



	// Name uan nodes
	for (uint32_t i = 0; i < numUan; ++i)
	{
		std::ostringstream os;
		os << "uan or buoy Node-" << i;
		Names::Add(os.str(), uanNode.Get(i));
	}

    // Name satellite nodes
	for (uint32_t i = 0; i < numLeo; ++i)
	{
		std::ostringstream os;
		os << "LeoNode-" << i;
		Names::Add (os.str (), leoNode.Get (i));
	}

}


void
ARCN::SetPosition()
{
        NS_LOG_DEBUG("-----------Initializing Position-----------");
	MobilityHelper mobility;
	Ptr<ListPositionAllocator> nodesPositionAlloc = CreateObject<ListPositionAllocator> ();

    double x = m_interval/2;
    double y = m_interval*sqrt(3)/2;

    //Set Uan node's position
    for(int n=0; n<9; n++)
    {
        if(n%2 == 0)
        {
            for(int m=0; m<11; m++)
            {
                nodesPositionAlloc->Add (Vector((1+2*m)*x, n*y, -1000));
            }
        }
        else
        {
            for(int m=0; m<12; m++)
            {
                nodesPositionAlloc->Add (Vector((2*m)*x, n*y, -1000));
            }
        }
    }


    //Set Buoy node's position

    //Set Leo Node's position
    nodesPositionAlloc->Add (Vector(11*x, 4*y, 2000000));

    mobility.SetPositionAllocator (nodesPositionAlloc);
    mobility.Install (uanNode);
    mobility.Install (leoNode);
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
    
    //1.set Mac and Phy
    uanMode = UanTxModeFactory::CreateMode(UanTxMode::FSK, 11000,
            11000, 2500, 2000, 2, "Default mode");
    UanModesList myModes;
    myModes.AppendMode(uanMode);
    uanHelper.SetPhy("ns3::UanPhyGen", 
            "PerModel", PointerValue(phyPer),
            "SinrModel", PointerValue(phySinr), 
            "SupportedModes",UanModesListValue(myModes), 
            "TxPower",DoubleValue(m_txPowerAcoustic));
    // double m_tFrame = m_interval*1.1/1500 + (5+1+3)*8/2000;
    uanHelper.SetMac(m_uanMacType);
    //2.set Channel
    Ptr<UanPropModelThorp> prop = CreateObject<UanPropModelThorp>();
    Ptr<UanNoiseModelDefault> noise = CreateObject<UanNoiseModelDefault> ();
    Ptr<UanChannel> uanChannel = CreateObject<UanChannel>();
    uanChannel->SetPropagationModel(prop);
    uanChannel->SetNoiseModel (noise);

    //3.Install
    uan_device = uanHelper.Install(uanNode, uanChannel);

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

    gateway_buoywireless_device = wifiHelper.Install (wifiPhy, wifiMac, gatewaybuoyNode);


    NS_LOG_DEBUG("-----------Initializing Saltellite-----------");
    LeoChannelHelper utCh;
    satellite_device = utCh.Install (leoNode, gatewaybuoyNode);

    NS_LOG_DEBUG("-----------Initializing Energy-----------");
    BasicEnergySourceHelper energySourceHelper;
    energySourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (900000));


    NS_LOG_DEBUG("Set Uan Energy.");
    EnergySourceContainer uanSources = energySourceHelper.Install (uanNode);
    AcousticModemEnergyModelHelper acousticEnergyHelper;
    for (NodeContainer::Iterator n = uanNode.Begin (); n != uanNode.End (); ++n)
    {
        uanSources.Add ((*n)->GetObject<EnergySourceContainer> ()->Get (0));
    }
    uan_energy = acousticEnergyHelper.Install (uan_device, uanSources);



    NS_LOG_DEBUG("Set Buoy Wifi Energy.");
    EnergySourceContainer buoywifiSources = energySourceHelper.Install (gatewaybuoyNode);
    WifiRadioEnergyModelHelper radioEnergyHelper;
    // configure radio energy model
    radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));
    // install device model
    buoy_wifi_energy = radioEnergyHelper.Install (gateway_buoywireless_device, buoywifiSources);


    NS_LOG_DEBUG("Set Buoy Satellite Energy.");
    //??好像没有实现这个

    NS_LOG_DEBUG("Set Leo Satellite Energy.");
    //??
}

void
ARCN::InstallInternetStack()
{
    //--------------------------------------------aodv routing--------------------------------------------
    NS_LOG_DEBUG("-----------Initializing Internet-----------");
    AodvHelper aodv;
	aodv.Set("HelloInterval", TimeValue(Seconds(1000))); //interval of sending Hello message
	aodv.Set("TimeoutBuffer", UintegerValue(2));
	aodv.Set("NodeTraversalTime", TimeValue(Seconds(15))); //one hop traversal time for packets
	aodv.Set("NextHopWait", TimeValue(Seconds(15.01))); 
	aodv.Set("ActiveRouteTimeout", TimeValue(Seconds(2000))); // time while the router is considered to be vaild
	aodv.Set("MyRouteTimeout", TimeValue(Seconds(2000)));
	aodv.Set("BlackListTimeout", TimeValue(Seconds(5.6)));
	aodv.Set("DeletePeriod", TimeValue(Seconds(500)));
	aodv.Set("NetDiameter", UintegerValue(20));
	aodv.Set("NetTraversalTime", TimeValue(Seconds(200)));
	aodv.Set("PathDiscoveryTime", TimeValue(Seconds(200)));
	aodv.Set("TtlStart", UintegerValue(15));

    vbfHelper vbf;
    selectRoutingHelper select;


	InternetStackHelper stack;
	stack.SetRoutingHelper (select);
	stack.Install (uanNode);
	stack.Install (leoNode);

	Ipv4AddressHelper address;
	address.SetBase("10.1.1.0", "255.255.255.0");
	NS_LOG_DEBUG ("Assign Uan IP Addresses.");
	uan_inter = address.Assign(uan_device);

	NS_LOG_DEBUG ("Assign Buoy Buoywifi IP Addresses.");
	address.SetBase("10.1.2.0", "255.255.255.0");
	gateway_buoywireless_inter = address.Assign(gateway_buoywireless_device);

	address.SetBase("10.1.3.0", "255.255.255.0");
	NS_LOG_DEBUG ("Assign Satellite IP Addresses.");
	satellite_inter = address.Assign(satellite_device);
    //satellite:10.1.3.1 //buoy:10.1.3.2-10.1.3.7

	//change arp cache wait time.
	// NodeContainer::Iterator m_uanNode = uanNode.Begin();
	// m_uanNode = uanNode.Begin();
	// while (m_uanNode != uanNode.End()) {
	// 	(*m_uanNode)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->SetWaitReplyTimeout(
	// 			Seconds(40));
	// 	m_uanNode++;
	// }

	// NodeContainer::Iterator m_gatewaybuoyNode = gatewaybuoyNode.Begin();
	// m_gatewaybuoyNode = gatewaybuoyNode.Begin();
	// while (m_gatewaybuoyNode != gatewaybuoyNode.End())
	// {
	// 	(*m_gatewaybuoyNode)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->SetWaitReplyTimeout(
	// 			Seconds(40));
	// 	m_gatewaybuoyNode++;
	// }

    // NodeContainer::Iterator m_leo = leoNode.Begin();
	// m_leo = leoNode.Begin();
	// while (m_leo != leoNode.End())
	// {
	// 	(*m_leo)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->SetWaitReplyTimeout(
	// 			Seconds(40));
	// 	m_leo++;
	// }

	if (printRoutes)
	{
		Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(
				"arcn-aodv-danbo.routes", std::ios::out);
		aodv.PrintRoutingTableAllAt(Seconds(294), routingStream);
	}
}


void
ARCN::SetupSocket()
{
     for(uint32_t i=0; i<uan_device.GetN(); i++)
    {
        Ptr<NetDevice> dev = uan_device.Get (i);
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
            return;
        }
        Ptr<Socket> cur_socket = Socket::CreateSocket(uannode,
                                UdpSocketFactory::GetTypeId ());
        NS_ASSERT (cur_socket != 0);

        cur_socket->SetRecvCallback (MakeCallback (&ARCN::ReceivePacket, this));
        cur_socket->BindToNetDevice (dev);
        cur_socket->Bind (InetSocketAddress (ip_addr, m_port));
        cur_socket->SetAllowBroadcast (true);
        //cur_socket->SetIpRecvTtl (true);
        m_UanSktAddr.insert(std::make_pair(cur_socket,iface));
        m_Uansockets.insert(std::make_pair(uannode,cur_socket));
        NS_LOG_DEBUG("Uan Node Uan ID =" << uannode->GetId() << " Socket =" <<cur_socket << " Ipv4Address " << ip_addr);
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


    for (uint32_t i = 0; i < satellite_device.GetN (); ++i)
    {

        //uan device
        Ptr<NetDevice> dev = satellite_device.Get (i);

        Ptr<Node> satellitenode = dev->GetNode ();
        Ptr<Ipv4> ipv4 = satellitenode->GetObject<Ipv4> ();
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
        Ptr<Socket> st_socket = Socket::CreateSocket (satellitenode,
                                                    UdpSocketFactory::GetTypeId ());
        NS_ASSERT (st_socket != 0);
        st_socket->SetRecvCallback (MakeCallback (&ARCN::ReceivePacket, this));
        st_socket->BindToNetDevice (dev);
        st_socket->Bind (InetSocketAddress (ip_addr, m_port));
        st_socket->SetAllowBroadcast (true);
        st_socket->SetIpRecvTtl (true);
        m_satelliteSktAddr.insert (std::make_pair (st_socket, iface));
        m_satellitesockets.insert (std::make_pair (satellitenode, st_socket));
        NS_LOG_DEBUG("Satellite Node ID =" << satellitenode->GetId () << " Socket =" <<st_socket << " Ipv4Address " << ip_addr);

    }
}


void
ARCN::ReceivePacket (Ptr<Socket> socket)
{
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
    NS_LOG_DEBUG("Packet "<<packet->GetUid()<<" has been received, transmitted delay is  "<<std::setprecision(4)<<delta<<"s.");
    m_totaldelay =m_totaldelay+delta;
}


void
ARCN::NodeSendPacket()
{
    Ptr<Packet> packet = Create<Packet>(m_pktSize);
    m_seqnum++;
    double sendtime = Simulator::Now().GetSeconds();
    SrcMap_IdSendTime[packet->GetUid()] = sendtime; 


    Ptr<Node> srcsend = uanNode.Get(m_uniformRandomVariable->GetValue(0, 102));
    Ptr<Socket> srcsocket =  m_Uansockets[srcsend];
    Ipv4Address destAddr = uan_inter.GetAddress(m_uniformRandomVariable->GetValue(0, 102),0);
    SendTo(srcsocket, packet, destAddr); 
    NS_LOG_DEBUG("Packet "<<packet->GetUid()<<" has been sent from "<< m_UanSktAddr[srcsocket].GetLocal() << 
                 " to " << destAddr);
    Time nextSend = Seconds(m_exponentiaflRandomVariable->GetValue());
    // Time nextSend = Seconds(100);
    NS_LOG_DEBUG("next Packet will be sent after " << nextSend.GetSeconds() << "s.");
    Simulator::Schedule(nextSend, &ARCN::NodeSendPacket, this);


    // if(Simulator::Now().GetSeconds() < 12000)
    // {
    // Simulator::Schedule(nextSend, &ARCN::NodeSendPacket, this);
    //     NS_LOG_DEBUG("next Packet will be sent after " << nextSend.GetSeconds() << "s.");
    // }
    // else
    // {
    //     NS_LOG_DEBUG("Stop sending.");
    // }


    // Ptr<Node> srcsend = uanNode.Get(92);
    // Ptr<Socket> srcsocket =  m_Uansockets[srcsend];
    // Ipv4Address destAddr = uan_inter.GetAddress(94,0);
    // SendTo(srcsocket, packet, destAddr); 

    // Time nextSend = Seconds(100);
    // NS_LOG_DEBUG("next Packet will be sent after " << nextSend.GetSeconds() << "s.");
    // Simulator::Schedule(nextSend, &ARCN::NodeSendPacket, this);

    // NS_LOG_DEBUG("Packet "<<packet->GetUid()<<" has been sent from "<< srcsend->GetId() << 
    //              " to " << destAddr);  
    // Vector pos1 = NodeList::GetNode(18)->GetObject<MobilityModel>()->GetPosition();
    // Vector pos2 = NodeList::GetNode(29)->GetObject<MobilityModel>()->GetPosition();

    // packet = Create<Packet>(m_pktSize);
    // m_seqnum++;
    // srcsend = uanNode.Get(18);
    // srcsocket =  m_Uansockets[srcsend];
    // destAddr = uan_inter.GetAddress(29,0);
    // SendTo(srcsocket, packet, destAddr); 
    // NS_LOG_DEBUG("Packet "<<packet->GetUid()<<" has been sent from "<< srcsend->GetId() << 
    //              " to " << destAddr);  

    // packet = Create<Packet>(m_pktSize);
    // m_seqnum++;
    // srcsend = uanNode.Get(31);
    // srcsocket =  m_Uansockets[srcsend];
    // destAddr = uan_inter.GetAddress(20,0);
    // SendTo(srcsocket, packet, destAddr); 
    // NS_LOG_DEBUG("Packet "<<packet->GetUid()<<" has been sent from "<< srcsend->GetId() << 
    //              " to " << destAddr);  

}


void
ARCN::SendTo (Ptr<Socket> socket, Ptr<Packet> packet, Ipv4Address destination)
{
  socket->SendTo (packet, 0, InetSocketAddress (destination, m_port));
}


void
ARCN::Calculate()
{
	EnergyConsum = 0;  //Uan energy consumption.
	calculatecount++;

	double uan_consumed;
	double buoy_uan_consumed;
	double buoy_wifi_consumed;

	NS_LOG_INFO("-----------Collect the node energy consumption.-----------");

	for (DeviceEnergyModelContainer::Iterator iter = uan_energy.Begin (); iter != uan_energy.End (); ++iter)
    {
		uan_consumed += (*iter)->GetTotalEnergyConsumption ();
	}
	for (DeviceEnergyModelContainer::Iterator iter = buoy_uan_energy.Begin (); iter != buoy_uan_energy.End (); ++iter)
    {
		buoy_uan_consumed += (*iter)->GetTotalEnergyConsumption ();
	}
	for (DeviceEnergyModelContainer::Iterator iter = buoy_wifi_energy.Begin (); iter != buoy_wifi_energy.End (); ++iter)
    {
		buoy_wifi_consumed += (*iter)->GetTotalEnergyConsumption ();
	}
    NS_LOG_INFO("uan_consumed:" << uan_consumed << ". buoyUan_consumed:" <<buoy_uan_consumed 
                << ". buoyWifi_consumed:" <<buoy_wifi_consumed );
	EnergyConsum = uan_consumed + buoy_uan_consumed + buoy_wifi_consumed;
    //拿到的是总的能量消耗，所以算效率也需要用总发送接受bit来计算




	NS_LOG_INFO("-----------Collect the End to End Delay.-----------");
	End2EndDelay = m_totaldelay/m_SinkRecvNum;

	NS_LOG_INFO("-----------Collect the node delivery ratio.-----------");
	DeliveryRatio = (double(m_SinkRecvNum)) / double(m_seqnum);   //to bind

	NS_LOG_INFO("-----------Collect the network throughput-----------");
	recvbitnum = double(m_SinkRecvNum * m_pktSize * 8);
//	Throughput = double(m_SinkRecvNum * m_pktSize * 8)/ totalTime;
	Throughput = double(m_SinkRecvNum * m_pktSize * 8)/ (calculatecount*m_calculateInterval.GetSeconds());
	pbconsumenergy = recvbitnum/double(EnergyConsum);

	NS_LOG_INFO("-----------Print the Network statistics-----------");
	NS_LOG_UNCOND("DeliveryRatio:"<<std::setprecision(4)<<DeliveryRatio
	<<" Throughput:"<<std::setprecision(4)<<Throughput
	<<" End2EndDelay:"<<std::setprecision(4)<<End2EndDelay
	<<" BitsPerJenergy:"<<std::setprecision(4)<<pbconsumenergy<<"." );


    //cauculate the network per 200s
	Simulator::Schedule (m_calculateInterval, &ARCN::Calculate, this);//print parameter per 200s

}



int main (int argc, char **argv)
{
  ARCN test;

  if (!test.Configure (argc, argv))
    NS_FATAL_ERROR ("Configuration failed. Aborted.");

  test.Run ();

  return 0;
}