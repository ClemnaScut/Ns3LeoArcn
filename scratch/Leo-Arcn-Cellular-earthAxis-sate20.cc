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
#include "ns3/uan-mac-sfama.h"
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
NS_LOG_COMPONENT_DEFINE("Leo-Arcn-Earth-Sate");


struct LonAndLat
{
    double lon; //经度
    double lat; //维度
};


//通过某一点的经纬度lon,lat计算另一点的经纬度,x为向右多少个间隔,y为向下多少个间隔，distance为间隔距离
LonAndLat CalculateLL(double lon, double lat, int x, int y, double dstx, double dsty)
{
    double arc = 6371.393 * 1000;
    // cout << sin(a*2*M_PI/ 360) << " " << cos(lat/180) << endl;
    // cout << dst * sin(a*2*M_PI/ 360) / (arc * cos(lat/180) * 2 * M_PI/ 360) << endl;

    lon += dstx * x / (arc * cos(lat/180) * 2 * M_PI/ 360);
    lat += dsty * y / (arc * 2 * M_PI / 360);
    LonAndLat newll;
    newll.lat = lat;
    newll.lon = lon;

    return newll;
}


//根据三维坐标计算该点的经纬度
LonAndLat position2LL(Vector position)
{
    double LatSource = atan(position.z/(sqrt(position.x*position.x + position.y*position.y)))/(M_PI / 180);
    double LonSource = atan(position.y / position.x)/(M_PI / 180);
    if(position.x<0 && position.y>0) LonSource+=180;
    if(position.x<0 && position.y<0) LonSource-=180;
    // NS_LOG_DEBUG("LatSource: " << LatSource << " LonSource " << LonSource );
    LonAndLat retll;
    retll.lat = LatSource;
    retll.lon = LonSource;

    return retll;
}


//计算卫星的三维坐标
Vector
getLeoPosition (LonAndLat leoll, double height)
{
  double lat = leoll.lat * (M_PI / 180);
  double lon = leoll.lon * (M_PI / 180);
  // Vector3D pos = Vector3D (LEO_GND_RAD_EARTH * sin (lat) * cos (lon),
  // 			   LEO_GND_RAD_EARTH * sin (lat) * sin (lon),
  // 			   LEO_GND_RAD_EARTH * cos (lat));

  //修改版
  //以北纬为正，以东经为正　　（即处在北纬0-90°＋东经0-90°的视为八个象限中的第一象限）
  Vector pos = Vector((LEO_GND_RAD_EARTH+height*1000) * cos (lat) * cos (lon),
  			   (LEO_GND_RAD_EARTH+height*1000) * cos (lat) * sin (lon),
  			   (LEO_GND_RAD_EARTH+height*1000) * sin (lat));

  return pos;
}

//卫星的运行轨迹
void CourseChange (std::string context, Ptr<const MobilityModel> position)
{
    if(int(Simulator::Now().GetSeconds())%1000==0)
    {
    Vector pos = position->GetPosition ();
    Ptr<Node> node = position->GetObject<Node> ();
    // std::cout << Simulator::Now () << " " << node->GetId () << " " << pos.x << " " << pos.y << " " << pos.z << " " << position->GetVelocity ().GetLength() << std::endl;
    double LatSource = atan(pos.z/(sqrt(pos.x*pos.x + pos.y*pos.y)))/(M_PI / 90);
    double LonSource = atan(pos.y / pos.x)/(M_PI / 180);
    if(pos.x<0 && pos.y>0) LonSource+=180;
    if(pos.x<0 && pos.y<0) LonSource-=180;
    NS_LOG_UNCOND("time:" << Simulator::Now().GetSeconds() << " SatelliteId "<< node->GetId() << " LatSource: " << LatSource << " LonSource " << LonSource );
    };

}



class ARCN
{
//private:
    /* Simulator */
	double totalTime;  /// Simulation time, seconds
	bool pcap;  /// Write per-device PCAP traces if true
	bool printRoutes;  /// Print routes if true
	Ptr<UniformRandomVariable> m_uniformRandomVariable;
	Ptr<ExponentialRandomVariable> m_exponentialRandomVariable;
    // double m_lambda;


    /* Node */
	uint32_t numUan;//number of uan node
	uint32_t numgtbuoy;//number of buoynode
    uint32_t numLeo; //number of leo

	NodeContainer uanNode;
	NodeContainer gatewaybuoyNode;
    NodeContainer leoNode;

    NodeContainer UanSrcNode; //chosen in uanNode

	NetDeviceContainer uan_device;
	NetDeviceContainer gateway_buoywireless_device;
	NetDeviceContainer gateway_buoyuan_device;
    // NetDeviceContainer gateway_satellite_device;
    NetDeviceContainer satellite_device;
    NetDeviceContainer satellite_isl_device; 

	Ipv4InterfaceContainer uan_inter;
	Ipv4InterfaceContainer gateway_buoywireless_inter;
	Ipv4InterfaceContainer gateway_buoyuan_inter;
    Ipv4InterfaceContainer gateway_satellite_inter;
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

	double DeliveryRatio;
    double End2EndDelay;
    double Throughput;
    double pbconsumenergy; // bit num per J

    /* map */
    std::map<Ptr<Node>, Ptr<Socket> > m_Uansockets; //!< send and receive sockets
	std::map<Ptr<Node>, Ptr<Socket> > m_buoyuansockets; //!< send and receive sockets
	std::map<Ptr<Node>, Ptr<Socket> > m_buoywifisockets; //!< send and receive sockets
	std::map<Ptr<Node>, Ptr<Socket> > m_satellitesockets; //!< send and receive sockets
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_UanSktAddr;
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_buoyWifiSktAddr;
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_buoyUanSktAddr;
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
    totalTime(1000),
    pcap (true),
    printRoutes (true),

    numUan(103),
    numgtbuoy(6),
    numLeo(20),
    m_uanMacType("ns3::UanMacAloha"),
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
    m_totaldelay(0),
    m_SinkRecvNum(0),
    calculatecount(0),
    m_seqnum(0)
    //m_lambda(0.2),
{
    m_uniformRandomVariable = CreateObject<UniformRandomVariable>();
    m_exponentialRandomVariable = CreateObjectWithAttributes<ExponentialRandomVariable>("Mean",DoubleValue(20));
}

ARCN::~ARCN()
{

  std::map<Ptr<Node>, Ptr<Socket> >::iterator socket;

  for (socket = m_Uansockets.begin (); socket != m_Uansockets.end (); socket++)
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

  for (socket = m_satellitesockets.begin (); socket != m_satellitesockets.end (); socket++)
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
    // LogComponentEnable("Leo-Arcn-Earth-Sate", LOG_LEVEL_DEBUG);
    // // LogComponentEnable("LeoGndNodeHelper",LOG_LEVEL_INFO);
    // LogComponentEnable("LeoPropagationLossModel",LOG_LEVEL_INFO);
    // LogComponentEnable("MockNetDevice", LOG_LEVEL_DEBUG);
    // LogComponentEnable("Ipv4L3Protocol", LOG_LEVEL_INFO);
    // LogComponentEnable("UanMacAloha", LOG_LEVEL_ALL);


	// LogComponentEnable("UanPhyGen", LOG_LEVEL_ALL);
    // LogComponentEnable("UanMacSFAMA", LOG_LEVEL_ALL);
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
ARCN::Run() 
{
    CreateNodes ();
    SetPosition ();
    UanHelper uanHelper;
    YansWifiPhyHelper wifiPhy;
    CreateDevices (uanHelper,wifiPhy);
    InstallInternetStack ();
    SetupSocket();
    Simulator::Schedule(Seconds(20), &ARCN::NodeSendPacket, this);

    std::cout << "Starting simulation for " << totalTime << " s ...\n";
    NS_LOG_UNCOND("Simulation run "<<totalTime<<" seconds....\n");
    Simulator::Schedule(Seconds(200), &ARCN::Calculate, this);

    Simulator::Stop (Seconds (totalTime+1));
    Simulator::Run ();
    Simulator::Destroy ();
}

void
ARCN::CreateNodes()
{
    NS_LOG_DEBUG("-----------Initializing Nodes-----------");
	// uanNode.Create(numUan);
    double lat_o = 10;
    double lon_o = 0;
    double lat_1 = CalculateLL(lon_o,lat_o,1,1,-10000,-10000*sqrt(3)).lat;
    double lon_1 = CalculateLL(lon_o,lat_o,1,1,-10000,-10000*sqrt(3)).lon;


    LeoGndNodeHelper ground;
    LonAndLat groundll;
    NS_LOG_INFO("The Gnd Node");
    for(int i=0; i<5; i++)
    {
        for(int j=0; j<11; j++)
        {
            groundll = CalculateLL(lon_o,lat_o,j,i,20000,-20000*sqrt(3));
            LeoLatLong station(groundll.lat, groundll.lon);
            ground.Install(uanNode,station,0);

        }

        for(int j=0; j<12 && i<4; j++)
        {
            groundll = CalculateLL(lon_1,lat_1,j,i,20000,-20000*sqrt(3));
            LeoLatLong station(groundll.lat, groundll.lon);
            ground.Install(uanNode,station,0);
        }    
    }



	// gatewaybuoyNode.Create(numgtbuoy);
    NS_LOG_INFO("The Buoy Node");
    double lat_buoy = CalculateLL(lon_o,lat_o,2,1,20000,-20000*sqrt(3)).lat;
    double lon_buoy = CalculateLL(lon_o,lat_o,2,1,20000,-20000*sqrt(3)).lon;
    LeoGndNodeHelper buoy;
    LonAndLat buoyll;
    for(int i=0; i<2; i++)
    {
        for(int j=0; j<3; j++)
        {
            buoyll = CalculateLL(lon_buoy,lat_buoy,3*j,2*i,20000,-20000*sqrt(3));
            LeoLatLong buoystation(buoyll.lat, buoyll.lon);
            buoy.Install(gatewaybuoyNode,buoystation,1);

        } 
    }

    // NS_LOG_DEBUG("   " << (uanNode.Get(25)->GetObject<MobilityModel>()->GetPosition() - gatewaybuoyNode.Get(0)->GetObject<MobilityModel>()->GetPosition()).GetLength());


    NS_LOG_INFO("The Leo Node");
    // leoNode.Create(1);
	// MobilityHelper mobility;
	// Ptr<ListPositionAllocator> nodesPositionAlloc = CreateObject<ListPositionAllocator> ();
    // LonAndLat leoll = {180,89.37697};
    // LonAndLat leoll = {0.900656,9.37697};
    // nodesPositionAlloc->Add (getLeoPosition(leoll,1200));

    // mobility.SetPositionAllocator (nodesPositionAlloc);
    // mobility.Install (leoNode);

    LeoOrbitNodeHelper orbit;
    leoNode = orbit.Install ( LeoOrbit (1200, 20, 1, 1));
    Vector Psource = leoNode.Get(0)->GetObject<MobilityModel>()->GetPosition();
    double LatSource = atan(Psource.z/(sqrt(Psource.x*Psource.x + Psource.y*Psource.y)))/(M_PI / 90);
    double LonSource = atan(Psource.y / Psource.x)/(M_PI / 180);
    if(Psource.x<0 && Psource.y>0) LonSource+=180;
    if(Psource.x<0 && Psource.y<0) LonSource-=180;
    NS_LOG_UNCOND("Satellite Node Id: " << 0);
    NS_LOG_UNCOND("LatSource: " << LatSource << " LonSource " << LonSource );
    NS_LOG_UNCOND("Position: " << Psource);

    ostringstream osscb1;
    osscb1 << "/NodeList/" << leoNode.Get(0)->GetId() <<"/$ns3::MobilityModel/CourseChange";
    Config::Connect (osscb1.str(), MakeCallback (&CourseChange));
    // osscb2 << "/NodeList/" << leoNode.Get(1)->GetId() <<"/$ns3::MobilityModel/CourseChange";
    // Config::Connect (osscb2.str(), MakeCallback (&CourseChange));
    // osscb3 << "/NodeList/" << leoNode.Get(2)->GetId() <<"/$ns3::MobilityModel/CourseChange";
    // Config::Connect (osscb3.str(), MakeCallback (&CourseChange));

	// Name uan nodes
	for (uint32_t i = 0; i < numUan; ++i)
	{
		std::ostringstream os;
		os << "uanNode-" << i;
		Names::Add(os.str(), uanNode.Get(i));
	}

    // Name buoy gateway nodes
	for (uint32_t i = 0; i < numgtbuoy; ++i)
	{
		std::ostringstream os;
		os << "BuoygatewayNode-" << i;
		Names::Add (os.str (), gatewaybuoyNode.Get (i));
	}

    // Name satellite nodes
	for (uint32_t i = 0; i < leoNode.GetN(); ++i)
	{
		std::ostringstream os;
		os << "LeoNode-" << i;
		Names::Add (os.str (), leoNode.Get (i));
	}


    for(int n=12; n<=21; n+=3)
    {
        for(int m=0; m<=3; m++)
        {
            NS_LOG_DEBUG("------Add Node " << uanNode.Get(n+23*m)->GetId() << " to SrcNode");
            UanSrcNode.Add(uanNode.Get(n+23*m));
        }
    }

}


void
ARCN::SetPosition()
{
    NS_LOG_DEBUG("-----------Initializing Position-----------");
	MobilityHelper mobility;
	Ptr<ListPositionAllocator> nodesPositionAlloc = CreateObject<ListPositionAllocator> ();

    // double x = m_interval/2;
    // double y = m_interval*sqrt(3)/2;

    // //Set Uan node's position
    // for(int n=0; n<9; n++)
    // {
    //     if(n%2 == 0)
    //     {
    //         for(int m=0; m<11; m++)
    //         {
    //             nodesPositionAlloc->Add (Vector((1+2*m)*x, n*y, -1000));
    //         }
    //     }
    //     else
    //     {
    //         for(int m=0; m<12; m++)
    //         {
    //             nodesPositionAlloc->Add (Vector((2*m)*x, n*y, -1000));
    //         }
    //     }
    // }


    // //Set Buoy node's position
    // nodesPositionAlloc->Add (Vector(5*x, 2*y, 0));
    // nodesPositionAlloc->Add (Vector(11*x, 2*y, 0));
    // nodesPositionAlloc->Add (Vector(17*x, 2*y, 0));
    // nodesPositionAlloc->Add (Vector(5*x, 6*y, 0));
    // nodesPositionAlloc->Add (Vector(11*x, 6*y, 0));
    // nodesPositionAlloc->Add (Vector(17*x, 6*y, 0));


    //Set Leo Node's position
    // LonAndLat leoll = {0.900656,9.37697};
    // nodesPositionAlloc->Add (getLeoPosition(leoll,1200));

    // mobility.SetPositionAllocator (nodesPositionAlloc);
    // // mobility.Install (uanNode);
    // // mobility.Install (gatewaybuoyNode);
    // mobility.Install (leoNode);



    for(uint32_t i=0; i<uanNode.GetN(); i++)
    {
        LonAndLat nodell = position2LL(uanNode.Get(i)->GetObject<MobilityModel>()->GetPosition());
        NS_LOG_DEBUG( i << " " << uanNode.Get(i)->GetObject<MobilityModel>()->GetPosition() << " " << nodell.lat << " " << nodell.lon);
    }
    for(uint32_t i=0; i<gatewaybuoyNode.GetN(); i++)
    {
        LonAndLat nodell = position2LL(gatewaybuoyNode.Get(i)->GetObject<MobilityModel>()->GetPosition());
        NS_LOG_DEBUG( i << " " << gatewaybuoyNode.Get(i)->GetObject<MobilityModel>()->GetPosition() << " "  << nodell.lat << " " << nodell.lon);
    }
    for(uint32_t i=0; i<leoNode.GetN(); i++)
    {
        LonAndLat nodell = position2LL(leoNode.Get(i)->GetObject<MobilityModel>()->GetPosition());
        NS_LOG_DEBUG( i << " "<< leoNode.Get(i)->GetObject<MobilityModel>()->GetPosition() << " "  << nodell.lat << " " << nodell.lon);
    }
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
    uanMode = UanTxModeFactory::CreateMode(UanTxMode::FSK, 2000,
            2000, 2500, 2000, 2, "Default mode");
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
    // NS_LOG_DEBUG("the src0 mac address: " << uan_device.Get(0)->GetAddress());
    
    uan_device = uanHelper.Install(uanNode, uanChannel);
    gateway_buoyuan_device = uanHelper.Install(gatewaybuoyNode, uanChannel);

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

    IslHelper islCh;
    satellite_isl_device = islCh.Install (leoNode);

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

    NS_LOG_DEBUG("Set Buoy Uan Energy.");
    EnergySourceContainer buoyuanSources;
    for(NodeContainer::Iterator node = gatewaybuoyNode.Begin (); node != gatewaybuoyNode.End (); node++)
    {
        buoyuanSources.Add ((*node)->GetObject<EnergySourceContainer> ()->Get (0));
    }
    buoy_uan_energy = acousticEnergyHelper.Install (gateway_buoyuan_device, buoyuanSources);

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

    vbfHelper vbf;

	InternetStackHelper stack;
	stack.SetRoutingHelper (vbf);
	stack.Install (uanNode);
	stack.Install (gatewaybuoyNode);
	stack.Install (leoNode);

	Ipv4AddressHelper address;
	address.SetBase("10.1.1.0", "255.255.255.0");
	NS_LOG_DEBUG ("Assign Uan IP Addresses.");
	uan_inter = address.Assign(uan_device);
	NS_LOG_DEBUG ("Assign Buoy uan IP Addresses.");
	gateway_buoyuan_inter = address.Assign(gateway_buoyuan_device);

	NS_LOG_DEBUG ("Assign Buoy Buoywifi IP Addresses.");
	address.SetBase("10.1.2.0", "255.255.255.0");
	gateway_buoywireless_inter = address.Assign(gateway_buoywireless_device);

	address.SetBase("10.1.3.0", "255.255.255.0");
	NS_LOG_DEBUG ("Assign Satellite IP Addresses.");
	satellite_inter = address.Assign(satellite_device);
    //satellite:10.1.3.1 //buoy:10.1.3.2-10.1.3.7


    address.SetBase ("10.1.4.0", "255.255.255.0");
    address.Assign (satellite_isl_device);

	//change arp cache wait time.
	NodeContainer::Iterator m_uanNode = uanNode.Begin();
	m_uanNode = uanNode.Begin();
	while (m_uanNode != uanNode.End()) {
		(*m_uanNode)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->SetWaitReplyTimeout(
				Seconds(40));
		m_uanNode++;
	}

	NodeContainer::Iterator m_gatewaybuoyNode = gatewaybuoyNode.Begin();
	m_gatewaybuoyNode = gatewaybuoyNode.Begin();
	while (m_gatewaybuoyNode != gatewaybuoyNode.End())
	{
		(*m_gatewaybuoyNode)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->SetWaitReplyTimeout(
				Seconds(40));
		m_gatewaybuoyNode++;
	}

    NodeContainer::Iterator m_leo = leoNode.Begin();
	m_leo = leoNode.Begin();
	while (m_leo != leoNode.End())
	{
		(*m_leo)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->SetWaitReplyTimeout(
				Seconds(40));
		m_leo++;
	}

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
    // NS_LOG_DEBUG("Source Send Packet sequence number is "<< m_seqnum);


    double sendtime = Simulator::Now().GetSeconds();
    SrcMap_IdSendTime[packet->GetUid()] = sendtime; 

    // Ptr<Node> srcsend = UanSrcNode.Get(m_uniformRandomVariable->GetInteger (0, 15));
    // Ptr<Socket> srcsocket =  m_Uansockets[srcsend];
    // Ipv4Address destAddr = uan_inter.GetAddress(m_uniformRandomVariable->GetInteger (0, 102),0);
    // SendTo(srcsocket, packet, destAddr);

    Ptr<Node> srcsend = uanNode.Get(25);
    Ptr<Socket> srcsocket =  m_Uansockets[srcsend];
    Ipv4Address destAddr = uan_inter.GetAddress(77,0);
    SendTo(srcsocket, packet, destAddr); 

    NS_LOG_DEBUG("Packet "<<packet->GetUid()<<" has been sent from "<< m_UanSktAddr[srcsocket].GetLocal() << 
                 " to " << destAddr);

//	erv = CreateObjectWithAttributes<ExponentialRandomVariable>("Mean",DoubleValue(1/m_lambda));
//	Simulator::Schedule (Seconds(erv->GetValue()), &AecnExample::NodeSendPacket, this);
    // Time nextSend = Seconds(m_exponentialRandomVariable->GetValue());
    Time nextSend = Seconds(50);
    Simulator::Schedule(nextSend, &ARCN::NodeSendPacket, this);
    NS_LOG_DEBUG("next Packet will be sent after " << nextSend.GetSeconds() << "s.");

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
	Throughput = double(m_SinkRecvNum * m_pktSize * 8)/ (calculatecount*200);
	pbconsumenergy = recvbitnum/double(EnergyConsum);

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