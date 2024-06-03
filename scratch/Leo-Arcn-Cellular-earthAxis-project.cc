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

struct LonAndLat
{
    double lon;
    double lat;
};

//通过某一点的经纬度lon,lat计算另一点的经纬度,x为向右多少个间隔,y为向下多少个间隔，distance为间隔距离
LonAndLat CalculateLL(double lon, double lat, int x, int y, double dstx, double dsty)
{
    double arc = 6371.393 * 1000;
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

//根据经纬度计算水下节点实际3维坐标，假设水下节点距离地心半径=地球半径
Vector
GetUanPosition (const LonAndLat &loc)
{
  double lat = loc.lat * (M_PI / 180);
  double lon = loc.lon * (M_PI / 180);
  // Vector3D pos = Vector3D (LEO_GND_RAD_EARTH * sin (lat) * cos (lon),
  // 			   LEO_GND_RAD_EARTH * sin (lat) * sin (lon),
  // 			   LEO_GND_RAD_EARTH * cos (lat));

  //修改版
  //以北纬为正，以东经为正　　（即处在北纬0-90°＋东经0-90°的视为八个象限中的第一象限）
  Vector pos = Vector3D (LEO_GND_RAD_EARTH * cos (lat) * cos (lon),
  			   LEO_GND_RAD_EARTH * cos (lat) * sin (lon),
  			   LEO_GND_RAD_EARTH * sin (lat));

  return pos;
}

//根据经纬度计算浮标节点实际3维坐标，假设浮标节点距离地心半径=地球半径+浮标高度：取浮标高度为500
Vector
GetBuoyPosition (const LonAndLat &loc)
{

  double lat = loc.lat * (M_PI / 180);
  double lon = loc.lon * (M_PI / 180);
  // Vector3D pos = Vector3D (LEO_GND_RAD_EARTH * sin (lat) * cos (lon),
  // 			   LEO_GND_RAD_EARTH * sin (lat) * sin (lon),
  // 			   LEO_GND_RAD_EARTH * cos (lat));

  //修改版
  //以北纬为正，以东经为正　　（即处在北纬0-90°＋东经0-90°的视为八个象限中的第一象限）
  double buoyHighth = 500;
  Vector pos = Vector3D ((LEO_GND_RAD_EARTH+buoyHighth) * cos (lat) * cos (lon),
  			   (LEO_GND_RAD_EARTH+buoyHighth)* cos (lat) * sin (lon),
  			   (LEO_GND_RAD_EARTH+buoyHighth) * sin (lat));

  return pos;
}

//计算卫星的三维坐标
Vector
getLeoPosition (LonAndLat leoll, double height)
{
  double lat = leoll.lat * (M_PI / 180);
  double lon = leoll.lon * (M_PI / 180);

  Vector pos = Vector((LEO_GND_RAD_EARTH+height*1000) * cos (lat) * cos (lon),
  			   (LEO_GND_RAD_EARTH+height*1000) * cos (lat) * sin (lon),
  			   (LEO_GND_RAD_EARTH+height*1000) * sin (lat));
  return pos;
}


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
    uint32_t numUav; //number of uav
    uint32_t numGround;

	NodeContainer uanNode;
	NodeContainer gatewaybuoyNode;
    NodeContainer leoNode;
    NodeContainer uavNode;
    NodeContainer groundNode;

	NetDeviceContainer uan_device;
	NetDeviceContainer gateway_buoywireless_device;
    NetDeviceContainer satellite_device;
    NetDeviceContainer uav_wireless_device;
    NetDeviceContainer ground_wireless_device;

	Ipv4InterfaceContainer uan_inter;
	Ipv4InterfaceContainer gateway_buoywireless_inter;
    Ipv4InterfaceContainer satellite_inter;
    Ipv4InterfaceContainer uav_wireless_inter;
    Ipv4InterfaceContainer ground_wireless_inter;

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
    Time TimeTurnRound;

    /* EnergyConsum */
	DeviceEnergyModelContainer uan_energy;
	DeviceEnergyModelContainer buoy_wifi_energy;
	DeviceEnergyModelContainer uav_wifi_energy;
	DeviceEnergyModelContainer ground_wifi_energy;

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
	std::map<Ptr<Node>, Ptr<Socket> > m_wifisockets; //!< send and receive sockets
	std::map<Ptr<Node>, Ptr<Socket> > m_satellitesockets; //!< send and receive sockets
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_UanSktAddr;
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_WifiSktAddr;
	std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_satelliteSktAddr;

    uint32_t rng;

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

    /// @brief Change the speed of uav
    void UavTurnRound(uint32_t uavNodeId);

};

//-----------------------------------------------------------------------------
ARCN::ARCN():
    totalTime(10000),
    pcap (true),
    printRoutes (true),
    numUan(103),
    numgtbuoy(6),
    numLeo(1),
    numUav(1),
    numGround(5),
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
    m_txPowerRadio(93), //radio--40km
    //m_step(1),
    TimeTurnRound(Seconds(70000/15)),
    m_totaldelay(0),
    m_SinkRecvNum(0),
    calculatecount(0),
    m_seqnum(0),
    rng(1)
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

  for (socket = m_wifisockets.begin (); socket != m_wifisockets.end (); socket++)
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

    // LogComponentEnable("SelectRoutingProtocol", LOG_LEVEL_ALL);
    // LogComponentEnable("UdpL4Protocol", LOG_LEVEL_ALL);
    // LogComponentEnable("UanMacSrTDMA", LOG_LEVEL_ALL);
    // LogComponentEnable("Ipv4L3Protocol",LOG_LEVEL_ALL);
    // LogComponentEnable("Leo-Arcn", LOG_LEVEL_ALL);
    // LogComponentEnable("LeoGndNodeHelper",LOG_LEVEL_INFO);
    // LogComponentEnable("LeoPropagationLossModel",LOG_LEVEL_INFO);
    // LogComponentEnable("MockNetDevice", LOG_LEVEL_DEBUG);
    // LogComponentEnable("Ipv4L3Protocol", LOG_LEVEL_INFO);


	// LogComponentEnable("UanPhyGen", LOG_LEVEL_ALL);
    // LogComponentEnable("UanMacSFAMA", LOG_LEVEL_ALL);
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

	cmd.Parse(argc, argv);
    RngSeedManager::SetSeed (12345); // Changes seed from default of 1 to 3
    RngSeedManager::SetRun (rng);
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

    m_uniformRandomVariable = CreateObject<UniformRandomVariable>();
    m_uniformRandomVariable->SetAttribute ("Min", DoubleValue (0.0));
    m_uniformRandomVariable->SetAttribute ("Max", DoubleValue (102.0));
    m_exponentiaflRandomVariable = CreateObjectWithAttributes<ExponentialRandomVariable>("Mean",DoubleValue(5));

    NS_LOG_UNCOND("Simulation run "<<totalTime<<" seconds....\n");
    Simulator::Schedule(Seconds(200), &ARCN::NodeSendPacket, this);
    Simulator::Schedule(Seconds(200), &ARCN::Calculate, this);

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

    NS_LOG_INFO("Create Ground Node");
    groundNode.Create(numGround);

    NS_LOG_INFO("Create Uav Node");
    uavNode.Create(numUav);

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

	for (uint32_t i = 0; i < numGround; ++i)
	{
		std::ostringstream os;
		os << "ground Node-" << i;
		Names::Add(os.str(), groundNode.Get(i));
	}

	for (uint32_t i = 0; i < numUav; ++i)
	{
		std::ostringstream os;
		os << "uav Node-" << i;
		Names::Add(os.str(), uavNode.Get(i));
	}

}


void
ARCN::SetPosition()
{
    NS_LOG_DEBUG("-----------Initializing Position-----------");
	MobilityHelper mobility;
	Ptr<ListPositionAllocator> nodesPositionAlloc = CreateObject<ListPositionAllocator> ();

    NS_LOG_DEBUG("初始化水下和浮标节点的位置.");
    // latitude-纬度  lontitude-经度
    double lat_o = 21;
    double lon_o = 119;
    double lat_1 = CalculateLL(lon_o,lat_o,1,1,-10000,-10000*sqrt(3)).lat;
    double lon_1 = CalculateLL(lon_o,lat_o,1,1,-10000,-10000*sqrt(3)).lon;
    NS_LOG_DEBUG(lat_o << " " << lon_o << " " << lat_1 << " " << lon_1);

    LonAndLat uanll;
    for(int i=0; i<5; i++)
    {
        for(int j=0; j<11; j++)
        {
            uanll = CalculateLL(lon_o,lat_o,j,i,20000,-20000*sqrt(3));
            Vector uanPos;
            if( !((i==1&&j==2)||((i==1&&j==5))||((i==1&&j==8))||(i==3&&j==2)||((i==3&&j==5))||((i==3&&j==8))))
            {
                uanPos = GetUanPosition(uanll);
                NS_LOG_DEBUG("The Uan Node: " << i*23+j);
            }
            else
            {
                uanPos = GetBuoyPosition(uanll);
                NS_LOG_DEBUG("The Buoy Node: " << i*23+j);
            }
            nodesPositionAlloc->Add(uanPos);
        }

        for(int j=0; j<12 && i<4; j++)
        {
            uanll = CalculateLL(lon_1,lat_1,j,i,20000,-20000*sqrt(3));
            Vector uanPos;
            uanPos = GetUanPosition(uanll);
            NS_LOG_DEBUG("The Uan Node: " << 11+i*23+j);
            nodesPositionAlloc->Add(uanPos);
        }    
    }

    NS_LOG_DEBUG("初始化卫星节点的位置.");
    //Set Leo Node's position
    LonAndLat leoll = {119.90065,20.37697};
    nodesPositionAlloc->Add (getLeoPosition(leoll,1200));
    NS_LOG_DEBUG("satellite lon: " << leoll.lon << " lat: " << leoll.lat << ".");

    NS_LOG_DEBUG("初始化岸基节点的位置.");
    LonAndLat groundll0 = CalculateLL(lon_o,lat_o,12,1,20000,-20000*sqrt(3));
    NS_LOG_DEBUG("ground0 lon: " << groundll0.lon << " lat: " << groundll0.lat << ".");

    LonAndLat ground;
    for(int i=0; i<5; i++)
    {
        ground = CalculateLL(groundll0.lon,groundll0.lat,0,i,20000,-10000*sqrt(3));
        Vector groundPos;
        groundPos = GetBuoyPosition(ground);
        NS_LOG_DEBUG("The Ground Node: " << i);
        nodesPositionAlloc->Add(groundPos);
    }

    mobility.SetPositionAllocator (nodesPositionAlloc);

    mobility.Install (uanNode);
    mobility.Install (leoNode);
    mobility.Install (groundNode);


    NS_LOG_DEBUG("初始化无人机节点的位置.");
    LonAndLat uavStartLL =  position2LL(uanNode.Get(54)->GetObject<MobilityModel>()->GetPosition());
    Vector uavStartPosition = GetBuoyPosition(uavStartLL);
    NS_LOG_DEBUG("uav position: " << uavStartPosition << ".");
	MobilityHelper mobilityUav;
	Ptr<ListPositionAllocator> nodesPositionAllocUav = CreateObject<ListPositionAllocator> ();
    nodesPositionAllocUav->Add (uavStartPosition);
    mobilityUav.SetPositionAllocator(nodesPositionAllocUav);
    mobilityUav.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobilityUav.Install (uavNode);

    Vector p1 = uanNode.Get(33)->GetObject<MobilityModel>()->GetPosition();
    Vector p2 = GetBuoyPosition(groundll0);
    double v = (p2-p1).GetLength();
    double value = 15; // 15m/s=54km/h
    Vector speed = Vector((p2.x-p1.x)*value/v,(p2.y-p1.y)*value/v,(p2.z-p1.z)*value/v); //归一化速度向量后乘以速度值
    NS_LOG_DEBUG("uav speed: " << speed << ".");
    for (uint32_t n=0 ; n < uavNode.GetN() ; n++)
    {
        Ptr<ConstantVelocityMobilityModel> mob = uavNode.Get(n)->GetObject<ConstantVelocityMobilityModel>();
        mob->SetVelocity(speed);
    }
    //需要写一个函数改变无人机速度
    Simulator::Schedule(TimeTurnRound,&ARCN::UavTurnRound,this,uavNode.Get(0)->GetId());


    for(uint32_t i=0; i<uanNode.GetN(); i++)
    {
        LonAndLat nodell = position2LL(uanNode.Get(i)->GetObject<MobilityModel>()->GetPosition());
        NS_LOG_DEBUG( i << " uan/buoy " << uanNode.Get(i)->GetObject<MobilityModel>()->GetPosition() << " " << nodell.lat << " " << nodell.lon);
    }
    for(uint32_t i=0; i<leoNode.GetN(); i++)
    {
        LonAndLat nodell = position2LL(leoNode.Get(i)->GetObject<MobilityModel>()->GetPosition());
        NS_LOG_DEBUG( i << " leo "<< leoNode.Get(i)->GetObject<MobilityModel>()->GetPosition() << " "  << nodell.lat << " " << nodell.lon);
    }

    for(uint32_t i=0; i<groundNode.GetN(); i++)
    {
        LonAndLat nodell = position2LL(groundNode.Get(i)->GetObject<MobilityModel>()->GetPosition());
        NS_LOG_DEBUG( i << " ground "<< groundNode.Get(i)->GetObject<MobilityModel>()->GetPosition() << " "  << nodell.lat << " " << nodell.lon);
    }

    for(uint32_t i=0; i<uavNode.GetN(); i++)
    {
        LonAndLat nodell = position2LL(uavNode.Get(i)->GetObject<MobilityModel>()->GetPosition());
        NS_LOG_DEBUG( i << " uav "<< uavNode.Get(i)->GetObject<MobilityModel>()->GetPosition() << " "  << nodell.lat << " " << nodell.lon);
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
    uanMode = UanTxModeFactory::CreateMode(UanTxMode::FSK, 6000,
            6000, 2500, 2000, 2, "Default mode");
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
    uan_device = uanHelper.Install(uanNode, uanChannel);

    NS_LOG_DEBUG("-----------Initializing Wireless-----------");
    WifiHelper wifiHelper;
    wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211a);
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
    wifiPhy = YansWifiPhyHelper::Default ();
    wifiPhy.Set ("RxGain", DoubleValue (0) );
    wifiPhy.Set ("TxGain", DoubleValue (m_offset + m_Prss)); //TxGain = 1dB
    wifiPhy.Set("TxPowerStart", DoubleValue(m_txPowerRadio));
    wifiPhy.Set("TxPowerEnd", DoubleValue(m_txPowerRadio));  ////TxPower = m_txPowerRadio = 92dbm--35000m 93dmb--40000m now is 93
    wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);

    wifiPhy.SetChannel (wifiChannel.Create());
    WifiMacHelper wifiMac;
    wifiMac.SetType ("ns3::AdhocWifiMac");
    wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (m_wifiPhyMode),
                                "ControlMode",StringValue (m_wifiPhyMode));

    gateway_buoywireless_device = wifiHelper.Install (wifiPhy, wifiMac, gatewaybuoyNode);
    uav_wireless_device = wifiHelper.Install (wifiPhy, wifiMac, uavNode);
    ground_wireless_device = wifiHelper.Install (wifiPhy, wifiMac, groundNode);


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
    NS_LOG_DEBUG("Set Uav and Ground Wifi Energy.");
    EnergySourceContainer buoywifiSources = energySourceHelper.Install (gatewaybuoyNode);
    EnergySourceContainer uavwifiSources = energySourceHelper.Install (uavNode);
    EnergySourceContainer groundwifiSources = energySourceHelper.Install (groundNode);
    WifiRadioEnergyModelHelper radioEnergyHelper;
    // configure radio energy model
    radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));
    // install device model
    buoy_wifi_energy = radioEnergyHelper.Install (gateway_buoywireless_device, buoywifiSources);
    uav_wifi_energy = radioEnergyHelper.Install (uav_wireless_device, uavwifiSources);
    ground_wifi_energy = radioEnergyHelper.Install (ground_wireless_device, groundwifiSources);


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
	// aodv.Set("TimeoutBuffer", UintegerValue(2));
	aodv.Set("NodeTraversalTime", TimeValue(Seconds(15))); //one hop traversal time for packets
	aodv.Set("NextHopWait", TimeValue(Seconds(15.01))); 
	aodv.Set("ActiveRouteTimeout", TimeValue(Seconds(2000))); // time while the router is considered to be vaild
	aodv.Set("MyRouteTimeout", TimeValue(Seconds(2000)));
	// aodv.Set("BlackListTimeout", TimeValue(Seconds(5.6)));
	// aodv.Set("DeletePeriod", TimeValue(Seconds(500)));
	aodv.Set("NetDiameter", UintegerValue(10));
	// aodv.Set("NetTraversalTime", TimeValue(Seconds(200)));
	// aodv.Set("PathDiscoveryTime", TimeValue(Seconds(200)));
	aodv.Set("TtlStart", UintegerValue(10));

    vbfHelper vbf;
    selectRoutingHelper select;


	InternetStackHelper stack;
   	stack.Install (uavNode);
	stack.Install (groundNode);

	stack.SetRoutingHelper (aodv);
	stack.Install (uanNode);
	stack.Install (leoNode);


	Ipv4AddressHelper address;
	address.SetBase("10.1.1.0", "255.255.255.0");
	NS_LOG_DEBUG ("Assign Uan IP Addresses.");
	uan_inter = address.Assign(uan_device);

	NS_LOG_DEBUG ("Assign Buoy Buoywifi IP Addresses.");
	address.SetBase("10.1.2.0", "255.255.255.0");
	gateway_buoywireless_inter = address.Assign(gateway_buoywireless_device);
    uav_wireless_inter = address.Assign(uav_wireless_device);
    ground_wireless_inter = address.Assign(ground_wireless_device);

	address.SetBase("10.1.3.0", "255.255.255.0");
	NS_LOG_DEBUG ("Assign Satellite IP Addresses.");
	satellite_inter = address.Assign(satellite_device);
    //satellite:10.1.3.1 //buoy:10.1.3.2-10.1.3.7

	// change arp cache wait time.
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
    /**
     * 细节：
     * 因为为socket在监听的时候绑定的是ip和端口，而节点收到包后是根据包的目的ip和端口来确定是不是发给自己的
     * 如果指定的网卡和目的节点监听的网卡ip不一致，就会un_rearch了，简单来说就是源节点在发出包的时候，
     * 要确定目的节点是用哪个网卡去接收的，以这个网卡ip地址作为发送目的地址
     * 
     * 问题的核心：这里原本是借鉴了aodv协议关于套接字的应用，因为在aodv协议中是需要区分不同网卡发出的aodv包
     * 所以每个套接字需要绑定在网卡上面，绑定到某个网卡后，如果你用10.1.3.1网卡收到一个发往10.1.1.1网卡的包，
     * 即便这两个网卡都属于你，你也没办法在udp层去接收这个包，因为包是发往10.1.1.1网卡绑定的套接字的，但是你
     * 是用10.1.3.1网卡接收的。
     * 
     * 解决方案：取消套接字绑定某个特定网卡ip的操作，而是绑定Ipv4Address::GetAny()=0.0.0.0地址，
     * 即任何网卡接收到的地址都视为可接收的包传入udp层。
     * 
     * */ 


    /**
     * 但是，如果是绑定的Ipv4Address::GetAny()=0.0.0.0地址，并且此时网络使用了具有协议头部的路由协议
     * 又会导致一个问题的出现。
     * １．正常情况：本来我们的头部顺序应该是mac-ip-route-udp，即udp把包传给ip（传入route为空的ipl3->send函数，这样在ipl3层就会
     * 去路由协议Routeoutput函数中找路由），ip把带有udp头部的包传给路由Routeoutput函数，函数中给包加上route头部，
     * 此时包变成带有route-udp头部，然后Routeoutput函数再把包和找到的路由项rtenry发回给ipl3层，ipl3层给包再加上ip头部，再发给链路层
     * 同理，读取的时候，Routeinput函数中处理的包其实是不带有ip头部，只带有route-udp头部的包，我们只需要在这类remove routeheader后
     * 即可以通过lcb函数把包发往udp层，udp再读取udpheader.
     * 
     * ２．但是，如果套接字绑定的是Ipv4Address::GetAny()=0.0.0.0地址，这时候就会导致一个问题，
     * 即udp连接的目的地址是可以指定的，但源地址是自动分配的，所以udp层一开始不知道源地址，就没办法
     * 处理udp头部，所以它需要知道发送的源地址才可以给包加上udp头部，但是udp要怎么去知道源地址呢，
     * 就是通过路由Routeoutput函数去获取，udp层如果一开始识别到源地址是Ipv4Address::GetAny()=0.0.0.0地址
     * 就会调用ipl3层的Routeoutput函数去获取一个路由，然后读取路由rtenry中的源地址，这样就知道源地址应该选哪个了，
     * 所以带来的问题就是在Routeoutput函数添加了route头部，然后再回到udp层中添加udp头部，这样就变成了头部顺序是
     * udp-route，然后因为udp层已经获取了route，所以它此时调用的ip层函数就是（传入route不为空的ipl3->send函数），
     * 这样在ipl3层就会发现已经有路由协议了，就不会再进入Routeoutput函数，而是直接加上ip头部，然后发往链路层
     * 所以这种情况导致的最终头部顺序就是：mac-ip-udp-route
     * 
     * ３．如果实在要用绑定Ipv4Address::GetAny()=0.0.0.0地址的套接字模式，就尽量用封装再发送的方式去处理包(aodv\vbf)，而不是
     * 添加额外头部的方式(select)，但是如果实在要用也有一种解决方法，因为前面提到最终头部顺序就是：mac-ip-udp-route，所以处理方法就是
     * 在RouteInput函数中处理的包其实是带有udp-route的包，所以就先remove udpheader，然后remove routeheader，最后在add刚才remove
     * 的udpheader，这样最终传入udp层的包就是去掉了route，只剩udp的包。
     * 
     * 
    */

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
        m_WifiSktAddr.insert (std::make_pair (bw_socket, iface));
        m_wifisockets.insert (std::make_pair (buoynode, bw_socket));
        NS_LOG_DEBUG("gt Node Wire ID =" << buoynode->GetId () << " Socket =" <<bw_socket << " Ipv4Address " << ip_addr);
    }

    for (uint32_t i = 0; i < uav_wireless_device.GetN (); ++i)
    {
        Ptr<NetDevice> dev = uav_wireless_device.Get (i);

        Ptr<Node> uavnode = dev->GetNode ();
        Ptr<Ipv4> ipv4 = uavnode->GetObject<Ipv4> ();
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
        Ptr<Socket> bw_socket = Socket::CreateSocket (uavnode,
                                                    UdpSocketFactory::GetTypeId ());
        NS_ASSERT (bw_socket != 0);
        bw_socket->SetRecvCallback (MakeCallback (&ARCN::ReceivePacket, this));
        bw_socket->BindToNetDevice (dev);
        bw_socket->Bind (InetSocketAddress (ip_addr, m_port));
        bw_socket->SetAllowBroadcast (true);
        bw_socket->SetIpRecvTtl (true);
        m_WifiSktAddr.insert (std::make_pair (bw_socket, iface));
        m_wifisockets.insert (std::make_pair (uavnode, bw_socket));
        NS_LOG_DEBUG("uav Node Wire ID =" << uavnode->GetId () << " Socket =" <<bw_socket << " Ipv4Address " << ip_addr);
    }

   for (uint32_t i = 0; i < ground_wireless_device.GetN (); ++i)
    {
        Ptr<NetDevice> dev = ground_wireless_device.Get (i);

        Ptr<Node> groundnode = dev->GetNode ();
        Ptr<Ipv4> ipv4 = groundnode->GetObject<Ipv4> ();
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
        Ptr<Socket> bw_socket = Socket::CreateSocket (groundnode,
                                                    UdpSocketFactory::GetTypeId ());
        NS_ASSERT (bw_socket != 0);
        bw_socket->SetRecvCallback (MakeCallback (&ARCN::ReceivePacket, this));
        bw_socket->BindToNetDevice (dev);
        bw_socket->Bind (InetSocketAddress (ip_addr, m_port));
        bw_socket->SetAllowBroadcast (true);
        bw_socket->SetIpRecvTtl (true);
        m_WifiSktAddr.insert (std::make_pair (bw_socket, iface));
        m_wifisockets.insert (std::make_pair (groundnode, bw_socket));
        NS_LOG_DEBUG("ground Node Wire ID =" << groundnode->GetId () << " Socket =" <<bw_socket << " Ipv4Address " << ip_addr);
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


    Ptr<Node> srcsend = uanNode.Get(m_uniformRandomVariable->GetInteger());
    Ptr<Socket> srcsocket =  m_Uansockets[srcsend];
    uint32_t destId = m_uniformRandomVariable->GetInteger();
    Ipv4Address destAddr = uan_inter.GetAddress(destId,0);
    SendTo(srcsocket, packet, destAddr); 
    NS_LOG_DEBUG("Packet "<<packet->GetUid()<<" has been sent from uanNode-"<< srcsend->GetId() << 
                 " to uanNode-" << destId);  
    Time nextSend = Seconds(m_exponentiaflRandomVariable->GetValue());
    // Time nextSend = Seconds(500);
    Simulator::Schedule(nextSend, &ARCN::NodeSendPacket, this);
    NS_LOG_DEBUG("next Packet will be sent after " << nextSend.GetSeconds() << "s.");


    // Ptr<Node> srcsend = uanNode.Get(29);
    // Ptr<Socket> srcsocket =  m_Uansockets[srcsend];
    // Ipv4Address destAddr = uan_inter.GetAddress(31,0); 
    // SendTo(srcsocket, packet, destAddr); 
    // NS_LOG_DEBUG("Packet "<<packet->GetUid()<<" has been sent from "<< srcsend->GetId() << 
    //              " to " << destAddr);  
    // Time nextSend = Seconds(200);
    // Simulator::Schedule(nextSend, &ARCN::NodeSendPacket, this);



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
	double buoy_wifi_consumed;
	double uav_wifi_consumed;
	double ground_wifi_consumed;

	NS_LOG_INFO("-----------Collect the node energy consumption.-----------");

	for (DeviceEnergyModelContainer::Iterator iter = uan_energy.Begin (); iter != uan_energy.End (); ++iter)
    {
		uan_consumed += (*iter)->GetTotalEnergyConsumption ();
	}
	for (DeviceEnergyModelContainer::Iterator iter = buoy_wifi_energy.Begin (); iter != buoy_wifi_energy.End (); ++iter)
    {
		buoy_wifi_consumed += (*iter)->GetTotalEnergyConsumption ();
	}
	for (DeviceEnergyModelContainer::Iterator iter = uav_wifi_energy.Begin (); iter != uav_wifi_energy.End (); ++iter)
    {
		uav_wifi_consumed += (*iter)->GetTotalEnergyConsumption ();
	}
	for (DeviceEnergyModelContainer::Iterator iter = ground_wifi_energy.Begin (); iter != ground_wifi_energy.End (); ++iter)
    {
		ground_wifi_consumed += (*iter)->GetTotalEnergyConsumption ();
	}


    NS_LOG_INFO("uan_consumed:" << uan_consumed << ". buoyWifi_consumed:" <<buoy_wifi_consumed<< ". uavWifi_consumed:" <<uav_wifi_consumed
    << ". groundWifi_consumed:" <<ground_wifi_consumed );
	EnergyConsum = uan_consumed + buoy_wifi_consumed + uav_wifi_consumed + ground_wifi_consumed;
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

void
ARCN::UavTurnRound(uint32_t uavNodeId)
{
    Ptr<Node> uavNode = NodeList::GetNode(uavNodeId);
    Vector uavPosition = uavNode->GetObject<MobilityModel>()->GetPosition();
    LonAndLat uavll = position2LL(uavPosition);
    NS_LOG_DEBUG("the position of uavNode " << uavNodeId << " is  lat:" << uavll.lat << " lon:" << uavll.lon);

    Vector uavVelocity = uavNode->GetObject<MobilityModel>()->GetVelocity();
    Vector uavNewVelocity(-uavVelocity.x,-uavVelocity.y,-uavVelocity.z);
    Ptr<ConstantVelocityMobilityModel> mob = uavNode->GetObject<ConstantVelocityMobilityModel>();
    mob->SetVelocity(uavNewVelocity);

    Simulator::Schedule(TimeTurnRound,&ARCN::UavTurnRound,this,uavNodeId);
}

int main (int argc, char **argv)
{
  ARCN test;

  if (!test.Configure (argc, argv))
    NS_FATAL_ERROR ("Configuration failed. Aborted.");

  test.Run ();

  return 0;
}