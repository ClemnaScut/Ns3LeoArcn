/*
 * uan-sfama-test.cc
 *
 *  Created on: Apr 2, 2021
 *      Author: SCUT-arcca-Wang
 */

#include "uan-sfama-test.h"
#include <fstream>
#include <math.h>
#include "uan-sfama-test.h"

NS_LOG_COMPONENT_DEFINE ("UanSFAMAExperiment");

NS_OBJECT_ENSURE_REGISTERED (SFAMATestTag);

/// Get TypeId
TypeId SFAMATestTag::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::UanClusterTest::UnderwaterClusterTestTag")
	.SetParent<Tag> ()
    .SetParent<Tag> ()
    .AddConstructor<SFAMATestTag> ()
  ;
  return tid;
}

/// Get instanceTypeId
TypeId  SFAMATestTag::GetInstanceTypeId () const
{
  return GetTypeId ();
}

/// Get epidemic tag
SFAMATestTag::TagType SFAMATestTag::GetTagType () const
{
  return m_tag;
}

bool
SFAMATestTag::IsTagType (const TagType type) const
{
  return m_tag == type;
}
/// Set epidemic tag

void
SFAMATestTag::SetTagType (const TagType tag)
{
  m_tag = tag;
}

/// Get size
uint32_t SFAMATestTag::GetSerializedSize () const
{
  return sizeof(uint8_t)+3*sizeof(uint32_t);
}


/// Serialize
void  SFAMATestTag::Serialize (TagBuffer i) const
{
  i.WriteU8 ((uint8_t) m_tag);
  i.WriteU32((uint32_t) m_TxId);
  i.WriteU32((uint32_t) m_RxId);
  i.WriteU32((uint32_t) m_pktId);

}

///Deserialize
void  SFAMATestTag::Deserialize (TagBuffer i)
{
  uint8_t type = i.ReadU8 ();
  switch (type)
    {
    case MSG:
    case ACK:
		{
		  m_tag = (TagType) type;
		  break;
		}
    default:
      break;
    }
  m_TxId = i.ReadU32 ();
  m_RxId = i.ReadU32 ();
  m_pktId = i.ReadU32 ();

}
void  SFAMATestTag::Print (std::ostream &os) const
{
	  switch (m_tag)
	    {
	    case MSG:
			{
				 os << "SFAMATestTag: MSG.";

			  break;
			}
	    case ACK:
			{
				 os << "SFAMATestTag: ACK.";
			break;
			}
	    }
 ;
}

std::ostream &
operator<< (std::ostream & os, SFAMATestTag const & p)
{
  p.Print (os);
  return os;
}

ExperimentSFAMA::ExperimentSFAMA ()
   :m_nct (5),
	m_ncr(4),
	m_moduleDataRate(4800),
	m_Tsinr(10),
	m_depth (1000),
	m_boundary (1500),
	m_lambda(1),
	m_txPowerAcoustic(190),
	seed_value(1),
	m_StopTime(2000),
	m_tFrame(0),
	m_packetSize(128),
	m_currentPktId(0),
	SinkrecvNum(0),
	SrcsendNum(0),
	RunTime(0),
	GeneratePkts_Sum(0),
	CurrentGenerate_Sum(0),
	Buffer_Sum(0),
	energyConsumed(0),
	ncr_Uan_Enr(0),
	nct_Uan_Enr(0),
	empty_slot(0),
	rec2send(0),
	test_pdr(0),
	NetworkTPut(0),
	Delay(0),
    m_asciitracefile ("IDU-Access.asc"),
    m_macType("ns3::UanMacSFAMA"),
	m_model("slot")
{

}

SFAMATestHeader::SFAMATestHeader () :
						 m_packetindex(0)
{
}

NS_OBJECT_ENSURE_REGISTERED (SFAMATestHeader);

TypeId
SFAMATestHeader::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::UanSFAMATest::UnderwaterGameMacHeader")
    .SetParent<Header> ()
    .AddConstructor<SFAMATestHeader> ()
  ;
  return tid;
}

TypeId
SFAMATestHeader::GetInstanceTypeId () const
{

  NS_LOG_FUNCTION (this);
  return GetTypeId ();
}

uint32_t
SFAMATestHeader::GetSerializedSize () const
{
  NS_LOG_FUNCTION (this);
  return sizeof(uint32_t);
}

void
SFAMATestHeader::Serialize (Buffer::Iterator i) const
{

	NS_LOG_FUNCTION (this);
	i.WriteU32 (m_packetindex);
}

uint32_t
SFAMATestHeader::Deserialize (Buffer::Iterator start)
{
	NS_LOG_FUNCTION (this);
	Buffer::Iterator i = start;
	m_packetindex = i.ReadU32 ();
	uint8_t dist = i.GetDistanceFrom (start);
	NS_ASSERT (dist == GetSerializedSize ());
	return dist;
}

void
SFAMATestHeader::Print (std::ostream &os) const
{

  NS_LOG_FUNCTION (this);
  os << " The packet index : "<< m_packetindex;

}

bool
SFAMATestHeader::operator == (SFAMATestHeader const & o) const
{
  NS_LOG_FUNCTION (this);
  return (m_packetindex == o.m_packetindex);
}

std::ostream &
operator<< (std::ostream & os, SFAMATestHeader const & h)
{
  h.Print (os);
  return os;
}

SFAMATestAckHeader::SFAMATestAckHeader () :
	m_packetindex(0)
{
}

SFAMATestAckHeader::~SFAMATestAckHeader ()
{
  NS_LOG_FUNCTION (this);
}

NS_OBJECT_ENSURE_REGISTERED (SFAMATestAckHeader);

TypeId
SFAMATestAckHeader::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::UanSFAMATest::UnderwaterGameMacAckHeader")
    .SetParent<Header> ()
    .AddConstructor<SFAMATestAckHeader> ()
  ;
  return tid;
}

TypeId
SFAMATestAckHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

uint32_t
SFAMATestAckHeader::GetSerializedSize () const
{
	NS_LOG_FUNCTION (this);

	return sizeof (uint8_t);
}

void
SFAMATestAckHeader::Serialize (Buffer::Iterator i ) const
{
	NS_LOG_FUNCTION (this);
	i.WriteU8 (m_packetindex);
}

uint32_t
SFAMATestAckHeader::Deserialize (Buffer::Iterator start )
{
	NS_LOG_FUNCTION (this);

  Buffer::Iterator i = start;
  m_packetindex = i.ReadU8();
  uint8_t dist = i.GetDistanceFrom (start);
  NS_ASSERT (dist == GetSerializedSize ());
  return dist;
}

void
SFAMATestAckHeader::Print (std::ostream &os ) const
{
	 os << "The packet index : "<< (*this).GetPacketInd() ;

}

bool
SFAMATestAckHeader::operator== (SFAMATestAckHeader const & o ) const
{
  return m_packetindex == o.m_packetindex  ;
}



std::ostream &
operator<< (std::ostream & os, SFAMATestAckHeader const & h )
{
  h.Print (os);
  return os;
}

uint32_t
ExperimentSFAMA::GetIdFromContext(std::string context)
{
	NS_LOG_FUNCTION (this);
	int size = context.size();
	int index = 0;
	int digit_len = 0;
	for (decltype(context.size()) j = 0; j != (unsigned)size; ++j)
	{
		if(isdigit(context[j]))
		{
			index = j;
			NS_LOG_DEBUG("The "<<j<<"'th index is "<<context[j]);
			for (int i = j; (unsigned)i < size - j; ++i)
			{
				if (isdigit(context[i]))
				{
					++digit_len;
				}
				else if(!isdigit(context[i]))
				{
					NS_LOG_INFO("The node is is "<<digit_len<<" length.");
					break;
				}
			}//end inner for loop
			break;
		}//end first if
	}
	std::stringstream os;
	vector <uint32_t> v_id;
	uint32_t Id = 0;

	for (int i = index; i < index + digit_len; ++i)
	{
		os<<context[i];
		NS_LOG_DEBUG("Put "<<context[i]<<" into os.");
	}
	os>>Id;
	NS_LOG_INFO("string flow collect digit is "<<Id<<".");

	if(Id < m_nct)
	{
		NS_LOG_INFO("This node is belong to tx container. This is src "<<Id<<".");
		uint32_t nodeid = Id;
		return nodeid;
	}
	else if(ncr_uan.Get(0)->GetId() <=Id && Id < ncr_uan.Get(0)->GetId() + m_ncr)
	{
		NS_LOG_INFO("This node is belong to rx underwater device container.");
		uint32_t nodeid = Id - m_nct;
		NS_LOG_INFO("This node is rx node "<<nodeid<<".");
		return nodeid;
	}
	NS_LOG_UNCOND("Error unknown case");
	return 9999;
}//end function get id from context

void
ExperimentSFAMA::PoissonTrafficGenerate(Ptr<Node> sender)
{
	NS_LOG_FUNCTION (this);
	if (Simulator::Now().GetSeconds()>m_StopTime)
	{
		return;
	}
	uint32_t sender_id = sender->GetId();
	double evalue = m_er->GetValue();
	++GeneratePkts_Sum;

	double c_tPkts = MAP_TID2TotalGeneratePkts[sender_id];
	double c_BufferPkts = MAP_TID2CurrentGeneratePkts[sender_id];
	++c_tPkts;//totall generated packets in current nodes
	++c_BufferPkts;//packets are not sent yet

	MAP_TID2TotalGeneratePkts[sender->GetId()]=c_tPkts;
	MAP_TID2CurrentGeneratePkts[sender->GetId()]=c_BufferPkts;

	SendNextPacket(sender->GetId());

	Time delay;
	delay = Seconds(evalue);
	Simulator::Schedule(delay,&ExperimentSFAMA::PoissonTrafficGenerate,this, sender);
}//end function PoissonTrafficGenerate

void
ExperimentSFAMA::SendNextPacket(uint32_t sender_Id)
{
	NS_LOG_FUNCTION (this);

	++m_currentPktId;
	uint32_t c_pktid = m_currentPktId;
	uint32_t des_id = MAP_SId2RId[sender_Id];

	Ptr<Packet> packet = Create<Packet>(m_packetSize);
	Ptr<Node> sender = nct_uan.Get(sender_Id);

	SFAMATestTag tag(SFAMATestTag::MSG);
	tag.SetTxInd(sender_Id);
	tag.SetRxInd(des_id);
	SFAMATestHeader msg_header;
	msg_header.SetPacketInd(c_pktid);
	packet->AddHeader(msg_header);
	packet->AddPacketTag(tag);

	Ptr<UanNetDevice> dev = sender->GetDevice(0)->GetObject<UanNetDevice>();
	//double current_time = Simulator::Now().GetSeconds();
	Time delay;
	Address uanrecvaddr;
	uanrecvaddr = ncr_uan.Get(des_id)->GetDevice(0)->GetObject<UanNetDevice>()->GetAddress();

	Ptr<UanPhyGen> uanphy=dev->GetPhy()->GetObject<UanPhyGen>();
	if (uanphy->IsStateTx())
	{
		return;
	}
	double c_BufferPkts = MAP_TID2CurrentGeneratePkts[sender->GetId()];
	dev->Send(packet,uanrecvaddr,0);
	NS_LOG_INFO("Node "<<sender_Id<<" transmits new packet, its index is "<<c_pktid<<".");
	--c_BufferPkts;
	MAP_TID2CurrentGeneratePkts[sender->GetId()]=c_BufferPkts;

	return;
}//end SendNextPacket

void
ExperimentSFAMA::MacEnqueueCB(std::string context,  Ptr< const Packet > packet)
{
	NS_LOG_FUNCTION(this);
	uint32_t sender_id = GetIdFromContext(context);
	Ptr<Packet> en_pkt = packet->Copy();
	SFAMATestTag tag(SFAMATestTag::ACK);
	en_pkt->RemovePacketTag(tag);
	NS_ASSERT(tag.GetTagType()==SFAMATestTag::MSG && tag.GetTxInd()==sender_id);
	SFAMATestHeader msg_header;
	en_pkt->RemoveHeader(msg_header);
	//uint32_t c_pktid=msg_header.GetPacketInd();
	double CPktSend = MAP_TID2SendNum[sender_id];
	CPktSend +=1;
	MAP_TID2SendNum[sender_id] = CPktSend;
}//end MacEnqueueCB

void
ExperimentSFAMA::PositionConfigure()
{
	NS_LOG_FUNCTION (this);

	if (m_ncr<=m_nct)
	{
		for(uint32_t i = 0, j = 0; i < m_nct; ++i, ++j)
		{
			if (j == m_ncr)
			{
				j=0;
			}
			MAP_SId2RId[i] = j;
		}
	}
	else
	{
		for(uint32_t i = 0, j = 0; i < m_ncr; ++i, ++j)
		{
			if (j == m_nct)
			{
				j=0;
			}
			MAP_SId2RId[j] = i;
		}
	}
	std::map<uint32_t,vector<double>> map_rx_position;
	for (uint32_t i = 0; i < m_ncr; i++)
	{
		//水底下node的位置
		vector<double> c_xy;
		double x = m_ur->GetValue (0, m_boundary);
		c_xy.push_back(x);
		double y = m_ur->GetValue (0, m_boundary);
		double z = m_ur->GetValue (0, m_boundary);
		c_xy.push_back(y);
		pos_uan_rx->Add (Vector (x, y, z));
		map_rx_position[i]=c_xy;
	}
//	pos_uan_rx->Add (Vector (0, 0, 0));
//	pos_uan_rx->Add (Vector (1500,0,0));
//	pos_uan_rx->Add (Vector (300,0,0));
	for (uint32_t i = 0; i < m_nct; i++)
	{
		double x = m_ur->GetValue (0, m_boundary);
		double y = m_ur->GetValue (0, m_boundary);
		double z = m_ur->GetValue (0, m_boundary);
		pos_uan_tx->Add (Vector (x, y, z));
	}
//	pos_uan_tx->Add (Vector (0,1500, 0));
//	pos_uan_tx->Add (Vector (1500, 3000, 0));
//	pos_uan_tx->Add (Vector (3000, 3000, 0));
}//end function PositionConfigure

void
ExperimentSFAMA::TxReceiveAck(std::string context, Ptr< const Packet > packet, Mac8Address address)
{
	NS_LOG_FUNCTION(this<<*packet<<address);

	uint32_t tx_id=GetIdFromContext(context);
	NS_LOG_DEBUG("Node "<<tx_id<<" receives ack.");
/*
	SFAMATestTag tag(SFAMATestTag::ACK);
	NS_ASSERT(tag.IsTagType(SFAMATestTag::ACK));
	Ptr<Packet> packet_copy = packet->Copy();
	packet_copy->RemovePacketTag(tag);
	SFAMATestAckHeader ack_header;
	packet_copy->RemoveHeader(ack_header);
	uint32_t rec_Id = ack_header.GetPacketInd();

	if (find(v_recvACKId.begin(),v_recvACKId.end(),rec_Id)==v_recvACKId.end())
	{
		v_recvACKId.push_back(rec_Id);
		double CPktNum=MAP_TID2RecvACKNum[rec_Id];
		CPktNum = CPktNum + 1;
		MAP_TID2RecvNum[tx_id]=CPktNum;
	}*/

	double CPktNum=MAP_TID2RecvACKNum[tx_id];
	CPktNum = CPktNum + 1;
	MAP_TID2RecvACKNum[tx_id]=CPktNum;
}//end function TxReceiveAck

void
ExperimentSFAMA::RxReceivePacket(std::string context, Ptr< const Packet > packet, Mac8Address address)
{
	NS_LOG_FUNCTION(this<<*packet<<address);

	uint32_t rx_id=GetIdFromContext(context);
	NS_LOG_DEBUG("Node "<<rx_id<<" receives data.");
	uint32_t senderId;
	SFAMATestTag tag(SFAMATestTag::MSG);
	Ptr<Packet> packet_copy = packet->Copy();
	packet_copy->RemovePacketTag(tag);
	if (tag.GetTagType()==SFAMATestTag::MSG)
	{
		senderId=tag.GetTxInd();
		SFAMATestHeader recv_header;
		packet_copy->RemoveHeader(recv_header);
		uint32_t pkt_id = recv_header.GetPacketInd();
		uint32_t pkts_num=packet_copy->GetSize()/m_packetSize;
		NS_ASSERT(pkts_num==1);

		if (find(v_recvPktId.begin(),v_recvPktId.end(),pkt_id)==v_recvPktId.end())
		{
			v_recvPktId.push_back(pkt_id);
			double CPktNum=MAP_TID2RecvNum[senderId];

			CPktNum = CPktNum + pkts_num;
			MAP_TID2RecvNum[senderId]=CPktNum;
		}

		SFAMATestAckHeader ack_header;
		ack_header.SetPacketInd(pkt_id);
		SFAMATestTag ack_tag(SFAMATestTag::ACK);
		Ptr<Packet> ack_pkt = Create<Packet>(1);
		ack_pkt->AddHeader(ack_header);
		ack_pkt->AddPacketTag(ack_tag);
		Ptr<UanNetDevice> recv_dev = ncr_uan.Get(rx_id)->GetDevice(0)->GetObject<UanNetDevice>();
	}
}//end function rxreceivepacket

void
ExperimentSFAMA::run()
{
	NS_LOG_FUNCTION (this);
	NS_LOG_INFO("Simulation begins");
	Delay = 0;
	RunTime = 0;
	ncr_Uan_Enr=0;
	nct_Uan_Enr=0;
	rec2send=0;
	NetworkTPut=0;
	m_currentPktId=0;
	GeneratePkts_Sum=0;
	CurrentGenerate_Sum=0;
	Buffer_Sum=0;
	empty_slot=0;
	std::map<uint32_t, double> MAP_TID2throughpy;
	std::map<uint32_t, double> MAP_TID2pdr;
//	m_tFrame=m_M*m_tslot+2*m_boundary*sqrt(3)/1500+16/m_moduleDataRate;
	m_tFrame=m_boundary*sqrt(3)/1500+(5+1+3)*8/m_moduleDataRate;//!Check to set the Frame length
	SeedManager::SetSeed (seed_value);
	SeedManager::SetRun(SeedManager::GetRun()+1);
	for(uint32_t i = 0; i < m_nct; ++i)
	{
		MAP_TID2RecvACKNum[i] = 0;
		MAP_TID2SendNum[i] = 0;
	}

	for(uint32_t i = 0; i < m_ncr; ++i)
	{
		MAP_TID2throughpy[i] = 0;
		MAP_TID2TotalGeneratePkts[i] = 0;
		MAP_TID2CurrentGeneratePkts[i] = 0;
		MAP_TID2BufferSum[i] = 0;
	}

	PositionConfigure();
	nct_uan.Create(m_nct);
	ncr_uan.Create(m_ncr);
	NS_LOG_INFO("Creating the uan node container.");

	MobilityHelper mobility4uan_tx;
	mobility4uan_tx.SetPositionAllocator(pos_uan_tx);
	mobility4uan_tx.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility4uan_tx.Install(nct_uan);

	MobilityHelper mobility4uan_rx;
	mobility4uan_rx.SetPositionAllocator(pos_uan_rx);
	mobility4uan_rx.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility4uan_rx.Install(ncr_uan);
	NS_LOG_INFO("Installing mobility helper.");

	std::string perModel = "ns3::UanPhyPerGenDefault";
	std::string sinrModel = "ns3::UanPhyCalcSinrDefault";
	ObjectFactory obf;
	obf.SetTypeId (perModel);
	Ptr<UanPhyPer> per = obf.Create<UanPhyPer> ();
	obf.SetTypeId (sinrModel);
	Ptr<UanPhyCalcSinr> sinr = obf.Create<UanPhyCalcSinr> ();
	UanTxMode mode;
	mode = UanTxModeFactory::CreateMode (UanTxMode::FSK, m_moduleDataRate,80000, 145000, 5000, 2,"Default mode");
	UanModesList myModes;
	myModes.AppendMode (mode);
	UanHelper uan;
	uan.SetPhy ("ns3::UanPhyGen",
	            "PerModel", PointerValue (per),
	            "SinrModel", PointerValue (sinr),
	            "SupportedModes", UanModesListValue (myModes),
		    	"TxPower",DoubleValue(m_txPowerAcoustic));
	uan.SetMac(m_macType,
			"SlotSkew", DoubleValue (0),
			"SlotLenth",TimeValue(Seconds(m_tFrame)));

	Ptr<UanPropModelThorp> prop = CreateObject<UanPropModelThorp>();
	Ptr<UanChannel> channel = CreateObjectWithAttributes<UanChannel>("PropagationModel",PointerValue(prop));
	devices_nct_uan = uan.Install(nct_uan,channel);
	devices_ncr_uan = uan.Install(ncr_uan,channel);

	NS_LOG_INFO("Creating underwater netdevices.");

	BasicEnergySourceHelper eh;
	eh.Set("BasicEnergySourceInitialEnergyJ",DoubleValue(8000000.0));
	EnergySourceContainer escont = eh.Install(nct_uan);    //发送节点的能量配置
	AcousticModemEnergyModelHelper modemHelper;
	DeviceEnergyModelContainer cont = modemHelper.Install(devices_nct_uan,escont);  // 发送网卡配置

	EnergySourceContainer escont4sink = eh.Install(ncr_uan);
	AcousticModemEnergyModelHelper modemHelper4sink;
	DeviceEnergyModelContainer cont4sink = modemHelper4sink.Install(devices_ncr_uan,escont4sink);  // 发送网卡配置

	for(uint32_t i = ncr_uan.Get(0)->GetId(); i < ncr_uan.Get(0)->GetId() + ncr_uan.GetN(); ++i)
		{

			std::ostringstream cbpath1;
			NS_LOG_DEBUG("Rx node id "<<i<<".");
			cbpath1<<"/NodeList/"<<i<<"/DeviceList/*/$ns3::UanNetDevice/Rx";
			Config::Connect(cbpath1.str(),MakeCallback(&ExperimentSFAMA::RxReceivePacket, this));
			std::ostringstream cbpath2;
			cbpath2<<"/NodeList/"<<i<<"/DeviceList/*/$ns3::UanNetDevice/Mac/$ns3::UanMacSFAMA/Enqueue";
			Config::Connect(cbpath2.str(),MakeCallback(&ExperimentSFAMA::MacEnqueueCB, this));

		}
	for(uint32_t i = nct_uan.Get(0)->GetId(); i < nct_uan.Get(0)->GetId() + nct_uan.GetN(); ++i)
		{

			std::ostringstream cbpath1;
			NS_LOG_DEBUG("Tx node id "<<i<<".");
			cbpath1<<"/NodeList/"<<i<<"/DeviceList/*/$ns3::UanNetDevice/Rx";
			Config::Connect(cbpath1.str(),MakeCallback(&ExperimentSFAMA::TxReceiveAck, this));
			std::ostringstream cbpath2;
			cbpath2<<"/NodeList/"<<i<<"/DeviceList/*/$ns3::UanNetDevice/Mac/$ns3::UanMacSFAMA/Enqueue";
			Config::Connect(cbpath2.str(),MakeCallback(&ExperimentSFAMA::MacEnqueueCB, this));
		}
	for (uint32_t i = nct_uan.Get(0)->GetId(); i < nct_uan.Get(0)->GetId() + nct_uan.GetN(); ++i)
	{
		Time start_T = Seconds(0);
		Simulator::Schedule( start_T, &ExperimentSFAMA::PoissonTrafficGenerate, this, nct_uan.Get(i));
	}//end loop for


//	AnimationInterface anim("SFAMA-TEST.xml");
//	std::string m_asciitracefile("uan-sfama.asc");
//	std::ofstream ascii (m_asciitracefile.c_str ());
//	uan.EnableAsciiAll(ascii);
	Simulator::Stop (Seconds(m_StopTime+20));
	Simulator::Run ();
	//NS_LOG_UNCOND("Simulation run "<<m_StopTime<<" seconds.");

	double total_recv=0;
	double total_send=0;
	double Recv_ACK = 0;

	for(uint32_t i = 0; i < m_nct; ++i)
	{
		double c_txpkts = MAP_TID2SendNum[i];
		total_send+=c_txpkts;
		double c_rxpkts = MAP_TID2RecvNum[i];
		total_recv+=c_rxpkts;
		Recv_ACK += MAP_TID2RecvACKNum[i];

		double c_pdr =c_rxpkts/c_txpkts;
		MAP_TID2throughpy[i] = c_rxpkts/m_StopTime;
		if (i==0)
		{
			test_pdr=c_pdr;
			//NS_LOG_UNCOND("node o pdr is "<<c_pdr<<" throughput is "<<c_rxpkts/m_StopTime);
		}
		if (c_rxpkts!=0)
		{
			MAP_TID2pdr[i] = c_pdr;
		}
		else
		{
			MAP_TID2pdr[i]=0;
		}
	}

	double check_genereatesum=0;
	for(uint32_t i = 0; i < m_nct; ++i)
	{
/*		double c_rec2send=MAP_TID2pdr[i];
		rec2send +=c_rec2send;
		double c_networktput=MAP_TID2throughpy[i];
		NetworkTPut += c_networktput;*/
		rec2send += MAP_TID2pdr[i];
		check_genereatesum += MAP_TID2TotalGeneratePkts[i];
		Buffer_Sum+=MAP_TID2BufferSum[i];
		Buffer_Sum+=MAP_TID2CurrentGeneratePkts[i];
	}

	RunTime = Simulator::Now().GetHours();
//	rec2send=total_recv/GeneratePkts_Sum;
	rec2send=total_recv/total_send;
	SinkrecvNum = total_recv;
	SrcsendNum = total_send;
	double pdr = rec2send;
	double ps = total_recv/total_send;
	NetworkTPut = total_recv/m_StopTime;

	NS_LOG_UNCOND("the delivery rate is "<<pdr<<", success transmission rate is "<<ps<<" and throughput is "<<NetworkTPut<<".");

	Simulator::Destroy ();
}//end run

string GetStringName(std::stringstream const& ss)
{
	string c_string = ss.str();
	string::iterator   it;
   for (it =c_string.begin(); it != c_string.end(); ++it)
	{
	   if ( *it == '.')
		   {
		   c_string.erase(it);
	   }
	}
	return 	c_string;
}//end GetStringName

void RunSimulationIDU(double stoptime, double Tsinr, double boundary, uint32_t current_seed, double lambda, double packetsize, uint32_t nt_num, uint32_t nr_num, double max_skew, uint32_t Times)
{
	std::stringstream DeliveryRatio_name;
	std::stringstream EnergyConsumption_name;
	std::stringstream End2EndDelay_name;
	std::stringstream ThrouPut_name;


	double depth = 70;
	double moduleDataRate=4000;
	double txPowerAcoustic = 300;
	double Max_skew=max_skew;
	uint32_t SimulatorTimes = Times;

	vector<Gnuplot2dDataset> v_rec2Send;
	vector<Gnuplot2dDataset> v_Energyconsumed;
	vector<Gnuplot2dDataset> v_ThroughPut;
	vector<Gnuplot2dDataset> v_Sendsum;
	vector<Gnuplot2dDataset> v_Recvsum;
	vector<Gnuplot2dDataset> v_Generatesum;

	//Raw data
	std::stringstream current_DeliveryRatio_name;
	std::stringstream current_EnergyConsumption_name;
	std::stringstream current_End2EndDelay_name;
	std::stringstream current_Sendsum_name;
	std::stringstream current_Recvsum_name;
	std::stringstream current_ThrouPut_name;
	std::stringstream current_Generate_name;
	std::stringstream current_Runtime_name;
	std::stringstream current_nct_Enr_name;

	int tf_l=10;
	int tf_r=10;

	current_DeliveryRatio_name<<"SFAMA_lambda_"<<lambda<<"_boundary_"<<boundary<<"__nct"<<nt_num<<"_ncr"<<nr_num<<"_tf"<<tf_l<<"-"<<tf_r<<"_pdr";
	current_EnergyConsumption_name<<"SFAMA_lambda_"<<lambda<<"_boundary_"<<boundary<<"_nct"<<nt_num<<"_ncr"<<nr_num<<"_tf"<<tf_l<<"-"<<tf_r<<"_enr";
	current_End2EndDelay_name<<"SFAMA_lambda_"<<lambda<<"_boundary_"<<boundary<<"_nct"<<nt_num<<"_ncr"<<nr_num<<"_tf"<<tf_l<<"-"<<tf_r<<"_delay";
	current_ThrouPut_name<<"SFAMA_lambda_"<<lambda<<"_boundary_"<<boundary<<"_nct"<<nt_num<<"_ncr"<<nr_num<<"_tf"<<tf_l<<"-"<<tf_r<<"_throughput";
	current_Generate_name<<"SFAMA_lambda_"<<lambda<<"_boundary_"<<boundary<<"_nct"<<nt_num<<"_ncr"<<nr_num<<"_tf"<<tf_l<<"-"<<tf_r<<"_generate";
	current_Runtime_name<<"SFAMA_lambda_"<<lambda<<"_boundary_"<<boundary<<"_nct"<<nt_num<<"_ncr"<<nr_num<<"_tf"<<tf_l<<"-"<<tf_r<<"_runtime";
	current_Sendsum_name<<"SFAMA_lambda_"<<lambda<<"_boundary_"<<boundary<<"_nct"<<nt_num<<"_ncr"<<nr_num<<"_tf"<<tf_l<<"-"<<tf_r<<"_sendsum";
	current_Recvsum_name<<"SFAMA_lambda_"<<lambda<<"_boundary_"<<boundary<<"_nct"<<nt_num<<"_ncr"<<nr_num<<"_tf"<<tf_l<<"-"<<tf_r<<"_recvsum";

	std::vector<Gnuplot3dDataset> v_raw_rec;
	std::vector<Gnuplot3dDataset> v_raw_runtime;
	std::vector<Gnuplot3dDataset> v_raw_energy;
	std::vector<Gnuplot3dDataset> v_raw_through;
	std::vector<Gnuplot3dDataset> v_raw_delay;
	std::vector<Gnuplot3dDataset> v_raw_nct_Enr;
	std::vector<Gnuplot3dDataset> v_raw_ncr_Enr;
	std::vector<Gnuplot3dDataset> v_raw_sendsum;
	std::vector<Gnuplot3dDataset> v_raw_recvsum;
	std::vector<Gnuplot3dDataset> v_raw_generatesum;

	vector<Gnuplot2dDataset> v_rec2Send2D;
	vector<Gnuplot2dDataset> v_througput2D;


	for (int i = tf_l; i<=tf_r; i +=1)
	{
    	uint32_t count = SimulatorTimes;
		double rec2sendG = 0;
		double test_pdr = 0;
		double throughput = 0;
		double sendsum = 0;
		double recvsum = 0;
		double generatesum = 0;
		double empty_slot = 0;

    	Gnuplot2dDataset MultiSkew_rec2Send;
    	Gnuplot2dDataset MultiSkew_Throughput;

    	std::vector<double> v_current_rec;
    	std::vector<double> v_current_test_rec;
    	std::vector<double> v_current_runtime;
    	std::vector<double> v_current_energy;
    	std::vector<double> v_current_generate;
    	std::vector<double> v_current_throughput;
    	std::vector<double> v_current_emptyslots;

    	Ptr<UniformRandomVariable> ur = CreateObject<UniformRandomVariable> ();
    	ur->SetAttribute ("Min", DoubleValue (0));
    	ur->SetAttribute ("Max", DoubleValue (boundary));

    	Ptr<ExponentialRandomVariable> er=CreateObjectWithAttributes<ExponentialRandomVariable>("Mean",DoubleValue(1/lambda));

    	double c_packetSize=packetsize*i-3-3-6;//5个bit是用来避免schedule的碰撞
		while(count)
		{
			Gnuplot3dDataset MultiSkew_current_rec2Send;
			Gnuplot3dDataset MultiSkew_current_Energyconsumed;
			Gnuplot3dDataset MultiSkew_current_Runtime;
			Gnuplot3dDataset MultiSkew_current_Throughput;
			Gnuplot3dDataset MultiSkew_current_Delay;
			Gnuplot3dDataset MultiSkew_current_nct_Enr;
			Gnuplot3dDataset MultiSkew_current_ncr_Enr;
			Gnuplot3dDataset MultiSkew_current_Sendsum;
			Gnuplot3dDataset MultiSkew_current_Recvsum;
			Gnuplot3dDataset MultiSkew_current_Generatesum;

			ExperimentSFAMA exp;
			exp.m_nct=nt_num;
			exp.m_ncr=nr_num;
			exp.m_moduleDataRate=moduleDataRate;
			exp.m_txPowerAcoustic=txPowerAcoustic;
			exp.m_boundary=boundary;
			exp.m_depth=depth;
			exp.m_packetSize=c_packetSize;
			exp.m_Tsinr=Tsinr;
			exp.m_ur=ur;
			exp.m_er=er;
			//double t_slot=double(i)/10;
			exp.m_lambda=lambda;
			exp.seed_value = current_seed;
			exp.m_StopTime = stoptime;
			SeedManager::SetRun(SeedManager::GetRun()+1);
			std::cout<<"Current simulation "<<count<<" begins."<<std::endl;
			time_t  t_begin= time (NULL);
			exp.run();
			time_t t_end = time(NULL);
			std::cout<<"Simulation "<<count<<" finishs and consumes "<<std::setprecision(5)<<difftime(t_end,t_begin)<<" seconds."<<std::endl;
			rec2sendG = exp.rec2send;
			test_pdr = exp.test_pdr;
			throughput = exp.NetworkTPut;
			sendsum = exp.SrcsendNum;
			recvsum = exp.SinkrecvNum;
			generatesum = exp.GeneratePkts_Sum;
			empty_slot = exp.empty_slot;

			//exp.ExperimentClear();

			MultiSkew_current_rec2Send.Add(max_skew, double(i)/10, rec2sendG);
			MultiSkew_current_Throughput.Add(max_skew, double(i)/10, throughput);
			MultiSkew_current_Sendsum.Add(max_skew, double(i)/10, sendsum);
			MultiSkew_current_Recvsum.Add(max_skew, double(i)/10, recvsum);
			MultiSkew_current_Generatesum.Add(max_skew, double(i)/10, generatesum);

			v_raw_rec.push_back(MultiSkew_current_rec2Send);
			v_raw_through.push_back(MultiSkew_current_Throughput);
			v_raw_sendsum.push_back(MultiSkew_current_Sendsum);
			v_raw_recvsum.push_back(MultiSkew_current_Recvsum);
			v_raw_generatesum.push_back(MultiSkew_current_Generatesum);

			v_current_rec.push_back(rec2sendG);
			v_current_test_rec.push_back(test_pdr);
			v_current_throughput.push_back(throughput);
			v_current_emptyslots.push_back(empty_slot);
			v_current_generate.push_back(generatesum);
			count--;

		}//end simulation times loop

		double c_tf = double(i);
		double c_rec =  accumulate(v_current_rec.begin(), v_current_rec.end(), double(0))/v_current_rec.size();
		double c_testrec =  accumulate(v_current_test_rec.begin(), v_current_test_rec.end(), double(0))/v_current_test_rec.size();

		double c_througput =  accumulate(v_current_throughput.begin(), v_current_throughput.end(), double(0))/v_current_throughput.size();
		double c_empty_slots = accumulate(v_current_emptyslots.begin(), v_current_emptyslots.end(), double(0))/v_current_emptyslots.size();

		double generate_mean = accumulate(v_current_generate.begin(), v_current_generate.end(), double(0))/v_current_generate.size();
		double emptyrate = 1-(c_empty_slots/((stoptime*nt_num)/((c_tf+Max_skew*10)/10)));

		NS_LOG_UNCOND("In SFAMA test, nt is "<<nt_num<<",1/lamda is "<<1/lambda<< ",tf is "<<c_tf/10<<", generate "<<generate_mean<<" pkts and the delivery rate is "<<c_rec<<" and test pdr is "<<c_testrec<<", throughput is "<<c_througput<<", have "<<c_empty_slots<<" empty slots, send rate is "<<emptyrate<<".");
		v_current_rec.clear();
		v_current_throughput.clear();

		MultiSkew_rec2Send.Add(8*c_packetSize/moduleDataRate, c_rec);
		MultiSkew_Throughput.Add(8*c_packetSize/moduleDataRate, c_througput);
	}//end skew loop
	stringstream Raw_current_DeliveryRatio_name("Raw_");
	Raw_current_DeliveryRatio_name<<current_DeliveryRatio_name.str();
	stringstream Raw_current_ThrouPut_name("Raw_");
	Raw_current_ThrouPut_name<<current_ThrouPut_name.str();

	string str_DeliveryRatio_name = GetStringName(current_DeliveryRatio_name);
	string str_ThrouPut_name = GetStringName(current_ThrouPut_name);
	string str_Sendsum_name = GetStringName(current_Sendsum_name);
	string str_Recvsum_name = GetStringName(current_Recvsum_name);
	string str_Generatesum_name = GetStringName(current_Generate_name);

	v_raw_rec.clear();
	v_raw_through.clear();
	v_raw_sendsum.clear();
	v_raw_recvsum.clear();

	v_rec2Send2D.clear();
	v_througput2D.clear();

	current_DeliveryRatio_name.str("");
	current_EnergyConsumption_name.str("");
	current_End2EndDelay_name.str("");
	current_ThrouPut_name.str("");
	current_Runtime_name.str("");
	current_Sendsum_name.str("");
	current_Recvsum_name.str("");
}//end run simulation

int
main(int argc,char **argv)

{
    //logo prefix
    LogComponentEnableAll (LOG_PREFIX_TIME);
    LogComponentEnableAll (LOG_PREFIX_NODE);
    LogComponentEnableAll (LOG_PREFIX_FUNC);
    LogComponentEnableAll (LOG_PREFIX_LEVEL);

//    LogComponentEnable ("UanMacSFAMA", LOG_LEVEL_DEBUG);
//    LogComponentEnable ("UanNetDevice", LOG_LEVEL_DEBUG);
//    LogComponentEnable ("UanChannel", LOG_LEVEL_DEBUG);
//    LogComponentEnable ("UanSFAMAExperiment", LOG_LEVEL_ALL);
//    LogComponentEnable ("UanPhyGen", LOG_LEVEL_DEBUG);
//    LogComponentEnable ("UanTransducerHd", LOG_LEVEL_ALL);
//
//
//    LogComponentEnable ("UanSFAMAExperiment", LOG_LEVEL_ALL);

    uint32_t seed_value = 19970420;
    double packetsize = 50; //size of one packet
	double tsinr=10;   
	double bound = 3000;  //
    double stoptime = 20000; //total time of sim

    uint32_t sim_Times =100;
    vector<double> v_lambda = {1.0/22};
    vector<uint32_t> v_nct={12};
    vector<uint32_t> v_ncr={12};
    vector<double> v_skew = {1.4};
    //bool is_bradcast=false;
	for (std::vector<double>::iterator i_lb = v_lambda.begin(); i_lb != v_lambda.end(); ++i_lb)
	{
		for (std::vector<uint32_t>::iterator i_t = v_nct.begin(); i_t != v_nct.end(); ++i_t)
		{
			for (std::vector<uint32_t>::iterator i_r = v_ncr.begin(); i_r != v_ncr.end(); ++i_r)
			{
				for (std::vector<double>::iterator i = v_skew.begin(); i != v_skew.end(); ++i)
				{
					RunSimulationIDU(stoptime, tsinr, bound, seed_value, *i_lb, packetsize, *i_t, *i_r, *i, sim_Times);
					//
				}
			}
		}
	}


    NS_LOG_UNCOND("Simulation finish");

}//end main
