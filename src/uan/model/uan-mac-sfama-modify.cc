/*
 * uan-mac-sfama.cc
 *
 *  Created on: Mar 29, 2021
 *      Author: SCUT-arcca-Wang
 */
#include "uan-mac-sfama-modify.h"
#include "ns3/attribute.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/nstime.h"
#include "ns3/uan-header-common.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/log.h"
using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("UanMacSFAMA");

NS_OBJECT_ENSURE_REGISTERED (UanMacSFAMA);
NS_OBJECT_ENSURE_REGISTERED (UanSFamaHeader);

UanSFamaHeader::UanSFamaHeader () :
		m_pType(UanSFamaHeader::RTS)
{
}

UanSFamaHeader::~UanSFamaHeader ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
UanSFamaHeader::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::UanMacSFAMA::UanSFamaHeader")
    .SetParent<Header> ()
    .AddConstructor<UanSFamaHeader> ()
  ;
  return tid;
}

TypeId
UanSFamaHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

uint32_t
UanSFamaHeader::GetSerializedSize () const
{
	NS_LOG_FUNCTION (this);

	return sizeof (uint8_t);
}

void
UanSFamaHeader::Serialize (Buffer::Iterator i ) const
{
	NS_LOG_FUNCTION (this);
	i.WriteU8 (m_pType);
}

uint32_t
UanSFamaHeader::Deserialize (Buffer::Iterator start )
{
	NS_LOG_FUNCTION (this);
	Buffer::Iterator i = start;
	uint8_t type = i.ReadU8();
	switch (type)
	{
		case FAMA_DATA:
		case RTS:
		case CTS:
		case ACK:
		  {
			  m_pType = (PacketType) type;
			break;
		  }
	}
	uint8_t dist = i.GetDistanceFrom (start);
	NS_ASSERT (dist == GetSerializedSize ());
	return dist;
}

void
UanSFamaHeader::Print (std::ostream &os ) const
{
	  switch (m_pType)
	    {
	    case FAMA_DATA:
			{
				 os << "This is FAMA Data packet";
			  break;
			}
	    case RTS:
			{
				 os << "This is FAMA RTS packet";
			break;
			}
		case CTS:
			{
				 os << "This is FAMA CTS packet";
			break;
			}
	    case ACK:
			{
				 os << "This is FAMA ACK packet";
			break;
			}
	    }
	  ;
}

UanSFamaHeader::PacketType
UanSFamaHeader::GetType ()
{
  return m_pType;
}

bool
UanSFamaHeader::IsType (PacketType type)
{
  return m_pType == type;
}

void
UanSFamaHeader::SetType (PacketType tag)
{
	m_pType = tag;
}


//-----------------------------------------------------------SFAMA-----------------------------------------------------------------
/*UanMacSFMAMA implentation*/

UanMacSFAMA::UanMacSFAMA ()
  : UanMac (),
    m_phy (0), //PHY layer attached to this MAC.
    m_state (IDLE), // init state = IDLE
    m_pktTx (0), //Next packet to send.
	m_proNumber (0), //Next packet sent protocolnumber
	m_holdPacket (false), //Check whether has got a packet
	m_meanSkew (1), //Slot's skew mean?
	m_cleared (false) //Flag when we've been cleared
{
	  m_rv = CreateObject<UniformRandomVariable> ();  //Provides uniform random variable for start skew.
	  m_struct.Struct_Role=SFAMA_STRUCT::IDLE;
	  m_struct.Struct_RTS=0;
	  m_struct.Struct_CTS=0;
	  m_struct.Struct_DATA=0;
	  m_struct.Struct_ACK=0;
	  m_struct.Struct_backoff=0;
}

UanMacSFAMA::~UanMacSFAMA ()
{
}

void
UanMacSFAMA::Clear ()
{
  if (m_cleared)
    {
      return;
    }
  m_cleared = true;
  m_pktTx = 0;
  if (m_phy)
    {
      m_phy->Clear ();
      m_phy = 0;
    }
  m_SlotTimer.Cancel ();
  m_waitCTSTimer.Cancel ();
  m_waitXTimer.Cancel ();
  m_waitACKTimer.Cancel ();
  m_waitDATATimer.Cancel ();
  m_backoffTimer.Cancel ();
}

TypeId
UanMacSFAMA::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanMacSFAMA")
    .SetParent<UanMac> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanMacSFAMA> ()
    .AddAttribute ("SlotLenth",
                   "Time slot duration for MAC.",
                   TimeValue (Seconds (1)),
                   MakeTimeAccessor (&UanMacSFAMA::m_slot),
                   MakeTimeChecker ())
	.AddAttribute ("SlotSkew",
				   "Mean of time slot skew duration for MAC in second.",
				   DoubleValue (1),
				   MakeDoubleAccessor (&UanMacSFAMA::m_meanSkew),
				   MakeDoubleChecker<double>  ())
	.AddTraceSource ("Enqueue",
					  "A packet arrived at the MAC for transmission.",
					  MakeTraceSourceAccessor (&UanMacSFAMA::m_enqueueLogger),
					  "ns3::UanMacSFAMA::QueueTracedCallback")
  ;
  return tid;
}

Address
UanMacSFAMA::GetAddress ()
{
  return this->m_address;
}//end GetAddress

void
UanMacSFAMA::SetAddress (Mac8Address addr)
{
  m_address = addr;
}//end SetAddress

bool
UanMacSFAMA::Enqueue (Ptr<Packet> packet, uint16_t protocolNumber, const Address &dest )
{
	NS_LOG_FUNCTION(this);
	if(dest == Mac8Address::GetBroadcast())
	{
		switch (m_state)
		{
			case IDLE:
			{
				if (!m_holdPacket)
				{
					NS_ASSERT(!m_pktTx);
					m_pktTx = packet->Copy();
					m_holdPacket=true;
					m_enqueueLogger(m_pktTx);
					m_proNumber = protocolNumber;
					m_struct.Struct_Role=SFAMA_STRUCT::Sender;
					m_struct.Counter_Addr=dest; //?
				}
				else
				{
					NS_ASSERT(m_pktTx);
					NS_LOG_DEBUG("Current node already have held a packet from this slot ready to send.");
					return false;
				}
				break;
			}
			case WAIT_X:
			{
				if (m_struct.Struct_Role==SFAMA_STRUCT::IDLE)
				{
					if (!m_holdPacket)
					{
						NS_ASSERT(!m_pktTx);
						m_pktTx = packet->Copy();
						m_holdPacket=true;
						m_enqueueLogger(m_pktTx);
						m_proNumber = protocolNumber;
						m_struct.Struct_Role=SFAMA_STRUCT::Sender;
						m_struct.Counter_Addr=dest;
					}
					else
					{
						NS_ASSERT(m_pktTx);
						NS_LOG_DEBUG("Current node already have held a packet from this slot ready to send.");
						return false;
					}
				}
				else
				{
					if (m_struct.Struct_Role==SFAMA_STRUCT::Receiver)
					{
						NS_LOG_DEBUG("Current node is acting as receiver.");
					}
					else if (m_struct.Struct_Role==SFAMA_STRUCT::Sender)
					{
						NS_LOG_DEBUG("Current node is acting as sender.");
					}
					return false;
				}
				break;
			}
			case WAIT_CTS:
				NS_ASSERT(m_struct.Struct_Role!=SFAMA_STRUCT::IDLE && m_struct.Struct_RTS);
				NS_LOG_DEBUG ("Source node is waiting the CTS message for sending previous packets.");
				return false;
			case WAIT_ACK:
				NS_ASSERT(m_struct.Struct_Role!=SFAMA_STRUCT::IDLE && m_struct.Struct_RTS && m_struct.Struct_CTS && m_struct.Struct_DATA);
				NS_LOG_DEBUG ("Source node is waiting the ACK message of previous packets.");
				return false;
			case WAIT_DATA:
				NS_ASSERT(m_struct.Struct_Role!=SFAMA_STRUCT::IDLE && m_struct.Struct_RTS && m_struct.Struct_CTS);
				NS_LOG_DEBUG ("Dest node is waiting the DATA message of previous packets.");
				return false;
			case BACKOFF:
				NS_ASSERT(m_struct.Struct_Role!=SFAMA_STRUCT::IDLE && m_struct.Struct_RTS && !m_struct.Struct_CTS);
				NS_LOG_DEBUG ("Source node is in backoff period to send to RTS message of previous packets again.");
				return false;
			default:
				NS_LOG_DEBUG ("MAC " << GetAddress () << " Starting enqueue SOMETHING ELSE");
				return false;
		}
	return true;
	}

	switch (m_state)
	{
	case IDLE:
	{
		if (!m_holdPacket)
		{
			NS_ASSERT(!m_pktTx);
			m_pktTx = packet->Copy();
			m_holdPacket=true;
			m_enqueueLogger(m_pktTx);
			m_proNumber = protocolNumber;
			m_struct.Struct_Role=SFAMA_STRUCT::Sender;
			m_struct.Counter_Addr=dest;
		}
		else
		{
			NS_ASSERT(m_pktTx);
			NS_LOG_DEBUG("Current node already have held a packet from this slot ready to send.");
			return false;
		}
		break;
	}
	case WAIT_X:
	{
		if (m_struct.Struct_Role==SFAMA_STRUCT::IDLE)
		{
			if (!m_holdPacket)
			{
				NS_ASSERT(!m_pktTx);
				m_pktTx = packet->Copy();
				m_holdPacket=true;
				m_enqueueLogger(m_pktTx);
				m_proNumber = protocolNumber;
				m_struct.Struct_Role=SFAMA_STRUCT::Sender;
				m_struct.Counter_Addr=dest;
				//m_waitXTimer.Cancel();
			}
			else
			{
				NS_ASSERT(m_pktTx);
				NS_LOG_DEBUG("Current node already have held a packet from this slot ready to send.");
				return false;
			}
		}
		else
		{
			if (m_struct.Struct_Role==SFAMA_STRUCT::Receiver)
			{
				NS_LOG_DEBUG("Current node is acting as receiver.");
			}
			else if (m_struct.Struct_Role==SFAMA_STRUCT::Sender)
			{
				NS_LOG_DEBUG("Current node is acting as sender.");
			}
			return false;
		}

		break;
	}
	case WAIT_CTS:
		NS_ASSERT(m_struct.Struct_Role!=SFAMA_STRUCT::IDLE && m_struct.Struct_RTS);
		NS_LOG_DEBUG ("Source node is waiting the CTS message for sending previous packets.");
		return false;
	case WAIT_ACK:
		NS_ASSERT(m_struct.Struct_Role!=SFAMA_STRUCT::IDLE && m_struct.Struct_RTS && m_struct.Struct_CTS && m_struct.Struct_DATA);
		NS_LOG_DEBUG ("Source node is waiting the ACK message of previous packets.");
		return false;
	case WAIT_DATA:
		NS_ASSERT(m_struct.Struct_Role!=SFAMA_STRUCT::IDLE && m_struct.Struct_RTS && m_struct.Struct_CTS);
		NS_LOG_DEBUG ("Dest node is waiting the DATA message of previous packets.");
		return false;
	case BACKOFF:
		NS_ASSERT(m_struct.Struct_Role!=SFAMA_STRUCT::IDLE && m_struct.Struct_RTS && !m_struct.Struct_CTS);
		NS_LOG_DEBUG ("Source node is in backoff period to send to RTS message of previous packets again.");
		return false;
	default:
	      NS_LOG_DEBUG ("MAC " << GetAddress () << " Starting enqueue SOMETHING ELSE");
	      return false;
	}
	return true;
}//end Enqueue

void
UanMacSFAMA::SetForwardUpCb (Callback<void, Ptr<Packet>, uint16_t, const Mac8Address&> cb)
{
  m_forwardUpCb = cb;
}//end SetForwardUpCb

void
UanMacSFAMA::AttachPhy (Ptr<UanPhy> phy)
{
  m_phy = phy;
  m_phy->SetReceiveOkCallback (MakeCallback (&UanMacSFAMA::PhyRxPacketGood, this));
  m_phy->SetReceiveErrorCallback (MakeCallback (&UanMacSFAMA::PhyRxPacketError, this));

  Time startDelay = Seconds(m_rv->GetValue(0, m_meanSkew));
  Simulator::Schedule (startDelay, &UanMacSFAMA::Start,this);
}//end AttachPhy

Address
UanMacSFAMA::GetBroadcast (void) const
{
  return Mac8Address::GetBroadcast ();
}//end GetBroadcast


void
UanMacSFAMA::Start()
{
	NS_LOG_FUNCTION(this);
	NS_LOG_DEBUG("Node begin slot at "<<Simulator::Now().GetSeconds()<<".");
//	m_SlotTimer.SetDelay(m_slot);
	m_SlotTimer.SetFunction(&UanMacSFAMA::SlottingSchedule,this);
	m_SlotTimer.Schedule(m_slot);
	m_rv->SetAttribute ("Min", DoubleValue (0));
	m_rv->SetAttribute ("Max", DoubleValue (m_meanSkew));

	m_waitCTSTimer.SetFunction(&UanMacSFAMA::WaitCTSFinish,this);
	m_waitXTimer.SetFunction(&UanMacSFAMA::WaitXPacketFinish,this);
	m_waitACKTimer.SetFunction(&UanMacSFAMA::WaitACKFinish,this);
	m_waitDATATimer.SetFunction(&UanMacSFAMA::WaitDATAFinish,this);
	m_backoffTimer.SetFunction(&UanMacSFAMA::BackoffFinish,this);

	Ptr<UanNetDevice> c_device =m_phy->GetDevice();
	m_nodeid=c_device->GetNode()->GetId();
}//end Start

void
UanMacSFAMA::PhySendPacket (Ptr<Packet> pkt, UanSFamaHeader::PacketType header_type)
{
	NS_LOG_FUNCTION(this);
    UanHeaderCommon header;
    header.SetDest (Mac8Address::ConvertFrom (m_struct.Counter_Addr));
    header.SetSrc (m_address);
    header.SetType (0);
	header.SetProtocolNumber(m_proNumber);

    UanSFamaHeader mac_header;
    mac_header.SetType(header_type);

    pkt->AddHeader(mac_header);
    pkt->AddHeader (header);
	NS_LOG_DEBUG("Node "<<m_nodeid<<" send "<<mac_header<<".");

    NS_ASSERT(!m_phy->IsStateTx());
    NS_LOG_DEBUG ("Time " << Simulator::Now ().GetSeconds () << ": Addr " << GetAddress () << ": Enqueuing new packet while idle (sending)");
    NS_ASSERT (m_phy->GetTransducer ()->GetArrivalList ().size () == 0 && !m_phy->IsStateTx ());

    m_phy->SendPacket (pkt,GetTxModeIndex ());
}//end PhySendPacket

void
UanMacSFAMA::LongWaitCheck(Time long_delay)
{
	NS_LOG_FUNCTION(this);
	NS_LOG_DEBUG(this);
	Time delay=long_delay+m_SlotTimer.GetDelayLeft();
    if(m_waitXTimer.IsExpired())
    {
  	    m_state=WAIT_X;

  	    m_waitXTimer.Schedule(delay);
//        m_waitXTimer.SetDelay(long_delay);
    }
    else
    {

  	  NS_ASSERT_MSG(m_state==WAIT_X,"This node "<<m_nodeid<<"'th state should be wait!!!");
  	  m_waitXTimer.Cancel();
  	  m_waitXTimer.Schedule(delay);
//  	  m_waitXTimer.SetDelay(long_delay);
    }
	NS_LOG_DEBUG("Node "<<m_nodeid<<" change state into WAIT_X to wait for "<<m_waitXTimer.GetDelayLeft().GetSeconds()<<"s.");

}//end LongWaitCheck

bool
UanMacSFAMA::StructResume()
{
	NS_LOG_FUNCTION (this);
	NS_ASSERT(m_struct.Struct_Role!=SFAMA_STRUCT::IDLE && m_struct.Struct_RTS && m_struct.Struct_CTS && m_struct.Struct_DATA && m_struct.Struct_ACK);
	m_struct.Struct_Role=SFAMA_STRUCT::IDLE;
	m_struct.Struct_RTS=0;
	m_struct.Struct_CTS=0;
	m_struct.Struct_DATA=0;
	m_struct.Struct_ACK=0;
	m_struct.Struct_backoff=0;
	m_holdPacket=0;
	m_pktTx=0;
	return true;
}//end StructResume

bool
UanMacSFAMA::ADTimeoutStructResume()
{
	NS_LOG_FUNCTION (this);
	NS_ASSERT(m_struct.Struct_Role!=SFAMA_STRUCT::IDLE && m_struct.Struct_RTS && m_struct.Struct_CTS && (!m_struct.Struct_DATA || !m_struct.Struct_ACK));
	m_struct.Struct_Role=SFAMA_STRUCT::IDLE;
	m_struct.Struct_RTS=0;
	m_struct.Struct_CTS=0;
	m_struct.Struct_DATA=0;
	m_struct.Struct_ACK=0;
	m_struct.Struct_backoff=0;
	m_holdPacket=0;
	m_pktTx=0;
	return true;
}//end ADTimeoutStructResume

void
UanMacSFAMA::SlottingSchedule() //在每个slot开始时，进行相应的动作并对m_state状态进行切换
{
//	if(Simulator::Now().GetSeconds()>23){
//		NS_LOG_DEBUG(">13");
//	};
	NS_LOG_FUNCTION(this);
	NS_LOG_DEBUG("Node "<<m_nodeid<<" start new slot at "<<Simulator::Now().GetSeconds()<<".");
	switch (m_state)
	{
	case IDLE:
	{
		if (m_struct.Struct_Role==SFAMA_STRUCT::Sender)
		{
			NS_ASSERT(m_pktTx && !m_struct.Struct_RTS);
			Ptr<Packet> rts_packet = Create<Packet>(1);
			PhySendPacket (rts_packet,UanSFamaHeader::RTS);
            m_struct.Struct_RTS=true;
            m_state = WAIT_CTS;
            Time wait_cts_delay=2*m_slot;
            NS_ASSERT(!m_waitCTSTimer.IsRunning());
//            m_waitCTSTimer.SetDelay(wait_cts_delay);
            m_waitCTSTimer.Schedule(wait_cts_delay);
		}
		else if(m_struct.Struct_Role==SFAMA_STRUCT::Receiver)
		{
			NS_ASSERT(m_struct.Struct_RTS);
			if (m_struct.Struct_CTS )
			{
				NS_LOG_DEBUG("Sender resend CTS.");
			}
			Ptr<Packet> cts_packet = Create<Packet>(5);
			PhySendPacket (cts_packet,UanSFamaHeader::CTS);
            m_struct.Struct_CTS=true;
            m_state = WAIT_DATA;
            Time wait_data_delay=3*m_slot;//!need to be checked the value of delay from the paper.
            NS_ASSERT(!m_waitDATATimer.IsRunning());
            m_waitDATATimer.Schedule(wait_data_delay);
//            m_waitDATATimer.SetDelay(wait_data_delay);
		}

		break;
	}
	case WAIT_X:
		break;
	case WAIT_CTS:
		NS_ASSERT(m_struct.Struct_Role==SFAMA_STRUCT::Sender);

		if (m_struct.Struct_CTS)
		{
			NS_ASSERT(m_waitCTSTimer.IsExpired());
//			m_waitCTSTimer.Cancel();//!node received data with 2slot delays
			NS_LOG_DEBUG ("Source node has received the CTS message and ready to send data packets.");

			PhySendPacket (m_pktTx,UanSFamaHeader::FAMA_DATA);
            m_struct.Struct_DATA=true;
            m_state = WAIT_ACK;

            Time wait_data_delay=3*m_slot;//!need to be checked the value of delay from the paper.
            NS_ASSERT(!m_waitACKTimer.IsRunning());
            m_waitACKTimer.Schedule(wait_data_delay);
//            m_waitDATATimer.SetDelay(wait_data_delay);
		}
		else if(m_struct.Struct_backoff)
		{
			NS_ASSERT(m_waitCTSTimer.IsExpired());
			NS_LOG_DEBUG ("Source node did not receive the CTS message and ready to re-send RTS packet.");
			NS_ASSERT(m_pktTx && m_struct.Struct_RTS);
			Ptr<Packet> rts_packet = Create<Packet>();

			PhySendPacket (rts_packet,UanSFamaHeader::RTS);
			m_struct.Struct_backoff=false;
            Time wait_cts_delay=2*m_slot;
            NS_ASSERT(!m_waitCTSTimer.IsRunning());
            m_waitCTSTimer.Schedule(wait_cts_delay);
//            m_waitCTSTimer.SetDelay(wait_cts_delay);
		}
		else
		{
			NS_LOG_DEBUG ("Source node is waiting the CTS message for sending previous packets.");
		}
		break;
	case WAIT_ACK:
		NS_ASSERT(m_struct.Struct_Role==SFAMA_STRUCT::Sender && m_struct.Struct_RTS && m_struct.Struct_CTS  && m_struct.Struct_DATA);
		//!need to be checked whether have to re transmit the data packet after a while.
		NS_LOG_DEBUG ("Source node is waiting the ACK message of previous packets, CHECK!!!");
		break;
	case WAIT_DATA:
		NS_ASSERT(m_struct.Struct_Role==SFAMA_STRUCT::Receiver);
//		NS_ASSERT(!m_waitDATATimer.IsExpired());
		if (m_struct.Struct_DATA)
		{
			NS_ASSERT(m_waitDATATimer.IsExpired());
			NS_LOG_DEBUG ("Dest node has received the DATA message and ready to send ACK packet,its state back to idle.");

			Ptr<Packet> ack_packet = Create<Packet>();
			PhySendPacket (ack_packet,UanSFamaHeader::ACK);
            m_struct.Struct_ACK=true;
            m_state = IDLE;
			NS_ASSERT(StructResume());
		}
		else
		{
			NS_LOG_DEBUG ("Dest node is waiting the DATA message of previous packets.");
		}
		break;
	case BACKOFF:
		NS_ASSERT(m_pktTx);
		NS_LOG_DEBUG ("Source node is in backoff period to send to RTS message of previous packets again.");
		break;
	default:
	    NS_LOG_DEBUG ("MAC " << GetAddress () << " Starting enqueue SOMETHING ELSE");
	    break;
	}
	m_SlotTimer.Schedule(m_slot);
	return;
}//end SlottingSchedule

void
UanMacSFAMA::PhyRxPacketGood (Ptr<Packet> packet, double sinr, UanTxMode mode)
{
	NS_LOG_FUNCTION(this << m_nodeid);
	UanHeaderCommon header;
	packet->RemoveHeader (header);
	// uint16_t protocolNumber = header.GetProtocolNumber();
	// NS_LOG_UNCOND(protocolNumber);
	Mac8Address dest_addr = header.GetDest ();
	UanSFamaHeader mac_header;
	packet->RemoveHeader (mac_header);
	UanSFamaHeader::PacketType mac_header_type = mac_header.GetType();

	if(dest_addr == Mac8Address::GetBroadcast())
	{
		switch (mac_header_type)
		{
		case UanSFamaHeader::RTS:
			if(m_state==IDLE && m_struct.Struct_Role==SFAMA_STRUCT::IDLE)
			{
				NS_LOG_DEBUG("Node "<<m_nodeid<<" firstly receive Broadcast RTS packet from "<<header.GetSrc()<<", change state and struct.");
				m_struct.Struct_Role=SFAMA_STRUCT::Receiver;
				m_struct.Struct_RTS=true;
				m_struct.Counter_Addr=header.GetSrc(); //记录第一次收到的RTS的sender地址
			}
			else if(m_struct.Struct_Role==SFAMA_STRUCT::Receiver)
			{
				if (header.GetSrc()!=m_struct.Counter_Addr)
				{
					m_struct.Counter_Addr=header.GetSrc();
					NS_LOG_DEBUG("Node "<<m_nodeid<<" receive Broadcast RTS packet from new source "<<header.GetSrc()<<", change counter addr.");
				}
			}
			else if(m_struct.Struct_Role==SFAMA_STRUCT::Sender)
			{
				NS_LOG_DEBUG("Node "<<m_nodeid<<" receive Broadcast RTS packet from source "<<header.GetSrc()<<", but I am a sender now, so drop packet.");
			}
			else if(m_state==WAIT_X && m_struct.Struct_Role==SFAMA_STRUCT::IDLE){
				NS_LOG_DEBUG("XRTS arrive earlier than RTS");
			}
			else
			{
				NS_LOG_UNCOND("Uneperted situation, check.");
			}
			break;
		case UanSFamaHeader::CTS:
			break;
		case UanSFamaHeader::FAMA_DATA:
				NS_ASSERT((m_state==WAIT_DATA||m_state==WAIT_X )&& m_struct.Struct_Role==SFAMA_STRUCT::Receiver && m_struct.Struct_CTS && m_struct.Struct_RTS);
				NS_ASSERT_MSG(m_struct.Counter_Addr == header.GetSrc(), "The sender is not my selected dest, check!");
				if (!m_struct.Struct_DATA)
				{
					m_struct.Struct_DATA=true;
					m_waitDATATimer.Cancel();
					NS_LOG_DEBUG("Node "<<m_nodeid<<" received Broadcast DATA from "<<header.GetSrc()<<", ready to send ACK at next slot.");
					m_forwardUpCb (packet,header.GetProtocolNumber(), header.GetSrc ()); //ljy
				}
			break;

		case UanSFamaHeader::ACK:
			break;
		}
	}
	else //单播情况
	{
		switch (mac_header_type)
		{
		case UanSFamaHeader::RTS:
			if (dest_addr == m_address)
			{
				if (m_state==IDLE && m_struct.Struct_Role==SFAMA_STRUCT::IDLE)
				{
					NS_LOG_DEBUG("Node "<<m_nodeid<<" firstly receive RTS packet from "<<header.GetSrc()<<", change state and struct.");
					m_struct.Struct_Role=SFAMA_STRUCT::Receiver;
					m_struct.Struct_RTS=true;
					m_struct.Counter_Addr=header.GetSrc();
				}
				else if(m_struct.Struct_Role==SFAMA_STRUCT::Receiver)
				{
					if (header.GetSrc()!=m_struct.Counter_Addr)
					{
						m_struct.Counter_Addr=header.GetSrc();
						NS_LOG_DEBUG("Node "<<m_nodeid<<" receive RTS packet from new source "<<header.GetSrc()<<", change counter addr.");
					}
				}
				else if(m_struct.Struct_Role==SFAMA_STRUCT::Sender)
				{
					NS_LOG_DEBUG("Node "<<m_nodeid<<" receive RTS packet from source "<<header.GetSrc()<<", but it's a sender drop packet.");
				}
				else if(m_state==WAIT_X && m_struct.Struct_Role==SFAMA_STRUCT::IDLE){
					NS_LOG_DEBUG("XRTS arrive earlier than RTS");
				}
				else
				{
					NS_LOG_UNCOND("Uneperted situation, check.");
				}
			}
			else
			{
				NS_LOG_DEBUG("Node "<<m_nodeid<<" receive xRTS packet from "<<header.GetSrc()<<", change state and wait 2 slots.");
				Time long_delay=2*m_slot;
		//		  NS_LOG_DEBUG("Node receive xRTS packet from "<<header.GetSrc()<<", change state and wait "<<long_delay.GetSeconds()<<" time.");
				LongWaitCheck(long_delay);
			}
			break;
		case UanSFamaHeader::CTS:
			if (dest_addr == m_address)
			{
				if(m_struct.Counter_Addr == Mac8Address::GetBroadcast()) //在广播RTS后收到某一个单播CTS回复
				{
					m_struct.Struct_CTS=true;
					if (m_state==BACKOFF)
					{
						NS_ASSERT(m_backoffTimer.IsRunning());
						m_backoffTimer.Cancel();
						m_state=WAIT_CTS;
					}

					m_waitCTSTimer.Cancel();//!node received data with 2slot delays
					if (m_state==WAIT_CTS)
					{
						NS_LOG_DEBUG("Node "<<m_nodeid<<" firstly received CTS from "<<header.GetSrc()<<", ready to send data at next slot.");
					}
					else
					{
						NS_ASSERT(m_state==WAIT_X);
						NS_LOG_DEBUG("Node "<<m_nodeid<<" firstly received CTS from "<<header.GetSrc()<<", but have to wait "<<m_waitXTimer.GetDelayLeft().GetSeconds()<<"s, due to the Xpackets.");
					}
				}
				else
				{
					NS_ASSERT_MSG((m_state==WAIT_CTS || m_state==WAIT_X|| m_state==BACKOFF) && m_struct.Struct_Role==SFAMA_STRUCT::Sender && !m_struct.Struct_CTS && m_struct.Struct_RTS,"Node should be firstly receive CTS.");
					NS_ASSERT_MSG(m_struct.Counter_Addr == header.GetSrc(), "The sender is not my selected dest, check!");
					m_struct.Struct_CTS=true;
					if (m_state==BACKOFF)
					{
						NS_ASSERT(m_backoffTimer.IsRunning());
						m_backoffTimer.Cancel();
						m_state=WAIT_CTS;
					}

					m_waitCTSTimer.Cancel();//!node received data with 2slot delays
					if (m_state==WAIT_CTS)
					{
						NS_LOG_DEBUG("Node "<<m_nodeid<<" firstly received CTS from "<<header.GetSrc()<<", ready to send data at next slot.");
					}
					else
					{
						NS_ASSERT(m_state==WAIT_X);
						NS_LOG_DEBUG("Node "<<m_nodeid<<" firstly received CTS from "<<header.GetSrc()<<", but have to wait "<<m_waitXTimer.GetDelayLeft().GetSeconds()<<"s, due to the Xpackets.");
					}
				}
			}
			else
			{
				Time long_delay=3*m_slot;//!need to be checked the value of delay from the paper.
				NS_LOG_DEBUG("Node "<<m_nodeid<<" receive xCTS packet from "<<header.GetSrc()<<", change state and wait "<<long_delay.GetSeconds()<<" time.");
				m_waitCTSTimer.Cancel();
				LongWaitCheck(long_delay);
			}

			break;
		case UanSFamaHeader::FAMA_DATA:
			if (dest_addr == m_address)
			{
				NS_ASSERT((m_state==WAIT_DATA||m_state==WAIT_X )&& m_struct.Struct_Role==SFAMA_STRUCT::Receiver && m_struct.Struct_CTS && m_struct.Struct_RTS);
				NS_ASSERT_MSG(m_struct.Counter_Addr == header.GetSrc(), "The sender is not my selected dest, check!");
				if (!m_struct.Struct_DATA)
				{
					m_struct.Struct_DATA=true;
					m_waitDATATimer.Cancel();
					NS_LOG_DEBUG("Node "<<m_nodeid<<" firstly received DATA from "<<header.GetSrc()<<", ready to send ACK at next slot.");
					m_forwardUpCb (packet,header.GetProtocolNumber (), header.GetSrc ());
				}
			}
			else
			{
				Time long_delay=2*m_slot;//!need to be checked the value of delay from the paper.
				NS_LOG_DEBUG("Node "<<m_nodeid<<" receive xDATA packet from "<<header.GetSrc()<<", change state and wait "<<long_delay.GetSeconds()<<" time.");
				LongWaitCheck(long_delay);

			}
			break;
		case UanSFamaHeader::ACK:
			if (dest_addr == m_address)
			{
				if(m_struct.Counter_Addr == Mac8Address::GetBroadcast())
				{
					m_struct.Struct_ACK=true;
					if(m_waitXTimer.IsExpired()){
						m_state=IDLE;
						NS_ASSERT(StructResume());
			//			  m_struct.Struct_Role=SFAMA_STRUCT::S;
					}

					if(!m_waitACKTimer.IsExpired())
					{
						m_waitACKTimer.Cancel();
					}
					m_forwardUpCb (packet,header.GetProtocolNumber (), header.GetSrc ());
					NS_LOG_DEBUG("Node "<<m_nodeid<<" has received ACK from "<<header.GetSrc()<<", transmission success, turn to IDLE.");
					
				}
				else
				{
					NS_ASSERT((m_state==WAIT_ACK||m_state==WAIT_X)&& m_struct.Struct_Role==SFAMA_STRUCT::Sender && m_struct.Struct_CTS && m_struct.Struct_RTS && m_struct.Struct_DATA);
					NS_ASSERT_MSG(m_struct.Counter_Addr == header.GetSrc(),"The sender is not my selected dest, check!");

					m_struct.Struct_ACK=true;
					if(m_waitXTimer.IsExpired()){
						m_state=IDLE;
						NS_ASSERT(StructResume());
			//			  m_struct.Struct_Role=SFAMA_STRUCT::S;
					}

					if(!m_waitACKTimer.IsExpired())
					{
						m_waitACKTimer.Cancel();
					}
					m_forwardUpCb (packet,header.GetProtocolNumber (), header.GetSrc ());
					NS_LOG_DEBUG("Node "<<m_nodeid<<" has received ACK from "<<header.GetSrc()<<", transmission success, turn to IDLE.");
				}
			}
			else
			{
				Time long_delay=0*m_slot;
				NS_LOG_DEBUG("Node "<<m_nodeid<<" has received XACK, return to previous state at next slot.");
		//		  Time long_delay=m_SlotTimer.GetDelayLeft();
				LongWaitCheck(long_delay);
			}
			break;
		}

}

}//end PhyRxPacketGood

void
UanMacSFAMA::BackoffFinish()
{
	NS_LOG_FUNCTION(this);
	NS_LOG_DEBUG("Node "<<m_nodeid<<" has backoff for a while and is ready to resend RTS at the begining of next slot");

	NS_ASSERT(m_struct.Struct_RTS);
	if(m_waitXTimer.IsRunning()){
		NS_ASSERT(m_state==WAIT_X);
	}else{
		m_state=WAIT_CTS;
	}
}

void
UanMacSFAMA::WaitCTSFinish()
//not receive the cts
{
	NS_LOG_FUNCTION(this);
	NS_ASSERT(m_struct.Struct_RTS);
	m_struct.Struct_backoff=true; //not receive cts -> into the backoff

	if (!m_waitCTSTimer.IsExpired())
	{
		m_waitCTSTimer.Cancel();
	}
	if (!m_waitXTimer.IsRunning())
	{
		m_state=BACKOFF;
	}
	else
	{
		NS_ASSERT(m_state==WAIT_X);
	}
//	Time backoff_delay=2*m_slot;
	Time backoff_delay=m_slot*((rand()%8)+3);
	NS_ASSERT(m_backoffTimer.IsExpired());
	m_backoffTimer.Schedule(backoff_delay);
	NS_LOG_DEBUG("Node "<<m_nodeid<<" has wait for a while and is ready to go into backoff state for "<<m_backoffTimer.GetDelayLeft().GetSeconds()<<"s.");

//	m_backoffTimer.SetDelay(backoff_delay);

}//end WaitCTSFinish

void
UanMacSFAMA::WaitACKFinish()
{
	NS_LOG_FUNCTION(this);
	NS_ASSERT(!m_struct.Struct_ACK);
	NS_LOG_DEBUG("Node "<<m_nodeid<<" does not receive the ACK packet, clear the struct.");
	//NS_ASSERT(ADTimeoutStructResume());
//	if (!m_waitXTimer.IsRunning())
//	{
//		m_state=IDLE;
//	}
}//WaitACKFinish

void
UanMacSFAMA::WaitDATAFinish()
{
	NS_LOG_FUNCTION(this);
	NS_LOG_DEBUG("Node "<<m_nodeid<<" does not receive the DATA packet, clear the struct.");
	NS_ASSERT(!m_struct.Struct_DATA);
	//NS_ASSERT(ADTimeoutStructResume ());
//	if (!m_waitXTimer.IsRunning())
//	{
//		m_state=IDLE;
//	}
//	NS_LOG_UNCOND("Node doese not receive the DATA packet.");
}//WaitDATAFinish

void
UanMacSFAMA::WaitXPacketFinish()
{
	NS_LOG_FUNCTION(this);
	NS_LOG_DEBUG("Node "<<m_nodeid<<" after the long wait about the Xpackets, state change to previous one");
	if (m_struct.Struct_Role==SFAMA_STRUCT::Sender)
	{
		NS_ASSERT(m_pktTx);
		if (!m_struct.Struct_RTS)
		{
			NS_ASSERT(!m_struct.Struct_CTS && !m_struct.Struct_DATA && !m_struct.Struct_ACK&& !m_struct.Struct_DATA);
			NS_LOG_DEBUG("Node has got packet but does not send RTS yet, change state into IDLE");
			m_state=IDLE;
		}
		else if (m_struct.Struct_RTS && !m_struct.Struct_CTS)
		{
			NS_ASSERT(!m_struct.Struct_ACK && !m_struct.Struct_DATA);
			if(m_backoffTimer.IsRunning()){
				NS_LOG_DEBUG("Node has sent RTS, but dose not receive CTS and, change state into BACKOFF");
				m_state=BACKOFF;
			}
			else
			{
				m_state=WAIT_CTS;
				NS_LOG_DEBUG("Node has sent RTS, but dose not receive CTS, change state into WAIT_CTS");
			}

		}
		else if (m_struct.Struct_RTS && m_struct.Struct_CTS && !m_struct.Struct_DATA)
		{
			NS_ASSERT(!m_struct.Struct_ACK);
			if(m_backoffTimer.IsRunning()){
				NS_LOG_DEBUG("Node has receive CTS, dose not need change back into backoff, cancel backoff");
				m_backoffTimer.Cancel();
			}
			NS_LOG_DEBUG("Node has receive CTS, but dose not send DATA yet, change state into WAIT_CTS");
			m_state=WAIT_CTS;
		}
		else if (m_struct.Struct_RTS && m_struct.Struct_CTS && m_struct.Struct_DATA && !m_struct.Struct_ACK)
		{
			NS_LOG_DEBUG("Node has sent DATA, but dose not receive ACK yet, change state into WAIT_ACK");
			m_state=WAIT_ACK;
		}
		else if (m_struct.Struct_RTS && m_struct.Struct_CTS && m_struct.Struct_DATA && m_struct.Struct_ACK)
		{
			NS_LOG_DEBUG("Node has sent DATA and received ACK yet, change state into Idle");
			NS_ASSERT(StructResume());
			m_state=IDLE;
		}
		else
		{
			NS_LOG_UNCOND("Unkonw situation in function WaitXPacketFinish() of sender case, CHECK!!!");
		}

	}
	else if(m_struct.Struct_Role==SFAMA_STRUCT::Receiver)
	{
		NS_ASSERT(m_struct.Struct_RTS);

		if (!m_struct.Struct_CTS)
		{
			NS_ASSERT(!m_struct.Struct_ACK && !m_struct.Struct_DATA);
			NS_LOG_DEBUG("Node has received RTS, but dose not send CTS, change state into IDLE");
			m_state=IDLE;
			m_struct.Struct_Role=SFAMA_STRUCT::IDLE;
			m_struct.Struct_RTS=false;
			NS_LOG_DEBUG("Node has received multiple RTS, change state into IDLE");
		}
		else if (m_struct.Struct_CTS && !m_struct.Struct_DATA)
		{
			NS_ASSERT(!m_struct.Struct_ACK);
			NS_LOG_DEBUG("Node has sent CTS, but dose not receive DATA yet, change state into WAIT_CTS");
			m_state=WAIT_DATA;
		}
		else if (m_struct.Struct_RTS && m_struct.Struct_CTS && m_struct.Struct_DATA && !m_struct.Struct_ACK)
		{
			NS_LOG_DEBUG("Node has received DATA, but dose not send ACK yet, change state into WAIT_ACK");
			m_state=WAIT_DATA;
		}
		else if (m_struct.Struct_RTS && m_struct.Struct_CTS && m_struct.Struct_DATA && m_struct.Struct_ACK)
		{
			NS_LOG_DEBUG("Node has received DATA and send ACK yet, change state into Idle");
			NS_ASSERT(StructResume());
			m_state=IDLE;
		}
		else
		{
			NS_LOG_UNCOND("Unkonw situation in function WaitXPacketFinish() of sender case, CHECK!!!");
		}
	}
	else
	{
		m_state=IDLE;
		NS_LOG_DEBUG("Struct_Role is idle, change state into Idle");
		return;
	}
}//end WaitXPacketFinish

void
UanMacSFAMA::PhyRxPacketError (Ptr<Packet> packet, double sinr)
{
	NS_LOG_DEBUG("Phy Receive Packet Error.");
}//end PhyRxPacketError

int64_t
UanMacSFAMA::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_rv->SetStream (stream);
  return 1;
}//end AssignStreams
