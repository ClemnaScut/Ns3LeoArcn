#include "uan-mac-fama.h"
#include "ns3/attribute.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/nstime.h"
#include "ns3/uan-header-common.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/log.h"
using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("UanMacFAMA");

NS_OBJECT_ENSURE_REGISTERED (UanMacFAMA);
NS_OBJECT_ENSURE_REGISTERED (UanFamaHeader);


//---------------------------------------------Header--------------------------------------------------
UanFamaHeader::UanFamaHeader () :
		m_pType(UanFamaHeader::RTS)
{
}

UanFamaHeader::~UanFamaHeader ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
UanFamaHeader::GetTypeId ()
{
	static TypeId tid = TypeId ("ns3::UanMacFAMA::UanFamaHeader")
	.SetParent<Header> ()
	.AddConstructor<UanFamaHeader> ()
	;
	return tid;
}

TypeId
UanFamaHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

uint32_t
UanFamaHeader::GetSerializedSize () const
{
	NS_LOG_FUNCTION (this);

	return sizeof (uint8_t);
}

void
UanFamaHeader::Serialize (Buffer::Iterator i ) const
{
	NS_LOG_FUNCTION (this);
	i.WriteU8 (m_pType);
}

uint32_t
UanFamaHeader::Deserialize (Buffer::Iterator start )
{
	NS_LOG_FUNCTION (this);
	Buffer::Iterator i = start;
	uint8_t type = i.ReadU8();
	switch (type)
	{
		case DATA:
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
UanFamaHeader::Print (std::ostream &os ) const
{
	switch (m_pType)
	{
	case DATA:
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
	};
}

UanFamaHeader::PacketType
UanFamaHeader::GetType ()
{
  return m_pType;
}

bool
UanFamaHeader::IsType (PacketType type)
{
  return m_pType == type;
}

void
UanFamaHeader::SetType (PacketType tag)
{
	m_pType = tag;
}


//-----------------------------------------------------------Sfama Broadcast MAC-----------------------------------------------------------------
UanMacFAMA::UanMacFAMA ()
: UanMac (),
	m_phy (0), //PHY layer attached to this MAC.
	m_state (IDLE), // init state = IDLE
	m_pktTx (0), //Next packet to send.
	m_proNumber (0), //Next packet sent protocolnumber
	m_cleared (false), //Flag when we've been cleared
	m_maxNodeInterval(22000),
	m_waitCTSTimer(Timer::CHECK_ON_DESTROY),
	m_backoffTimer(Timer::CHECK_ON_DESTROY),
	m_remoteTimer(Timer::CANCEL_ON_DESTROY)
{
	m_rv = CreateObject<UniformRandomVariable> ();  //Provides uniform random variable for start skew.
	m_struct.Struct_SendRTS=false;
	m_struct.Struct_SendCTS=false;
	m_struct.Struct_SendDATA=false;
	m_struct.Struct_SendACK=false;
	m_struct.Struct_ReceiveRTS=false;
	m_struct.Struct_ReceiveCTS=false;
	m_struct.Struct_ReceiveDATA=false;
	m_struct.Struct_ReceiveACK=false;
	m_struct.Struct_backoff=0;
}

UanMacFAMA::~UanMacFAMA ()
{
}

void
UanMacFAMA::Clear ()
{

}


TypeId
UanMacFAMA::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanMacFAMA")
	.SetParent<UanMac> ()
	.SetGroupName ("Uan")
	.AddConstructor<UanMacFAMA> ()
	.AddTraceSource ("Enqueue",
                    "A packet arrived at the MAC for transmission.",
                    MakeTraceSourceAccessor (&UanMacFAMA::m_enqueueLogger),
                    "ns3::UanMacSFAMA::QueueTracedCallback")
  ;
  return tid;
}


Address
UanMacFAMA::GetAddress ()
{
  return this->m_address;
}//end GetAddress

void
UanMacFAMA::SetAddress (Mac8Address addr)
{
  m_address = addr;
}//end SetAddress

int64_t
UanMacFAMA::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_rv->SetStream (stream);
  return 1;
}//end AssignStreams

void
UanMacFAMA::AttachPhy (Ptr<UanPhy> phy)
{
  m_phy = phy;
  m_phy->SetReceiveOkCallback (MakeCallback (&UanMacFAMA::PhyRxPacketGood, this));
  m_phy->SetReceiveErrorCallback (MakeCallback (&UanMacFAMA::PhyRxPacketError, this));
	Ptr<UanNetDevice> c_device =m_phy->GetDevice();
	m_nodeid=c_device->GetNode()->GetId();

  Simulator::Schedule (Seconds(0), &UanMacFAMA::Start,this);
}//end AttachPhy

Address
UanMacFAMA::GetBroadcast (void) const
{
  return Mac8Address::GetBroadcast ();
}//end GetBroadcast

void
UanMacFAMA::SetForwardUpCb (Callback<void, Ptr<Packet>, uint16_t, const Mac8Address&> cb)
{
  m_forwardUpCb = cb;
}//end SetForwardUpCb

void
UanMacFAMA::PhyRxPacketError (Ptr<Packet> packet, double sinr)
{
	NS_LOG_DEBUG("Phy Receive Packet Error.");
}//end PhyRxPacketError


//---------------------------------------Mac Logic--------------------------------------------
void
UanMacFAMA::Start()
{
	m_maxPropDelay = Seconds(m_maxNodeInterval/1500);
}

bool
UanMacFAMA::Enqueue(Ptr<Packet> pkt, uint16_t protocolNumber,const Address &dest)
{
	NS_LOG_FUNCTION(this);
	NS_LOG_DEBUG("node " << m_nodeid << ": enqueue a packet.");
	EnqueuePacket newPacket(pkt,protocolNumber,dest);
	m_packetQueue.push(newPacket);
	Simulator::Schedule(Seconds(0), &UanMacFAMA::CheckPacketSend, this);
	//这里还没有处理发送失败的，需要重新唤醒CheckPacketSend去发送
}


void
UanMacFAMA::CheckPacketSend()
{
	NS_LOG_FUNCTION(this);
	if(m_packetQueue.empty())
	{
		NS_LOG_DEBUG("node " << m_nodeid << ": packetQ is empty.");
	}
	else //队列中有数据
	{
		if(m_state == IDLE)
		{
			EnqueuePacket pktSend =  m_packetQueue.front();
			StartSend(pktSend);
			NS_LOG_DEBUG("node " << m_nodeid << ": try to send a packet.");
		}
		else
		{
			NS_LOG_DEBUG("node " << m_nodeid << " in " << m_state << ". do nothing");
		}
	}
	return;

}


void
UanMacFAMA::StartSend(EnqueuePacket pkt)
{
	NS_LOG_FUNCTION(this);
  if(m_waitCTSTimer.IsRunning())
  {
      return;
  }
	NS_ASSERT(m_state==IDLE);
	m_struct.Struct_Addr = pkt.dest;
	SendRTS(pkt);
}

void
UanMacFAMA::SendRTS(EnqueuePacket pkt)
{
	NS_LOG_FUNCTION(this);
	UanHeaderCommon macHeader;
	macHeader.SetDest (Mac8Address::ConvertFrom (pkt.dest));
	macHeader.SetSrc (m_address);

	UanFamaHeader famaHeader;
	famaHeader.SetType(UanFamaHeader::RTS);

	Ptr<Packet> rts_packet = Create<Packet>(1);
	rts_packet->AddHeader(famaHeader);
	rts_packet->AddHeader(macHeader);

	PhySendPacket(rts_packet);

	NS_LOG_DEBUG("node " << m_nodeid << ": send RTS success. Now wait CTS.");
	m_state = WAIT_CTS;
  m_waitCTSTimer.SetFunction(&UanMacFAMA::WaitCTSFinish,this);
  m_waitCTSTimer.Schedule(2*m_maxPropDelay);
  NS_LOG_DEBUG("m_waitCTSTimer: " << m_waitCTSTimer.GetDelayLeft());

}


void
UanMacFAMA::PhySendPacket(Ptr<Packet> pkt)
{
	NS_ASSERT(!m_phy->IsStateTx());
	NS_LOG_DEBUG ("Time " << Simulator::Now ().GetSeconds () << ": Addr " << GetAddress () << ": Enqueuing new packet while idle (sending)");
	NS_ASSERT (m_phy->GetTransducer ()->GetArrivalList ().size () == 0 && !m_phy->IsStateTx ());

	m_phy->SendPacket (pkt,GetTxModeIndex ());
}


void
UanMacFAMA::WaitCTSFinish()
{
	NS_LOG_FUNCTION(this);
	NS_LOG_DEBUG("node " << m_nodeid << " does not receive CTS in 2 maxPropDelay.");
	uint16_t randBack = rand()%5+2;
  Time backoffTime = m_maxPropDelay * randBack;
	m_state = BACKOFF;
  if( m_backoffTimer.IsRunning() ) {
      m_backoffTimer.Cancel();
  }

  m_backoffTimer.SetFunction(&UanMacFAMA::BackoffFinish,this);
  m_backoffTimer.Schedule(backoffTime);
	NS_LOG_DEBUG("node " << m_nodeid << " start to backoff. for " << backoffTime);
}


void
UanMacFAMA::BackoffFinish()
{
	m_state = IDLE;

}






void
UanMacFAMA::PhyRxPacketGood (Ptr<Packet> packet, double sinr, UanTxMode mode)
{

}


