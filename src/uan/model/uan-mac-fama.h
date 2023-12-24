#ifndef UAN_MAC_FAMA
#define UAN_MAC_FAMA
#include "ns3/network-module.h"
#include "ns3/stats-module.h"
#include "ns3/core-module.h"
#include "ns3/uan-mac.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/uan-phy.h"
#include "ns3/uan-tx-mode.h"
#include "ns3/mac8-address.h"
#include "ns3/random-variable-stream.h"
#include "ns3/tag.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/uan-net-device.h"
using namespace ns3;

/**
 * \ingroup UAN
 *
 * \brief Generic MAC header with demux
 */
class UanFamaHeader : public Header
{
public:
	typedef enum  {
	DATA,
    RTS,
    CTS,
	ACK
  } PacketType;

  UanFamaHeader();
  virtual ~UanFamaHeader();
  static TypeId GetTypeId(void);

  void SetType(PacketType type);
  bool IsType(PacketType type);
  PacketType GetType();

  //inherited methods
  virtual uint32_t GetSerializedSize(void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;
  virtual TypeId GetInstanceTypeId(void) const;
private:
  PacketType m_pType;
};  // class UanSFamaHeader

class EnqueuePacket
{
	public:
		Ptr<Packet> packet;
		uint16_t protocolNumber;
		Address dest;

		EnqueuePacket::EnqueuePacket(Ptr<Packet> pkt, uint16_t pN, Address adr)
		{
			packet = pkt;
			protocolNumber = pN;
			dest = adr;
		}

		EnqueuePacket::~EnqueuePacket(){};

};


struct  SFAMA_STRUCT
{
// typedef enum {
// 		 IDLE,
// 		 Sender,
// 		 Receiver
// }Struct_state;

// Struct_state Struct_Role;//! 1 for sender 0 for receiver

Address Struct_Addr;//! 在这次握手通信过程中的对方mac地址？

//这些状态会在rx或者send中发生改变，而每次slot更新就会依据这些状态和IDLE进行相应的更新
bool Struct_SendRTS;//! 若为true，代表这次通信过程中已经发送了RTS
bool Struct_SendCTS;//! true for received or sent CTS
bool Struct_SendDATA;//! true for received or sent DATA
bool Struct_SendACK;//! true for received or sent ACK

bool Struct_ReceiveRTS;//! true for received or sent RTS
bool Struct_ReceiveCTS;//! true for received or sent CTS
bool Struct_ReceiveDATA;//! true for received or sent DATA
bool Struct_ReceiveACK;//! true for received or sent ACK

bool Struct_ReceiveX; 
bool Struct_backoff;//! true for node has backoff to send rts again
};

class UanMacFAMA : public UanMac
{
	public:
	  /** Default constructor */
	  UanMacFAMA ();
	  /** Dummy destructor, DoDispose. */
	  virtual ~UanMacFAMA ();
	  /**
	   * Register this type.
	   * \return The TypeId.
	   */
	  static TypeId GetTypeId (void);

	  // Inherited methods from UanMac
	  virtual Address GetAddress ();
	  virtual void SetAddress (Mac8Address addr);
	  virtual bool Enqueue (Ptr<Packet> pkt, uint16_t protocolNumber,const Address &dest);
	  virtual void SetForwardUpCb (Callback<void, Ptr<Packet>, uint16_t, const Mac8Address&> cb);
	  virtual void AttachPhy (Ptr<UanPhy> phy);
	  virtual Address GetBroadcast (void) const;
	  virtual void Clear (void);
	  int64_t AssignStreams (int64_t stream);

	  //SFAMA functional
	  //! Start after a skew
	  void Start();
	  //! Slotting schedule process
	  void SlottingSchedule();
	  void PhySendPacket (Ptr<Packet> pkt);
	  void LongWaitCheck (Time long_delay);

	  void BackoffFinish();
	  void WaitXPacketFinish();
	  void WaitCTSFinish();
	  void WaitACKFinish();
	  void WaitDATAFinish();
	  void CheckPacketSend();
	  void StartSend(EnqueuePacket pkt);
	  void SendRTS(EnqueuePacket pkt);

	private:
	  /** Enum defining possible states. */
	  typedef enum {
		IDLE,
		WAIT_CTS,	//!Source has sent RTS wait for CTS
		WAIT_ACK,	//!Source has received CTS and sent data wait for ACK
		WAIT_DATA,	//!Dest has sent ACK and wait data.
		BACKOFF,	//!does not receive CTS, backoff for a while
		WAIT_X		//!receive x packet(packets to others) and wait for a long while
	  } Node_State;
	  typedef void (* QueueTracedCallback)(Ptr<const Packet> packet);
	  uint32_t m_nodeid;
	  SFAMA_STRUCT m_struct;
	  /** The MAC address. */
	  Mac8Address m_address;
	  /** PHY layer attached to this MAC. */
	  Ptr<UanPhy> m_phy;
	  /** Current state. */
	  Node_State m_state;
	//   /** Next packet to send. */
	  Ptr<Packet> m_pktTx;
	  /*Next packet sent protocolnumber*/
	  uint16_t m_proNumber;
	  /*Check whether has got a packet*/
	  bool m_holdPacket;
	//   double m_meanSkew;
	//   /** Provides uniform random variable for start skew. */
	  Ptr<UniformRandomVariable> m_rv;


	  /* Timer */
	  Timer m_waitCTSTimer; //!Source 等待CTS计时器
	  Timer m_backoffTimer; //!Source 退避计时器
	  Timer m_remoteTimer; 
	  Timer m_sendPacketTimer; //发送packet的定时器


	  Timer m_waitXTimer;	//!Any 长/短躲避计时器
	  Timer m_waitACKTimer;	//!Source 等待ACK计时器
	  Timer m_waitDATATimer;	//!Dest 等待数据计时器

	  /** Flag when we've been cleared */
	  bool m_cleared;

	  //!Physical layer has successfully received packet and check whether this packet is for me or a X packet
	  void PhyRxPacketGood (Ptr<Packet> packet, double sinr, UanTxMode mode);
	  void PhyRxPacketError (Ptr<Packet> packet, double sinr);
	//   bool StructResume ();//used for clear the struct when the data or ack is successfully received.
	//   bool ADTimeoutStructResume ();//used for clear the struct when the wait data or ack timer is time out.

	  /** Forwarding up callback. */
	  Callback<void, Ptr<Packet>, uint16_t, const Mac8Address& > m_forwardUpCb;
	  /*Index whether this packets has been enqueued*/
	  TracedCallback<Ptr<const Packet>> m_enqueueLogger;


	  //queue of packet
	  std::queue<EnqueuePacket> m_packetQueue;
	  Time m_maxPropDelay;
	  double m_maxNodeInterval;

	  
};


#endif /* UAN_MAC_SFAMA */
