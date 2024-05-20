/*
 * uan-mac-sfama.h
 *
 *  Created on: Mar 28, 2021
 *      Author: SCUT-arcca-Wang
 */

#ifndef UAN_MAC_SFAMA
#define UAN_MAC_SFAMA
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
class UanSFamaHeader : public Header
{
public:
	typedef enum  {
	FAMA_DATA,
    RTS,
    CTS,
	ACK
  } PacketType;

  UanSFamaHeader();
  virtual ~UanSFamaHeader();
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

struct  SFAMA_STRUCT
{
typedef enum {
		 IDLE,
		 Sender,
		 Receiver
	 }Struct_state;

Struct_state Struct_Role;//! 1 for sender 0 for receiver
Address Counter_Addr;//! 1 for receiver's address and 0 for sender's address
bool Struct_RTS;//! true for received or sent RTS
bool Struct_CTS;//! true for received or sent CTS
bool Struct_DATA;//! true for received or sent DATA
bool Struct_ACK;//! true for received or sent ACK
bool Struct_backoff;//! true for node has backoff to send rts again
};

class UanMacSFAMA : public UanMac
{
	public:
	  /** Default constructor */
	  UanMacSFAMA ();
	  /** Dummy destructor, DoDispose. */
	  virtual ~UanMacSFAMA ();
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
	  void PhySendPacket (Ptr<Packet> pkt, UanSFamaHeader::PacketType header_type);
	  void LongWaitCheck (Time long_delay);

	  void BackoffFinish();
	  void WaitXPacketFinish();
	  void WaitCTSFinish();
	  void WaitACKFinish();
	  void WaitDATAFinish();

	private:
	  /** Enum defining possible states. */
	  typedef enum {
		IDLE,
		WAIT_CTS,	//!Source has sent RTS wait for CTS
		WAIT_ACK,	//!Source has received CTS and sent data wait for ACK
		WAIT_DATA,	//!Dest has sent ACK and wait data.
		BACKOFF,	//!does not receive CTS, backoff for a while
		WAIT_X		//!receive x packet(packets to others) and wait for a long while
	  } State;
	  typedef void (* QueueTracedCallback)(Ptr<const Packet> packet);
	  uint32_t m_nodeid;
	  SFAMA_STRUCT m_struct;
	  /** The MAC address. */
	  Mac8Address m_address;
	  /** PHY layer attached to this MAC. */
	  Ptr<UanPhy> m_phy;
	  /** Current state. */
	  State m_state;
	  /** Slot length. */
	  Time m_slot;
	  /** Next packet to send. */
	  Ptr<Packet> m_pktTx;
	  /*Next packet sent protocolnumber*/
	  uint16_t m_proNumber;
	  /*Check whether has got a packet*/
	  bool m_holdPacket;
	  /** Slot's skew mean.
	   *Define Mac starting time*/
	  double m_meanSkew;
	  /** Provides uniform random variable for start skew. */
	  Ptr<UniformRandomVariable> m_rv;

	  /* Timer */
	  Timer m_SlotTimer;	//!Schedule slot 计时器
	  Timer m_waitCTSTimer; //!Source 等待CTS计时器
	  Timer m_backoffTimer; //!Source 退避计时器
	  Timer m_waitXTimer;	//!Any 长/短躲避计时器
	  Timer m_waitACKTimer;	//!Source 等待ACK计时器
	  Timer m_waitDATATimer;	//!Dest 等待数据计时器

	  /** Flag when we've been cleared */
	  bool m_cleared;

	  //!Physical layer has successfully received packet and check whether this packet is for me or a X packet
	  void PhyRxPacketGood (Ptr<Packet> packet, double sinr, UanTxMode mode);
	  void PhyRxPacketError (Ptr<Packet> packet, double sinr);
	  bool StructResume ();//used for clear the struct when the data or ack is successfully received.
	  bool ADTimeoutStructResume ();//used for clear the struct when the wait data or ack timer is time out.

	  /** Forwarding up callback. */
	  Callback<void, Ptr<Packet>, uint16_t, const Mac8Address& > m_forwardUpCb;
	  /*Index whether this packets has been enqueued*/
	  TracedCallback<Ptr<const Packet>> m_enqueueLogger;


	  //queue of packet
	  std::queue<Ptr<Packet>> m_packetQueue;
};


#endif /* UAN_MAC_SFAMA */
