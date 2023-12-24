/*
 * uan-sfama-test.h
 *
 *  Created on: Apr 2, 2021
 *      Author: SCUT-arcca-Wang
 */

#ifndef UAN_SFAMA_TEST_H
#define UAN_SFAMA_TEST_H

#include "ns3/network-module.h"
#include "ns3/stats-module.h"
#include "ns3/uan-module.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/energy-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/aodv-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/v4ping-helper.h"
#include "ns3/tag.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/netanim-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/ptr.h"
#include <iostream>
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>
#include <numeric>
#include <algorithm>
#include "ns3/timer.h"
#include "ns3/traffic-control-module.h"
#include "ns3/gnuplot.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
using namespace ns3;
using namespace std;


class SFAMATestHeader : public Header
{
public:

  SFAMATestHeader ();

  // Header serialization/deserialization
  static TypeId GetTypeId ();
  TypeId GetInstanceTypeId () const;
  uint32_t GetSerializedSize () const;
  void Serialize (Buffer::Iterator start) const;
  uint32_t Deserialize (Buffer::Iterator start);
  void Print (std::ostream &os) const;


  //added
  void SetPacketInd (uint32_t ind) { m_packetindex = ind;}
  uint32_t GetPacketInd () const { return m_packetindex; }//set and get the packet's id in a set

  bool operator== (SFAMATestHeader const & o) const;

private:

  uint32_t		 m_packetindex;	   ///<Packet index in a set

};

std::ostream & operator<< (std::ostream & os, SFAMATestHeader const &);

//end the header of underwater rateless coding packet
//----------------------------------------------------------------------------------
//The uan reply header
/**
* \copy from Uan game mac for zqy
* \brief Sink Acknowledgment  Message Format
* This packet header is added to a ACK packet to uan src, which is used to identify the sink
* has been receive all the packet in same block.
  0                   1
  0 1 2 3 4 5 6 7 8 0 1 2 3 4 5 6 7
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  | 	   	   packerid			  |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  \endverbatim
*/
class SFAMATestAckHeader : public Header
{
public:
  /// c-tor
	SFAMATestAckHeader ();
  virtual ~SFAMATestAckHeader ();

  // Header serialization/deserialization
  static TypeId GetTypeId ();//use the tyoeid define the ack
  TypeId GetInstanceTypeId () const;
  uint32_t GetSerializedSize () const;
  void Serialize (Buffer::Iterator start) const;
  uint32_t Deserialize (Buffer::Iterator start);
  void Print (std::ostream &os) const;

  bool operator== (SFAMATestAckHeader const & o) const;
  void SetPacketInd (uint32_t ind) {m_packetindex = ind;}
  uint32_t GetPacketInd () const {return m_packetindex;}

private:
  uint32_t m_packetindex;

};
std::ostream & operator<< (std::ostream & os, SFAMATestAckHeader const &);


class SFAMATestTag : public Tag
{
public:
	enum TagType
	{
		MSG = 0,
		ACK = 1,
	};
	SFAMATestTag (TagType o = TagType::MSG) : Tag (),
	                                     m_tag (o)
	  {
	  }

	static TypeId GetTypeId ();
	// Inherited
	TypeId  GetInstanceTypeId () const;
	/// \returns control tag type
	TagType GetTagType () const;
	bool IsTagType (const TagType type) const;
	/// Set suropp tag by \p tag
	void SetTagType (const TagType tag);
	// Inherited
	uint32_t GetSerializedSize () const;
	void  Serialize (TagBuffer i) const;
	void  Deserialize (TagBuffer i);
	void  Print (std::ostream &os) const;
	void SetTxInd (uint32_t ind) { m_TxId = ind;}
	uint32_t GetTxInd () const { return m_TxId; }
	void SetRxInd (uint32_t ind) { m_RxId = ind;}
	uint32_t GetRxInd () const { return m_RxId; }
	void SetPktInd (uint32_t ind) { m_pktId = ind;}
	uint32_t GetPktInd () const { return m_pktId; }


private:
	TagType m_tag;
	uint32_t		 m_TxId;	   ///<Sender index in a set
	uint32_t 		 m_RxId;	   ///<Receiver index
	uint32_t		 m_pktId;	   ///<Packet index in a set
};



class ExperimentSFAMA
{
public:
	ExperimentSFAMA ();
	void run();
	void PoissonTrafficGenerate(Ptr<Node> sender);
	void PurePoissonTraffic(Ptr<Node> sender, double slot);
	void SendNextPacket(uint32_t sender_Id);
	void RxReceivePacket(std::string context, Ptr< const Packet > packet, Mac8Address address);
	void TxReceiveAck(std::string context, Ptr< const Packet > packet, Mac8Address address);
	void PositionConfigure ();
	void MacEnqueueCB (std::string context, Ptr< const Packet > packet);

	uint32_t GetIdFromContext(std::string context);
	void ExperimentClear();
	uint32_t m_nct;		   //<number of transmitting nodes
	uint32_t m_ncr;		   //<number of receiving nodes
	double m_moduleDataRate;
	double m_Tsinr;
	double m_depth;
	double m_boundary;         //<size of bundary in meters.
	double m_lambda;//<Src depth.
	double  m_txPowerAcoustic;
	uint32_t seed_value;
	double m_StopTime;
	double m_tFrame;
	double m_packetSize;
   	double m_currentPktId;

	/** Provides uniform random variable. */
	Ptr<UniformRandomVariable> m_ur;
	Ptr<ExponentialRandomVariable> m_er;

	//------------------underwater_nodecontainer and netdevice
	NodeContainer nct_uan;
	NetDeviceContainer devices_nct_uan;
	NodeContainer ncr_uan;
	NetDeviceContainer devices_ncr_uan;

	uint32_t SinkrecvNum;
	uint32_t SrcsendNum;
	double RunTime;
	double GeneratePkts_Sum;
	double CurrentGenerate_Sum;
	uint32_t Buffer_Sum;
	double energyConsumed;
	double ncr_Uan_Enr;
	double nct_Uan_Enr;
	double empty_slot;

	double rec2send;
	double test_pdr;
	double NetworkTPut;
   	double Delay;

   	std::vector<uint32_t> v_recvPktId;
   	std::vector<uint32_t> v_recvACKId;

	std::string m_asciitracefile;       //!< Name for ascii trace file, default uan-cw-example.asc.
	std::string m_macType;
	std::string m_model;

	Ptr<ListPositionAllocator> pos_uan_tx = CreateObject<ListPositionAllocator> ();
	Ptr<ListPositionAllocator> pos_uan_rx = CreateObject<ListPositionAllocator> ();

	std::map<uint32_t, double> MAP_TID2TotalGeneratePkts;//!collect the totals packets generated in src
	std::map<uint32_t, double> MAP_TID2CurrentGeneratePkts;//!collect the totals packets generated in one slot in src
	std::map<uint32_t, double> MAP_TID2BufferSum;//!collect the packets stored in buffer update each slot

	std::map<uint32_t, double> MAP_TID2RecvNum;//!collect the packets received in rx(is the key) node from each tx node
	std::map<uint32_t, double> MAP_TID2RecvACKNum;//!collect the packets received in rx(is the key) node from each tx node

	std::map<uint32_t, double> MAP_TID2SendNum;//!collect the number of packets sent in each tx node

	std::map<uint32_t, uint32_t> MAP_SId2RId;//!collect the sender_id to receiver_id pair
	std::map<uint32_t, uint32_t> MAP_SId2cnum;//!collect the sender_id to receiver_id pair
	std::map<uint32_t,EventId> MAP_SenderUpdateEvent;//stored the event id of the scheduling UpdateRevenuWeight
};

#endif /* UAN_SFAMA_TEST_H */
