/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 IITP RAS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Based on
 *      NS-2 vbf model developed by the CMU/MONARCH group and optimized and
 *      tuned by Samir Das and Mahesh Marina, University of Cincinnati;
 *
 *      vbf-UU implementation by Erik Nordström of Uppsala University
 *      http://core.it.uu.se/core/number.php/vbf-UU
 *
 * Authors: Elena Buchatskaia <borovkovaes@iitp.ru>
 *          Pavel Boyko <boyko@iitp.ru>
 */
#ifndef vbfHybirdROUTINGPROTOCOL_H
#define vbfHybirdROUTINGPROTOCOL_H

#include "vbf-hybird-packet.h"
#include "ns3/node.h"
#include "ns3/random-variable-stream.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include <map>
#include "ns3/vector.h"


namespace ns3 {
namespace vbf_hybird {
// ------------------------------------------Neighbor Table---------------------------------------------------
/**
 * 下面的参数请根据网络节点数和类型自适应更改
*/


enum NodeType{BUOY,BUOYNBH,UAN};


struct NeighborInfo
{
  uint32_t n_nodeid;
  Ipv4Address n_ip;
  Vector n_position;
  uint32_t n_flagAdjoin;
  uint32_t n_flagBuoy;
};

class NeighborInfoTable
{
  public:
    std::vector<NeighborInfo> m_table;
    uint32_t m_length;
    bool m_isInit;
    NeighborInfoTable();
    ~NeighborInfoTable();
    //判断邻居表中有没有浮标节点，有则返回节点NodeId，否则返回-1
    uint32_t findNeighborBuoy();
    //判断表中邻居有没有该id的节点
    bool findNodeInTable(uint32_t id);
    //判断表有没有进行初始化，有则返回true，否则返回false
    bool isInit();
    //打印邻居表
    void PrintTable();
};

// ------------------------------------------Hash Table---------------------------------------------------


#define MAX_NEIGHBOR 10
#define WINDOW_SIZE  19

/// @brief 用于储存邻居位置信息的结构体
struct vbf_neighborhood{
  //邻居数，用于查看已经存在多少个邻居转发过此packet
  int number;
  /*创建了一个长度为MAX_NEIGHBOR的Vector数组，比如neighbor[0]为数组首元素，它是一个Vector对象，
  存放了三维信息：neighbor[0].x，neighbor[0].y，neighbor[0].z*/
  Vector neighbor[MAX_NEIGHBOR]; 
  double neighborFactor[MAX_NEIGHBOR];
};

typedef std::pair<Ipv4Address, unsigned int> hash_entry; 
//ipv4address:VBFpacket-source_ip  //u_int: VBFpacket-pakcetNum

/**
 * \ingroup aqua-sim-ng
 *
 * \brief Packet Hash table for VBF to assist in specialized tables.
 */
/*当节点收到一个VBFpacket，会先调用GetHash，
即查找当前节点的hashTable是否已经存过这个VBFpacket，有则不处理这个VBFpacket（代表这个是重复的）。
若没有接收过这个VBFpacket，则会把这个VBFpacket中存放的source_ip和packetUid生成一条hash_entry，
调用PutInHash存放到这个节点的hashTable中。*/
class AquaSimPktHashTable {
public:
  std::map<hash_entry,vbf_neighborhood*> m_htable; //每一个hash_entry对应一个vbf_neighborhood

  AquaSimPktHashTable();
  ~AquaSimPktHashTable();

  //m_htable表能够存放的entry数目
  int  m_windowSize; 
  void Reset();
  // void PutInHash(Ipv4Address sAddr, unsigned int pkNum, double);
  void PutInHash(Ipv4Address sAddr, unsigned int pkNum, Vector p, double factor);
  vbf_neighborhood* GetHash(Ipv4Address senderAddr, unsigned int pkt_num);
}; 


// ----------------------------------------- --Routing Protocol---------------------------------------------------


#define DELAY_PRE 1.0

/**
 * \ingroup vbf
 *
 * \brief vbf routing protocol
 */
class RoutingProtocol : public Ipv4RoutingProtocol
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  static const uint32_t vbf_PORT;

  /// constructor
  RoutingProtocol ();
  virtual ~RoutingProtocol ();
  virtual void DoDispose ();
  int64_t AssignStreams (int64_t stream);
  // Inherited from Ipv4RoutingProtocol
  Ptr<Ipv4Route> RouteOutput (Ptr<Packet> p, const Ipv4Header &header, Ptr<NetDevice> oif, Socket::SocketErrno &sockerr);
  bool RouteInput (Ptr<const Packet> p, const Ipv4Header &header, Ptr<const NetDevice> idev,
                   UnicastForwardCallback ucb, MulticastForwardCallback mcb,
                   LocalDeliverCallback lcb, ErrorCallback ecb);

  virtual void NotifyInterfaceUp (uint32_t interface);
  virtual void NotifyInterfaceDown (uint32_t interface);
  virtual void NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address);
  virtual void NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address);
  virtual void SetIpv4 (Ptr<Ipv4> ipv4);
  virtual void PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit = Time::S) const;

  NeighborInfoTable m_nbhTable;  //NeighborTabel 2024 4.11 定义为public是因为其他实例需要访问

protected:
  virtual void DoInitialize (void);

private:
  double m_width;
  int m_pkNum;
  /// whether to enable the hopbyhop model. default: false
  int m_hopByHop;
  /// if true, VBF can perform routing functionality. Otherwise, not perform, default:true
  int m_enableRouting; 
  double m_priority;
  double m_TransRange;
  double m_SoundSpeed;
  AquaSimPktHashTable PktTable;
  /// the width is used to test if the node is close enough to the path specified by the packet

  Vector m_targetPos;
  Ptr<UniformRandomVariable> m_uniformRandomVariable;


  /// IP protocol
  Ptr<Ipv4> m_ipv4;
  /// Raw unicast socket per each IP interface, map socket -> iface address (IP + mask)
  std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_socketAddresses;
  /// Raw subnet directed broadcast socket per each IP interface, map socket -> iface address (IP + mask)
  std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_socketSubnetBroadcastAddresses;
  /// Loopback device
  Ptr<NetDevice> m_loopNet;

  static std::map<uint32_t, NeighborInfoTable*> nodeId2Table;




private:

  /**
   * 判断ip地址是否为自己地址
   * \param src the ip address
  */
  bool IsMyOwnAddress (Ipv4Address src);
  /**
   * 通过Ipv4interface去获取到其对应的socket
   *
   * \param iface the interface
   * \returns the socket associated with the interface
   */
  Ptr<Socket> FindSocketWithInterfaceAddress (Ipv4InterfaceAddress iface) const;
  /**
   * 通过Ipv4interface去获取到其对应的socket
   *
   * \param iface the interface
   * \returns the socket associated with the interface
   */
  Ptr<Socket> FindSubnetBroadcastSocketWithInterfaceAddress (Ipv4InterfaceAddress iface) const;

  /**
   * 接收到VBFpacket时会进入这里处理
   * \param socket the socket used in vbf
   * 
  */
  void RecvVBF (Ptr<Socket> socket);


  /**
   * 在该程序中调用vbfSocket去发送vbfpacket到传输层
   * \param p vbfpacket
   * \param source source_ip used in socket
   * \param dst dst_ip used in socket
  */
  void vbfSend(Ptr<Packet> p, Ipv4Address source, Ipv4Address dst);

  /**
   * 调用socket去发送vbfpacket
   * \param destination dst_ip used in socket
  */
  void SendTo (Ptr<Socket> socket, Ptr<Packet> packet, Ipv4Address destination);

  /**
   * 从Ipv4地址获取节点（用于获取位置信息）
  */
  Ptr<Node> GetNodeWithIpv4Address(Ipv4Address addr);

  /**
   * 从位置坐标中获取节点
  */
  Ptr<Node> GetNodeWithPosition(Vector pos);

  /**
   * 从节点获取Ipv4地址（多ip的只获取水下那个ip）
  */
  Ipv4Address GetUanIpv4WithNode(Ptr<Node> node);

  /**
   * 协议在这里判断节点是否是vbfpacket的目标节点，若不是则判断能够作为转发节点
   * \param pkt 传入的带有vbf头部的vbfpacket
   * \param receiver 接收到这个vbfpacket的地址，即本节点地址
  */
  void ConsiderNew(Ptr<Packet> pkt, Ipv4Address receiver, double range, double speed);


  /**
   * 作为目标节点，接受这个vbfpacket
   * \param pkt 传入带有vbf头部的vbfpacket
  */
  void TargetReceivePacket(Ptr<Packet> pkt);


  /**
   * 判断节点是否位于路由管道内，根据点到直线的距离去判断
   * \param pkt 传入带有vbf头部的vbfpacket
  */
  bool IsCloseEnough(Ptr<Packet> pkt);


  /**
   * 求节点距离路由向量的垂直距离
   * \param pkt 传入带有vbf头部的vbfpacket
  */
  double Projection(Ptr<Packet> pkt);

  /**
   * 计算节点距离路由平面的垂直距离
   * \param pkt 传入带有vbf头部的vbfpacket
  */
  double Distance(Ptr<Packet> pkt);


  /**
   * 在节点第一次收到packet时，并发现自己是转发节点时，会调用该函数计算出Tadaptation Delay
   * \param pkt 传入带有vbf头部的vbfpacket
  */
  double CalculateDelay(Ptr<Packet> pkt, double range, double speed);


  /**
   * 计算所有关于pkt的邻居的factor因子
   * \param pkt 传入带有vbf头部的vbfpacket
   * \param neighborPos 邻居位置
  */
  double CalculateNeighborFactor(Ptr<Packet> pkt, Vector neighborPos, double range);


  /**
   * 作为转发节点进行延时
   * \param pkt 传入带有vbf头部的vbfpacket
   * \param delay 延时时间
  */
  void SetDelayTimer(Ptr<Packet> pkt, double delay);


  /**
   * 延时结束，进行转发packet的处理(从HashTable存放的关于该packet邻居数去判断是转发还是抛弃)
   * \param pkt 传入带有vbf头部的vbfpacket
  */
  void Timeout(Ptr<Packet> pkt);

  /**
   * 如果本节点是卫星节点则返回true
  */
  bool IsSatellite(Ptr<Node> node);


  //--------------------------------------------------NeighborTabel func-----------------------------------------------
  /**
   * 浮标节点邻居表初始化（第一批初始化节点）
  */
  bool buoyTableInit(uint32_t nodeId);

  /**
   * 浮标节点的邻居的邻居表初始化（第二批初始化节点）
  */
  bool buoynbhTableInit(uint32_t nodeId);

  /**
   * 水下节点邻居表初始化（第三批初始化节点）
  */
  bool uanTableInit(uint32_t nodeId);

  /**
   * 添加节点信息到邻居表中
   * \param id 邻居节点nodeId
   * \param flag 用于标示是第几批初始化节点，共三批：1浮标--2浮标邻居--3水下其他节点
  */
  void AddNeighborToTable(uint32_t id, NodeType type, uint32_t flagAddj, uint32_t flagBuoys);


  /**
   * 查找某个水下节点是不是浮标节点的邻居，前提是浮标节点已经建立好了邻居表
   * 找到则返回该浮标节点的nodeId，否则返回0
   * \param id 节点nodeId
  */
  uint32_t findBuoyNbhNode(uint32_t id);
};

} //namespace vbf
} //namespace ns3

#endif /* vbfROUTINGPROTOCOL_H */
