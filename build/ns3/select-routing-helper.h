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
 * Authors: Pavel Boyko <boyko@iitp.ru>, written after OlsrHelper by Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */

#ifndef select_Routing_HELPER_H
#define select_Routing_HELPER_H

#include "ns3/object-factory.h"
#include "ns3/node.h"
#include "ns3/node-container.h"
#include "ns3/ipv4-routing-helper.h"

namespace ns3 {
/**
 * \ingroup vbf
 * \brief Helper class that adds vbf routing to nodes.
 */
class selectRoutingHelper : public Ipv4RoutingHelper
{
public:
  selectRoutingHelper ();

  /**
   * \returns pointer to clone of this vbfHybirdHelper
   *
   * \internal
   * This method is mainly for internal use by the other helpers;
   * clients are expected to free the dynamic memory allocated by this method
   */
  selectRoutingHelper* Copy (void) const;

  /**
   * \param node the node on which the routing protocol will run
   * \returns a newly-created routing protocol
   *
   * This method will be called by ns3::InternetStackHelper::Install
   *
   * \todo support installing vbf on the subset of all available IP interfaces
   */
  virtual Ptr<Ipv4RoutingProtocol> Create (Ptr<Node> node) const;
  /**
   * \param name the name of the attribute to set
   * \param value the value of the attribute to set.
   *
   * This method controls the attributes of ns3::vbf::RoutingProtocol
   */
  void Set (std::string name, const AttributeValue &value);
  /**
   * Assign a fixed random variable stream number to the random variables
   * used by this model.  Return the number of streams (possibly zero) that
   * have been assigned.  The Install() method of the InternetStackHelper
   * should have previously been called by the user.
   *
   * \param stream first stream index to use
   * \param c NodeContainer of the set of nodes for which vbf
   *          should be modified to use a fixed stream
   * \return the number of stream indices assigned by this helper
   */
  int64_t AssignStreams (NodeContainer c, int64_t stream);


  //2024-6-7
  void SetSatelliteId(std::vector<uint32_t> v);
  void SetBuoyId(std::vector<uint32_t> v);
  void SetUanMaxId(uint32_t id);
  void SetBuoyNbhId(std::vector<uint32_t> v);
  void SetBuoySatelliteMode(bool flag);

private:
  /** the factory to create vbf routing object */
  ObjectFactory m_agentFactory;
  //2024-6-7
  std::vector<uint32_t> m_satelliteId; //卫星容器，储存卫星ID
  std::vector<uint32_t>  m_buoyId; //浮标容器，储存浮标ID
  std::vector<uint32_t> m_buoynhbId; //浮标的邻居水下节点ID
  uint32_t m_uanMaxNodeId; //水下节点最大id
  bool m_bsmode = false; //是否带有卫星节点和浮标节点
};

}

#endif /* vbf_HELPER_H */
