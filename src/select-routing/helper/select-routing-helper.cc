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
#include "select-routing-helper.h"
#include "ns3/select-routing-protocol.h"
#include "ns3/node-list.h"
#include "ns3/names.h"
#include "ns3/ptr.h"
#include "ns3/ipv4-list-routing.h"

namespace ns3
{

selectRoutingHelper::selectRoutingHelper() : 
  Ipv4RoutingHelper ()
{
  m_agentFactory.SetTypeId ("ns3::select_route::RoutingProtocol");
}

selectRoutingHelper* 
selectRoutingHelper::Copy (void) const 
{
  return new selectRoutingHelper (*this); 
}

Ptr<Ipv4RoutingProtocol> 
selectRoutingHelper::Create (Ptr<Node> node) const
{
  Ptr<select_route::RoutingProtocol> agent = m_agentFactory.Create<select_route::RoutingProtocol> ();
  //2024-6-7
  agent->SetBuoySatelliteMode(m_bsmode);
  agent->SetUanMaxId(m_uanMaxNodeId);
  agent->SetSatelliteId(m_satelliteId);
  agent->SetBuoyNbhId(m_buoynhbId);
  agent->SetBuoyId(m_buoyId);

  node->AggregateObject (agent);
  return agent;
}

void
selectRoutingHelper::SetSatelliteId(std::vector<uint32_t> v)
{
  m_satelliteId = v;
}


void 
selectRoutingHelper::SetBuoyId(std::vector<uint32_t> v)
{
  m_buoyId = v;
}

void 
selectRoutingHelper::SetUanMaxId(uint32_t id)
{
  m_uanMaxNodeId=id;
}

void
selectRoutingHelper::SetBuoyNbhId(std::vector<uint32_t> v)
{
  m_buoynhbId=v;
}

void
selectRoutingHelper::SetBuoySatelliteMode(bool flag)
{
  m_bsmode=flag;
}



void 
selectRoutingHelper::Set (std::string name, const AttributeValue &value)
{
  m_agentFactory.Set (name, value);
}

int64_t
selectRoutingHelper::AssignStreams (NodeContainer c, int64_t stream)
{
  int64_t currentStream = stream;
  Ptr<Node> node;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      node = (*i);
      Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
      NS_ASSERT_MSG (ipv4, "Ipv4 not installed on node");
      Ptr<Ipv4RoutingProtocol> proto = ipv4->GetRoutingProtocol ();
      NS_ASSERT_MSG (proto, "Ipv4 routing not installed on node");
      Ptr<select_route::RoutingProtocol> select = DynamicCast<select_route::RoutingProtocol> (proto);
      if (select)
        {
          currentStream += select->AssignStreams (currentStream);
          continue;
        }
      // select may also be in a list
      Ptr<Ipv4ListRouting> list = DynamicCast<Ipv4ListRouting> (proto);
      if (list)
        {
          int16_t priority;
          Ptr<Ipv4RoutingProtocol> listProto;
          Ptr<select_route::RoutingProtocol> listselect;
          for (uint32_t i = 0; i < list->GetNRoutingProtocols (); i++)
            {
              listProto = list->GetRoutingProtocol (i, priority);
              listselect = DynamicCast<select_route::RoutingProtocol> (listProto);
              if (listselect)
                {
                  currentStream += listselect->AssignStreams (currentStream);
                  break;
                }
            }
        }
    }
  return (currentStream - stream);
}

}
