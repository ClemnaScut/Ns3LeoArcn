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
 *      http://core.it.uu.se/core/index.php/vbf-UU
 *
 * Authors: Elena Buchatskaia <borovkovaes@iitp.ru>
 *          Pavel Boyko <boyko@iitp.ru>
 */
#include "select-routing-packet.h"
#include "ns3/address-utils.h"
#include "ns3/packet.h"

namespace ns3 {
namespace select_route {
//-----------------------------------------------------------------------------
// forwardHeader
//-----------------------------------------------------------------------------
forwardHeader::forwardHeader()
{};


forwardHeader::forwardHeader (Ipv4Address forwardAddr):
    m_forwardAddr(forwardAddr)
{};

NS_OBJECT_ENSURE_REGISTERED (forwardHeader);

TypeId
forwardHeader::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::select_route::forwardHeader")
    .SetParent<Header> ()
    .SetGroupName ("select_route")
    .AddConstructor<forwardHeader> ()
  ;
  return tid;
}

TypeId
forwardHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

uint32_t
forwardHeader::GetSerializedSize () const
{
  return 4; //4bytes
}

void
forwardHeader::Serialize (Buffer::Iterator i ) const
{
  WriteTo (i,m_forwardAddr);
}

uint32_t
forwardHeader::Deserialize (Buffer::Iterator start )
{
  Buffer::Iterator i = start;
  ReadFrom(i,m_forwardAddr);

  uint32_t dist = i.GetDistanceFrom (start);
  NS_ASSERT (dist == GetSerializedSize ());
  return dist;
}

void
forwardHeader::Print (std::ostream &os ) const
{
  os << m_forwardAddr << std::endl;
}

bool
forwardHeader::operator== (forwardHeader const & o ) const
{
  if ( m_forwardAddr != o.m_forwardAddr )
    {
      return false;
    }
  return true;
}

std::ostream &
operator<< (std::ostream & os, forwardHeader const & h )
{
  h.Print (os);
  return os;
}
}
}
