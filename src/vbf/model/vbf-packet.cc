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
#include "vbf-packet.h"
#include "ns3/address-utils.h"
#include "ns3/packet.h"

namespace ns3 {
namespace vbf {
//-----------------------------------------------------------------------------
// VBFHeader
//-----------------------------------------------------------------------------
vbfHeader::vbfHeader()
{};


vbfHeader::vbfHeader (MessageType messageType, uint16_t pkNum, uint8_t hopCount, 
                      Ipv4Address targetAddr, Ipv4Address sourceAddr, Ipv4Address forwardAddr,
                      Time timeGenerate, Time timeForward):
    m_messType(messageType),
    m_pkNum(pkNum),
    m_hopCount(hopCount),
    m_targetAddr(targetAddr),
    m_sourceAddr(sourceAddr),
    m_forwardAddr(forwardAddr),
    m_timegenerate(uint32_t(timeGenerate.GetSeconds())),
    m_timeforward(uint32_t(timeForward.GetSeconds()))
{};

NS_OBJECT_ENSURE_REGISTERED (vbfHeader);

TypeId
vbfHeader::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::vbf::vbfHeader")
    .SetParent<Header> ()
    .SetGroupName ("vbf")
    .AddConstructor<vbfHeader> ()
  ;
  return tid;
}

TypeId
vbfHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

uint32_t
vbfHeader::GetSerializedSize () const
{
  return (1+2+1+4+4+4+4+4); //24bytes
}

void
vbfHeader::Serialize (Buffer::Iterator i ) const
{
  i.WriteU8 (m_messType);
  i.WriteU16 (m_pkNum);
  i.WriteU8 (m_hopCount);
  WriteTo (i,m_targetAddr);
  WriteTo (i,m_sourceAddr);
  WriteTo (i,m_forwardAddr);
  i.WriteU32 (m_timegenerate);
  i.WriteU32 (m_timeforward);
}

uint32_t
vbfHeader::Deserialize (Buffer::Iterator start )
{
  Buffer::Iterator i = start;
  m_messType = i.ReadU8();
  m_pkNum = i.ReadU16();
  m_hopCount = i.ReadU8();
  ReadFrom(i,m_targetAddr);
  ReadFrom(i,m_sourceAddr);
  ReadFrom(i,m_forwardAddr);
  m_timegenerate = i.ReadU32();
  m_timeforward = i.ReadU32();

  uint32_t dist = i.GetDistanceFrom (start);
  NS_ASSERT (dist == GetSerializedSize ());
  return dist;
}

void
vbfHeader::Print (std::ostream &os ) const
{
  os << "---vbfHeader--- MessageType:" << m_messType << " PacketNum:" << m_pkNum
     << " HopCount:" << m_hopCount << " targetIp:" << m_targetAddr << " sourceIp:" 
     << m_sourceAddr << " forwardIp:" << m_forwardAddr << " TimeGene:" << m_timegenerate
     << " TimeForw:" << m_timeforward << std::endl;
}

bool
vbfHeader::operator== (vbfHeader const & o ) const
{
  if (m_messType != o.m_messType || m_pkNum != o.m_pkNum || m_targetAddr!=o.m_targetAddr
      ||m_sourceAddr != o.m_sourceAddr || m_forwardAddr != o.m_forwardAddr || m_timegenerate != o.m_timegenerate
      || m_timeforward != o.m_timeforward)
    {
      return false;
    }
  return true;
}

std::ostream &
operator<< (std::ostream & os, vbfHeader const & h )
{
  h.Print (os);
  return os;
}
}
}
