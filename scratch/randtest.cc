/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 */

#include "ns3/core-module.h"
#include "ns3/core-config.h"
#include "ns3/command-line.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("ScratchSimulator");

int 
main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse(argc,argv);
  RngSeedManager::SetSeed(12345);

  Ptr<RandomVariableStream> uv = CreateObjectWithAttributes<UniformRandomVariable>("Min",DoubleValue(0),"Max",DoubleValue(10));
  Ptr<ExponentialRandomVariable> ev = CreateObjectWithAttributes<ExponentialRandomVariable>("Mean",DoubleValue(10));
  NS_LOG_UNCOND ("Scratch Simulator");

  NS_LOG_UNCOND (uv->GetValue());
  NS_LOG_UNCOND (uv->GetValue());
  NS_LOG_UNCOND (uv->GetValue());
  NS_LOG_UNCOND (uv->GetValue());
  NS_LOG_UNCOND (uv->GetValue());
  NS_LOG_UNCOND (uv->GetValue());
  NS_LOG_UNCOND (uv->GetValue());

  NS_LOG_UNCOND ("Scratch Simulator");
  NS_LOG_UNCOND (ev->GetValue());
  NS_LOG_UNCOND (ev->GetValue());
  NS_LOG_UNCOND (ev->GetValue());
  NS_LOG_UNCOND (ev->GetValue());
  NS_LOG_UNCOND (ev->GetValue());
  NS_LOG_UNCOND (ev->GetValue());


  

  Simulator::Run ();
  Simulator::Destroy ();
}
