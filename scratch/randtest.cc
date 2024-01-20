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

    Vector myPos{5,5,200000};
    Vector senderPos{0,0,0};
    Vector s2t{10,10,0};

    Vector n = {s2t.y, -s2t.x, 0};
    double lengthN = n.GetLength();
    n = {s2t.y/lengthN, -s2t.x/lengthN, 0}; //归一化
    std::cout << "n: " << n.x << ' ' << n.y << ' ' << n.z << std::endl;
  
    double distance = fabs(n.x*(myPos.x-senderPos.x)+n.y*(myPos.y-senderPos.y)+0)/n.GetLength();
    std::cout << distance << std::endl;

  Simulator::Run ();
  Simulator::Destroy ();
}
