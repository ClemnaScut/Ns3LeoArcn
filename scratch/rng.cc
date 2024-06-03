#include "ns3/core-module.h"
#include "ns3/random-variable-stream.h"
#include <iostream>
#include "ns3/ptr.h"
#include "ns3/double.h"
#include "ns3/rng-seed-manager.h"

using namespace std;
using namespace ns3;

int main(int argc, char *argv[])
{
        uint32_t rng=1;//自定义一个变量  
        CommandLine cmd;
        cmd.AddValue("rng","Number of rng",rng);
        cmd.Parse(argc,argv);//必须在前三行之后 
        RngSeedManager::SetSeed (12345); // Changes seed from default of 1 to 3
        RngSeedManager::SetRun (rng);

        Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
        x->SetAttribute ("Min", DoubleValue (0.0));
        x->SetAttribute ("Max", DoubleValue (102.0));

        double value = x->GetValue ();
        cout <<"value:"<<value<< endl;

        cout<<"randint:"<<x->GetInteger ()<<endl;
        return 0;
}

