/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 AGH University of Science and Technology
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Author: Lukasz Prasnal <prasnal@kt.agh.edu.pl>
   edited by Paweł Bańbuła
 */

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/wifi-module.h"
#include "ns3/propagation-module.h"
#include "ns3/mobility-module.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"

//for building positioning modelling
#include <ns3/buildings-module.h>
#include <ns3/building.h>
#include <ns3/buildings-helper.h>
#include <ns3/buildings-propagation-loss-model.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/hybrid-buildings-propagation-loss-model.h>

using namespace ns3; 

NS_LOG_COMPONENT_DEFINE ("wifi-qos-test");

class SimulationHelper 
{
public:
	SimulationHelper ();
	
	static OnOffHelper CreateOnOffHelper(InetSocketAddress socketAddress, DataRate dataRate, int packetSize, uint8_t tid, Time start, Time stop);
	static void PopulateArpCache ();
};

SimulationHelper::SimulationHelper () 
{
}

//prepare CBR traffic source
OnOffHelper
SimulationHelper::CreateOnOffHelper(InetSocketAddress socketAddress, DataRate dataRate, int packetSize, uint8_t tid, Time start, Time stop) 
{
  socketAddress.SetTos (tid << 5); //(see: https://www.tucny.com/Home/dscp-tos and http://www.revolutionwifi.net/revolutionwifi/2010/08/wireless-qos-part-3-user-priorities.html)

  OnOffHelper onOffHelper  ("ns3::UdpSocketFactory", socketAddress);
  onOffHelper.SetAttribute ("OnTime",     StringValue   ("ns3::ConstantRandomVariable[Constant=100000]"));
  onOffHelper.SetAttribute ("OffTime",    StringValue   ("ns3::ConstantRandomVariable[Constant=0]") );
  onOffHelper.SetAttribute ("DataRate",   DataRateValue (dataRate) );
  onOffHelper.SetAttribute ("PacketSize", UintegerValue (packetSize) );
  //onOffHelper.SetAttribute ("Jitter",     DoubleValue (1.0)); //packets generation times modified by random value between -50% and +50% of constant time step between packets
  //onOffHelper.SetAttribute ("PoissonTraffic", BooleanValue (true)); //generate packets in Poisson process
  onOffHelper.SetAttribute ("MaxBytes",   UintegerValue (0));
  onOffHelper.SetAttribute ("StartTime",  TimeValue (start));
  onOffHelper.SetAttribute ("StopTime",   TimeValue (stop));

  return onOffHelper;
}

//fulfill the ARP cache prior to simulation run
void
SimulationHelper::PopulateArpCache () 
{
  Ptr<ArpCache> arp = CreateObject<ArpCache> ();
  arp->SetAliveTimeout (Seconds (3600 * 24 * 365) );
	
  for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i) 
    {	
      Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
      NS_ASSERT (ip != 0);
      ObjectVectorValue interfaces;
      ip->GetAttribute ("InterfaceList", interfaces);

      for (ObjectVectorValue::Iterator j = interfaces.Begin (); j != interfaces.End (); j++) 
        {		
          Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface> ();
          NS_ASSERT (ipIface != 0);
          Ptr<NetDevice> device = ipIface->GetDevice ();
          NS_ASSERT (device != 0);
          Mac48Address addr = Mac48Address::ConvertFrom (device->GetAddress () );
      
          for (uint32_t k = 0; k < ipIface->GetNAddresses (); k++) 
            {			
              Ipv4Address ipAddr = ipIface->GetAddress (k).GetLocal();		
              if (ipAddr == Ipv4Address::GetLoopback ()) 
                continue;

              ArpCache::Entry *entry = arp->Add (ipAddr);
              Ipv4Header ipv4Hdr;
              ipv4Hdr.SetDestination (ipAddr);
              Ptr<Packet> p = Create<Packet> (100);  
              entry->MarkWaitReply (ArpCache::Ipv4PayloadHeaderPair (p, ipv4Hdr) );
              entry->MarkAlive (addr);
            }
        }
    }

    for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i) 
      {
        Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
		NS_ASSERT (ip != 0);
		ObjectVectorValue interfaces;
		ip->GetAttribute ("InterfaceList", interfaces);

        for (ObjectVectorValue::Iterator j = interfaces.Begin (); j != interfaces.End (); j ++)
          {
            Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface> ();
            ipIface->SetAttribute ("ArpCache", PointerValue (arp) );
          }
      }
}



/* ===== main function ===== */

int main (int argc, char *argv[])
{
  uint32_t nSTA = 3;
  uint32_t packetSize = 1470;
  float simTime = 20;
  Time appsStart = Seconds(0);
  float radius = 5.0;
  float calcStart = 1;
  bool oneDest = true;
  bool rtsCts = false;
  bool A_VO = true;
  bool VO = true;
  bool VI = true;
  bool A_VI = true;
  bool BE = true;
  bool BK = true;
  double Mbps = 54;
  uint32_t seed = 1;
  uint32_t attackSTA = 0;
  double BeaconInt = 100; // Normalnie 100 * 1024 - Dlugi moze miec 300
  uint32_t i;
/* ===== Command Line parameters ===== */

  CommandLine cmd;
  cmd.AddValue ("nSTA",       "Number of stations",                            nSTA);
  cmd.AddValue ("packetSize", "Packet size [B]",                               packetSize);
  cmd.AddValue ("simTime",    "simulation time [s]",                           simTime);
  cmd.AddValue ("calcStart",  "start of results analysis [s]",                 calcStart);
  cmd.AddValue ("radius",     "Radius of area [m] to randomly place stations", radius);
  cmd.AddValue ("oneDest",    "use one traffic destination?",                  oneDest);
  cmd.AddValue ("RTSCTS",     "use RTS/CTS?",                                  rtsCts);
  cmd.AddValue ("A_VO",       "run A_VO traffic?",                             A_VO);
  cmd.AddValue ("VO",         "run VO traffic?",                               VO);
  cmd.AddValue ("VI",         "run VI traffic?",                               VI);
  cmd.AddValue ("A_VI",       "run A_VI traffic?",                             A_VI);
  cmd.AddValue ("BE",         "run BE traffic?",                               BE);
  cmd.AddValue ("BK",         "run BK traffic?",                               BK);
  cmd.AddValue ("Mbps",       "traffic generated per queue [Mbps]",            Mbps);
  cmd.AddValue ("seed",       "Seed",                                          seed);
  cmd.AddValue ("attackSTA",  "attackSTA",                                     attackSTA);
  cmd.AddValue ("BeaconInt",  "BeaconInt",                                     BeaconInt);
  cmd.Parse (argc, argv);
  

  // proba ustalenia beacon interval
  double BeaconInterval = BeaconInt*1024;
  // -------------

  Time simulationTime = Seconds (simTime);
  ns3::RngSeedManager::SetSeed (seed);
 
  Packet::EnablePrinting ();
/* Create stations and AP, stworzenie stacji i AP */
NodeContainer wifiStaNodes;
wifiStaNodes.Create (nSTA+1);
NodeContainer wifiApNode;
wifiApNode.Create (1);
// Attack stations/Stacje atakujace
NodeContainer wifiAttackNodes;
wifiAttackNodes.Create (attackSTA);



/* ======== Positioning / Mobility ======= */

  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  for (i = 0 ; i < nSTA+1; i++){
    positionAlloc->Add (Vector (4.0, 0.0, 0.0));
  }
  for (i = 0 ; i < attackSTA; i++){
    positionAlloc->Add (Vector (8.0, 0.0, 0.0));
  }
  
  MobilityHelper mobility;
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");

  //constant speed movement configuration
  /*Ptr<ConstantVelocityMobilityModel> mob = sta[std][grp].Get (n)->GetObject<ConstantVelocityMobilityModel> ();
  mob->SetVelocity (Vector3D (movX, movY, movZ) );*/

 
  mobility.Install (wifiApNode);
  mobility.Install (wifiStaNodes);
  // Attack stations/Stacje atakujace
  mobility.Install (wifiAttackNodes);

/* ===== Propagation Model configuration ===== */
  //default model (i.e. LogDistancePropagationLossModel)
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default (); //default

/* ===== MAC and PHY configuration ===== */

  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ());

  WifiHelper wifi;
  WifiMacHelper mac;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211a);


  //PHY parameters 
  //for complete list of available parameters - see Attributes on https://www.nsnam.org/doxygen/classns3_1_1_wifi_phy.html#pri-static-attribs
  phy.Set ("RxNoiseFigure",                DoubleValue   (7.0) );
  phy.Set ("TxPowerStart",                 DoubleValue   (15.0) );
  phy.Set ("TxPowerEnd",                   DoubleValue   (15.0) );
  phy.Set ("CcaEdThreshold",               DoubleValue   (-82.0) );
  phy.Set ("RxSensitivity",                DoubleValue   (-88.0) );
  phy.Set ("Antennas",                     UintegerValue (1) ); //[1-4] for 802.11n/ac - see http://mcsindex.com/
  phy.Set ("MaxSupportedTxSpatialStreams", UintegerValue (1) ); //[1-4] for 802.11n/ac - see http://mcsindex.com/
  phy.Set ("MaxSupportedRxSpatialStreams", UintegerValue (1) ); //[1-4] for 802.11n/ac - see http://mcsindex.com/
  

  //WiFi Remote Station Manager parameters 

  //Constant Rate setting - see Attributes on https://www.nsnam.org/doxygen/classns3_1_1_constant_rate_wifi_manager.html#pri-attribs
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
		  	  	  	  	  	  	/***** setting constant transmission mode (MCS) *****/
							    "DataMode",    StringValue ("OfdmRate54Mbps"), //[6, 9, 12, 18, 24, 36, 48, 54] for 802.11a
							    "ControlMode", StringValue ("OfdmRate6Mbps"),  //[6, 9, 12, 18, 24, 36, 48, 54] for 802.11a
							    "RtsCtsThreshold",        UintegerValue (rtsCts ? 0 : 2500),
							    "FragmentationThreshold", UintegerValue (2500));

  Ssid ssid = Ssid ("test"); 

  // Stacja AP
  mac.SetType ("ns3::ApWifiMac",
              "QosSupported", BooleanValue (true),
               "Ssid", SsidValue (ssid),
               "AltEDCASupported", BooleanValue (true));  
  NetDeviceContainer apDevice;
  apDevice = wifi.Install (phy, mac, wifiApNode);
  // Stacje klienckie 
  mac.SetType ("ns3::StaWifiMac",
               "QosSupported", BooleanValue (true),
               "Ssid", SsidValue (ssid),   
               "AltEDCASupported", BooleanValue (true));  
  NetDeviceContainer staDevices;
  staDevices = wifi.Install (phy, mac, wifiStaNodes);
  
  // Attack stations/Stacje atakujace 
  mac.SetType ("ns3::ApWifiMac",
               "Ssid", SsidValue (ssid),
               "BeaconInterval", TimeValue (MicroSeconds(BeaconInterval)),
               "EnableBeaconJitter",BooleanValue (true));
               
               
  NetDeviceContainer AttackDevices;
  AttackDevices = wifi.Install (phy, mac, wifiAttackNodes);


//Configs with paths:
  /* !!! IMPORTANT - HERE WE SHOULD SET CHANNEL WIDTH */
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (20) ); //for 802.11n/ac - see http://mcsindex.com/

//Queue Size (in packets):
  Config::Set ("/NodeList/*/DeviceList/*/Mac/VO_Txop/HiTidQueue/MaxSize",  QueueSizeValue (QueueSize ("10000p")) ); //setting A_VO queue size
  Config::Set ("/NodeList/*/DeviceList/*/Mac/VO_Txop/LowTidQueue/MaxSize", QueueSizeValue (QueueSize ("10000p")) ); //setting VO queue size
  Config::Set ("/NodeList/*/DeviceList/*/Mac/VI_Txop/HiTidQueue/MaxSize",  QueueSizeValue (QueueSize ("10000p")) ); //setting VI queue size
  Config::Set ("/NodeList/*/DeviceList/*/Mac/VI_Txop/LowTidQueue/MaxSize", QueueSizeValue (QueueSize ("10000p")) ); //setting A_VI queue size
  Config::Set ("/NodeList/*/DeviceList/*/Mac/BE_Txop/Queue/MaxSize",       QueueSizeValue (QueueSize ("10000p")) ); //setting BE queue size
  Config::Set ("/NodeList/*/DeviceList/*/Mac/BK_Txop/Queue/MaxSize",       QueueSizeValue (QueueSize ("10000p")) ); //setting BK queue size

  //int64_t streamIndex = 0;
  //wifi.AssignStreams (staDevices, streamIndex);

/* ===== Internet stack ===== */
  Ipv4AddressHelper address;
  address.SetBase ("192.168.1.0", "255.255.255.0");

  InternetStackHelper stack;
  stack.Install (wifiApNode);
  stack.Install (wifiStaNodes);
  // Attack stations/Stacje atakujace
  stack.Install (wifiAttackNodes);

  
  Ipv4InterfaceContainer ApInterface;
  ApInterface = address.Assign (apDevice);
  
  Ipv4InterfaceContainer StaInterface;
  StaInterface = address.Assign (staDevices);
  
  // Attack stations/Stacje atakujace
  Ipv4InterfaceContainer AttackInterface;
  AttackInterface = address.Assign (AttackDevices);
  

/* ===== Traffic Control (TC) Layer ==== */

  //TrafficControlHelper tch;
  //tch.Install (staDevices);

/* ===== Setting applications ===== */

 /* proba ustawienia czasu startu stacji atakujacych 
 ApplicationContainer AttackApp;
 AttackApp.Add (packetSinkHelper.Install (wifiAttackNodes.Get(0));
   AttackApp.Start (Seconds (1.0));
   AttackApp.Stop (Seconds (simulationTime+1));
   */

  DataRate dataRate = DataRate (1000000 * Mbps);

  uint32_t destinationSTANumber = nSTA; //for one common traffic destination

  Ipv4Address destination = StaInterface.GetAddress(destinationSTANumber);
  Ptr<Node> dest = wifiStaNodes.Get(destinationSTANumber);

  if (oneDest)
    {
      if (A_VO) 
        {
          PacketSinkHelper sink_A_VO ("ns3::UdpSocketFactory", InetSocketAddress (destination, 1007));
          sink_A_VO.Install (dest);
        }
      if (VO) 
        {
          PacketSinkHelper sink_VO ("ns3::UdpSocketFactory", InetSocketAddress (destination, 1006));
          sink_VO.Install (dest);
        }
      if (VI) 
        {
          PacketSinkHelper sink_VI ("ns3::UdpSocketFactory", InetSocketAddress (destination, 1005));
          sink_VI.Install (dest);
        }
      if (A_VI) 
        {
          PacketSinkHelper sink_A_VI ("ns3::UdpSocketFactory", InetSocketAddress (destination, 1004));
          sink_A_VI.Install (dest);
        }
      if (BE) 
        {
          PacketSinkHelper sink_BE ("ns3::UdpSocketFactory", InetSocketAddress (destination, 1000));
          sink_BE.Install (dest);
        }
      if (BK) 
        {
          PacketSinkHelper sink_BK ("ns3::UdpSocketFactory", InetSocketAddress (destination, 1001));
          sink_BK.Install (dest);
        }
    }

  for(uint32_t i = 0; i < nSTA; i++) 
    {
      Ptr<Node> node = wifiStaNodes.Get(i);

      if (!oneDest) //overwrite for different traffic destinations
        {
          destinationSTANumber = (i+1 == nSTA) ? (0) : (i+1); 
          destination = StaInterface.GetAddress(destinationSTANumber);
          dest = wifiStaNodes.Get(destinationSTANumber);
        }

      if (A_VO) 
        {
          OnOffHelper onOffHelper_A_VO = SimulationHelper::CreateOnOffHelper(InetSocketAddress (destination, 1007), dataRate, packetSize, 7, appsStart, simulationTime);
          onOffHelper_A_VO.Install(node);
        }
      if (VO) 
        {
          OnOffHelper onOffHelper_VO = SimulationHelper::CreateOnOffHelper(InetSocketAddress (destination, 1006), dataRate, packetSize, 6, appsStart, simulationTime);
          onOffHelper_VO.Install(node);
        }
      if (VI) 
        {
          OnOffHelper onOffHelper_VI = SimulationHelper::CreateOnOffHelper(InetSocketAddress (destination, 1005), dataRate, packetSize, 5, appsStart, simulationTime);
          onOffHelper_VI.Install(node);
        }
      if (A_VI) 
        {
          OnOffHelper onOffHelper_A_VI = SimulationHelper::CreateOnOffHelper(InetSocketAddress (destination, 1004), dataRate, packetSize, 4, appsStart, simulationTime);
          onOffHelper_A_VI.Install(node);
        }
      if (BE) 
        {
          OnOffHelper onOffHelper_BE = SimulationHelper::CreateOnOffHelper(InetSocketAddress (destination, 1000), dataRate, packetSize, 0, appsStart, simulationTime);
          onOffHelper_BE.Install(node);
        }
      if (BK) 
        {
          OnOffHelper onOffHelper_BK = SimulationHelper::CreateOnOffHelper(InetSocketAddress (destination, 1001), dataRate, packetSize, 1, appsStart, simulationTime);
          onOffHelper_BK.Install(node);
        }
    }


/* ===== tracing configuration and running simulation === */

  SimulationHelper::PopulateArpCache ();
  //Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  Simulator::Stop (simulationTime);

  FlowMonitorHelper flowmon_helper;
  Ptr<FlowMonitor> monitor = flowmon_helper.InstallAll ();
  monitor->SetAttribute ("StartTime", TimeValue (Seconds (calcStart) ) ); //Time from which flowmonitor statistics are gathered.
  monitor->SetAttribute ("DelayBinWidth", DoubleValue (0.001));
  monitor->SetAttribute ("JitterBinWidth", DoubleValue (0.001));
  monitor->SetAttribute ("PacketSizeBinWidth", DoubleValue (20));

  Simulator::Run ();
  Simulator::Destroy ();



/* ===== printing results ===== */

  monitor->CheckForLostPackets();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon_helper.GetClassifier ());
  //monitor->SerializeToXmlFile ("out.xml", true, true);

  std::string proto;
  uint64_t txBytes = 0, rxBytes = 0, txPackets = 0, rxPackets = 0, lostPackets = 0;
  double throughput;
  Time delaySum = Seconds (0), jitterSum = Seconds (0);

  std::vector<uint64_t> txBytesPerTid     = std::vector<uint64_t> (8, 0);
  std::vector<uint64_t> rxBytesPerTid     = std::vector<uint64_t> (8, 0);
  std::vector<uint64_t> txPacketsPerTid   = std::vector<uint64_t> (8, 0);
  std::vector<uint64_t> rxPacketsPerTid   = std::vector<uint64_t> (8, 0);
  std::vector<uint64_t> lostPacketsPerTid = std::vector<uint64_t> (8, 0);
  std::vector<double>   throughputPerTid  = std::vector<double>   (8, 0.0);
  std::vector<Time>     delaySumPerTid    = std::vector<Time>     (8, Seconds (0) );
  std::vector<Time>     jitterSumPerTid   = std::vector<Time>     (8, Seconds (0) );

  std::map< FlowId, FlowMonitor::FlowStats > stats = monitor->GetFlowStats();
  for (std::map< FlowId, FlowMonitor::FlowStats >::iterator flow = stats.begin (); flow != stats.end (); flow++)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (flow->first);
      switch (t.protocol)
        {
          case (6):
            proto = "TCP";
            break;
          case (17):
            proto = "UDP";
            break;
          default:
            exit (1);
        }
      std::cout << "FlowID: " << flow->first << "(" << proto << " "
                << t.sourceAddress << "/" << t.sourcePort << " --> "
                << t.destinationAddress << "/" << t.destinationPort << ")" <<
      std::endl;

      std::cout << "  Tx bytes:\t"     << flow->second.txBytes << std::endl;
      std::cout << "  Rx bytes:\t"     << flow->second.rxBytes << std::endl;
      std::cout << "  Tx packets:\t"   << flow->second.txPackets << std::endl;
      std::cout << "  Rx packets:\t"   << flow->second.rxPackets << std::endl;
      std::cout << "  Lost packets:\t" << flow->second.lostPackets << std::endl;
      if (flow->second.rxPackets > 0)
        {
          //std::cout << "  Throughput:\t"   << flow->second.rxBytes * 8.0 / (flow->second.timeLastRxPacket.GetSeconds ()-flow->second.timeFirstTxPacket.GetSeconds ()) / 1000000  << " Mb/s" << std::endl;
          std::cout << "  Throughput:\t"   << flow->second.rxBytes * 8.0 / (simulationTime - Seconds (calcStart)).GetMicroSeconds ()  << " Mb/s" << std::endl;
          std::cout << "  Mean delay:\t"   << (double)(flow->second.delaySum / (flow->second.rxPackets)).GetMicroSeconds () / 1000 << " ms" << std::endl;    
          if (flow->second.rxPackets > 1)
            std::cout << "  Mean jitter:\t"  << (double)(flow->second.jitterSum / (flow->second.rxPackets - 1)).GetMicroSeconds () / 1000 << " ms" << std::endl;   
          else
            std::cout << "  Mean jitter:\t---"   << std::endl;
        }
      else
        {
          std::cout << "  Throughput:\t0 Mb/s" << std::endl;
          std::cout << "  Mean delay:\t---"    << std::endl;    
          std::cout << "  Mean jitter:\t---"   << std::endl;
        }

      uint16_t tid = t.destinationPort-1000;
      txBytesPerTid[tid]     += flow->second.txBytes;
      rxBytesPerTid[tid]     += flow->second.rxBytes;
      txPacketsPerTid[tid]   += flow->second.txPackets;
      rxPacketsPerTid[tid]   += flow->second.rxPackets;
      lostPacketsPerTid[tid] += flow->second.lostPackets;
      //throughputPerTid[tid]  += (flow->second.rxPackets > 0 ? flow->second.rxBytes * 8.0 / (flow->second.timeLastRxPacket.GetSeconds ()-flow->second.timeFirstTxPacket.GetSeconds ()) / 1000000 : 0);
      throughputPerTid[tid]  += (flow->second.rxPackets > 0 ? flow->second.rxBytes * 8.0 / (simulationTime - Seconds (calcStart)).GetMicroSeconds () : 0);
      delaySumPerTid[tid]    += flow->second.delaySum;
      jitterSumPerTid[tid]   += flow->second.jitterSum;

      txBytes     += flow->second.txBytes;
      rxBytes     += flow->second.rxBytes;
      txPackets   += flow->second.txPackets;
      rxPackets   += flow->second.rxPackets;
      lostPackets += flow->second.lostPackets;
      //throughput  += (flow->second.rxPackets > 0 ? flow->second.rxBytes * 8.0 / (flow->second.timeLastRxPacket.GetSeconds ()-flow->second.timeFirstTxPacket.GetSeconds ()) / 1000000 : 0);
      throughput  += (flow->second.rxPackets > 0 ? flow->second.rxBytes * 8.0 / (simulationTime - Seconds (calcStart)).GetMicroSeconds () : 0);
      delaySum    += flow->second.delaySum;
      jitterSum   += flow->second.jitterSum;
    }

  for (uint16_t tid = 0; tid < 8; tid++)
    if ((tid != 2) && (tid != 3))
      {
        std::cout << "=======================TID: " << tid << " =====================================" << std::endl;

        std::cout << "  Tx bytes:\t"     << txBytesPerTid[tid]     << std::endl;
        std::cout << "  Rx bytes:\t"     << rxBytesPerTid[tid]     << std::endl;
        std::cout << "  Tx packets:\t"   << txPacketsPerTid[tid]   << std::endl;
        std::cout << "  Rx packets:\t"   << rxPacketsPerTid[tid]   << std::endl;
        std::cout << "  Lost packets:\t" << lostPacketsPerTid[tid] << std::endl;
        std::cout << "  Throughput:\t"   << throughputPerTid[tid]  << " Mb/s" << std::endl;
        if (rxPacketsPerTid[tid] > 0)
          {
            std::cout << "  Mean delay:\t"   << (double)(delaySumPerTid[tid] / (rxPacketsPerTid[tid])).GetMicroSeconds () / 1000 << " ms" << std::endl;    
            if (rxPacketsPerTid[tid] > 1)  
              std::cout << "  Mean jitter:\t"  << (double)(jitterSumPerTid[tid] / (rxPacketsPerTid[tid] - 1)).GetMicroSeconds () / 1000  << " ms" << std::endl;   
            else
              std::cout << "  Mean jitter:\t---"   << std::endl;
          }
        else
          {
            std::cout << "  Mean delay:\t---"    << std::endl;    
            std::cout << "  Mean jitter:\t---"   << std::endl;
          }
      }

  std::cout << "=======================Total: =====================================" << std::endl;

  std::cout << "  Tx bytes:\t"     << txBytes     << std::endl;
  std::cout << "  Rx bytes:\t"     << rxBytes     << std::endl;
  std::cout << "  Tx packets:\t"   << txPackets   << std::endl;
  std::cout << "  Rx packets:\t"   << rxPackets   << std::endl;
  std::cout << "  Lost packets:\t" << lostPackets << std::endl;
  std::cout << "  Throughput:\t"   << throughput  << " Mb/s" << std::endl;
  if (rxPackets > 0)
    {
      std::cout << "  Mean delay:\t"   << (double)(delaySum / (rxPackets)).GetMicroSeconds () / 1000 << " ms" << std::endl;    
      if (rxPackets > 1)  
        std::cout << "  Mean jitter:\t"  << (double)(jitterSum / (rxPackets - 1)).GetMicroSeconds () / 1000  << " ms" << std::endl;   
      else
        std::cout << "  Mean jitter:\t---"   << std::endl;
    }
  else
    {
      std::cout << "  Mean delay:\t---"    << std::endl;    
      std::cout << "  Mean jitter:\t---"   << std::endl;
    }

  return 0;
}