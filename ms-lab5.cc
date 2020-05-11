/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 SEBASTIEN DERONNE
 * Copyright (c) 2020 AGH University of Science and Technology
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
 * Author: Szymon Szott <szott@kt.agh.edu.pl>
 * Based on he-wifi-network.cc by S. Deronne <sebastien.deronne@gmail.com>
 * Last update: 2020-03-24 17:59:15
 */

#include "ns3/core-module.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/yans-wifi-channel.h"
#include <chrono>  // For high resolution clock
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/abort.h"
#include "ns3/mobility-model.h"
#include "ns3/flow-monitor-module.h"
#include <fstream>
#include <iostream>
#include <ctime>
#include <iomanip>

// Course: Simulation Methods (Metody symulacyjne)
// Lab exercise: 5
// 
// This scenario allows to measure the performance of an IEEE 802.11ax Wi-Fi network.
// 
//   AP  STA ... STA
//    *  *       *
//    |  |       |
//   n0  n1      nWifi
//
// The stations generate constant traffic so as to saturate the channel.
// The user can specify the number of stations and other parameters.
// The simulation output is the network throughput.
// With the --useCsv flag, the output is saved to a CSV file.

bool fileExists(const std::string& filename);

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("ms-lab5");

int main (int argc, char *argv[])
{
  
  // Initialize default simulation parameters
  uint32_t nWifi = 1;   //Number of transmitting stations
  int mcs = 11; // Default MCS is set to highest value
  int channelWidth = 20; //Default channel width [MHz]
  int gi = 800; //Default guard interval [ns]
  std::string lossModel = "LogDistance"; //Propagation loss model
  std::string positioning = "disc"; //Position allocator
  double simulationTime = 10; // Simulation time [s]
  double radius = 10; // Radius of node placement disc [m]
  bool useCsv = false; // Flag for saving output to CSV file
  
  // Parse command line arguments
  CommandLine cmd;
  cmd.AddValue ("mcs", "Select a specific MCS (0-11)", mcs);
  cmd.AddValue ("lossModel", "Propagation loss model to use (Friis, LogDistance, TwoRayGround, Nakagami)", lossModel);       
  cmd.AddValue ("simulationTime", "Duration of simulation", simulationTime);
  cmd.AddValue ("nWifi", "Number of station", nWifi);  
  cmd.AddValue ("positioning", "Position allocator (grid, rectangle, disc)", positioning);     
  cmd.AddValue ("radius", "Radius of disc within which stations are randomly distributed", radius);  
  cmd.AddValue ("useCsv", "Flag for saving output to CSV file", useCsv);  
  cmd.Parse (argc,argv);

  // Print simulation settings to screen
  std::cout << std::endl << "Simulating an IEEE 802.11ax network with the following settings:" << std::endl;
  std::cout << "- number of transmitting stations: " << nWifi << std::endl;  
  std::cout << "- frequency band: 5 GHz" << std::endl;  
  std::cout << "- modulation and coding scheme (MCS): " << mcs << std::endl;  
  std::cout << "- channel width: " << channelWidth << " MHz" << std::endl;  
  std::cout << "- guard interval: " << gi << " ms" << std::endl;    
  std::cout << "- loss model: " << lossModel << std::endl;  
  std::cout << "- position allocator: " << positioning << std::endl; 
  std::cout << "- disc radius: " << radius << std::endl;  



  // Create AP and stations
  NodeContainer wifiApNode;
  wifiApNode.Create (1);
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (nWifi);

  // Configure wireless channel
  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  Ptr<YansWifiChannel> channel;
  YansWifiChannelHelper channelHelper = YansWifiChannelHelper::Default ();

  if (lossModel=="LogDistance") {
    phy.SetChannel (channelHelper.Create ());  
  }
  else if (lossModel=="Friis") {
    channel = channelHelper.Create ();
    channel->SetPropagationLossModel (CreateObject<FriisPropagationLossModel>());
    phy.SetChannel (channel);
  }  
  else if (lossModel=="TwoRayGround") {
    channel = channelHelper.Create ();
    Ptr<TwoRayGroundPropagationLossModel> lossModel = CreateObject<TwoRayGroundPropagationLossModel>();
    lossModel->SetSystemLoss(3);
    channel->SetPropagationLossModel (lossModel);
    phy.SetChannel (channel);
  } 
  else if (lossModel=="Nakagami") {
    // Add Nakagami fading to the default log distance model
    channelHelper.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
    phy.SetChannel (channelHelper.Create ());
  }     
  else {
    NS_ABORT_MSG("Wrong propagation model selected. Valid models are: Friis, LogDistance, TwoRayGround, Nakagami\n");
  }
  


  // Create and configure Wi-Fi network
  WifiMacHelper mac;
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211ax_5GHZ);

  std::ostringstream oss;
  oss << "HeMcs" << mcs;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue (oss.str ()),
                                "ControlMode", StringValue (oss.str ())); //Set MCS

  Ssid ssid = Ssid ("ns3-80211ax"); //Set SSID

  mac.SetType ("ns3::StaWifiMac",
                "Ssid", SsidValue (ssid));

  // Create and configure Wi-Fi interfaces
  NetDeviceContainer staDevice;
  staDevice = wifi.Install (phy, mac, wifiStaNodes);

  mac.SetType ("ns3::ApWifiMac",
                "Ssid", SsidValue (ssid));

  NetDeviceContainer apDevice;
  apDevice = wifi.Install (phy, mac, wifiApNode);

  // Set channel width and guard interval on all interfaces of all nodes
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (channelWidth));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval", TimeValue (NanoSeconds (gi)));  

  // Configure mobility
  MobilityHelper mobility;

  // Use a position allocator for station placement
  if (positioning=="grid") {
    mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                   "MinX", DoubleValue (0.0),
                                   "MinY", DoubleValue (0.0),
                                   "DeltaX", DoubleValue (1.0),
                                   "DeltaY", DoubleValue (1.0),
                                   "GridWidth", UintegerValue (10));
  }
  else if (positioning=="rectangle") {
    mobility.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator",
                                   "X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1.0]"),
                                   "Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1.0]"));
  }
  else if (positioning=="disc") {
    mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                   "X", DoubleValue (0.0),
                                   "Y", DoubleValue (0.0),
                                   "rho", DoubleValue (radius)); 
  }    
  else {
    NS_ABORT_MSG("Wrong positioning allocator selected.\n");
  }  
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNode);
  mobility.Install (wifiStaNodes);

  // For random positioning models, make sure AP is at (0, 0)
  Ptr<MobilityModel> mobilityAp = wifiApNode.Get(0)->GetObject<MobilityModel> ();
  Vector pos = mobilityAp->GetPosition ();  
  pos.x = 0;
  pos.y = 0;
  mobilityAp->SetPosition (pos);

  // Print position of each node
  std::cout << std::endl << "Node positions" << std::endl;

  //  - AP position
  Ptr<MobilityModel> position = wifiApNode.Get(0)->GetObject<MobilityModel> ();
  pos = position->GetPosition ();
  std::cout << "AP:\tx=" << pos.x << ", y=" << pos.y << std::endl;

  // - station positions
  for (NodeContainer::Iterator j = wifiStaNodes.Begin (); j != wifiStaNodes.End (); ++j) {
    Ptr<Node> object = *j;
    Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
    Vector pos = position->GetPosition ();
    std::cout << "Sta " << (uint32_t) object->GetId() << ":\tx=" << pos.x << ", y=" << pos.y << std::endl;
  }


  // Install an Internet stack
  InternetStackHelper stack;
  stack.Install (wifiApNode);
  stack.Install (wifiStaNodes);

  // Configure IP addressing
  Ipv4AddressHelper address;
  address.SetBase ("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer staNodeInterface;
  Ipv4InterfaceContainer apNodeInterface;

  staNodeInterface = address.Assign (staDevice);
  apNodeInterface = address.Assign (apDevice);

  // Install applications (traffic generators)
  ApplicationContainer sourceApplications, sinkApplications;
  uint32_t portNumber = 9;
  for (uint32_t index = 0; index < nWifi; ++index) //Loop over all stations (which transmit to the AP)
    {
      auto ipv4 = wifiApNode.Get (0)->GetObject<Ipv4> (); //Get destination's IP interface
      const auto address = ipv4->GetAddress (1, 0).GetLocal (); //Get destination's IP address
      InetSocketAddress sinkSocket (address, portNumber++); //Configure destination socket
      OnOffHelper onOffHelper ("ns3::UdpSocketFactory", sinkSocket); //Configure traffic generator: UDP, destination socket
      onOffHelper.SetConstantRate (DataRate (150e6 / nWifi), 1000);  //Set data rate (150 Mb/s divided by no. of transmitting stations) and packet size [B]
      sourceApplications.Add (onOffHelper.Install (wifiStaNodes.Get (index))); //Install traffic generator on station
      PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", sinkSocket); //Configure traffic sink
      sinkApplications.Add (packetSinkHelper.Install (wifiApNode.Get (0))); //Install traffic sink on AP
    }

  // Configure application start/stop times
  // Note: 
  // - source starts transmission at 1.0 s
  // - source stops at simulationTime+1
  // - simulationTime reflects the time when data is sent
  sinkApplications.Start (Seconds (0.0));
  sinkApplications.Stop (Seconds (simulationTime + 1));
  sourceApplications.Start (Seconds (1.0));
  sourceApplications.Stop (Seconds (simulationTime + 1));

  //Install FlowMonitor
  FlowMonitorHelper flowmon;
	Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

  // Define simulation stop time
  Simulator::Stop (Seconds (simulationTime + 1));
  
  // Print information that the simulation will be executed
  std::clog << std::endl << "Starting simulation... ";
  // Record start time
  auto start = std::chrono::high_resolution_clock::now();
 

  // Run the simulation!
  Simulator::Run ();

  // Record stop time and count duration
  auto finish = std::chrono::high_resolution_clock::now();
  std::clog << ("done!") << std::endl;  
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Elapsed time: " << elapsed.count() << " s\n\n";
  
  // Save FlowMonitor results to an XML file
  flowmon.SerializeToXmlFile ("ms-lab5.xml", false, false);

  if (useCsv) {
    std::ofstream myfile;
    std::string outputCsv = "ms-lab5.csv";
    if (fileExists(outputCsv)) {
      // If the file exists, append to it
      myfile.open (outputCsv, std::ios::app); 
    }
    else {
      // If the file does not exist, create it and set the header line
      myfile.open (outputCsv, std::ios::app);  
      myfile << "Timestamp,nWifi,RngRun,FlowSrc,Throughput" << std::endl;
    }

    //Get timestamp
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    // Calculate per-flow throughput and print results to file
    double flowThr=0;
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i) {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      flowThr=i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds () - i->second.timeFirstTxPacket.GetSeconds ()) / 1e6;
    	myfile << std::put_time(&tm, "%Y-%m-%d %H:%M") << "," << nWifi << "," << RngSeedManager::GetRun() << "," << t.sourceAddress << "," << flowThr << std::endl;
	  }
	  myfile.close();
  }

  // Calculate network throughput
  double throughput = 0;
  for (uint32_t index = 0; index < sinkApplications.GetN (); ++index) //Loop over all traffic sinks
  {
    uint64_t totalBytesThrough = DynamicCast<PacketSink> (sinkApplications.Get (index))->GetTotalRx (); //Get amount of bytes received
    throughput += ((totalBytesThrough * 8) / (simulationTime * 1000000.0)); //Mbit/s 
  }

  //Print results
  std::cout << "Results: " << std::endl;
  std::cout << "- network throughput: " << throughput << " Mbit/s" << std::endl;

  //Clean-up
  Simulator::Destroy ();

  return 0;
}

bool fileExists(const std::string& filename)
{
    std::ifstream f(filename.c_str());
    return f.good();   
}