/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015 Universita' degli Studi di Napoli "Federico II"
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
 * Based on examples/traffic-control/traffic-control.cc by
 * 		Author: Pasquale Imputato <p.imputato@gmail.com>
 * 		Author: Stefano Avallone <stefano.avallone@unina.it>
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/config-store.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("MS-LAB7-QUEUE");


void TcPacketsInQueue (QueueDiscContainer qdiscs, Ptr<Queue<Packet> > p2pqueue, Ptr<OutputStreamWrapper> stream)
{
	//Get current queue size value and save to file.
	Ptr<QueueDisc> p = qdiscs.Get (1);
	int size = p->GetCurrentSize().GetValue (); 
	int p2psize = p2pqueue->GetCurrentSize().GetValue();
	*stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << size + p2psize << std::endl;
}

static void GenerateTraffic (Ptr<Socket> socket, Ptr<ExponentialRandomVariable> randomSize,	Ptr<ExponentialRandomVariable> randomTime)
{
	uint32_t pktSize = randomSize->GetInteger (); //Get random value for packet size
	socket->Send (Create<Packet> (pktSize));

	Time pktInterval = Seconds(randomTime->GetValue ()); //Get random value for next packet generation time 
	Simulator::Schedule (pktInterval, &GenerateTraffic, socket, randomSize, randomTime); //Schedule next packet generation
}

int main (int argc, char *argv[])
{
	double simulationTime = 10; //seconds
	double mu = 330;
	double lambda = 300;
	uint32_t queueSize = 1000;

	CommandLine cmd;
	cmd.AddValue ("simulationTime", "Simulation time [s]", simulationTime);
	cmd.AddValue ("queueSize", "Size of queue [no. of packets]", queueSize);
	cmd.AddValue ("lambda", "Arrival rate [packets/s]", lambda);
	cmd.AddValue ("mu", "Service rate [packets/s]", mu);
	cmd.Parse (argc, argv);

	NS_ASSERT_MSG (queueSize > 1, "This implementation does not support a queue size than 2.");

	NodeContainer nodes;
	nodes.Create (2);
	//nazwa_pliku="queue"+str(queueSize)+".tr"

	PointToPointHelper pointToPoint;
	pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("1Mbps"));
	pointToPoint.SetChannelAttribute ("Delay", StringValue ("1ms"));
	pointToPoint.SetQueue ("ns3::DropTailQueue", "MaxSize", StringValue ("1p"));

	NetDeviceContainer devices;
	devices = pointToPoint.Install (nodes);

	InternetStackHelper stack;
	stack.Install (nodes);

	TrafficControlHelper tch;
	tch.SetRootQueueDisc ("ns3::FifoQueueDisc", "MaxSize", StringValue (std::to_string(queueSize-1)+"p")); //-1 because there MAC layer queue holds one packet
	QueueDiscContainer qdiscs = tch.Install (devices);

	AsciiTraceHelper asciiTraceHelper;
	Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream ("queue.tr");

	Ptr<NetDevice> nd = devices.Get (1);
	Ptr<PointToPointNetDevice> ptpnd = DynamicCast<PointToPointNetDevice> (nd);
	Ptr<Queue<Packet> > p2pqueue = ptpnd->GetQueue ();

	for (float t=1.0; t < simulationTime; t+=0.001) {
		Simulator::Schedule (Seconds(t), &TcPacketsInQueue, qdiscs, p2pqueue, stream);
	}

	Ipv4AddressHelper address;
	address.SetBase ("10.1.1.0", "255.255.255.0");

	Ipv4InterfaceContainer interfaces = address.Assign (devices);

	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

	Ptr<Socket> recvSink = Socket::CreateSocket (nodes.Get (0), tid);
	InetSocketAddress local = InetSocketAddress (interfaces.GetAddress (0), 80);
	recvSink->Bind (local);

	Ptr<Socket> source = Socket::CreateSocket (nodes.Get (1), tid);
	InetSocketAddress remote = InetSocketAddress (interfaces.GetAddress (0), 80);
	source->Connect (remote);

	double mean = 1.0/lambda;
	Ptr<ExponentialRandomVariable> randomTime = CreateObject<ExponentialRandomVariable> ();
	randomTime->SetAttribute ("Mean", DoubleValue (mean));

	mean = (1000000.0/(8*mu)-30); // (1 000 000 [b/s])/(8 [b/B] * packet service rate [1/s]) - 30 [B (header bytes)]
	Ptr<ExponentialRandomVariable> randomSize = CreateObject<ExponentialRandomVariable> ();
	randomSize->SetAttribute ("Mean", DoubleValue (mean));

	Simulator::ScheduleWithContext (source->GetNode ()->GetId (), Seconds (1.0), &GenerateTraffic, source, randomSize, randomTime);

	FlowMonitorHelper flowmon;
	Ptr<FlowMonitor> monitor = flowmon.InstallAll();

	pointToPoint.EnablePcapAll ("ms-lab7-queue");


	Simulator::Stop (Seconds (simulationTime));
	Simulator::Run ();

	/* Calculation of experiment statistics, no need to analyze */
	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
	std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
	std::cout << std::endl << "*** Flow monitor statistics ***" << std::endl;
	std::cout << "  Tx Packets/Bytes:   " << stats[1].txPackets
		<< " / " << stats[1].txBytes << std::endl;
	std::cout << "  Offered Load: " << stats[1].txBytes * 8.0 / (stats[1].timeLastTxPacket.GetSeconds () - stats[1].timeFirstTxPacket.GetSeconds ()) / 1000000 << " Mbps" << std::endl;
	std::cout << "  Rx Packets/Bytes:   " << stats[1].rxPackets
		<< " / " << stats[1].rxBytes << std::endl;
	uint32_t packetsDroppedByQueueDisc = 0;
	uint64_t bytesDroppedByQueueDisc = 0;
	if (stats[1].packetsDropped.size () > Ipv4FlowProbe::DROP_QUEUE_DISC)
	{
		packetsDroppedByQueueDisc = stats[1].packetsDropped[Ipv4FlowProbe::DROP_QUEUE_DISC];
		bytesDroppedByQueueDisc = stats[1].bytesDropped[Ipv4FlowProbe::DROP_QUEUE_DISC];
	}
	std::cout << "  Packets/Bytes Dropped by Queue Disc:   " << packetsDroppedByQueueDisc
		<< " / " << bytesDroppedByQueueDisc << std::endl;
	uint32_t packetsDroppedByNetDevice = 0;
	uint64_t bytesDroppedByNetDevice = 0;
	if (stats[1].packetsDropped.size () > Ipv4FlowProbe::DROP_QUEUE)
	{
		packetsDroppedByNetDevice = stats[1].packetsDropped[Ipv4FlowProbe::DROP_QUEUE];
		bytesDroppedByNetDevice = stats[1].bytesDropped[Ipv4FlowProbe::DROP_QUEUE];
	}
	std::cout << "  Packets/Bytes Dropped by NetDevice:   " << packetsDroppedByNetDevice
		<< " / " << bytesDroppedByNetDevice << std::endl;
	std::cout << "  Queue blocking probability:\t" << (float) packetsDroppedByQueueDisc/stats[1].txPackets << std::endl;
	std::cout << "  Throughput: " << stats[1].rxBytes * 8.0 / (stats[1].timeLastRxPacket.GetSeconds () - stats[1].timeFirstRxPacket.GetSeconds ()) / 1000000 << " Mbps" << std::endl;
	/* End of statistics calculation */
	
	Simulator::Destroy ();

	return 0;
}
