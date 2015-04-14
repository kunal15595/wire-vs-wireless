#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <bits/stdc++.h>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/error-model.h"
#include "ns3/tcp-header.h"
#include "ns3/udp-header.h"
#include "ns3/enum.h"
#include "ns3/event-id.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wimax-module.h"
#include "ns3/global-route-manager.h"

using namespace ns3;
using namespace std;


NS_LOG_COMPONENT_DEFINE ("compare");

bool firstCwnd = true;
bool firstSshThr = true;
bool firstRtt = true;
bool firstRto = true;
Ptr<OutputStreamWrapper> cWndStream;
Ptr<OutputStreamWrapper> ssThreshStream;
Ptr<OutputStreamWrapper> rttStream;
Ptr<OutputStreamWrapper> rtoStream;
uint32_t cWndValue;
uint32_t ssThreshValue;

// uncomment to simulate wired
// #define __WIRED__

// uncomment to simulate wireless
#define __WIRELESS__

#ifdef __WIRED__

	static void
	CwndTracer (uint32_t oldval, uint32_t newval){
		if (firstCwnd){
			*cWndStream->GetStream () << "0.0 " << oldval << std::endl;
			firstCwnd = false;
		}
		*cWndStream->GetStream () << Simulator::Now ().GetSeconds () << " " << newval << std::endl;
		cWndValue = newval;

		if (!firstSshThr){
			*ssThreshStream->GetStream () << Simulator::Now ().GetSeconds () << " " << ssThreshValue << std::endl;
		}
	}

	static void
	SsThreshTracer (uint32_t oldval, uint32_t newval){
		if (firstSshThr){
			*ssThreshStream->GetStream () << "0.0 " << oldval << std::endl;
			firstSshThr = false;
		}
		*ssThreshStream->GetStream () << Simulator::Now ().GetSeconds () << " " << newval << std::endl;
		ssThreshValue = newval;

		if (!firstCwnd){
			*cWndStream->GetStream () << Simulator::Now ().GetSeconds () << " " << cWndValue << std::endl;
		}
	}

	static void
	RttTracer (Time oldval, Time newval){
		if (firstRtt){
			*rttStream->GetStream () << "0.0 " << oldval.GetSeconds () << std::endl;
			firstRtt = false;
		}
		*rttStream->GetStream () << Simulator::Now ().GetSeconds () << " " << newval.GetSeconds () << std::endl;
	}

	static void
	RtoTracer (Time oldval, Time newval){
		if (firstRto){
			*rtoStream->GetStream () << "0.0 " << oldval.GetSeconds () << std::endl;
			firstRto = false;
		}
		*rtoStream->GetStream () << Simulator::Now ().GetSeconds () << " " << newval.GetSeconds () << std::endl;
	}


	static void
	TraceCwnd (std::string cwnd_tr_file_name){
		AsciiTraceHelper ascii;
		cWndStream = ascii.CreateFileStream (cwnd_tr_file_name.c_str ());
		Config::ConnectWithoutContext ("/NodeList/1/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow", MakeCallback (&CwndTracer));
	}

	static void
	TraceSsThresh (std::string ssthresh_tr_file_name){
		AsciiTraceHelper ascii;
		ssThreshStream = ascii.CreateFileStream (ssthresh_tr_file_name.c_str ());
		Config::ConnectWithoutContext ("/NodeList/1/$ns3::TcpL4Protocol/SocketList/0/SlowStartThreshold", MakeCallback (&SsThreshTracer));
	}

	static void
	TraceRtt (std::string rtt_tr_file_name){
		AsciiTraceHelper ascii;
		rttStream = ascii.CreateFileStream (rtt_tr_file_name.c_str ());
		Config::ConnectWithoutContext ("/NodeList/1/$ns3::TcpL4Protocol/SocketList/0/RTT", MakeCallback (&RttTracer));
	}

	static void
	TraceRto (std::string rto_tr_file_name){
		AsciiTraceHelper ascii;
		rtoStream = ascii.CreateFileStream (rto_tr_file_name.c_str ());
		Config::ConnectWithoutContext ("/NodeList/1/$ns3::TcpL4Protocol/SocketList/0/RTO", MakeCallback (&RtoTracer));
	}

#endif

int main (int argc, char *argv[])
{
	Time::SetResolution(Time::NS);

	float duration;

#ifdef __WIRED__

	std::string transport_prot = "TcpReno";
	double error_p = 0.001;
	std::string bandwidth = "2Mbps";
	std::string access_bandwidth = "10Mbps";
	std::string access_delay = "45ms";
	bool tracing = true;
	std::string tr_file_name = "reno_tr";
	std::string cwnd_tr_file_name = "reno_cwnd_tr";
	std::string ssthresh_tr_file_name = "reno_ssthresh_tr";
	std::string rtt_tr_file_name = "reno_rtt_tr";
	std::string rto_tr_file_name = "reno_rto_tr";
	double data_mbytes = 0;
	uint32_t mtu_bytes = 400;
	uint16_t num_flows = 1;
	float duration_wired = 20;
	duration = duration_wired;
	uint32_t run = 0;
	bool flow_monitor = true;

#endif

#ifdef __WIRELESS__

	int nbSS = 2;
	float duration_wimax = 7;
	duration = duration_wimax;
	int schedType = 0;
	bool verbose = false;
	WimaxHelper::SchedulerType scheduler = WimaxHelper::SCHED_TYPE_SIMPLE;
	LogComponentEnable ("UdpClient", LOG_LEVEL_INFO);
	LogComponentEnable ("UdpServer", LOG_LEVEL_INFO);

#endif

	CommandLine cmd;

#ifdef __WIRED__

	cmd.AddValue ("transport_prot", "Transport protocol to use: TcpTahoe, TcpReno, TcpNewReno, TcpWestwood, TcpWestwoodPlus ", transport_prot);
	cmd.AddValue ("error_p", "Packet error rate", error_p);
	cmd.AddValue ("bandwidth", "Bottleneck bandwidth", bandwidth);
	cmd.AddValue ("access_bandwidth", "Access link bandwidth", access_bandwidth);
	cmd.AddValue ("delay", "Access link delay", access_delay);
	cmd.AddValue ("tracing", "Flag to enable/disable tracing", tracing);
	cmd.AddValue ("tr_name", "Name of output trace file", tr_file_name);
	cmd.AddValue ("cwnd_tr_name", "Name of output trace file", cwnd_tr_file_name);
	cmd.AddValue ("ssthresh_tr_name", "Name of output trace file", ssthresh_tr_file_name);
	cmd.AddValue ("rtt_tr_name", "Name of output trace file", rtt_tr_file_name);
	cmd.AddValue ("rto_tr_name", "Name of output trace file", rto_tr_file_name);
	cmd.AddValue ("data", "Number of Megabytes of data to transmit", data_mbytes);
	cmd.AddValue ("mtu", "Size of IP packets to send in bytes", mtu_bytes);
	cmd.AddValue ("num_flows", "Number of flows", num_flows);
	cmd.AddValue ("duration", "Time to allow flows to run in seconds", duration_wired);
	cmd.AddValue ("run", "Run index (for setting repeatable seeds)", run);
	cmd.AddValue ("flow_monitor", "Enable flow monitor", flow_monitor);
#endif

#ifdef __WIRELESS__

	cmd.AddValue ("nbSS", "number of subscriber station to create", nbSS);
	cmd.AddValue ("scheduler", "type of scheduler to use with the network devices", schedType);
	cmd.AddValue ("duration", "duration of the simulation in seconds", duration_wimax);
	cmd.AddValue ("verbose", "turn on all WimaxNetDevice log components", verbose);
#endif

	cmd.Parse (argc, argv);

	MobilityHelper mobility;
	

#ifdef __WIRELESS__

	switch (schedType){
		case 0:
			scheduler = WimaxHelper::SCHED_TYPE_SIMPLE;
			break;
		case 1:
			scheduler = WimaxHelper::SCHED_TYPE_MBQOS;
			break;
		case 2:
			scheduler = WimaxHelper::SCHED_TYPE_RTPS;
			break;
		default:
			scheduler = WimaxHelper::SCHED_TYPE_SIMPLE;
	}


	NodeContainer ssNodes;
	NodeContainer bsNodes;

	ssNodes.Create (2);
	bsNodes.Create (2);

	WimaxHelper wimax;

	Ipv4AddressHelper address_wimax;

	NetDeviceContainer ssDevs, bsDevs;

	ssDevs = wimax.Install (ssNodes, WimaxHelper::DEVICE_TYPE_SUBSCRIBER_STATION, WimaxHelper::SIMPLE_PHY_TYPE_OFDM, scheduler);
	bsDevs = wimax.Install (bsNodes, WimaxHelper::DEVICE_TYPE_BASE_STATION, WimaxHelper::SIMPLE_PHY_TYPE_OFDM, scheduler);

	Ptr<SubscriberStationNetDevice>* ss = new Ptr<SubscriberStationNetDevice>[nbSS];

	for (int i = 0; i < nbSS; i++){
		ss[i] = ssDevs.Get (i)->GetObject<SubscriberStationNetDevice> ();
		ss[i]->SetModulationType (WimaxPhy::MODULATION_TYPE_QAM16_12);
	}

	Ptr<BaseStationNetDevice>* bs = new Ptr<BaseStationNetDevice>[2];
	bs[0] = bsDevs.Get (0)->GetObject<BaseStationNetDevice> ();
	bs[1] = bsDevs.Get (1)->GetObject<BaseStationNetDevice> ();
	
	mobility.Install (bsNodes);
	mobility.Install (ssNodes);

	InternetStackHelper stack_wimax;
	stack_wimax.Install (bsNodes);
	stack_wimax.Install (ssNodes);

	
	address_wimax.SetBase ("10.1.1.0", "255.255.255.0");
	Ipv4InterfaceContainer SSinterfaces = address_wimax.Assign (ssDevs);
	Ipv4InterfaceContainer BSinterface = address_wimax.Assign (bsDevs);


	if (verbose){
		wimax.EnableLogComponents ();  // Turn on all wimax logging
	}
	/*------------------------------*/
	UdpServerHelper* udpServer = new UdpServerHelper[1];
	ApplicationContainer* serverApps = new ApplicationContainer[1];
	
	UdpClientHelper* udpClient = new UdpClientHelper[1];
	ApplicationContainer* clientApps = new ApplicationContainer[1];

	
	// set server port to 100+(i*10)
	udpServer[0] = UdpServerHelper (100);

	serverApps[0] = udpServer[0].Install (ssNodes.Get (0));
	serverApps[0].Start (Seconds (0.1));
	serverApps[0].Stop (Seconds (duration_wimax));

	udpClient[0] = UdpClientHelper (SSinterfaces.GetAddress (0), 100);
	udpClient[0].SetAttribute ("MaxPackets", UintegerValue (1200));
	udpClient[0].SetAttribute ("Interval", TimeValue (Seconds (0.12)));
	udpClient[0].SetAttribute ("PacketSize", UintegerValue (800));

	clientApps[0] = udpClient[0].Install (ssNodes.Get (1));
	clientApps[0].Start (Seconds (0.1));
	clientApps[0].Stop (Seconds (duration_wimax));

	cout << "Base Src : " << BSinterface.GetAddress(0) << endl;
	cout << "Base Dest : " << BSinterface.GetAddress(1) << endl;
	cout << "Subs Src : " << SSinterfaces.GetAddress(0) << endl;
	cout << "Subs Dest : " << SSinterfaces.GetAddress(1) << endl;
	
	/*
		IpcsClassifierRecord (Ipv4Address srcAddress, Ipv4Mask srcMask, Ipv4Address dstAddress, 
			Ipv4Mask dstMask, uint16_t srcPortLow, uint16_t srcPortHigh, uint16_t dstPortLow, 
			uint16_t dstPortHigh, uint8_t protocol, uint8_t priority)
	*/


	// client
	// capture traffic from all nodes
	IpcsClassifierRecord DlClassifierBe (BSinterface.GetAddress (1),
										Ipv4Mask ("255.255.255.255"),
										SSinterfaces.GetAddress (0),
										Ipv4Mask ("255.255.255.255"),
										0,
										65000,
										100,
										100,
										17,
										1);
	ServiceFlow DlServiceFlowBe = wimax.CreateServiceFlow (ServiceFlow::SF_DIRECTION_DOWN,
	                              ServiceFlow::SF_TYPE_BE,
	                              DlClassifierBe);
	ss[1]->AddServiceFlow (DlServiceFlowBe);

	// server
	// deliver traffic to all nodes
	IpcsClassifierRecord ulClassifierBe (SSinterfaces.GetAddress (1),
										Ipv4Mask ("255.255.255.255"),
										BSinterface.GetAddress (0),
										Ipv4Mask ("255.255.255.255"),
										0,
										65000,
										100,
										100,
										17,
										1);
	ServiceFlow ulServiceFlowBe = wimax.CreateServiceFlow (ServiceFlow::SF_DIRECTION_UP,
	                              ServiceFlow::SF_TYPE_BE,
	                              ulClassifierBe);
	ss[0]->AddServiceFlow (ulServiceFlowBe);


#endif

	mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
	                               "MinX", DoubleValue (0.0),
	                               "MinY", DoubleValue (0.0),
	                               "DeltaX", DoubleValue (5.0),
	                               "DeltaY", DoubleValue (10.0),
	                               "GridWidth", UintegerValue (3),
	                               "LayoutType", StringValue ("RowFirst"));

	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

#ifdef __WIRED__

	Ipv4AddressHelper address_wired;

	SeedManager::SetSeed (1);
	SeedManager::SetRun (run);

	// User may find it convenient to enable logging
	LogComponentEnable("compare", LOG_LEVEL_ALL);
	// LogComponentEnable("BulkSendApplication", LOG_LEVEL_INFO);
	// LogComponentEnable("DropTailQueue", LOG_LEVEL_ALL);

	// Calculate the ADU size
	Header* temp_header = new Ipv4Header ();
	uint32_t ip_header = temp_header->GetSerializedSize ();
	NS_LOG_LOGIC ("IP Header size is: " << ip_header);
	delete temp_header;
	temp_header = new TcpHeader ();
	uint32_t tcp_header = temp_header->GetSerializedSize ();
	NS_LOG_LOGIC ("TCP Header size is: " << tcp_header);
	delete temp_header;
	uint32_t tcp_adu_size = mtu_bytes - (ip_header + tcp_header);
	NS_LOG_LOGIC ("TCP ADU size is: " << tcp_adu_size);

	// Set the simulation start and stop time
	float start_time = 0.1;
	float stop_time = start_time + duration_wired;

	Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TcpReno::GetTypeId ()));

	// Create gateways, sources, and sinks
	NodeContainer gateways;
	gateways.Create (2);
	NodeContainer sources;
	sources.Create (num_flows);
	NodeContainer sinks;
	sinks.Create (num_flows);

	// Configure the error model
	// Here we use RateErrorModel with packet error rate
	Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
	uv->SetStream (50);
	RateErrorModel error_model;
	error_model.SetRandomVariable (uv);
	error_model.SetUnit (RateErrorModel::ERROR_UNIT_PACKET);
	error_model.SetRate (error_p);

	PointToPointHelper src_gate_link;
	src_gate_link.SetDeviceAttribute ("DataRate", StringValue (bandwidth));
	src_gate_link.SetChannelAttribute ("Delay", StringValue ("1ms"));
	// src_gate_link.SetDeviceAttribute ("ReceiveErrorModel", PointerValue (&error_model));

	PointToPointHelper gate_dest_link;
	gate_dest_link.SetDeviceAttribute ("DataRate", StringValue (bandwidth));
	gate_dest_link.SetChannelAttribute ("Delay", StringValue ("10ms"));
	// gate_dest_link.SetDeviceAttribute ("ReceiveErrorModel", PointerValue (&error_model));

	PointToPointHelper gate_gate_link;
	gate_gate_link.SetDeviceAttribute ("DataRate", StringValue (bandwidth));
	gate_gate_link.SetChannelAttribute ("Delay", StringValue ("1ms"));

	InternetStackHelper stack_wired;
	stack_wired.Install (gateways);
	stack_wired.Install (sources);
	stack_wired.Install (sinks);
	

	address_wired.SetBase ("10.0.0.0", "255.255.255.0");

	

	Ipv4InterfaceContainer sink_interfaces;

	NetDeviceContainer devices;
	devices = src_gate_link.Install (sources.Get (0), gateways.Get (0));
	
	address_wired.NewNetwork ();
	Ipv4InterfaceContainer interfaces = address_wired.Assign (devices);
	devices = gate_gate_link.Install (gateways.Get (0), gateways.Get (1));
	
	address_wired.NewNetwork ();
	interfaces = address_wired.Assign (devices);
	devices = gate_dest_link.Install (gateways.Get (1), sinks.Get (0));
	
	address_wired.NewNetwork ();
	interfaces = address_wired.Assign (devices);
	sink_interfaces.Add (interfaces.Get (1));

	NS_LOG_INFO ("Initialize Global Routing.");
	Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

	uint16_t port = 50000;
	Address sinkLocalAddress (InetSocketAddress (Ipv4Address::GetAny (), port));
	PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", sinkLocalAddress);

	for (uint16_t i = 0; i < sources.GetN (); i++){
		AddressValue remoteAddress (InetSocketAddress (sink_interfaces.GetAddress (i, 0), port));

		if (transport_prot.compare ("TcpReno") == 0){
			Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (tcp_adu_size));
			BulkSendHelper ftp ("ns3::TcpSocketFactory", Address ());
			ftp.SetAttribute ("Remote", remoteAddress);
			ftp.SetAttribute ("SendSize", UintegerValue (tcp_adu_size));
			ftp.SetAttribute ("MaxBytes", UintegerValue (int(data_mbytes * 1000000)));

			ApplicationContainer sourceApp = ftp.Install (sources.Get (i));
			sourceApp.Start (Seconds (start_time * i));
			sourceApp.Stop (Seconds (stop_time - 3));

			sinkHelper.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
			ApplicationContainer sinkApp = sinkHelper.Install (sinks);
			sinkApp.Start (Seconds (start_time * i));
			sinkApp.Stop (Seconds (stop_time));
		}
		else{
			NS_LOG_DEBUG ("Invalid transport protocol " << transport_prot << " specified");
			exit (1);
		}
	}

	// Set up tracing if enabled
	if (tracing){
		if (tr_file_name.compare ("") != 0){
			std::ofstream ascii;
			Ptr<OutputStreamWrapper> ascii_wrap;
			ascii.open (tr_file_name.c_str ());
			ascii_wrap = new OutputStreamWrapper (tr_file_name.c_str (), std::ios::out);
			stack_wired.EnableAsciiIpv4 (ascii_wrap, gateways);
			stack_wired.EnableAsciiIpv4 (ascii_wrap, sources);
			stack_wired.EnableAsciiIpv4 (ascii_wrap, sinks);
		}
	
		if (cwnd_tr_file_name.compare ("") != 0){
			Simulator::Schedule (Seconds (0.00001), &TraceCwnd, cwnd_tr_file_name);
		}

		if (ssthresh_tr_file_name.compare ("") != 0){
			Simulator::Schedule (Seconds (0.00001), &TraceSsThresh, ssthresh_tr_file_name);
		}

		if (rtt_tr_file_name.compare ("") != 0){
			Simulator::Schedule (Seconds (0.00001), &TraceRtt, rtt_tr_file_name);
		}

		if (rto_tr_file_name.compare ("") != 0){
			Simulator::Schedule (Seconds (0.00001), &TraceRto, rto_tr_file_name);
		}
	
	}

	src_gate_link.EnablePcapAll ("reno_pcap", true);
	gate_gate_link.EnablePcapAll ("reno_pcap", true);

	// Flow monitor
	FlowMonitorHelper flowHelper;
	if (flow_monitor){
		flowHelper.Install (gateways);
		flowHelper.Install (sources);
		flowHelper.Install (sinks);
	}

	mobility.Install(sources);
	mobility.Install(gateways);
	mobility.Install(sinks);


	AnimationInterface::SetConstantPosition(sources.Get(0), 100, 300);
	// AnimationInterface::SetConstantPosition(sources.Get(1), 100, 100);

	AnimationInterface::SetConstantPosition(gateways.Get(0), 150, 250);
	AnimationInterface::SetConstantPosition(gateways.Get(1), 250, 150);
	

	AnimationInterface::SetConstantPosition(sinks.Get(0), 300, 100);
	// AnimationInterface::SetConstantPosition(sinks.Get(1), 300, 300);


#endif

	Simulator::Stop (Seconds (duration + 0.1));

	AnimationInterface anim ("compare.xml");
	anim.SetMobilityPollInterval (Seconds (0.01));

#ifdef __WIRED__	

	if (flow_monitor){
		flowHelper.SerializeToXmlFile ("reno_tcp_flow.xml", true, true);
	}

#endif

#ifdef __WIRELESS__

	mobility.Install(ssNodes);
	mobility.Install(bsNodes);

	for (int i = 0; i < nbSS; ++i){
		AnimationInterface::SetConstantPosition(ssNodes.Get(i), 100, 50*i);
	}
	AnimationInterface::SetConstantPosition(bsNodes.Get(0), 40, 70);
	AnimationInterface::SetConstantPosition(bsNodes.Get(1), 40, 80);
	
	
#endif

	Simulator::Run ();

#ifdef __WIRELESS__

	delete[] clientApps;
	delete[] udpClient;
	delete[] serverApps;
	delete[] udpServer;
	for (int i = 0; i < nbSS; i++){
		ss[i] = 0;
	}
	delete[] ss;

	bs = 0;

#endif

	Simulator::Destroy ();
	return 0;
}

