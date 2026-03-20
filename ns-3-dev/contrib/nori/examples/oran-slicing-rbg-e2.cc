/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * O-RAN Slice-Aware RBG Scheduling with E2 Integration (NORI-based)
 *
 * Uses NORI E2TermHelperSlicing for proper E2/O-RAN integration.
 * Connects to an external Near-RT RIC via SCTP.
 *
 * Usage:
 *   Terminal 1 (RIC): Run ricsim or OSC RIC on port 36422
 *   Terminal 2 (gNB): ./ns3 run "oran-slicing-rbg-e2 --ricAddress=127.0.0.1"
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-generator-3gpp-generic-video.h"
#include "ns3/E2-term-helper-slicing.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranSlicingRbgE2");

int main(int argc, char* argv[])
{
    uint32_t simTime = 10;
    uint32_t nEmbbUes = 10;
    uint32_t nIotUes = 10;
    double ueDistance = 100.0;
    std::string ricAddress = "10.107.233.133";
    std::string outputDir = "results";
    double embbStaticWeight = 0.5;
    double iotStaticWeight = 0.5;
    double embbDynamicShare = 0.5;
    double iotDynamicShare = 0.5;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("nEmbbUes", "Number of eMBB UEs", nEmbbUes);
    cmd.AddValue("nIotUes", "Number of IoT UEs", nIotUes);
    cmd.AddValue("ueDistance", "UE distance (m)", ueDistance);
    cmd.AddValue("ricAddress", "RIC E2Term IP address", ricAddress);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("embbStaticWeight", "eMBB static weight", embbStaticWeight);
    cmd.AddValue("iotStaticWeight", "IoT static weight", iotStaticWeight);
    cmd.Parse(argc, argv);

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    auto nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    auto idealBeamforming = CreateObject<IdealBeamformingHelper>();
    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamforming);
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerOfdmaSliceQos"));
    nrHelper->SetSchedulerAttribute("EmbbStaticWeight", DoubleValue(embbStaticWeight));
    nrHelper->SetSchedulerAttribute("MmtcStaticWeight", DoubleValue(iotStaticWeight));
    nrHelper->SetSchedulerAttribute("EmbbDynamicShare", DoubleValue(embbDynamicShare));
    nrHelper->SetSchedulerAttribute("MmtcDynamicShare", DoubleValue(iotDynamicShare));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 200e6, 1);
    bandConf.m_numBwp = 1;
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    auto channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = ccBwpCreator.GetAllBwps({band});

    idealBeamforming->SetAttribute("BeamformingMethod",
        TypeIdValue(DirectPathBeamforming::GetTypeId()));

    NodeContainer gnbNodes, ueEmbbNodes, ueIotNodes;
    gnbNodes.Create(1);
    ueEmbbNodes.Create(nEmbbUes);
    ueIotNodes.Create(nIotUes);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodes);
    gnbNodes.Get(0)->GetObject<ConstantPositionMobilityModel>()->SetPosition(Vector(0, 0, 10));

    double angleStep = 2.0 * M_PI / (nEmbbUes + nIotUes);
    for (uint32_t i = 0; i < nEmbbUes; ++i)
    {
        mobility.Install(ueEmbbNodes.Get(i));
        ueEmbbNodes.Get(i)->GetObject<ConstantPositionMobilityModel>()->
            SetPosition(Vector(ueDistance * cos(i * angleStep), ueDistance * sin(i * angleStep), 1.5));
    }
    for (uint32_t i = 0; i < nIotUes; ++i)
    {
        mobility.Install(ueIotNodes.Get(i));
        ueIotNodes.Get(i)->GetObject<ConstantPositionMobilityModel>()->
            SetPosition(Vector(ueDistance * cos((i + nEmbbUes) * angleStep),
                               ueDistance * sin((i + nEmbbUes) * angleStep), 1.5));
    }

    NetDeviceContainer gnbDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueEmbbDev = nrHelper->InstallUeDevice(ueEmbbNodes, allBwps);
    NetDeviceContainer ueIotDev = nrHelper->InstallUeDevice(ueIotNodes, allBwps);

    DynamicCast<NrGnbPhy>(gnbDev.Get(0))->SetAttribute("Numerology", UintegerValue(2));
    DynamicCast<NrGnbPhy>(gnbDev.Get(0))->SetAttribute("TxPower", DoubleValue(43.0));

    InternetStackHelper internet;
    internet.Install(ueEmbbNodes);
    internet.Install(ueIotNodes);

    Ipv4InterfaceContainer ueEmbbIp = nrEpcHelper->AssignUeIpv4Address(ueEmbbDev);
    Ipv4InterfaceContainer ueIotIp = nrEpcHelper->AssignUeIpv4Address(ueIotDev);

    nrHelper->AttachToClosestGnb(ueEmbbDev, gnbDev);
    nrHelper->AttachToClosestGnb(ueIotDev, gnbDev);

    // E2 Integration using NORI
    auto e2Helper = CreateObject<E2TermHelperSlicing>();
    e2Helper->SetAttribute("E2TermIp", StringValue(ricAddress));
    e2Helper->InstallE2Term(gnbDev.Get(0));

    NS_LOG_INFO("E2 integration enabled - connecting to RIC at " << ricAddress);

    // Applications
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    internet.Install(remoteHostContainer);
    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(10)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>())->
        AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    ApplicationContainer serverApps, clientApps;
    uint16_t dlPort = 1234;

    for (uint32_t i = 0; i < nEmbbUes; ++i)
    {
        PacketSinkHelper sink("ns3::UdpSocketFactory",
            InetSocketAddress(IIpv4Address::GetAny(), dlPort + i));
        serverApps.Add(sink.Install(ueEmbbNodes.Get(i)));
    }

    serverApps.Start(Seconds(0.1));
    serverApps.Stop(Seconds(simTime));

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    NS_LOG_INFO("Starting simulation for " << simTime << " seconds...");
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Simulator::Destroy();

    NS_LOG_INFO("Simulation complete");
    return 0;
}
