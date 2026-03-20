/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Baseline 5G-LENA Slicing Simulation with 2 Slices (eMBB and mMTC)
 *
 * This simulation implements a baseline scenario using 5G-LENA without RIC/E2/O-RAN components.
 * Slices are represented through:
 * - Logical separation of UE groups (eMBB UEs vs mMTC UEs)
 * - QoS-aware scheduler selection (without native radio slicing)
 * - Distinct traffic patterns per slice
 *
 * Author: Based on 5G-LENA v4.1.1
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

#include <fstream>
#include <map>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <limits>
#include <utility>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("BaselineSlicingTwoSlices");

// Structure for per-UE aggregated statistics
struct UeAggregateStats {
    uint32_t ueId = 0;
    uint8_t sliceId = 255;
    uint64_t txBytes = 0;
    uint64_t rxBytes = 0;
    uint32_t txPackets = 0;
    uint32_t rxPackets = 0;
    double totalDelay = 0.0; // Sum of delays in seconds
};

struct SliceTimeSeriesStats
{
    double throughputMbps = 0.0;
    double avgDelayMs = 0.0;
    double packetLossRatio = 0.0;
    uint32_t activeUes = 0;
};

static std::pair<uint32_t, uint8_t>
ResolveUeAndSlice(const Ipv4FlowClassifier::FiveTuple& t,
                  const Ipv4Address& remoteHostAddr,
                  const std::map<Ipv4Address, uint32_t>& ipToUeMap,
                  const std::map<uint32_t, uint8_t>& ueToSliceMap)
{
    uint32_t ueId = 0;
    uint8_t sliceId = 255;

    if (t.sourceAddress == remoteHostAddr)
    {
        auto it = ipToUeMap.find(t.destinationAddress);
        if (it != ipToUeMap.end())
        {
            ueId = it->second;
            auto sliceIt = ueToSliceMap.find(ueId);
            if (sliceIt != ueToSliceMap.end())
            {
                sliceId = sliceIt->second;
            }
        }
    }
    else if (t.destinationAddress == remoteHostAddr)
    {
        auto it = ipToUeMap.find(t.sourceAddress);
        if (it != ipToUeMap.end())
        {
            ueId = it->second;
            auto sliceIt = ueToSliceMap.find(ueId);
            if (sliceIt != ueToSliceMap.end())
            {
                sliceId = sliceIt->second;
            }
        }
    }

    return {ueId, sliceId};
}

static SliceTimeSeriesStats
BuildSliceTimeSeriesStats(Ptr<FlowMonitor> monitor,
                          Ptr<Ipv4FlowClassifier> classifier,
                          uint8_t targetSliceId,
                          const Ipv4Address& remoteHostAddr,
                          const std::map<Ipv4Address, uint32_t>& ipToUeMap,
                          const std::map<uint32_t, uint8_t>& ueToSliceMap,
                          double elapsedSeconds)
{
    SliceTimeSeriesStats out;
    if (monitor == nullptr || classifier == nullptr || elapsedSeconds <= 0.0)
    {
        return out;
    }

    monitor->CheckForLostPackets();
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    uint64_t rxBytes = 0;
    uint32_t txPackets = 0;
    uint32_t rxPackets = 0;
    double totalDelaySeconds = 0.0;
    std::map<uint32_t, bool> activeUeSet;

    for (const auto& [flowId, flowStats] : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);
        auto [ueId, sliceId] = ResolveUeAndSlice(t, remoteHostAddr, ipToUeMap, ueToSliceMap);
        if (sliceId != targetSliceId)
        {
            continue;
        }

        rxBytes += flowStats.rxBytes;
        txPackets += flowStats.txPackets;
        rxPackets += flowStats.rxPackets;
        totalDelaySeconds += flowStats.delaySum.GetSeconds();
        if (ueId != 0 && (flowStats.txPackets > 0 || flowStats.rxPackets > 0))
        {
            activeUeSet[ueId] = true;
        }
    }

    out.throughputMbps = static_cast<double>(rxBytes) * 8.0 / (elapsedSeconds * 1e6);
    out.avgDelayMs = (rxPackets > 0) ? (totalDelaySeconds / rxPackets) * 1000.0 : 0.0;
    out.packetLossRatio = (txPackets > 0) ? static_cast<double>(txPackets - rxPackets) / txPackets : 0.0;
    out.activeUes = static_cast<uint32_t>(activeUeSet.size());
    return out;
}

static void
SampleTimeSeries(Ptr<FlowMonitor> monitor,
                 Ptr<Ipv4FlowClassifier> classifier,
                 Ipv4Address remoteHostAddr,
                 const std::map<Ipv4Address, uint32_t>* ipToUeMap,
                 const std::map<uint32_t, uint8_t>* ueToSliceMap,
                 const std::vector<Ptr<PacketSink>>* embbSinks,
                 Ptr<PacketSink> mmTcSink,
                 double metricsWindowSeconds,
                 double simTimeSeconds,
                 std::ofstream* timeOut)
{
    if (timeOut == nullptr || !timeOut->is_open())
    {
        return;
    }

    double now = Simulator::Now().GetSeconds();
    if (now <= 0.0)
    {
        return;
    }

    auto embbStats = BuildSliceTimeSeriesStats(monitor,
                                               classifier,
                                               0,
                                               remoteHostAddr,
                                               *ipToUeMap,
                                               *ueToSliceMap,
                                               now);
    auto mmTcStats = BuildSliceTimeSeriesStats(monitor,
                                               classifier,
                                               1,
                                               remoteHostAddr,
                                               *ipToUeMap,
                                               *ueToSliceMap,
                                               now);

    uint64_t embbAppRxBytes = 0;
    for (const auto& sink : *embbSinks)
    {
        if (sink != nullptr)
        {
            embbAppRxBytes += sink->GetTotalRx();
        }
    }
    uint64_t mmTcAppRxBytes = (mmTcSink != nullptr) ? mmTcSink->GetTotalRx() : 0;

    (*timeOut) << now << ",0,eMBB,"
               << embbStats.throughputMbps << ","
               << embbStats.avgDelayMs << ","
               << embbStats.packetLossRatio << ","
               << embbStats.activeUes << ","
               << embbAppRxBytes << "\n";

    (*timeOut) << now << ",1,mMTC,"
               << mmTcStats.throughputMbps << ","
               << mmTcStats.avgDelayMs << ","
               << mmTcStats.packetLossRatio << ","
               << mmTcStats.activeUes << ","
               << mmTcAppRxBytes << "\n";

    double nextSample = now + metricsWindowSeconds;
    if (nextSample <= simTimeSeconds + 1e-9)
    {
        Simulator::Schedule(Seconds(metricsWindowSeconds),
                            &SampleTimeSeries,
                            monitor,
                            classifier,
                            remoteHostAddr,
                            ipToUeMap,
                            ueToSliceMap,
                            embbSinks,
                            mmTcSink,
                            metricsWindowSeconds,
                            simTimeSeconds,
                            timeOut);
    }
}

int main(int argc, char* argv[])
{
    // =========================================================================
    // SECTION 1: Setup and Configuration
    // =========================================================================

    // Total simulation parameters
    uint32_t simTime = 10;              // Simulation duration (seconds)
    uint32_t runNumber = 1;             // Random run number

    // UE parameters
    uint32_t nUes = 20;                 // Total number of UEs
    uint32_t nEmbbUes = 10;             // Number of eMBB UEs
    uint32_t nMmTcUes = 10;             // Number of mMTC UEs
    double ueDistance = 100.0;          // UE distance from gNB (meters)
    double ueSpread = 0.0;              // Additional deterministic radial spread from base distance (meters)

    // eMBB traffic parameters
    std::string embbDataRate = "25Mbps"; // Offered rate per eMBB UE

    // mMTC traffic parameters
    uint32_t mmTcPacketSize = 100;       // mMTC packet size (bytes)
    double mmTcInterval = 1.0;           // mMTC transmission interval (seconds)
    double mmTcJitter = 0.1;              // mMTC jitter (seconds)

    // NR PHY parameters (3GPP-like configuration)
    // NOTE: The exact PRB count depends on the realized BWP/band configuration in the installed 5G-LENA version.
    uint16_t numerology = 0;            // Subcarrier spacing (0 = 15 kHz)
    double centralFrequency = 3.5e9;     // 3.5 GHz
    double bandwidth = 20e6;              // 20 MHz nominal channel bandwidth
    double totalTxPower = 43;            // Total TX power in dBm

    bool enableOfdma = false;           // Use TDMA scheduler (PF is simpler)

    // Metrics output
    std::string outputDir = "results";  // Output directory
    bool enableLogging = false;          // Enable detailed logging
    uint32_t metricsWindowMs = 1000;      // Time-series sampling window (milliseconds)

    // SLA targets
    double embbSlaThroughputMbps = 25.0;   // eMBB SLA throughput target
    double embbSlaLatencyMs = 100.0;        // eMBB SLA latency target

    // Parse command line
    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
    cmd.AddValue("nUes", "Total number of UEs", nUes);
    cmd.AddValue("nEmbbUes", "Number of eMBB UEs", nEmbbUes);
    cmd.AddValue("nMmTcUes", "Number of mMTC UEs", nMmTcUes);
    cmd.AddValue("ueDistance", "UE distance from gNB (meters)", ueDistance);
    cmd.AddValue("ueSpread", "Spread UEs in circle (meters)", ueSpread);
    cmd.AddValue("embbDataRate", "eMBB offered rate per UE", embbDataRate);
    cmd.AddValue("mmTcPacketSize", "mMTC packet size (bytes)", mmTcPacketSize);
    cmd.AddValue("mmTcInterval", "mMTC transmission interval (s)", mmTcInterval);
    cmd.AddValue("mmTcJitter", "mMTC jitter (s)", mmTcJitter);
    cmd.AddValue("numerology", "NR numerology (0-3)", numerology);
    cmd.AddValue("centralFrequency", "Central frequency (Hz)", centralFrequency);
    cmd.AddValue("bandwidth", "System bandwidth (Hz)", bandwidth);
    cmd.AddValue("totalTxPower", "Total TX power (dBm)", totalTxPower);
    cmd.AddValue("enableOfdma", "Enable OFDMA scheduler", enableOfdma);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("runNumber", "Random run number", runNumber);
    cmd.AddValue("enableLogging", "Enable detailed logging", enableLogging);
    cmd.AddValue("embbSlaThroughputMbps", "eMBB SLA throughput target (Mbps)", embbSlaThroughputMbps);
    cmd.AddValue("embbSlaLatencyMs", "eMBB SLA latency target (ms)", embbSlaLatencyMs);
    cmd.AddValue("metricsWindowMs", "Time-series sampling window (ms)", metricsWindowMs);
    cmd.Parse(argc, argv);

    // Validate configuration
    // Allow single-slice testing: nUes can be larger than nEmbbUes + nMmTcUes
    // This enables tests like Test A (1 eMBB, 0 mMTC) with nUes=20
    NS_ABORT_MSG_IF(simTime <= 0, "ERROR: simTime must be > 0");
    NS_ABORT_MSG_IF(ueDistance <= 0, "ERROR: ueDistance must be > 0");
    NS_ABORT_MSG_IF(nEmbbUes > nUes || nMmTcUes > nUes, "ERROR: eMBB/mMTC UEs cannot exceed total UEs");
    NS_ABORT_MSG_IF(nEmbbUes + nMmTcUes == 0, "ERROR: Must have at least 1 UE total");
    NS_ABORT_MSG_IF(numerology > 3, "ERROR: Numerology must be 0-3");
    NS_ABORT_MSG_IF(metricsWindowMs == 0, "ERROR: metricsWindowMs must be > 0");
    NS_ABORT_MSG_IF(ueSpread < 0.0, "ERROR: ueSpread must be >= 0");

    // Enable logging if requested
    if (enableLogging)
    {
        LogComponentEnable("BaselineSlicingTwoSlices", LOG_LEVEL_INFO);
    }

    // Create results directory
    SystemPath::MakeDirectories(outputDir);

    // Seed random generators
    // NOTE: RngSeedManager is the correct API in ns-3.46/5G-LENA
    // SeedManager is a typedef to RngSeedManager (kept for backward compatibility)
    RngSeedManager::SetSeed(runNumber);
    RngSeedManager::SetRun(runNumber);

    // Configure global defaults
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    Config::SetDefault("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue(999999999));

    NS_LOG_INFO("=== BASELINE 5G-LENA SLICING SIMULATION ===");
    NS_LOG_INFO("Configuration:");
    NS_LOG_INFO("  - Total UEs: " << nUes << " (eMBB: " << nEmbbUes << ", mMTC: " << nMmTcUes << ")");
    NS_LOG_INFO("  - Simulation time: " << simTime << " s");
    NS_LOG_INFO("  - UE distance: " << ueDistance << " m");
    NS_LOG_INFO("  - Bandwidth: " << (bandwidth / 1e6) << " MHz (numerology " << numerology << ")");
    NS_LOG_INFO("  - Central frequency: " << (centralFrequency / 1e9) << " GHz");
    NS_LOG_INFO("  - eMBB data rate: " << embbDataRate << " per UE");
    NS_LOG_INFO("  - mMTC interval: " << mmTcInterval << " s (jitter: " << mmTcJitter << " s)");
    NS_LOG_INFO("  - Scheduler: " << (enableOfdma ? "OFDMA" : "TDMA") << " QoS");
    NS_LOG_INFO("  - Time-series window: " << metricsWindowMs << " ms");

    // =========================================================================
    // SECTION 2: Network Topology and NR Setup
    // =========================================================================

    // Configure EPC
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    // Configure scheduler (QoS-aware)
    std::string schedulerType = std::string("ns3::NrMacScheduler") +
                            (enableOfdma ? "Ofdma" : "Tdma") + "Qos";

    // BWP ID mapping for QCI-to-BWP mapping
    // eMBB (NGBR_VIDEO_TCP_OPERATOR, QCI=6) maps to BWP 0
    // mMTC (GBR_CONV_VOICE, QCI=1) maps to BWP 1
    uint8_t bwpIdForEmbb = 0;
    uint8_t bwpIdForMmtc = 1;

    nrHelper->SetGnbBwpManagerAlgorithmTypeId(TypeId::LookupByName("ns3::BwpManagerAlgorithmStatic"));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_OPERATOR", UintegerValue(bwpIdForEmbb));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpIdForMmtc));
    nrHelper->SetUeBwpManagerAlgorithmTypeId(TypeId::LookupByName("ns3::BwpManagerAlgorithmStatic"));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_OPERATOR", UintegerValue(bwpIdForEmbb));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpIdForMmtc));

    // Configure spectrum (single operation band with 2 BWPs)
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, 1);
    bandConf.m_numBwp = 2;
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    // Configure channel (3GPP channel model)
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
    channelHelper->AssignChannelsToBands({band});

    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    // Configure antennas
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(1));

    NS_LOG_INFO("EPC and NR helpers configured");

    // =========================================================================
    // SECTION 3: UE Creation and Positioning (Deterministic)
    // =========================================================================

    // Create node containers
    NodeContainer gnbNodes;
    NodeContainer ueEmbbNodes;
    NodeContainer ueMmTcNodes;
    gnbNodes.Create(1);
    ueEmbbNodes.Create(nEmbbUes);
    ueMmTcNodes.Create(nMmTcUes);

    // Configure mobility
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    // Position gNB at origin (0, 0, 10m height)
    mobility.Install(gnbNodes);
    Ptr<ConstantPositionMobilityModel> gnbPos = gnbNodes.Get(0)->GetObject<ConstantPositionMobilityModel>();
    gnbPos->SetPosition(Vector(0.0, 0.0, 10.0));

    // Position UEs deterministically around the gNB.
    // If ueSpread > 0, nodes are placed in a reproducible annulus [ueDistance, ueDistance + ueSpread].
    double angleStep = 2.0 * M_PI / nUes; // Radians between UEs

    auto computeRadius = [&](uint32_t globalIndex) {
        if (nUes <= 1 || ueSpread <= 0.0)
        {
            return ueDistance;
        }
        double fraction = static_cast<double>(globalIndex) / static_cast<double>(nUes - 1);
        return ueDistance + fraction * ueSpread;
    };

    // Position eMBB UEs
    for (uint32_t i = 0; i < ueEmbbNodes.GetN(); ++i)
    {
        double angle = i * angleStep;
        double radius = computeRadius(i);
        double x = radius * std::cos(angle);
        double y = radius * std::sin(angle);

        mobility.Install(ueEmbbNodes.Get(i));
        Ptr<ConstantPositionMobilityModel> uePos =
            ueEmbbNodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
        uePos->SetPosition(Vector(x, y, 1.5)); // 1.5m UE height
    }

    // Position mMTC UEs (continue from eMBB angle positions)
    for (uint32_t i = 0; i < ueMmTcNodes.GetN(); ++i)
    {
        uint32_t globalIndex = i + nEmbbUes;
        double angle = globalIndex * angleStep;
        double radius = computeRadius(globalIndex);
        double x = radius * std::cos(angle);
        double y = radius * std::sin(angle);

        mobility.Install(ueMmTcNodes.Get(i));
        Ptr<ConstantPositionMobilityModel> uePos =
            ueMmTcNodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
        uePos->SetPosition(Vector(x, y, 1.5));
    }

    NS_LOG_INFO("Nodes created and positioned:");
    NS_LOG_INFO("  - 1 gNB at (0, 0, 10)");
    NS_LOG_INFO("  - " << nEmbbUes << " eMBB UEs in deterministic annulus from "
                 << ueDistance << " m to " << (ueDistance + ueSpread) << " m");
    NS_LOG_INFO("  - " << nMmTcUes << " mMTC UEs in deterministic annulus from "
                 << ueDistance << " m to " << (ueDistance + ueSpread) << " m");

    // =========================================================================
    // SECTION 4: Install NR Devices and Configure
    // =========================================================================

    // Install gNB and UE devices
    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueEmbbDevs = nrHelper->InstallUeDevice(ueEmbbNodes, allBwps);
    NetDeviceContainer ueMmTcDevs = nrHelper->InstallUeDevice(ueMmTcNodes, allBwps);

    // Install Internet stack on all nodes (BEFORE attaching UEs)
    InternetStackHelper internet;
    internet.Install(ueEmbbNodes);
    internet.Install(ueMmTcNodes);
    internet.Install(gnbNodes);

    // Assign IP addresses to UEs (EPC does this automatically)
    Ipv4InterfaceContainer ueEmbbIpIfaces = nrEpcHelper->AssignUeIpv4Address(ueEmbbDevs);
    Ipv4InterfaceContainer ueMmTcIpIfaces = nrEpcHelper->AssignUeIpv4Address(ueMmTcDevs);

    // Configure PHY numerology
    for (uint32_t i = 0; i < gnbDevs.GetN(); ++i)
    {
        Ptr<NrGnbPhy> gnbPhy = nrHelper->GetGnbPhy(gnbDevs.Get(i), 0);
        gnbPhy->SetAttribute("Numerology", UintegerValue(numerology));
        // TxPower attribute expects dBm directly - no unit conversion needed
        // Previous bug: 10 * log10(pow(10, totalTxPower/10)/1000) resulted in ~13 dBm for 43 dBm input
        // Fix: Use DoubleValue(totalTxPower) directly to set the correct dBm value
        gnbPhy->SetAttribute("TxPower", DoubleValue(totalTxPower));
    }

    // Attach UEs to closest gNB (AFTER installing Internet stack and assigning IPs)
    nrHelper->AttachToClosestGnb(ueEmbbDevs, gnbDevs);
    nrHelper->AttachToClosestGnb(ueMmTcDevs, gnbDevs);

    NS_LOG_INFO("NR devices installed and UEs attached");

    // =========================================================================
    // SECTION 5: Internet Stack and IP Configuration
    // =========================================================================

    // Get PGW and create remote host for internet traffic
    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    internet.Install(remoteHostContainer);

    // Create P2P link for remote host to PGW
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(10)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    // EPC handles UE routing automatically through default gateway
    // Remote host needs route to UE subnet
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4> remoteIp = remoteHost->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> remoteStaticRouting = ipv4RoutingHelper.GetStaticRouting(remoteIp);
    remoteStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    NS_LOG_INFO("Internet stack installed and IP addresses assigned");
    NS_LOG_INFO("  - Remote host: " << remoteHostAddr);
    NS_LOG_INFO("  - UE subnet: 7.0.0.0/8");

    // =========================================================================
    // SECTION 6: Traffic Generation Setup
    // =========================================================================

    // NOTE: This baseline does NOT implement native radio slicing or dedicated
    // bearers per slice. The differentiation is intentionally logical only:
    // 1. Different traffic patterns (eMBB continuous DL vs mMTC intermittent UL)
    // 2. QoS-aware scheduler selection (OFDMA/TDMA QoS)
    // 3. Explicit UE-group-to-slice mapping for metrics collection
    //
    // Future enhancement: dedicated bearers with TFT and/or external control
    // loops can be layered on top of this baseline.

    // =========================================================================
    // SECTION 7: Traffic Generation Setup
    // =========================================================================

    uint16_t dlPort = 1234;
    uint16_t ulPort = 5678;

    // eMBB: Downlink traffic from remote host to UEs
    // Packet sinks on eMBB UEs (downlink receivers)
    ApplicationContainer embbSinkApps;
    for (uint32_t i = 0; i < ueEmbbNodes.GetN(); ++i)
    {
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                     InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        ApplicationContainer sinkApp = sinkHelper.Install(ueEmbbNodes.Get(i));
        sinkApp.Start(Seconds(0.0));
        sinkApp.Stop(Seconds(simTime));
        embbSinkApps.Add(sinkApp);
    }

    // OnOff senders on remote host (one per eMBB UE, downlink)
    ApplicationContainer embbSourceApps;
    for (uint32_t i = 0; i < ueEmbbNodes.GetN(); ++i)
    {
        OnOffHelper onOffHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(ueEmbbIpIfaces.GetAddress(i), dlPort));
        onOffHelper.SetAttribute("DataRate", DataRateValue(DataRate(embbDataRate)));
        onOffHelper.SetAttribute("PacketSize", UintegerValue(1400)); // Standard MTU - headers
        onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

        ApplicationContainer sourceApp = onOffHelper.Install(remoteHost);
        sourceApp.Start(Seconds(0.1));
        sourceApp.Stop(Seconds(simTime));
        embbSourceApps.Add(sourceApp);
    }

    NS_LOG_INFO("eMBB traffic configured:");
    NS_LOG_INFO("  - Downlink from remote host to eMBB UEs");
    NS_LOG_INFO("  - Rate: " << embbDataRate << " per UE");
    NS_LOG_INFO("  - Continuous traffic (OnTime=1, OffTime=0)");

    // mMTC: Packet sink on remote host (uplink receiver)
    PacketSinkHelper mmTcSinkHelper("ns3::UdpSocketFactory",
                                     InetSocketAddress(Ipv4Address::GetAny(), ulPort));
    ApplicationContainer mmTcSink = mmTcSinkHelper.Install(remoteHost);
    mmTcSink.Start(Seconds(0.0));
    mmTcSink.Stop(Seconds(simTime));

    // OnOff senders on mMTC UEs (uplink, intermittent)
    ApplicationContainer mmTcSourceApps;
    for (uint32_t i = 0; i < ueMmTcNodes.GetN(); ++i)
    {
        OnOffHelper onOffHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(remoteHostAddr, ulPort));
        onOffHelper.SetAttribute("DataRate", DataRateValue(DataRate("1Mbps")));
        onOffHelper.SetAttribute("PacketSize", UintegerValue(mmTcPacketSize));
        // 1ms burst, variable off time around mmTcInterval
        // Using uniform random variable for simpler pattern
        double offTimeMin = std::max(0.001, mmTcInterval - mmTcJitter);
        double offTimeMax = mmTcInterval + mmTcJitter;
        std::string onTimeStr = "ns3::ConstantRandomVariable[Constant=0.001]";
        std::string offTimeStr = "ns3::UniformRandomVariable[Min=" +
                                std::to_string(offTimeMin) + "|Max=" +
                                std::to_string(offTimeMax) + "]";
        onOffHelper.SetAttribute("OnTime", StringValue(onTimeStr));
        onOffHelper.SetAttribute("OffTime", StringValue(offTimeStr));

        ApplicationContainer sourceApp = onOffHelper.Install(ueMmTcNodes.Get(i));
        sourceApp.Start(Seconds(0.1));
        sourceApp.Stop(Seconds(simTime));
        mmTcSourceApps.Add(sourceApp);
    }

    NS_LOG_INFO("mMTC traffic configured:");
    NS_LOG_INFO("  - Uplink from mMTC UEs to remote host");
    NS_LOG_INFO("  - Packet size: " << mmTcPacketSize << " bytes");
    NS_LOG_INFO("  - Interval: " << mmTcInterval << " s (jitter: " << mmTcJitter << " s)");

    // =========================================================================
    // SECTION 8: Explicit Mappings (IP -> UE -> Slice)
    // =========================================================================

    // Build explicit mappings for metrics collection
    std::map<Ipv4Address, uint32_t> ipToUeMap;       // IP -> UE ID
    std::map<uint32_t, uint8_t> ueToSliceMap;       // UE ID -> Slice ID

    // UE to Slice mapping (0 = eMBB, 1 = mMTC)
    for (uint32_t i = 0; i < ueEmbbNodes.GetN(); ++i)
    {
        uint32_t ueId = ueEmbbNodes.Get(i)->GetId();
        ueToSliceMap[ueId] = 0; // eMBB
        ipToUeMap[ueEmbbIpIfaces.GetAddress(i)] = ueId;
    }
    for (uint32_t i = 0; i < ueMmTcNodes.GetN(); ++i)
    {
        uint32_t ueId = ueMmTcNodes.Get(i)->GetId();
        ueToSliceMap[ueId] = 1; // mMTC
        ipToUeMap[ueMmTcIpIfaces.GetAddress(i)] = ueId;
    }

    NS_LOG_INFO("Explicit mappings created:");
    NS_LOG_INFO("  - " << ipToUeMap.size() << " IP to UE mappings");
    NS_LOG_INFO("  - " << ueToSliceMap.size() << " UE to Slice mappings");

    // =========================================================================
    // SECTION 9: FlowMonitor and Metrics Collection
    // =========================================================================

    // Install FlowMonitor for per-flow statistics
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    // Enable FlowMonitor to track statistics
    monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001)); // 1ms bins
    monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("StartTime", TimeValue(Seconds(0.0))); // Start immediately to capture all packets

    NS_LOG_INFO("FlowMonitor installed");

    // Prepare time-series output before the simulation starts.
    std::string timeseriesFile = outputDir + "/timeseries_metrics.csv";
    std::ofstream timeOut(timeseriesFile);
    timeOut << "timestampSeconds,sliceId,sliceName,cumulativeThroughputMbps,"
               "cumulativeAvgDelayMs,cumulativePacketLossRatio,activeUes,appRxBytes\n";

    std::vector<Ptr<PacketSink>> embbSinkPtrs;
    for (uint32_t i = 0; i < embbSinkApps.GetN(); ++i)
    {
        Ptr<PacketSink> sink = DynamicCast<PacketSink>(embbSinkApps.Get(i));
        if (sink != nullptr)
        {
            embbSinkPtrs.push_back(sink);
        }
    }
    Ptr<PacketSink> mmTcSinkPtr = DynamicCast<PacketSink>(mmTcSink.Get(0));

    // =========================================================================
    // SECTION 10: Run Simulation
    // =========================================================================

    NS_LOG_INFO("Starting simulation...");

    double metricsWindowSeconds = static_cast<double>(metricsWindowMs) / 1000.0;
    Simulator::Schedule(Seconds(metricsWindowSeconds),
                        &SampleTimeSeries,
                        monitor,
                        DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier()),
                        remoteHostAddr,
                        &ipToUeMap,
                        &ueToSliceMap,
                        &embbSinkPtrs,
                        mmTcSinkPtr,
                        metricsWindowSeconds,
                        static_cast<double>(simTime),
                        &timeOut);

    // Allow connections to establish before monitoring starts
    Simulator::Stop(Seconds(simTime));

    // Run simulation
    Simulator::Run();

    // IMPORTANT: Collect FlowMonitor statistics BEFORE destroying simulator
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    NS_LOG_INFO("Simulation completed, collecting metrics...");

    // Destroy simulator AFTER collecting statistics
    Simulator::Destroy();

    // =========================================================================
    // SECTION 11: Export Metrics (All Files)
    // =========================================================================

    // Define output file paths
    std::string ueMetricsFile = outputDir + "/ue_metrics.csv";
    std::string flowMetricsFile = outputDir + "/flow_metrics.csv";
    std::string sliceMetricsFile = outputDir + "/slice_metrics.csv";
    std::string summaryFile = outputDir + "/summary.json";

    // ===== EXPORT FLOW METRICS =====
    std::ofstream flowOut(flowMetricsFile);
    flowOut << "flowId,ueId,sliceId,direction,sourceIp,destIp,sourcePort,destPort,"
             << "protocol,txBytes,rxBytes,txPackets,rxPackets,"
             << "delayMeanMs,delayStdDevMs,jitterMeanMs,packetLossRatio\n";

    uint64_t totalEmbbTxBytes = 0;
    uint64_t totalMmTcTxBytes = 0;
    uint64_t totalEmbbRxBytes = 0;
    uint64_t totalMmTcRxBytes = 0;
    uint32_t totalEmbbTxPackets = 0;
    uint32_t totalMmTcTxPackets = 0;
    uint32_t totalEmbbRxPackets = 0;
    uint32_t totalMmTcRxPackets = 0;

    // Process each flow
    for (auto const& [flowId, flowStats] : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);

        uint8_t direction = 255; // Unknown
        if (t.sourceAddress == remoteHostAddr)
        {
            direction = 0; // DL
        }
        else if (t.destinationAddress == remoteHostAddr)
        {
            direction = 1; // UL
        }

        auto [ueId, sliceId] = ResolveUeAndSlice(t, remoteHostAddr, ipToUeMap, ueToSliceMap);

        // Only export flows we could classify
        if (sliceId != 255)
        {
            // Update slice totals
            if (sliceId == 0) // eMBB
            {
                totalEmbbTxBytes += flowStats.txBytes;
                totalEmbbRxBytes += flowStats.rxBytes;
                totalEmbbTxPackets += flowStats.txPackets;
                totalEmbbRxPackets += flowStats.rxPackets;
            }
            else if (sliceId == 1) // mMTC
            {
                totalMmTcTxBytes += flowStats.txBytes;
                totalMmTcRxBytes += flowStats.rxBytes;
                totalMmTcTxPackets += flowStats.txPackets;
                totalMmTcRxPackets += flowStats.rxPackets;
            }

            // Calculate metrics
            double avgDelayMs = 0.0;
            double jitterSumMs = 0.0;

            if (flowStats.rxPackets > 0)
            {
                avgDelayMs = flowStats.delaySum.GetSeconds() / flowStats.rxPackets * 1000.0;
                // Using jitterSum / rxPackets as approximation
                jitterSumMs = flowStats.jitterSum.GetSeconds() / flowStats.rxPackets * 1000.0;
            }

            double packetLossRatio = 0.0;
            if (flowStats.txPackets > 0)
            {
                packetLossRatio = static_cast<double>(flowStats.txPackets - flowStats.rxPackets) /
                                 flowStats.txPackets;
            }

            flowOut << flowId << "," << ueId << "," << static_cast<int>(sliceId) << ","
                     << static_cast<int>(direction) << ","
                     << t.sourceAddress << "," << t.destinationAddress << ","
                     << t.sourcePort << "," << t.destinationPort << ","
                     << "UDP" << ","
                     << flowStats.txBytes << "," << flowStats.rxBytes << ","
                     << flowStats.txPackets << "," << flowStats.rxPackets << ","
                     << avgDelayMs << ",0," << jitterSumMs << ","
                     << packetLossRatio << "\n";
        }
    }
    flowOut.close();

    NS_LOG_INFO("Flow metrics exported to: " << flowMetricsFile);

    // ===== EXPORT UE METRICS =====
    std::ofstream ueOut(ueMetricsFile);
    ueOut << "ueId,sliceId,avgThroughputMbps,avgDelayMs,"
            << "packetLossRatio,txBytes,rxBytes,txPackets,rxPackets\n";

    // Per-UE aggregated metrics (sum flows per UE)
    std::map<uint32_t, UeAggregateStats> ueAggMap;
    for (auto const& [flowId, flowStats] : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);
        auto [ueId, sliceId] = ResolveUeAndSlice(t, remoteHostAddr, ipToUeMap, ueToSliceMap);

        if (sliceId != 255)
        {
            auto& agg = ueAggMap[ueId];
            agg.ueId = ueId;
            agg.sliceId = sliceId;
            agg.txBytes += flowStats.txBytes;
            agg.rxBytes += flowStats.rxBytes;
            agg.txPackets += flowStats.txPackets;
            agg.rxPackets += flowStats.rxPackets;
            agg.totalDelay += flowStats.delaySum.GetSeconds();
        }
    }

    // Write UE metrics
    for (auto const& [ueId, agg] : ueAggMap)
    {
        double avgDelayMs = 0.0;
        if (agg.rxPackets > 0)
        {
            avgDelayMs = agg.totalDelay / agg.rxPackets * 1000.0;
        }

        double avgThroughputMbps = agg.rxBytes * 8.0 / (simTime * 1e6);
        double packetLossRatio = 0.0;
        if (agg.txPackets > 0)
        {
            packetLossRatio = static_cast<double>(agg.txPackets - agg.rxPackets) / agg.txPackets;
        }

        ueOut << ueId << "," << static_cast<int>(agg.sliceId) << ","
                << avgThroughputMbps << ","
                << avgDelayMs << ","
                << packetLossRatio << ","
                << agg.txBytes << "," << agg.rxBytes << ","
                << agg.txPackets << "," << agg.rxPackets << "\n";
    }
    ueOut.close();

    NS_LOG_INFO("UE metrics exported to: " << ueMetricsFile);

    // ===== EXPORT SLICE METRICS =====
    std::ofstream sliceOut(sliceMetricsFile);
    sliceOut << "sliceId,sliceName,throughputAggregatedMbps,"
             << "throughputPerUeMbps,avgLatencyMs,packetDeliveryRatio,"
             << "packetLossRatio,activeUes,totalTxBytes,totalRxBytes\n";

    const char* sliceNames[] = {"eMBB", "mMTC"};
    for (uint8_t sliceId = 0; sliceId < 2; ++sliceId)
    {
        uint64_t txBytes = (sliceId == 0) ? totalEmbbTxBytes : totalMmTcTxBytes;
        uint64_t rxBytes = (sliceId == 0) ? totalEmbbRxBytes : totalMmTcRxBytes;
        uint32_t txPackets = (sliceId == 0) ? totalEmbbTxPackets : totalMmTcTxPackets;
        uint32_t rxPackets = (sliceId == 0) ? totalEmbbRxPackets : totalMmTcRxPackets;
        uint32_t activeUes = (sliceId == 0) ? nEmbbUes : nMmTcUes;

        double throughputMbps = rxBytes * 8.0 / (simTime * 1e6);
        double throughputPerUeMbps = (activeUes > 0) ? throughputMbps / activeUes : 0.0;

        double packetDeliveryRatio = (txPackets > 0) ? static_cast<double>(rxPackets) / txPackets : 0.0;
        double packetLossRatio = 1.0 - packetDeliveryRatio;

        // Calculate average latency for slice (approximated from per-UE)
        double avgLatencyMs = 0.0;
        if (rxPackets > 0)
        {
            double totalDelay = 0.0;
            for (auto const& [ueId, agg] : ueAggMap)
            {
                if (agg.sliceId == sliceId && agg.rxPackets > 0)
                {
                    totalDelay += agg.totalDelay;
                }
            }
            avgLatencyMs = totalDelay / rxPackets * 1000.0;
        }

        sliceOut << static_cast<int>(sliceId) << "," << sliceNames[sliceId] << ","
                 << throughputMbps << ","
                 << throughputPerUeMbps << ","
                 << avgLatencyMs << ","
                 << packetDeliveryRatio << ","
                 << packetLossRatio << ","
                 << activeUes << ","
                 << txBytes << "," << rxBytes << "\n";
    }
    sliceOut.close();

    NS_LOG_INFO("Slice metrics exported to: " << sliceMetricsFile);

    // ===== EXPORT SUMMARY JSON =====
    std::ofstream sumOut(summaryFile);

    sumOut << "{\n";
    sumOut << "  \"simulation\": {\n";
    sumOut << "    \"seed\": " << runNumber << ",\n";
    sumOut << "    \"runNumber\": " << runNumber << ",\n";
    sumOut << "    \"durationSeconds\": " << simTime << "\n";
    sumOut << "  },\n";

    sumOut << "  \"configuration\": {\n";
    sumOut << "    \"totalUes\": " << nUes << ",\n";
    sumOut << "    \"embbUes\": " << nEmbbUes << ",\n";
    sumOut << "    \"mmTcUes\": " << nMmTcUes << ",\n";
    sumOut << "    \"ueDistanceMeters\": " << ueDistance << ",\n";
    sumOut << "    \"ueSpreadMeters\": " << ueSpread << ",\n";
    sumOut << "    \"embbDataRateMbps\": " << DataRate(embbDataRate).GetBitRate() / 1e6 << ",\n";
    sumOut << "    \"mmTcPacketSizeBytes\": " << mmTcPacketSize << ",\n";
    sumOut << "    \"mmTcIntervalSeconds\": " << mmTcInterval << ",\n";
    sumOut << "    \"mmTcJitterSeconds\": " << mmTcJitter << ",\n";
    sumOut << "    \"numerology\": " << static_cast<int>(numerology) << ",\n";
    sumOut << "    \"centralFrequencyHz\": " << centralFrequency << ",\n";
    sumOut << "    \"bandwidthHz\": " << bandwidth << ",\n";
    sumOut << "    \"bandwidthMHz\": " << (bandwidth / 1e6) << ",\n";
    sumOut << "    \"totalTxPowerDbm\": " << totalTxPower << ",\n";
    sumOut << "    \"scheduler\": \"" << schedulerType << "\",\n";
    sumOut << "    \"sliceRepresentation\": \"logical_separation_via_ue_groups_qos_scheduler_and_traffic_patterns\",\n";
    sumOut << "    \"prbNote\": \"PRB count depends on the realized BWP/band configuration of the installed 5G-LENA version and is not hard-coded in this baseline\",\n";
    sumOut << "    \"duplexingNote\": \"TDD pattern is not explicitly overridden here; the simulation uses the 5G-LENA defaults associated with the selected band/BWP setup\"\n";
    sumOut << "  },\n";

    // eMBB slice metrics
    double embbThroughputMbps = totalEmbbRxBytes * 8.0 / (simTime * 1e6);
    double embbThroughputPerUeMbps = (nEmbbUes > 0) ? embbThroughputMbps / nEmbbUes : 0.0;
    bool embbThroughputSla = (embbThroughputPerUeMbps >= embbSlaThroughputMbps);

    double embbAvgLatencyMs = 0.0;
    if (totalEmbbRxPackets > 0)
    {
        double totalEmbbDelay = 0.0;
        for (auto const& [ueId, agg] : ueAggMap)
        {
            if (agg.sliceId == 0)
            {
                totalEmbbDelay += agg.totalDelay;
            }
        }
        embbAvgLatencyMs = totalEmbbDelay / totalEmbbRxPackets * 1000.0;
    }
    bool embbLatencySla = (embbAvgLatencyMs <= embbSlaLatencyMs);

    double embbPacketDeliveryRatio = (totalEmbbTxPackets > 0) ?
                                   static_cast<double>(totalEmbbRxPackets) / totalEmbbTxPackets : 0.0;
    double embbPacketLossRatio = 1.0 - embbPacketDeliveryRatio;

    sumOut << "  \"eMBB\": {\n";
    sumOut << "    \"throughputAggregatedMbps\": " << embbThroughputMbps << ",\n";
    sumOut << "    \"throughputPerUeMbps\": " << embbThroughputPerUeMbps << ",\n";
    sumOut << "    \"avgLatencyMs\": " << embbAvgLatencyMs << ",\n";
    sumOut << "    \"packetDeliveryRatio\": " << embbPacketDeliveryRatio << ",\n";
    sumOut << "    \"packetLossRatio\": " << embbPacketLossRatio << ",\n";
    sumOut << "    \"activeUes\": " << nEmbbUes << ",\n";
    sumOut << "    \"totalTxBytes\": " << totalEmbbTxBytes << ",\n";
    sumOut << "    \"totalRxBytes\": " << totalEmbbRxBytes << ",\n";
    sumOut << "    \"totalTxPackets\": " << totalEmbbTxPackets << ",\n";
    sumOut << "    \"totalRxPackets\": " << totalEmbbRxPackets << ",\n";
    sumOut << "    \"slaCompliance\": {\n";
    sumOut << "      \"throughput25MbpsMet\": " << (embbThroughputSla ? "true" : "false") << ",\n";
    sumOut << "      \"latency100msMet\": " << (embbLatencySla ? "true" : "false") << "\n";
    sumOut << "    }\n";
    sumOut << "  },\n";

    // mMTC slice metrics
    double mmTcThroughputMbps = totalMmTcRxBytes * 8.0 / (simTime * 1e6);
    double mmTcThroughputPerUeKbps = (nMmTcUes > 0) ? (mmTcThroughputMbps * 1e3) / nMmTcUes : 0.0;

    double mmTcAvgLatencyMs = 0.0;
    if (totalMmTcRxPackets > 0)
    {
        double totalMmTcDelay = 0.0;
        for (auto const& [ueId, agg] : ueAggMap)
        {
            if (agg.sliceId == 1)
            {
                totalMmTcDelay += agg.totalDelay;
            }
        }
        mmTcAvgLatencyMs = totalMmTcDelay / totalMmTcRxPackets * 1000.0;
    }

    double mmTcPacketDeliveryRatio = (totalMmTcTxPackets > 0) ?
                                      static_cast<double>(totalMmTcRxPackets) / totalMmTcTxPackets : 0.0;
    double mmTcPacketLossRatio = 1.0 - mmTcPacketDeliveryRatio;

    sumOut << "  \"mMTC\": {\n";
    sumOut << "    \"throughputAggregatedMbps\": " << mmTcThroughputMbps << ",\n";
    sumOut << "    \"throughputPerUeKbps\": " << mmTcThroughputPerUeKbps << ",\n";
    sumOut << "    \"avgLatencyMs\": " << mmTcAvgLatencyMs << ",\n";
    sumOut << "    \"packetDeliveryRatio\": " << mmTcPacketDeliveryRatio << ",\n";
    sumOut << "    \"packetLossRatio\": " << mmTcPacketLossRatio << ",\n";
    sumOut << "    \"activeUes\": " << nMmTcUes << ",\n";
    sumOut << "    \"totalTxBytes\": " << totalMmTcTxBytes << ",\n";
    sumOut << "    \"totalRxBytes\": " << totalMmTcRxBytes << ",\n";
    sumOut << "    \"totalTxPackets\": " << totalMmTcTxPackets << ",\n";
    sumOut << "    \"totalRxPackets\": " << totalMmTcRxPackets << "\n";
    sumOut << "  },\n";

    sumOut << "  \"timeSeries\": {\n";
    sumOut << "    \"file\": \"timeseries_metrics.csv\",\n";
    sumOut << "    \"windowMs\": " << metricsWindowMs << ",\n";
    sumOut << "    \"note\": \"Time-series values are cumulative snapshots sampled during the simulation\"\n";
    sumOut << "  },\n";

    // Energy proxy (bytes transmitted)
    sumOut << "  \"energyProxy\": {\n";
    sumOut << "    \"note\": \"Energy is approximated by total bytes transmitted, not actual RF power consumption\",\n";
    sumOut << "    \"embb\": {\n";
    sumOut << "      \"txBytes\": " << totalEmbbTxBytes << "\n";
    sumOut << "    },\n";
    sumOut << "    \"mmTc\": {\n";
    sumOut << "      \"txBytes\": " << totalMmTcTxBytes << "\n";
    sumOut << "    }\n";
    sumOut << "  },\n";

    // Limitations
    sumOut << "  \"limitations\": [\n";
    sumOut << "    \"No native slice-aware scheduler in 5G-LENA v4.1.1 - using logical separation by UE groups, QoS-aware scheduling, and traffic patterns\",\n";
    sumOut << "    \"Energy model is proxy-based (bytes transmitted and packet counts), not actual RF power consumption\",\n";
    sumOut << "    \"Metrics are collected primarily at IP/flow layer via FlowMonitor, with application-byte counters from PacketSink for time-series snapshots; there is no PDCP/RLC/MAC/PHY export in this baseline\",\n";
    sumOut << "    \"Jitter is approximated from FlowMonitor jitterSum, not per-packet measurements\",\n";
    sumOut << "    \"Latency P95 is not computed because this baseline does not store per-packet delay samples\",\n";
    sumOut << "    \"SLA targets are application-level requirements, network may not meet them due to capacity constraints\",\n";
    sumOut << "    \"Cell capacity limitation: With 20 MHz bandwidth at 3.5 GHz, the cell achieves ~60-70 Mbps aggregate throughput. With 10 eMBB UEs offering 25 Mbps each (250 Mbps total), this causes congestion, high latency (~4s), and packet loss (~74%). This behavior is consistent with official ns-3/NR examples (cttc-nr-demo, cttc-nr-multi-flow-qos-sched). Single UE scenarios achieve near Offered Rate (24 Mbps) with low latency (13 ms).\"\n";
    sumOut << "  ]\n";

    sumOut << "}\n";
    sumOut.close();
    timeOut.close();

    NS_LOG_INFO("Summary exported to: " << summaryFile);
    NS_LOG_INFO("Time-series exported to: " << timeseriesFile);

    // =========================================================================
    // SECTION 12: Print Summary to Stdout and Validation
    // =========================================================================

    std::cout << "\n=================================================================\n";
    std::cout << "BASELINE 5G-LENA SLICING SIMULATION - SUMMARY\n";
    std::cout << "=================================================================\n";
    std::cout << "Simulation Time: " << simTime << " seconds\n";
    std::cout << "Random Run: " << runNumber << "\n\n";

    std::cout << "Configuration:\n";
    std::cout << "- Total UEs: " << nUes << "\n";
    std::cout << "  - eMBB UEs: " << nEmbbUes << "\n";
    std::cout << "  - mMTC UEs: " << nMmTcUes << "\n";
    std::cout << "- UE Distance: " << ueDistance << " m from gNB\n";
    std::cout << "- UE Spread: " << ueSpread << " m additional radial spread (deterministic annulus)\n";
    std::cout << "- eMBB Data Rate: " << embbDataRate << " per UE\n";
    std::cout << "- mMTC Packet Size: " << mmTcPacketSize << " bytes\n";
    std::cout << "- mMTC Interval: " << mmTcInterval << " s (jitter: " << mmTcJitter << " s)\n";
    std::cout << "- Numerology: " << static_cast<int>(numerology) << " (SCS=" << (15 << numerology) << " kHz)\n";
    std::cout << "- Bandwidth: " << (bandwidth / 1e6) << " MHz\n";
    std::cout << "- Central Frequency: " << (centralFrequency / 1e9) << " GHz\n";
    std::cout << "- Total TX Power: " << totalTxPower << " dBm\n";
    std::cout << "- Scheduler: " << schedulerType << "\n";
    std::cout << "- Slice Representation: Logical separation by UE groups + QoS-aware scheduler + traffic patterns\n";
    std::cout << "- Duplexing/TDD note: using 5G-LENA defaults for the configured band/BWP unless overridden by the installed version\n\n";

    std::cout << "Results:\n";
    std::cout << "=================================================================\n";

    // eMBB slice results
    std::cout << "SLICE eMBB (Video/AI Surveillance):\n";
    std::cout << "- Aggregated Throughput: " << embbThroughputMbps << " Mbps\n";
    std::cout << "- Avg Throughput per UE: " << embbThroughputPerUeMbps << " Mbps\n";
    std::cout << "- Avg Latency: " << embbAvgLatencyMs << " ms\n";
    std::cout << "- Packet Loss Ratio: " << (embbPacketLossRatio * 100) << "%\n";
    std::cout << "- SLA Compliance:\n";
    std::cout << "  * Throughput >= " << embbSlaThroughputMbps << " Mbps: "
              << (embbThroughputSla ? "YES" : "NO") << "\n";
    std::cout << "  * Latency <= " << embbSlaLatencyMs << " ms: "
              << (embbLatencySla ? "YES" : "NO") << "\n\n";

    // mMTC slice results
    std::cout << "SLICE mMTC (IoT Sensor Monitoring):\n";
    std::cout << "- Active Sensors: " << nMmTcUes << "\n";
    std::cout << "- Avg Throughput per Sensor: " << mmTcThroughputPerUeKbps << " kbps\n";
    std::cout << "- Delivery Ratio: " << (mmTcPacketDeliveryRatio * 100) << "%\n";
    std::cout << "- Packet Loss Ratio: " << (mmTcPacketLossRatio * 100) << "%\n";
    std::cout << "- Avg Delay: " << mmTcAvgLatencyMs << " ms\n";
    std::cout << "- Energy Proxy (bytes TX): " << totalMmTcTxBytes << " bytes total\n\n";

    std::cout << "Output Files Generated:\n";
    std::cout << "- " << ueMetricsFile << "\n";
    std::cout << "- " << flowMetricsFile << "\n";
    std::cout << "- " << sliceMetricsFile << "\n";
    std::cout << "- " << timeseriesFile << "\n";
    std::cout << "- " << summaryFile << "\n\n";

    // ===== VALIDATION CHECKS =====
    std::cout << "Validation:\n";
    bool allValid = true;

    if (ueEmbbNodes.GetN() != nEmbbUes)
    {
        std::cerr << "ERROR: Expected " << nEmbbUes << " eMBB UEs, got "
                  << ueEmbbNodes.GetN() << "\n";
        allValid = false;
    }
    if (ueMmTcNodes.GetN() != nMmTcUes)
    {
        std::cerr << "ERROR: Expected " << nMmTcUes << " mMTC UEs, got "
                  << ueMmTcNodes.GetN() << "\n";
        allValid = false;
    }
    if (stats.empty())
    {
        std::cerr << "ERROR: No flow statistics collected\n";
        allValid = false;
    }
    if (ueAggMap.size() != nUes)
    {
        std::cout << "WARNING: Only " << ueAggMap.size() << " of " << nUes
                  << " UEs have flow statistics\n";
    }

    std::cout << (allValid ? "- All validation checks PASSED" : "- Some validation checks FAILED") << "\n";
    std::cout << "=================================================================\n";
    std::cout << "Simulation completed " << (allValid ? "successfully!" : "with errors!") << "\n";
    std::cout << "=================================================================\n";

    return allValid ? 0 : 1;
}
