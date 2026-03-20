#!/bin/bash
# Test script for O-RAN Slice-Aware RBG Scheduling with E2 Integration
# This script demonstrates the complete functionality

echo "=============================================="
echo "  O-RAN Slice-Aware Scheduling Test Suite"
echo "=============================================="
echo ""

# Set working directory
cd /home/elioth/Documentos/artigo_jussi/ns-3-dev

# Create results directory
mkdir -p results

echo "=== TEST 1: Baseline Simulation (E2 Disabled) ==="
echo "Running 2s simulation with 10 eMBB + 10 IoT UEs..."
echo ""

./ns3 run "scratch/oran_slicing_rbg \
    --simTime=2 \
    --enableE2=false \
    --outputDir=results/baseline"

echo ""
echo "=== Baseline Results ==="
echo ""
cat results/baseline/summary.json | python3 -c "
import json, sys
data = json.load(sys.stdin)
print(f\"eMBB: {data['eMBB']['throughputAggregatedMbps']:.2f} Mbps, {data['eMBB']['avgLatencyMs']:.2f} ms, PDR={data['eMBB']['packetDeliveryRatio']:.2%}\")
print(f\"IoT:  {data['iot']['throughputAggregatedMbps']:.4f} Mbps, {data['iot']['avgLatencyMs']:.2f} ms, PDR={data['iot']['packetDeliveryRatio']:.2%}\")
print(f\"SLA Met: {data['slaVerification']['embbThroughputSlaMet'] and data['slaVerification']['embbLatencySlaMet']}\")
"

echo ""
echo "=== TEST 2: E2 Integration (RIC xApp Ready) ==="
echo ""
echo "The following files demonstrate E2/O-RAN integration:"
echo "  - scratch/oran_slicing_rbg.cc   : ns-3 gNB with E2Termination"
echo "  - scratch/ric_xapp_simple.py    : Python RIC/xApp controller"
echo ""
echo "To test with E2 integration, run in separate terminals:"
echo ""
echo "  Terminal 1 (RIC):"
echo "    python3 scratch/ric_xapp_simple.py --server-ip 127.0.0.1"
echo ""
echo "  Terminal 2 (gNB):"
echo "    ./ns3 run 'scratch/oran_slicing_rbg --enableE2=true --ricAddress=127.0.0.1'"
echo ""

echo "=== TEST 3: Video Traffic Configuration ==="
echo ""
echo "The 3GPP TR 38.838 video model is configured for:"
echo "  - Min Rate: 5 Mbps"
echo "  - Max Rate: 10 Mbps"
echo "  - Avg Rate: 7.5 Mbps"
echo "  - FPS: 30-60 (avg 30)"
echo ""

echo "=== Summary Files Generated ==="
echo ""
ls -la results/baseline/
echo ""

echo "=== Test Complete ==="
