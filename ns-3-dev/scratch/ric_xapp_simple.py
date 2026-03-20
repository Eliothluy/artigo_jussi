#!/usr/bin/env python3
"""
Simple O-RAN Near-RT RIC/xApp for Slice Control

This xApp implements a rule-based controller that:
1. Connects to gNB via E2 interface (SCTP)
2. Subscribes to KPM (Key Performance Metrics) data
3. Monitors slice KPIs (throughput, latency, PRB usage)
4. Sends RIC control commands to adjust slice weights when SLA violations are detected

SLA Targets:
- eMBB: >= 7.5 Mbps throughput, < 100ms latency
- IoT: >= 0.01 Mbps throughput, < 100ms latency
"""

import socket
import struct
import json
import time
import os
from datetime import datetime

class SimpleRicXapp:
    """Simple Near-RT RIC xApp for slice control"""
    
    def __init__(self, config):
        self.server_ip = config.get('server_ip', '10.0.2.10')
        self.server_port = config.get('server_port', 36422)
        self.client_port = config.get('client_port', 38472)
        self.log_level = config.get('log_level', 2)  # 0=UNCOND, 1=ERROR, 2=INFO, 3=DEBUG
        
        # SLA Configuration
        self.slas = config.get('sla_config', {
            'embb': {'min_throughput_mbps': 7.5, 'max_latency_ms': 100.0, 'priority': 1.0},
            'iot': {'min_throughput_mbps': 0.01, 'max_latency_ms': 100.0, 'priority': 0.5}
        })
        
        # Slice state
        self.slice_weights = {'embb': 0.5, 'iot': 0.5}
        self.slice_metrics = {}
        
        # Decision history
        self.decision_history = []
        
        # Output directory
        self.output_dir = config.get('output_dir', 'results')
        os.makedirs(self.output_dir, exist_ok=True)
        
    def log(self, level, message):
        """Log message based on log level"""
        if level <= self.log_level:
            level_names = {0: 'UNCOND', 1: 'ERROR', 2: 'INFO', 3: 'DEBUG'}
            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            print(f"[{timestamp}] [{level_names.get(level, 'UNKNOWN')}] {message}")
            
    def start_server(self):
        """Start SCTP server to listen for E2 connections"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.server_ip, self.server_port))
            self.sock.listen(1)
            self.log(2, f"RIC listening on {self.server_ip}:{self.server_port}")
            return True
        except Exception as e:
            self.log(1, f"Failed to start server: {e}")
            return False
            
    def wait_for_e2_setup(self):
        """Wait for E2 Setup Request from gNB"""
        self.log(2, "Waiting for E2 Setup Request...")
        try:
            self.conn, addr = self.sock.accept()
            self.log(2, f"Connected by {addr}")
            
            # Receive E2 Setup Request (simplified)
            data = self.conn.recv(4096)
            if data:
                self.log(3, f"Received {len(data)} bytes")
                self.log(2, "E2 Setup Request received")
                return True
        except Exception as e:
            self.log(1, f"E2 Setup failed: {e}")
        return False
        
    def send_subscription_request(self):
        """Send RIC Subscription Request for KPM data"""
        self.log(2, "Sending RIC Subscription Request for KPM...")
        
        # Simplified subscription request (in practice, this would be ASN.1 encoded)
        subscription = {
            'requestor_id': 1,
            'instance_id': 1,
            'ran_function_id': 200,  # KPM function
            'action_id': 1,
            'action_type': 'report',
            'report_interval_ms': 100
        }
        
        try:
            # Send subscription (simplified - just JSON)
            self.conn.send(json.dumps(subscription).encode())
            self.log(2, "Subscription request sent")
            return True
        except Exception as e:
            self.log(1, f"Failed to send subscription: {e}")
            return False
            
    def receive_kpm_indication(self):
        """Receive KPM Indication from gNB"""
        try:
            data = self.conn.recv(4096)
            if data:
                # Parse KPM indication (simplified - expects JSON)
                try:
                    indication = json.loads(data.decode())
                    return indication
                except json.JSONDecodeError:
                    # Binary data - extract metrics heuristically
                    self.log(3, f"Received binary data: {len(data)} bytes")
                    return None
        except socket.timeout:
            return None
        except Exception as e:
            self.log(1, f"Error receiving KPM: {e}")
            return None
            
    def analyze_sla_compliance(self, metrics):
        """Analyze metrics against SLA requirements"""
        violations = []
        
        for slice_name, slice_metrics in metrics.items():
            if slice_name not in self.slas:
                continue
                
            sla = self.slas[slice_name]
            
            # Check throughput
            if slice_metrics.get('throughput_mbps', 0) < sla['min_throughput_mbps']:
                violations.append({
                    'slice': slice_name,
                    'type': 'throughput',
                    'target': sla['min_throughput_mbps'],
                    'actual': slice_metrics.get('throughput_mbps', 0),
                    'severity': 'high'
                })
                
            # Check latency
            if slice_metrics.get('avg_latency_ms', 0) > sla['max_latency_ms']:
                violations.append({
                    'slice': slice_name,
                    'type': 'latency',
                    'target': sla['max_latency_ms'],
                    'actual': slice_metrics.get('avg_latency_ms', 0),
                    'severity': 'high'
                })
                
        return violations
        
    def make_control_decision(self, violations):
        """Make control decisions based on SLA violations"""
        decisions = []
        
        for violation in violations:
            slice_name = violation['slice']
            current_weight = self.slice_weights.get(slice_name, 0.5)
            
            if violation['type'] == 'throughput':
                # Increase weight to improve throughput
                shortfall = violation['target'] - violation['actual']
                new_weight = min(0.8, current_weight + shortfall * 0.1)
            elif violation['type'] == 'latency':
                # Increase weight to reduce latency (more resources = less queuing)
                excess = violation['actual'] - violation['target']
                new_weight = min(0.8, current_weight + excess * 0.01)
            else:
                new_weight = current_weight
                
            if new_weight != current_weight:
                decisions.append({
                    'slice': slice_name,
                    'old_weight': current_weight,
                    'new_weight': new_weight,
                    'reason': violation['type'],
                    'timestamp': time.time()
                })
                self.slice_weights[slice_name] = new_weight
                
        return decisions
        
    def send_ric_control(self, decisions):
        """Send RIC Control messages to adjust slice weights"""
        for decision in decisions:
            self.log(2, f"RIC Control: Slice {decision['slice']} "
                       f"weight {decision['old_weight']:.3f} -> {decision['new_weight']:.3f} "
                       f"(reason: {decision['reason']})")
            
            # Encode control message (simplified)
            # Format: (sliceId << 8) | (weight * 100)
            slice_id = 0 if decision['slice'] == 'embb' else 1
            weight_int = int(decision['new_weight'] * 100)
            control_value = (slice_id << 8) | weight_int
            
            control_msg = {
                'request_type': 'QoS',
                'ran_function_id': 300,  # RC function
                'slice_id': slice_id,
                'weight': decision['new_weight'],
                'control_value': control_value
            }
            
            try:
                self.conn.send(json.dumps(control_msg).encode())
                self.log(3, f"Control message sent: {control_msg}")
            except Exception as e:
                self.log(1, f"Failed to send control: {e}")
                
            # Log decision
            self.decision_history.append({
                'timestamp': decision['timestamp'],
                'slice_id': slice_id,
                'slice_name': decision['slice'],
                'old_weight': decision['old_weight'],
                'new_weight': decision['new_weight'],
                'reason': decision['reason']
            })
            
    def export_results(self):
        """Export results to JSON files"""
        # Export decision history
        decisions_file = os.path.join(self.output_dir, 'ric_decisions.json')
        with open(decisions_file, 'w') as f:
            json.dump({
                'timestamp': datetime.now().isoformat(),
                'total_decisions': len(self.decision_history),
                'final_weights': self.slice_weights,
                'decisions': self.decision_history
            }, f, indent=2)
        self.log(2, f"Decisions exported to {decisions_file}")
        
        # Export slice weights history
        weights_file = os.path.join(self.output_dir, 'slice_weights_history.csv')
        with open(weights_file, 'w') as f:
            f.write('timestamp,slice_name,weight,reason\n')
            for d in self.decision_history:
                f.write(f"{d['timestamp']},{d['slice_name']},{d['new_weight']:.4f},{d['reason']}\n")
        self.log(2, f"Weights history exported to {weights_file}")
        
    def run(self):
        """Main execution loop"""
        self.log(0, "=== O-RAN Near-RT RIC/xApp Started ===")
        self.log(0, f"Server: {self.server_ip}:{self.server_port}")
        self.log(0, f"SLAs: {json.dumps(self.slas, indent=2)}")
        
        # Start server
        if not self.start_server():
            return
            
        # Wait for E2 setup
        if not self.wait_for_e2_setup():
            self.log(1, "E2 Setup failed")
            return
            
        # Send subscription request
        if not self.send_subscription_request():
            self.log(1, "Subscription failed")
            return
            
        # Set socket timeout for non-blocking receive
        self.conn.settimeout(0.5)
        
        # Main control loop
        self.log(0, "Entering main control loop...")
        cycle_count = 0
        
        try:
            while True:
                cycle_count += 1
                
                # Receive KPM indication
                indication = self.receive_kpm_indication()
                
                if indication:
                    # Update metrics
                    if 'eMBB' in indication:
                        self.slice_metrics['embb'] = indication['eMBB']
                    if 'IoT' in indication:
                        self.slice_metrics['iot'] = indication['IoT']
                    
                    # Analyze SLA compliance
                    violations = self.analyze_sla_compliance(self.slice_metrics)
                    
                    if violations:
                        self.log(2, f"SLA violations detected: {len(violations)}")
                        for v in violations:
                            self.log(2, f"  {v['slice']}: {v['type']} "
                                       f"target={v['target']:.2f}, actual={v['actual']:.2f}")
                        
                        # Make control decisions
                        decisions = self.make_control_decision(violations)
                        
                        if decisions:
                            self.send_ric_control(decisions)
                
                # Export results periodically
                if cycle_count % 100 == 0:
                    self.export_results()
                    
                time.sleep(0.1)  # 100ms control loop
                
        except KeyboardInterrupt:
            self.log(0, "\nShutting down RIC...")
        finally:
            self.export_results()
            self.conn.close()
            self.sock.close()
            self.log(0, "=== RIC/xApp Shutdown Complete ===")


def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Simple O-RAN RIC/xApp for Slice Control')
    parser.add_argument('--server-ip', default='10.0.2.10', help='RIC IP address')
    parser.add_argument('--server-port', type=int, default=36422, help='RIC port')
    parser.add_argument('--client-port', type=int, default=38472, help='Client port')
    parser.add_argument('--log-level', type=int, default=2, 
                       choices=[0, 1, 2, 3], help='Log level (0=UNCOND, 3=DEBUG)')
    parser.add_argument('--output-dir', default='results', help='Output directory')
    
    args = parser.parse_args()
    
    config = {
        'server_ip': args.server_ip,
        'server_port': args.server_port,
        'client_port': args.client_port,
        'log_level': args.log_level,
        'output_dir': args.output_dir
    }
    
    ric = SimpleRicXapp(config)
    ric.run()


if __name__ == '__main__':
    main()
