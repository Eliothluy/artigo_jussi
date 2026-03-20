/*
 * Simple RIC/xApp for O-RAN Slice Control
 * Uses e2sim library for SCTP communication
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <thread>
#include <chrono>

#include "e2sim.hpp"
#include "e2sim_sctp.hpp"
#include "encode_e2apv1.hpp"
#include "e2ap_message_handler.hpp"

extern "C" {
    #include "E2AP-PDU.h"
    #include "e2sim_defs.h"
}

static bool g_running = true;
static int g_indicationCount = 0;

void signalHandler(int sig) {
    printf("\n[SIGNAL] Caught signal %d, shutting down...\n", sig);
    g_running = false;
}

void handleSubscription(E2AP_PDU_t* pdu) {
    printf("[RIC] Received RIC Subscription Request\n");
    // The e2sim handles the subscription response automatically
}

void handleControl(E2AP_PDU_t* pdu) {
    printf("[RIC] Received RIC Control Response\n");
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    printf("===================================================\n");
    printf("  Simple O-RAN RIC/xApp for Slice Control\n");
    printf("===================================================\n\n");
    
    // Configuration
    std::string serverIp = "127.0.0.1";
    uint16_t serverPort = 36422;
    uint16_t localPort = 38472;
    std::string gnbId = "1";
    std::string plmnId = "111";
    
    if (argc >= 3) {
        serverIp = argv[1];
        serverPort = atoi(argv[2]);
    }
    
    printf("[RIC] Configuration:\n");
    printf("  Server IP: %s\n", serverIp.c_str());
    printf("  Server Port: %u\n", serverPort);
    printf("  gNB ID: %s\n", gnbId.c_str());
    printf("  PLMN ID: %s\n", plmnId.c_str());
    printf("\n");
    
    // Create E2Sim instance
    E2Sim e2sim;
    
    printf("[RIC] Starting SCTP server on %s:%u...\n", serverIp.c_str(), serverPort);
    
    // Start the server
    int server_fd = sctp_start_server(serverIp.c_str(), serverPort);
    if (server_fd < 0) {
        printf("[ERROR] Failed to start SCTP server\n");
        return 1;
    }
    
    printf("[RIC] Waiting for E2 agent connection...\n");
    
    // Accept connection from ns-3 gNB
    int client_fd = sctp_accept_connection(serverIp.c_str(), server_fd);
    if (client_fd < 0) {
        printf("[ERROR] Failed to accept connection\n");
        return 1;
    }
    
    printf("[RIC] Connected to E2 agent!\n\n");
    
    // Receive E2 Setup Request
    sctp_buffer_t recv_buf;
    printf("[RIC] Waiting for E2 Setup Request...\n");
    
    if (sctp_receive_data(client_fd, recv_buf) > 0) {
        printf("[RIC] Received E2 message (%d bytes)\n", recv_buf.len);
        
        // Decode the message
        auto* pdu = (E2AP_PDU_t*)calloc(1, sizeof(E2AP_PDU));
        e2ap_asn1c_decode_pdu(pdu, ATS_ALIGNED_BASIC_PER, recv_buf.buffer, recv_buf.len);
        
        long procedureCode = e2ap_asn1c_get_procedureCode(pdu);
        printf("[RIC] Procedure Code: %ld\n", procedureCode);
        
        if (procedureCode == 1) {  // E2 Setup Request
            printf("[RIC] E2 Setup Request received - sending response...\n");
            
            // Create E2 Setup Response
            E2AP_PDU* setup_resp = new E2AP_PDU;
            encoding::generate_e2apv1_setup_response(setup_resp);
            
            // Encode and send
            uint8_t* buf;
            sctp_buffer_t data;
            data.len = e2ap_asn1c_encode_pdu(setup_resp, &buf);
            memcpy(data.buffer, buf, (data.len < MAX_SCTP_BUFFER ? data.len : MAX_SCTP_BUFFER));
            sctp_send_data(client_fd, data);
            
            printf("[RIC] E2 Setup Response sent\n");
            delete setup_resp;
        }
        
        free(pdu);
    }
    
    printf("\n[RIC] Entering main loop - waiting for KPM indications...\n");
    printf("[RIC] Press Ctrl+C to stop\n\n");
    
    // Main receive loop
    while (g_running) {
        if (sctp_receive_data(client_fd, recv_buf) > 0) {
            g_indicationCount++;
            printf("[RIC] Received message #%d (%d bytes)\n", g_indicationCount, recv_buf.len);
            
            // Try to decode
            auto* pdu = (E2AP_PDU_t*)calloc(1, sizeof(E2AP_PDU));
            e2ap_asn1c_decode_pdu(pdu, ATS_ALIGNED_BASIC_PER, recv_buf.buffer, recv_buf.len);
            
            long procedureCode = e2ap_asn1c_get_procedureCode(pdu);
            printf("[RIC] Procedure Code: %ld\n", procedureCode);
            
            if (procedureCode == 201) {  // RIC Indication
                printf("[RIC] KPM Indication received - metrics updated\n");
            }
            
            free(pdu);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    printf("\n[RIC] Shutting down...\n");
    close(client_fd);
    close(server_fd);
    printf("[RIC] Shutdown complete. Received %d messages.\n", g_indicationCount);
    
    return 0;
}
