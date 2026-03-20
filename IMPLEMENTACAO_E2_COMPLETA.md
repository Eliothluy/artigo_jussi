# Implementação Completa E2/O-RAN - Análise e Implementação

## 1. Análise do NORI (`nori2/`)

### Arquitetura do NORI
O NORI implementa E2 com 4 camadas:

```
E2Termination (model/oran-interface.h/cc)
  └── E2Interface (model/E2-interface.h/cc) 
        └── E2TermHelper (helper/E2-term-helper.h/cc)
              └── NoriE2Report (model/E2-report.h/cc)
```

### Fluxo de Comunicação E2
1. **E2 Setup**: gNB → RIC (SCTP + E2AP)
2. **RIC Subscription**: RIC → gNB (subscrição KPM)
3. **KPM Indication**: gNB → RIC (métricas periódicas, 10ms)
4. **RIC Control**: RIC → gNB (comandos de slicing)

### Service Models
- **KPM (Function ID 200)**: Telemetria (CU-UP, CU-CP, DU)
- **RC (Function ID 300)**: Controle (RAN Slicing)

### Diferenças do NORI vs `oran-interface` do ns-3-dev

| Feature | `oran-interface` | `nori2` |
|---------|------------------|---------|
| E2Termination | ✓ | ✓ (identical) |
| KPM SM | ✓ | ✓ (identical) |
| RC SM | ✓ | ✓ (enhanced for slicing) |
| E2Interface | ✗ | ✓ bridges NR sim to E2 |
| E2TermHelper | Partial | ✓ orchestrator |
| PHY Traces | ✗ | ✓ NoriE2Report |
| RL Scheduler | ✗ | ✓ NrRLMacSchedulerOfdma |

---

## 2. Implementação no Projeto

### Módulo NORI Criado
```
ns-3-dev/contrib/nori/
├── CMakeLists.txt
├── model/
│   ├── E2-interface-slicing.h
│   └── E2-interface-slicing.cc
└── helper/
    ├── E2-term-helper-slicing.h
    └── E2-term-helper-slicing.cc
```

### Componentes

#### E2InterfaceSlicing
- Conecta traces PHY/RLC/PDCP ao protocolo E2
- Envia KPM Indications periódicas (CU-UP, DU)
- Processa RIC Control Messages

#### E2TermHelperSlicing
- Orquestra E2Termination + E2Interface
- Configura PLMN ID, Cell ID, portas SCTP
- Registra callbacks KPM e RC
- Inicia thread E2Termination

### Código de Exemplo
```cpp
// Criar helper E2
auto e2Helper = CreateObject<E2TermHelperSlicing>();
e2Helper->SetAttribute("E2TermIp", StringValue("10.244.0.246"));
e2Helper->InstallE2Term(gnbDev.Get(0));
```

---

## 3. Como Executar

### Pré-requisitos
1. e2sim instalado (`/usr/local/lib/libe2sim.a`)
2. Near-RT RIC disponível (ex: OSC RIC)

### Executar com RIC

**Terminal 1 (RIC - usando ricsim do e2sim):**
```bash
cd /home/elioth/Documentos/artigo_jussi/oran-e2sim/e2sim/previous/e2apv1sim/ricsim
./build_e2sim
./ricsim -s 10.0.2.10 -p 36422
```

**Terminal 2 (ns-3 gNB):**
```bash
cd /home/elioth/Documentos/artigo_jussi/ns-3-dev
./ns3 run "oran-slicing-rbg-e2 --ricAddress=10.0.2.10"
```

### Executar sem RIC (Baseline)
```bash
./ns3 run "scratch/oran_slicing_rbg --enableE2=false --simTime=10"
```

---

## 4. Fluxo de Execução

### Sem RIC (Baseline)
```
┌─────────────┐
│ ns-3 gNB    │
│ (slicing)   │──→ FlowMonitor ──→ results/*.csv/json
│ static      │
└─────────────┘
```

### Com RIC (E2 Enabled)
```
┌─────────────┐      SCTP+E2AP     ┌─────────────┐
│ ns-3 gNB    │────────────────────│ Near-RT RIC │
│             │                    │             │
│ E2Setup ────┼───────────────────→│             │
│             │←───────────────────┼ E2SetupResp │
│             │                    │             │
│             │←───────────────────┼ RIC Subscr  │
│ KPM Ind ────┼───────────────────→│             │
│ (100ms)     │                    │             │
│             │←───────────────────┼ RIC Control │
│ Apply Weight│                    │             │
└─────────────┘                    └─────────────┘
```

---

## 5. KPIs Coletados

### KPM Indications
- **CU-UP**: PDCP bytes, throughput por UE
- **CU-CP**: UEs ativos, SINR L3
- **DU**: PRBs alocados, MCS, SINR bins

### RIC Control
- **RAN Slicing**: Ajuste de pesos dos slices

### FlowMonitor
- Throughput por slice
- Latência por slice
- PDR por slice
- Perda de pacotes

---

## 6. Limitações

1. **SCTP Required**: E2Termination usa SCTP, requer RIC real (não Python TCP)
2. **ricsim básico**: O ricsim do e2sim não implementa KPM decode completo
3. **Para produção**: Usar OSC RIC ou Near-RT RIC completo
4. **NR module version**: Algumas APIs não disponíveis (NrUeManager)

---

## 7. Próximos Passos

1. **Deploy OSC RIC**: Para RIC completo com xApp
2. **Implementar xApp**: Baseado no ricsim pattern
3. **Integrar DRL**: Usar KPIs como state, RIC Control como action
4. **Multi-gNB**: Estender para múltiplas células

---

## 8. Arquivos de Saída

```
results/
├── summary.json         # Resumo completo
├── ue_metrics.csv       # Métricas por UE
├── slice_metrics.csv    # Métricas por slice
└── timeseries_metrics.csv # Time series

DlE2PdcpStats.txt       # PDCP stats (NORI)
DlE2RlcStats.txt        # RLC stats (NORI)
```
