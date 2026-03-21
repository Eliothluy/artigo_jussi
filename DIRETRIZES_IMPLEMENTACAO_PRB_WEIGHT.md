# Documentacao de Implementacao: O-RAN Slice-Aware PRB Scheduling com xApp

## 1. Visao Geral

Este documento descreve a implementacao do cenario de simulacao `oran_slicing_prb_weight.cc`, que implementa fatiamento (slicing) de rede em nivel de PRB (Physical Resource Block) no gNB simulado do ns-3, integrado com a interface E2 do O-RAN para controle via xApp/RIC.

### Conceito Central: "Enganando" o xApp

O xApp envia **pesos abstratos** (ex: `0.7`) pensando que controla uma particao proporcional. Mas o scheduler `NrMacSchedulerOfdmaSlicePrb` converte esse peso internamente em uma **quota rigida de PRBs**:

```
prbQuota = totalPrbs * weight
```

Exemplo: com 273 PRBs totais e `weight = 0.7`, o eMBB recebe exatamente 191 PRBs como quota fixa.

### Diferenca em relacao ao cenario RBG (`oran_slicing_rbg.cc`)

| Aspecto | RBG (`oran_slicing_rbg.cc`) | PRB Weight (`oran_slicing_prb_weight.cc`) |
|---|---|---|
| Scheduler | `NrMacSchedulerOfdmaSliceQos` | `NrMacSchedulerOfdmaSlicePrb` |
| Granularidade | RBG (grupo de PRBs) | PRB individual |
| Interface xApp | `SetSliceStaticWeight(weight)` | `SetSliceStaticWeight(weight)` (mesma!) |
| Efeito real | Particao proporcional de RBGs | Conversao direta para quota PRB |
| KPM metricas | `allocatedRbgs` | `allocatedPrbs` (3GPP TS 28.552) |
| Callback RIC | Nao aplica | **Aplica via `SetSliceStaticWeight`** |

---

## 2. Arquitetura do Sistema

```
┌─────────────────┐         E2 (SCTP)         ┌──────────────────────┐
│                 │ ◄──────────────────────── │                      │
│   Near-RT RIC   │                            │   ns-3 gNB Simulado  │
│   (xApp)        │ ──────────────────────► │                      │
│                 │    RIC Control Msg         │  ┌────────────────┐ │
│  - KPM Subscribe│                            │  │ E2Termination  │ │
│  - SLA Monitor  │                            │  │  (oran-iface)  │ │
│  - Weight Ctrl  │                            │  └───────┬────────┘ │
└─────────────────┘                            │          │          │
                                               │  ┌───────▼────────┐ │
                                               │  │ RicControlMsg  │ │
                                               │  │   Callback     │ │
                                               │  └───────┬────────┘ │
                                               │          │          │
                                               │  ┌───────▼────────┐ │
                                               │  │ SetSliceStatic │ │
                                               │  │ Weight(weight) │ │
                                               │  └───────┬────────┘ │
                                               │          │          │
                                               │  ┌───────▼────────┐ │
                                               │  │ NrMacScheduler │ │
                                               │  │  OfdmaSlicePrb │ │
                                               │  │                │ │
                                               │  │ weight → PRB   │ │
                                               │  │ quota (hard)   │ │
                                               │  └────────────────┘ │
                                               └──────────────────────┘
```

### Fluxo de Controle E2 (Closed-Loop)

```
1. xApp envia RIC Subscription Request (KPM, Function ID 200)
        │
2. E2Termination::KpmSubscriptionCallback() recebe
        │
3. gNB envia KPM Indications a cada 100ms:
   - DRB.IPThpDl.UEID (throughput)
   - DRB.IPLateDl.UEID (latencia)
   - DRB.PrbAllocated.UEID (PRBs alocados)
        │
4. xApp analisa KPIs contra SLAs
        │
5. xApp envia RIC Control Message (RC, Function ID 300):
   - Request Type: QoS (1002)
   - Valor: (sliceId << 8) | (weight * 100)
        │
6. E2Termination::RicControlMessageCallback() recebe
        │
7. Extrai sliceId e weight do valor inteiro
        │
8. Chama scheduler->SetSliceStaticWeight(sliceId, weight)
        │
9. Scheduler converte: prbQuota = totalPrbs * weight
   e aloca PRBs rigidamente para o slice
```

---

## 3. Componentes do Codigo

### 3.1 Arquivo Principal: `oran_slicing_prb_weight.cc`

**Localizacao:** `ns-3-dev/scratch/oran_slicing_prb_weight.cc`

**Componentes:**

| Componente | Linhas | Descricao |
|---|---|---|
| Includes e configuracao | 1-68 | Headers ns-3, e2sim, ASN.1 |
| Variaveis globais | 70-109 | Mapas IP->UE, UE->Slice, E2Termination |
| `GetUeAndSliceFromFlow()` | 133-163 | Mapeamento de fluxo para slice |
| `BuildAndSendKpmIndication()` | 165-216 | Constroi indicacao KPM com metricas PRB |
| `SendKpmIndications()` | 218-230 | Loop periodico de envio KPM (100ms) |
| `KpmSubscriptionCallback()` | 232-244 | Processa subscription do xApp |
| `RicControlMessageCallback()` | 246-285 | **Callback principal**: converte peso em PRB |
| `main()` | 287-913 | Configuracao completa da simulacao |

### 3.2 xApp Python: `ric_xapp_simple.py`

**Localizacao:** `ns-3-dev/scratch/ric_xapp_simple.py`

Implementacao de um xApp simplificado com:
- Monitoramento de SLA (throughput e latencia)
- Controle baseado em regras (rule-based)
- Historico de decisoes exportado em JSON/CSV

### 3.3 RIC C++: `simple_ric.cpp`

**Localizacao:** `ns-3-dev/scratch/simple_ric.cpp`

Implementacao em C++ usando a biblioteca e2sim:
- Servidor SCTP para conexao E2
- Recebe KPM indications
- Pode enviar RIC control messages

---

## 4. Interface E2 (Service Models)

### 4.1 E2SM-KPM (Function ID 200) — Telemetria

**Metricas reportadas (a cada 100ms):**

```cpp
// Indicacao KPM Header
headerValues.m_plmId = "111";
headerValues.m_gnbId = 1;
headerValues.m_nrCellId = 1;
headerValues.m_timestamp = Simulator::Now().GetMilliSeconds();

// Container CU-UP (PRBs alocados)
cuUpValues->m_pDCPBytesUL = g_sliceMetrics[0].allocatedPrbs * 100;
cuUpValues->m_pDCPBytesDL = g_sliceMetrics[0].allocatedPrbs * 100;

// Metricas por slice (eMBB)
embbUeValues->AddItem<double>("DRB.IPThpDl.UEID", throughput_mbps);
embbUeValues->AddItem<double>("DRB.IPLateDl.UEID", avg_delay_ms);
embbUeValues->AddItem<long>("DRB.PrbAllocated.UEID", allocated_prbs);

// Metricas por slice (IoT)
iotUeValues->AddItem<double>("DRB.IPThpDl.UEID", throughput_mbps);
iotUeValues->AddItem<double>("DRB.IPLateDl.UEID", avg_delay_ms);
iotUeValues->AddItem<long>("DRB.PrbAllocated.UEID", allocated_prbs);
```

### 4.2 E2SM-RC (Function ID 300) — Controle

**Formato da mensagem de controle do xApp:**

```
Valor inteiro = (sliceId << 8) | (weight * 100)

Exemplo:
  sliceId = 0 (eMBB), weight = 0.7
  valor = (0 << 8) | 70 = 70

  sliceId = 1 (IoT), weight = 0.3
  valor = (1 << 8) | 30 = 286
```

**Callback no gNB (linha 246-285):**

```cpp
static void RicControlMessageCallback(E2AP_PDU_t* ric_ctrl_pdu)
{
    RicControlMessage msg = RicControlMessage(ric_ctrl_pdu);

    if (msg.m_requestType == RicControlMessage::ControlMessageRequestIdType::QoS)
    {
        for (const auto& param : msg.m_valuesExtracted)
        {
            if (param.m_valueType == RANParameterItem::ValueType::Int)
            {
                int value = param.m_valueInt;

                // Decodificar: sliceId no byte alto, weight no byte baixo
                uint8_t sliceId = (value >> 8) & 0xFF;
                double weight = (value & 0xFF) / 100.0;

                // O TRUCO: xApp manda "peso", scheduler converte em PRB
                Ptr<NrMacSchedulerOfdmaSlicePrb> scheduler =
                    DynamicCast<NrMacSchedulerOfdmaSlicePrb>(
                        NrHelper::GetScheduler(g_gnbNetDev, 0));

                scheduler->SetSliceStaticWeight(sliceId, weight);
                // Internamente: prbQuota = totalPrbs * weight
            }
        }
    }
}
```

---

## 5. Mapeamento xApp <-> Scheduler

### 5.1 Codigo do xApp (envio)

```python
# ric_xapp_simple.py — linha 198-210
slice_id = 0 if decision['slice'] == 'embb' else 1
weight_int = int(decision['new_weight'] * 100)
control_value = (slice_id << 8) | weight_int

control_msg = {
    'request_type': 'QoS',
    'ran_function_id': 300,
    'slice_id': slice_id,
    'weight': decision['new_weight'],
    'control_value': control_value
}
```

### 5.2 Scheduler (recebimento e conversao)

```cpp
// nr-mac-scheduler-ofdma-slice-prb.cc — linha 116-132
void NrMacSchedulerOfdmaSlicePrb::SetSliceStaticWeight(uint8_t slice, double weight)
{
    uint32_t totalPrbs = GetBandwidthInRbg() * GetNumRbPerRbg();

    if (slice == SLICE_EMBB)
    {
        m_embbPrbQuota = static_cast<uint32_t>(totalPrbs * weight);  // PESO -> PRB RIGIDO
        m_mmtcPrbQuota = totalPrbs - m_embbPrbQuota;
    }
    else if (slice == SLICE_MMTC)
    {
        m_mmtcPrbQuota = static_cast<uint32_t>(totalPrbs * weight);  // PESO -> PRB RIGIDO
        m_embbPrbQuota = totalPrbs - m_mmtcPrbQuota;
    }
}
```

### 5.3 Exemplo de Conversao (200 MHz, numerology 2)

| Parametro | Valor |
|---|---|
| Banda | 200 MHz |
| SCS | 60 kHz (numerology 2) |
| Total PRBs | 273 |
| PRBs por RBG | 1 |
| xApp envia weight | 0.5 |
| **PRB quota resultante** | `273 * 0.5 = 136` |
| xApp envia weight | 0.7 |
| **PRB quota resultante** | `273 * 0.7 = 191` |

---

## 6. Politica de Fatiamento (Slicing Policy)

### 6.1 Configuracao Inicial (command line)

```bash
./ns3 run "oran_slicing_prb_weight --embbPrbQuota=136 --mmtcPrbQuota=137"
```

### 6.2 Identificacao de Slices por QCI

| Slice | QCI | EPS Bearer | Carga de Trabalho |
|---|---|---|---|
| eMBB (Slice 0) | 6 | `NGBR_VIDEO_TCP_OPERATOR` | Video 3GPP TR 38.838 |
| IoT (Slice 1) | 80 | `NGBR_LOW_LAT_EMBB` | UDP periodic (100 bytes/s) |

### 6.3 Work-Conserving

PRBs nao utilizados por um slice sao redistribuidos para o outro slice, garantindo que nenhum recurso fique ocioso.

---

## 7. Verificacao de SLA

O xApp monitora KPIs recebidos via KPM e compara com metas:

| SLA | Meta | Acao do xApp se violado |
|---|---|---|
| eMBB Throughput | >= 1.0 Mbps/UE | Aumenta `weight` do eMBB |
| eMBB Latencia | <= 100 ms | Aumenta `weight` do eMBB |
| IoT Throughput | >= 0.01 Mbps | Aumenta `weight` do IoT |

O xApp calcula:
```python
# Se throughput abaixo da meta:
shortfall = target - actual
new_weight = min(0.8, current_weight + shortfall * 0.1)
```

---

## 8. Saidas e Metricas

### 8.1 Arquivos Gerados

| Arquivo | Conteudo |
|---|---|
| `results_prb_weight/ue_metrics.csv` | Metricas por UE (throughput, latencia, perda) |
| `results_prb_weight/slice_metrics.csv` | Metricas agregadas por slice |
| `results_prb_weight/summary.json` | Resumo completo com politica PRB e verificacao SLA |

### 8.2 Estrutura do summary.json (campos PRB)

```json
{
  "prbPolicy": {
    "embbPrbQuota": 136,
    "mmtcPrbQuota": 137,
    "totalPrbQuota": 273,
    "embbPrbFraction": 0.498,
    "xAppControlMechanism": "abstract_weights_converted_to_prb_quotas",
    "weightToPrbFormula": "prbQuota = totalPrbs * weight"
  },
  "technicalNotes": {
    "scheduler": "NrMacSchedulerOfdmaSlicePrb",
    "granularity": "PRB (Physical Resource Block)",
    "xAppInterface": "SetSliceStaticWeight(slice, weight) - same as RBG scenario",
    "schedulerConversion": "SetSliceStaticWeight -> totalPrbs * weight = prbQuota"
  }
}
```

---

## 9. Execucao

### 9.1 Sem xApp (standalone)

```bash
cd /home/elioth/Documentos/artigo_jussi/ns-3-dev/

# Configuracao padrao (20 UEs, 10s)
./ns3 run "oran_slicing_prb_weight --enableE2=false"

# Configuracao customizada
./ns3 run "oran_slicing_prb_weight \
  --enableE2=false \
  --simTime=10 \
  --nEmbbUes=15 \
  --nIotUes=5 \
  --embbPrbQuota=200 \
  --mmtcPrbQuota=73 \
  --embbSlaThroughputMbps=1.0 \
  --embbSlaLatencyMs=50"
```

### 9.2 Com xApp (E2 habilitado)

```bash
# Terminal 1: Iniciar a simulacao ns-3 com E2
./ns3 run "oran_slicing_prb_weight \
  --enableE2=true \
  --ricAddress=10.0.2.10 \
  --ricPort=36422 \
  --simTime=30"

# Terminal 2: Iniciar o xApp Python
python3 scratch/ric_xapp_simple.py \
  --server-ip 10.0.2.10 \
  --server-port 36422 \
  --output-dir results_xapp
```

### 9.3 Com RIC C++ (e2sim)

```bash
# Terminal 1: Compilar e rodar o RIC
cd /home/elioth/Documentos/artigo_jussi/ns-3-dev/
g++ -o simple_ric scratch/simple_ric.cpp \
  -I/usr/local/include/e2sim \
  -le2sim -lsctp
./simple_ric 127.0.0.1 36422

# Terminal 2: Simulacao ns-3
./ns3 run "oran_slicing_prb_weight --enableE2=true --ricAddress=127.0.0.1"
```

---

## 10. Limitacoes

| Limitacao | Descricao |
|---|---|
| Granularidade minima | 1 RBG (nao e possivel alocar PRB fracionario) |
| Maximo de slices | 2 (eMBB + mMTC) |
| Protocolo xApp Python | TCP simplificado (nao SCTP real) — necessario e2sim para integracao real |
| Tamanho maximo pacote UDP | 65535 bytes (data rate de video limitado a ~15 Mbps @ 30fps) |
| Work-conserving | Recursos ociosos podem violar garantias estritas de SLA |
| Metricas | FlowMonitor (camada IP), nao camada MAC/PHY |

---

## 11. Padronizacao e Conformidade

| Padro | Aplicacao |
|---|---|
| 3GPP TS 28.552 | Metricas de uso de PRB por slice |
| 3GPP TS 38.314 | Metricas de PRB na camada L2 |
| O-RAN E2SM-RC §8.4.3.6 | Quota de PRB por nivel de slice |
| 3GPP TR 38.838 | Modelo de trafego de video generico |
