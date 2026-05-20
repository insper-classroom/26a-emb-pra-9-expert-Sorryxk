# Lab 9 — Expert Firmware: RTOS Metrics + SMP

Reutilização do lab de I²C (MPU6050 → Fusion AHRS → UART/PWM) para medir
métricas de tempo real em **single core** e em **SMP (2 cores)** no RP2350.

## Hardware

- Raspberry Pi Pico 2 (RP2350)
- MPU6050: SDA=GPIO4, SCL=GPIO5, **VCC alimentado por GPIO14**, GND no GND
- LED único via PWM em GPIO15
- Saída do "mouse" via USB-CDC (cliente Python no PC)
- Saleae Logic 8 nos GPIOs 16/17/18/19 + GND comum

### Fiação

| Sinal             | Pico (GPIO) | Vai para           |
|-------------------|-------------|--------------------|
| MPU SDA           | GP4         | MPU SDA            |
| MPU SCL           | GP5         | MPU SCL            |
| MPU VCC           | GP14        | MPU VCC (3.3 V)    |
| MPU GND           | GND         | MPU GND            |
| LED               | GP15        | anodo do LED       |
| Probe mpu_task    | GP16        | Saleae CH0         |
| Probe fusion_task | GP17        | Saleae CH1         |
| Probe uart_task   | GP18        | Saleae CH2         |
| Probe pwm_task    | GP19        | Saleae CH3         |
| GND               | GND         | Saleae GND         |

> Mapeamento canal-task (validado pela frequência observada):
> - ch1 = mpu_task (100 Hz, Tstd grande por causa do I²C)
> - ch2 = uart_task (20 Hz no caso normal — divisor /5 no fusion)
> - ch3 = pwm_task (100 Hz)
> - ch4 = fusion_task (100 Hz)

### Como medir

Em `main/main.c`:
```c
#define INSTRUMENT_ENABLED      1   // toggles dos GPIOs
#define STACK_MONITOR_ENABLED   0   // 1 só para coletar stack (printf afeta o tempo)
```

Em `main/CMakeLists.txt`:
```cmake
configNUMBER_OF_CORES=1   # Single Core
configNUMBER_OF_CORES=2   # SMP
```

---

## Resultados

### Tabela — Single Core

| Métrica / Task        | mpu_task   | fusion_task | uart_task     | pwm_task |
|-----------------------|------------|-------------|---------------|----------|
| **WCET**              | ≥ 370 µs ¹ | 66 µs       | 177 µs        | 27 µs    |
| **Jitter**            | 86 µs      | 3.3 µs      | 330 µs        | 25 µs    |
| **Deadline Miss**     | < 1 %      | 0 %         | N/A (event)   | 0 %      |
| **Stack alocada (words)** | 8192   | 1024        | 512           | 512      |
| **Stack usada (words)**   | 110    | 216         | 72            | 44       |
| **% uso**             | 1.3 %      | 21 %        | 14 %          | 8.6 %    |

¹ O Logic 2 mostrou ~200 ns no click-and-drag, mas é artefato de captura — a
transação I²C de 14 bytes a 400 kHz consome **~370 µs no mínimo**, sendo esse o
WCET físico mínimo.

### Tabela — SMP (2 cores)

| Métrica / Task        | mpu_task   | fusion_task | uart_task               | pwm_task |
|-----------------------|------------|-------------|-------------------------|----------|
| **WCET**              | ≥ 370 µs ¹ | 73 µs       | ~97 µs / 50 ms (click)² | 43 µs    |
| **Jitter**            | 35 µs      | 7 µs        | caótico (Tstd 30 ms)²   | 24 µs    |
| **Deadline Miss**     | < 1 %      | 0 %         | N/A (event)             | 0 %      |
| **Stack alocada (words)** | 192    | 320         | 128                     | 96       |
| **Stack usada (words)**   | 94     | 193         | 84                      | 40       |
| **% uso**             | 49 %       | 60 %        | 66 %                    | 42 %     |

² Captura SMP incluiu eventos de clique (a uart_task fica HIGH por 50 ms
durante o gesto, por `vTaskDelay(50)` intencional). Para o caminho normal
(`putchar_raw + stdio_flush`), o WCET é **~97 µs**.

### Distribuição entre cores (SMP)

| Task          | Core | Prioridade | Justificativa |
|---------------|------|------------|---------------|
| `mpu_task`    | 1    | 2          | pipeline I²C → fusion |
| `fusion_task` | 1    | 2          | AHRS consome direto da fila do mpu |
| `uart_task`   | 0    | 1          | USB-CDC bloqueante isolado do core do sensor |
| `pwm_task`    | 0    | 1          | leve, event-driven |

### Comparação Single Core vs SMP — Jitter

| Task    | SC         | SMP        | Δ                  |
|---------|------------|------------|--------------------|
| mpu     | 86 µs      | **35 µs**  | ⬇️ **−59 %**       |
| fusion  | 3.3 µs     | 7 µs       | ⬆️ +112 % (pequeno) |
| pwm     | 25 µs      | 24 µs      | ≈ igual             |
| uart    | 330 µs     | caótico¹   | comparação inválida |

¹ No single core o gesto de clique não disparou; no SMP foi capturado.

---

## Perguntas extras

### 1. Qual é a frequência máxima que a task fusion pode executar?

Com WCET de fusion ≈ **73 µs** no SMP, a frequência máxima é:

`f_max = 1 / 73 µs ≈ 13.7 kHz`

Ou seja, há margem de pelo menos **100×** sobre os 100 Hz que estamos usando.
Na prática o gargalo é o `mpu_task` (limitado pelo I²C a 400 kHz → ≥ 370 µs
por leitura), que limita o pipeline a ~2.7 kHz no melhor caso.

### 2. 100 Hz para o fusion estava correto? (`SAMPLE_PERIOD = 0.1f` no enunciado)

**Errado por fator 10.** `SAMPLE_PERIOD` deve estar em **segundos** e bater
com o período real entre chamadas a `FusionAhrsUpdateNoMagnetometer`. Como o
`mpu_task` usa `vTaskDelay(10 ms)`, o valor correto é:

```c
#define SAMPLE_PERIOD (0.01f)   // 10 ms = 100 Hz
```

O `0.1f` do enunciado original passaria 100 ms ao filtro AHRS, fazendo o
algoritmo subestimar a integração do giro por um fator 10 — a orientação
ficaria correta no longo prazo, mas com resposta extremamente lenta.

### 3. Como otimizar a task da uart para que ela não ocupe tanto processamento?

A `uart_task` é a mais cara (WCET ~177 µs SC, jitter 330 µs). Causas e
otimizações:

- **`stdio_flush()` chamado a cada 4 bytes** — força a stack USB-CDC a
  finalizar transferência. **Otimização**: juntar os 4 bytes em um buffer e
  chamar `fwrite`/`puts` uma única vez por pacote.
- **USB-CDC tem latência inerente** (~1 ms frame USB). **Otimização**: usar
  UART hardware (uart0) com DMA + IRQ TX em vez de USB-CDC.
- **Divisor /5 no `fusion_task`** já reduz a taxa de envio para 20 Hz —
  pode-se subir para /10 ou /20 se a aplicação tolerar.
- **Gesto de clique segura HIGH por 50 ms** com `vTaskDelay(50)` — substituir
  por um timer FreeRTOS de software não-bloqueante.

### 4. Por que o valor do jitter é baixo/alto nessa aplicação?

- **fusion e pwm têm jitter muito baixo** (3–25 µs) porque rodam dentro do
  ciclo do sistema, sem I/O externo, e o RP2350 tem tick rate de 1 kHz com
  scheduler preemptivo determinístico.
- **mpu tem jitter médio** (86 µs SC, 35 µs SMP) porque depende do barramento
  I²C, cuja conclusão de transação varia conforme ACK/clock stretching.
- **uart tem jitter alto** (330 µs ou mais) porque depende do consumo do host
  USB-CDC, que tem janelas de polling assíncronas.

### 5. O jitter aumentou ou diminuiu com SMP? O que contribuiu?

**Diminuiu para o mpu** (86 → 35 µs, −59 %) — fator dominante: **afinidade**.
Colocar `mpu_task` no core 1 isolou-a da `uart_task` (que bloqueia o core 0
com USB flushes), eliminando a maior fonte de interferência.

**Para fusion teve leve aumento** (3 → 7 µs) — provavelmente disputa de
memória/cache entre mpu e fusion no mesmo core, e overhead do scheduler SMP.

**Para pwm ficou estável** (25 → 24 µs) — pwm sempre foi event-driven leve.

Contribuição relativa (subjetivo):
1. **Afinidade** — maior impacto (isolou cadeia sensorial do I/O USB).
2. **Prioridades** — pouco impacto aqui (não mexi).
3. **Locks** — não há contenção significativa (queues do FreeRTOS são leves).
4. **ISR** — USB-CDC IRQ continua no core 0; afetava ambos antes.

### 6. Houve deadline miss? Em qual condição e como mitigou?

- `mpu_task`: **< 1 %** em ambos os modos. T_max chegou a 10.04 ms (deadline
  10 ms) — alguns ciclos atrasados, magnitude desprezível. Causa: variação
  do bus I²C. Mitigação aplicada: SMP isolou o core do mpu, reduzindo
  T_max de 10.04 ms para 10.02 ms.
- `fusion_task`, `pwm_task`: **0 %** em ambos.
- `uart_task`: **N/A** (event-driven, sem deadline periódico). O pulso de
  50 ms durante clique é por design (não é miss).

### 7. Como você mediu o jitter?

- **Sinal**: GPIO instrumentado em cada task (16/17/18/19), HIGH durante a
  execução, capturado no Saleae Logic 8.
- **Definição**: período entre **bordas de subida consecutivas** do GPIO.
  Estatística `Period` no painel Measurements → `Jitter = T_max − T_min`
  com `T_max = 1/fmin` e `T_min = 1/fmax`.

### 8. Qual foi o efeito de mudar prioridades vs mudar afinidade?

Neste projeto, **a afinidade teve impacto MUITO maior**. As prioridades
foram mantidas iguais (mpu=fusion=2, uart=pwm=1) entre as duas medições, e
a melhoria de jitter no mpu (−59 %) veio exclusivamente da realocação de
tasks entre os cores.

Em sistemas com muita contenção por um único core (como este antes do SMP),
**afinidade > prioridade**, porque resolve fisicamente a competição. Em
sistemas single core, prioridade é a única ferramenta — mas aqui ela nem
mostraria efeito porque as prioridades atuais já evitam inversão.

### 9. O tamanho de stack ficou dentro da regra dos 80%?

**Após ajuste** (com base nas medidas SMP):

| Task     | Alocado | High Water Mark | Usado | % uso |
|----------|---------|------------------|-------|-------|
| mpu      | 192     | 98 free          | 94    | 49 %  |
| fusion   | 320     | 127 free         | 193   | 60 %  |
| uart     | 128     | 44 free          | 84    | 66 %  |
| pwm      | 96      | 56 free          | 40    | 42 %  |

Racional do ajuste (todos abaixo de 80%, com margem):
- `mpu`: usado 94 → alocado 192 (49%). Não fui mais agressivo porque o
  Cortex-M33 com FPU exige ~32 words só de save area de ISR; queremos
  margem para preempção.
- `fusion`: usado 193 → alocado 320 (60%). FPU + AHRS math + queue ops.
- `uart`: usado 84 → alocado 128 (66%). USB-CDC TX usa buffer pequeno do
  PicoSDK; margem para `stdio_flush`.
- `pwm`: usado 40 → alocado 96 (42%). Não pode ir abaixo de ~64 words por
  segurança de contexto FPU.

> Antes do ajuste, todas estavam massivamente superdimensionadas
> (mpu 1.3%, fusion 21%, uart 14%, pwm 8.6%) — desperdiçando ~9 KB de heap.

