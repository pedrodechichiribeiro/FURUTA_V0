# Pêndulo Invertido de Furuta

Projeto experimental de implementação de um **Pêndulo Invertido Rotacional de Furuta**, utilizando Arduino Nano, motor de passo NEMA 17, driver A4988 e sensor magnético AS5600.

O desenvolvimento está sendo realizado de forma incremental no **VS Code + PlatformIO**, dividindo o projeto em fases independentes. Cada fase possui seu próprio ambiente (`env`) no PlatformIO, permitindo testar individualmente sensores, atuadores e algoritmos antes da implementação do controle final.

---

## 1. Objetivo

O objetivo final é realizar:

1. aquisição da posição angular do pêndulo;
2. cálculo da velocidade angular;
3. controle do braço rotacional;
4. caracterização experimental do motor;
5. swing-up do pêndulo;
6. captura próxima à posição vertical;
7. estabilização do pêndulo na posição vertical instável;
8. controle da posição do braço em torno do centro.

A estratégia de controle será construída progressivamente, evitando implementar diretamente o controlador final antes da caracterização do sistema físico.

---

## 2. Hardware

### Controlador

- Arduino Nano
- ATmega328P
- comunicação serial: 115200 baud
- desenvolvimento com PlatformIO

### Motor

- NEMA 17 17HS8401
- 200 passos por revolução
- driver A4988
- alimentação: 12 V
- fonte: 2 A máx.
- microstepping: 1/8

### Sensor do pêndulo

- AS5600
- encoder magnético absoluto
- resolução: 12 bits / 4096 posições por revolução
- comunicação I²C
- biblioteca utilizada: `robtillaart/AS5600`

### Geometria aproximada

| Parâmetro | Valor |
|---|---:|
| Comprimento do pêndulo | 30 cm |
| Massa na extremidade do pêndulo | 71,45 g |
| Centro de massa informado | 18,473 cm do eixo |
| Comprimento do braço rotacional | 20 cm |
| Massa do braço + conjunto do sensor | 261,55 g |

---

## 3. Bibliotecas

Principais bibliotecas utilizadas:

- `robtillaart/AS5600`
- `waspinator/AccelStepper`

Também estão sendo desenvolvidas bibliotecas próprias para abstrair o hardware do projeto.

Entre elas:

```text
MotorPosition
```

responsável por converter posições angulares em comandos para o motor de passo.

---

# Desenvolvimento por fases

## Fase 00 — Estrutura inicial

**Status: concluída**

Objetivos:

- criação do projeto PlatformIO;
- configuração do Arduino Nano;
- organização do código em fases;
- criação de um `env` específico para cada etapa;
- validação de compilação e upload.

Foi necessário configurar corretamente o Arduino Nano utilizado, que possui **bootloader antigo**.

A partir desta fase, cada etapa do projeto passou a possuir seu próprio diretório:

```text
src/
├── fase_00_estrutura/
├── fase_01_sensor/
├── fase_02_motor/
├── fase_03_referencia/
└── fase_04_limites_motor/
```

---

## Fase 01 — Sensor AS5600

**Status: concluída**

Objetivo: validar isoladamente a aquisição angular do pêndulo.

Foram implementados e testados:

- comunicação I²C;
- detecção do AS5600;
- detecção do ímã;
- leitura da posição angular;
- tratamento da passagem 0° ↔ 360°;
- posição angular acumulada;
- cálculo da velocidade angular.

Foi utilizada como referência a biblioteca oficial:

```text
robtillaart/AS5600
```

A aquisição foi posteriormente organizada de forma modular, separando as responsabilidades de:

```text
posição angular
velocidade angular
```

Essa estrutura será reutilizada posteriormente no controle do pêndulo.

---

## Fase 02 — Controle isolado do motor

**Status: concluída**

Objetivo: controlar e validar o conjunto:

```text
Arduino Nano
    ↓
A4988
    ↓
NEMA 17
```

Configuração utilizada:

```cpp
PASSOS_POR_VOLTA = 200;
MICRO_PASSO = 8;
```

Portanto:

```text
1600 micropassos/revolução
```

ou aproximadamente:

```text
4,444 micropassos/grau
```

Foi criada a biblioteca:

```text
MotorPosition
```

que encapsula o `AccelStepper` e permite trabalhar diretamente em graus.

Exemplo:

```text
P 30
P -30
P 0
```

em vez de trabalhar diretamente com números de passos.

Também foram implementados comandos pela Serial para:

```text
E          habilitar motor
D / L      liberar motor
Z          definir zero
P <graus>  mover para posição
STOP       parar
S          mostrar status
```

A liberação do A4988 permite posicionar manualmente o braço antes de definir sua referência.

---

## Fase 03 — Referência e limites do braço

**Status: concluída**

Foi adotado um sistema de coordenadas centrado no braço:

```text
          +90°
            |
            |
-90° ------ 0° ------ +90°
            |
         centro
```

Assim:

```text
0° = posição central do braço
```

e posições para lados opostos possuem sinais diferentes.

### Referência

Como atualmente não existe encoder no braço, sua posição é determinada pela contagem dos passos enviados ao motor.

O procedimento inicial é:

```text
L
```

Posicionar manualmente o braço no centro e então:

```text
Z
E
```

O comando `Z` informa ao sistema que a posição física atual corresponde a:

```text
0°
```

Ao liberar o motor, a referência é considerada perdida, pois o braço pode ser movimentado manualmente.

### Limites

Os limites físicos considerados são aproximadamente:

```text
-90° a +90°
```

Para evitar colisões mecânicas foi adotada uma margem de segurança:

```text
limite operacional = -80° a +80°
```

Movimentos fora dessa região são recusados pelo software.

---

# Fase 04 — Caracterização dinâmica do motor

**Status: em desenvolvimento**

Esta fase busca determinar experimentalmente os limites reais do conjunto:

```text
NEMA 17 + A4988 + braço do Furuta
```

antes da implementação do controle do pêndulo.

Os principais parâmetros investigados são:

- velocidade máxima confiável;
- aceleração máxima confiável;
- perda de passos;
- comportamento durante reversões;
- repetibilidade;
- possíveis diferenças entre os dois sentidos;
- margem operacional adequada.

A biblioteca `MotorPosition` foi ampliada para permitir alteração dinâmica de:

```cpp
setMaxSpeed(...)
setAcceleration(...)
```

em unidades angulares:

```text
graus/s
graus/s²
```

A conversão para micropassos é realizada internamente pela biblioteca.

---

## Teste automático

Foi criado um ensaio automático com movimento:

```text
0° → +60° → -60° → 0°
```

O comando:

```text
T 10
```

executa dez ciclos.

Os parâmetros podem ser alterados pela Serial:

```text
V 180
```

define:

```text
velocidade = 180°/s
```

e:

```text
A 450
```

define:

```text
aceleração = 450°/s²
```

---

## Procedimento experimental

Inicialização:

```text
L
```

Posicionar manualmente o braço no centro.

Depois:

```text
Z
E
```

Configuração inicial:

```text
V 180
A 450
T 10
```

O braço executará:

```text
0 → +60 → -60 → 0
```

dez vezes.

---

## Detecção de perda de passos

O braço **não possui encoder**.

Portanto, a posição indicada pelo software não é suficiente para detectar perda de passos.

Uma marca física no braço e na estrutura deve indicar o zero real.

Depois do ensaio:

```text
T 10
```

o programa poderá indicar:

```text
Posicao calculada: 0.00 graus
```

mas é necessário verificar se o braço realmente retornou à marca física.

Se as marcas não coincidirem, ocorreu perda de referência, provavelmente devido à perda de passos.

---

## Sequência planejada de testes

Inicialmente, manter a velocidade baixa e aumentar progressivamente a aceleração:

```text
V 180

A 450
A 600
A 900
A 1200
A 1500
```

Depois, utilizando uma aceleração comprovadamente segura, aumentar a velocidade:

```text
V 180
V 270
V 360
V 450
V 600
```

Cada configuração deve ser inicialmente testada com:

```text
T 10
```

e configurações promissoras devem posteriormente passar por testes mais longos:

```text
T 50
T 100
```

---

## Interpretação dos testes

| Comportamento | Possível causa |
|---|---|
| Perda de passos durante reversões | aceleração excessiva |
| Perda de passos durante movimento contínuo | velocidade excessiva / torque insuficiente |
| Vibração intensa | aceleração excessiva ou região de ressonância |
| Erro diferente entre sentidos | assimetria mecânica |
| Funciona frio e falha depois | aquecimento do driver/motor |
| Software retorna a 0°, mas braço não | perda de passos |
| Ângulo físico diferente do comandado | microstepping incorreto |

Os valores máximos encontrados experimentalmente **não serão necessariamente utilizados no controle**.

Será adotada uma margem abaixo dos limites encontrados para aumentar a confiabilidade.

---

# Estrutura do software

A estrutura geral segue aproximadamente:

```text
FURUTA_V0/
│
├── platformio.ini
│
├── lib/
│   └── motor/
│       └── Position/
│           └── src/
│               ├── MotorPosition.h
│               └── MotorPosition.cpp
│
└── src/
    ├── fase_00_estrutura/
    │   └── main.cpp
    │
    ├── fase_01_sensor/
    │   └── main.cpp
    │
    ├── fase_02_motor/
    │   └── main.cpp
    │
    ├── fase_03_referencia/
    │   └── main.cpp
    │
    └── fase_04_limites_motor/
        └── main.cpp
```

Cada fase possui um ambiente correspondente no `platformio.ini`.

Isso permite compilar individualmente uma etapa sem interferência das demais.

---

# Próximas etapas

Após a conclusão da Fase 04, o projeto avançará progressivamente para a integração entre motor e sensor.

As etapas seguintes deverão incluir:

```text
caracterização dinâmica
        ↓
integração motor + AS5600
        ↓
aquisição sincronizada dos estados
        ↓
swing-up
        ↓
detecção da região de captura
        ↓
controle próximo da vertical
        ↓
estabilização do pêndulo
        ↓
controle simultâneo do braço
```

O controlador final será desenvolvido somente depois que os limites reais do hardware e a qualidade das medições forem conhecidos.

---

## Estado atual

```text
Fase 00  Estrutura do projeto             ✓ concluída
Fase 01  Sensor AS5600                    ✓ concluída
Fase 02  Controle isolado do motor        ✓ concluída
Fase 03  Referência e limites             ✓ concluída
Fase 04  Caracterização do motor          → em desenvolvimento
```

---

## Observação importante

Este projeto envolve um sistema mecanicamente instável e um braço rotacional em movimento rápido.

Os limites de velocidade, aceleração e posição devem ser aumentados progressivamente durante os testes, mantendo margem em relação aos obstáculos físicos do mecanismo.

A posição do braço atualmente é estimada pela contagem de passos do motor. Qualquer perda de passos provoca perda da referência absoluta até que o sistema seja novamente referenciado.