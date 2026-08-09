# Pêndulo de Furuta --- Controle em Arduino Nano

Projeto experimental de um **Pêndulo de Furuta (Rotary Inverted
Pendulum)** desenvolvido com Arduino Nano, motor de passo NEMA 17,
driver A4988 e sensor magnético AS5600.

O projeto foi construído de forma incremental: primeiro foram validados
individualmente o sensor e o atuador; depois foi desenvolvida a
estimativa de velocidade do pêndulo, realizada a identificação
experimental da dinâmica próxima à posição vertical e, finalmente,
projetado e testado um controlador por realimentação de quatro estados.

> **Estado atual:** estabilização local do pêndulo na posição vertical
> implementada e validada experimentalmente.\
> O **swing-up automático ainda não faz parte da versão atual**.

------------------------------------------------------------------------

## Sumário

1.  [Objetivo](#objetivo)
2.  [Visão geral do sistema](#visão-geral-do-sistema)
3.  [Hardware](#hardware)
4.  [Variáveis do sistema](#variáveis-do-sistema)
5.  [Estratégia de desenvolvimento](#estratégia-de-desenvolvimento)
6.  [Estrutura do projeto](#estrutura-do-projeto)
7.  [Bibliotecas](#bibliotecas)
8.  [Medição do pêndulo](#medição-do-pêndulo)
9.  [Estimativa da velocidade](#estimativa-da-velocidade)
10. [Caracterização do atuador](#caracterização-do-atuador)
11. [Referências do sistema](#referências-do-sistema)
12. [Identificação experimental](#identificação-experimental)
13. [Evolução do controle](#evolução-do-controle)
14. [Controlador atual](#controlador-atual)
15. [Captura e BALANCE](#captura-e-balance)
16. [Limites e proteções](#limites-e-proteções)
17. [Telemetria](#telemetria)
18. [Como compilar e executar](#como-compilar-e-executar)
19. [Sequência básica de ensaio](#sequência-básica-de-ensaio)
20. [Principais resultados](#principais-resultados)
21. [Seis decisões importantes](#seis-decisões-importantes)
22. [Limitações atuais](#limitações-atuais)
23. [Próximas etapas](#próximas-etapas)

------------------------------------------------------------------------

# Objetivo

O objetivo deste projeto é desenvolver experimentalmente um sistema
capaz de controlar um **Pêndulo de Furuta**, utilizando hardware de
baixo custo e uma metodologia progressiva de validação.

Nesta etapa do trabalho, o objetivo principal foi:

**manter o pêndulo estabilizado próximo da posição vertical instável.**

O desenvolvimento do swing-up --- movimento que leva automaticamente o
pêndulo da posição inferior até a região de equilíbrio --- foi
deliberadamente deixado para uma etapa posterior.

------------------------------------------------------------------------

# Visão geral do sistema

O sistema é formado por um braço horizontal acionado por um motor de
passo. Na extremidade desse braço encontra-se o eixo do pêndulo.

O sensor AS5600 mede a posição angular do pêndulo. O Arduino estima sua
velocidade, calcula o estado do sistema e determina a aceleração que
deve ser comandada ao braço.

Fluxo simplificado:

``` text
                +-------------------+
                |      AS5600       |
                | posição pêndulo   |
                +---------+---------+
                          |
                          v
                +-------------------+
                | posição/velocidade|
                |    do pêndulo     |
                +---------+---------+
                          |
                          v
+---------------+   +-------------------+
| estado braço  |-->| vetor de estados  |
+---------------+   +---------+---------+
                              |
                              v
                    +-------------------+
                    | controlador de    |
                    | quatro estados    |
                    +---------+---------+
                              |
                              v
                    +-------------------+
                    | MotorVelocity     |
                    +---------+---------+
                              |
                              v
                    +-------------------+
                    | A4988 + NEMA 17   |
                    +-------------------+
```

------------------------------------------------------------------------

# Hardware

## Controlador

-   Arduino Nano
-   ATmega328P
-   comunicação serial a 115200 baud
-   I2C utilizado para comunicação com o AS5600

## Sensor do pêndulo

-   AS5600
-   sensor magnético absoluto
-   endereço I2C padrão: `0x36`
-   biblioteca utilizada: `robtillaart/AS5600`

## Atuador

-   motor de passo NEMA 17
-   driver A4988
-   alimentação do conjunto de potência em 12 V
-   microstepping utilizado no desenvolvimento
-   acionamento por sinais STEP, DIR e ENABLE

## Pinagem principal utilizada

A pinagem deve ser conferida no código da versão utilizada antes de
qualquer alteração de hardware.

Na configuração consolidada do projeto foram utilizados, entre outros:

``` text
STEP    -> D6
DIR     -> D8
ENABLE  -> D4
AS5600  -> barramento I2C
```

------------------------------------------------------------------------

# Variáveis do sistema

O controlador final considera quatro estados:

``` text
phi       posição angular do braço
phiDot    velocidade angular do braço
beta      erro angular do pêndulo em relação à vertical
betaDot   velocidade angular do pêndulo
```

O vetor de estado pode ser representado conceitualmente como:

``` text
x = [phi, phiDot, beta, betaDot]
```

A variável de controle é:

``` text
u = aceleração angular comandada ao braço
```

## Atenção

`beta` e `betaDot` são obtidos a partir do sensor do pêndulo.

Já `phi` e `phiDot` **não são medidos por um encoder independente**:

-   `phi` é estimado pela contagem dos passos comandados;
-   `phiDot` deriva do movimento comandado ao motor.

Essa é uma limitação importante da implementação atual.

------------------------------------------------------------------------

# Estratégia de desenvolvimento

O projeto não foi desenvolvido diretamente como um controlador completo.

A estratégia adotada foi:

``` text
MEDIR
  ↓
VALIDAR
  ↓
ATUAR
  ↓
VALIDAR
  ↓
IDENTIFICAR
  ↓
CONTROLAR
  ↓
INTEGRAR
```

Essa abordagem permitiu verificar cada elemento separadamente antes de
atribuir eventuais problemas ao controlador.

------------------------------------------------------------------------

# Estrutura do projeto

O projeto utiliza **PlatformIO** e foi organizado em ambientes
correspondentes às etapas de desenvolvimento.

Estrutura conceitual:

``` text
FURUTA_V0/
│
├── platformio.ini
│
├── src/
│   ├── fase_00_estrutura/
│   ├── fase_01_sensor/
│   ├── fase_02_motor/
│   ├── ...
│   └── fase_10_integracao_final/
│       └── main.cpp
│
└── lib/
    ├── PendulumPosition/
    ├── PendulumVelocity/
    ├── MotorVelocity/
    ├── StateVector/
    ├── FourStateController/
    ├── BalanceTelemetry/
    └── FurutaSystem/
```

As fases antigas permanecem úteis como registro da evolução e como
programas de teste isolado.

A Fase 10 concentra a arquitetura integrada.

------------------------------------------------------------------------

# Bibliotecas

## Bibliotecas externas

### AS5600

Utilizada para comunicação com o sensor magnético:

``` ini
robtillaart/AS5600
```

### AccelStepper

Utilizada durante o desenvolvimento do acionamento do motor:

``` ini
waspinator/AccelStepper
```

## Bibliotecas desenvolvidas no projeto

### PendulumPosition

Responsável pela leitura e tratamento da posição angular do pêndulo.

### PendulumVelocity

Responsável pela estimativa filtrada da velocidade angular.

### MotorVelocity

Abstrai o comando de velocidade/aceleração aplicado ao motor de passo.

### StateVector

Organiza os quatro estados utilizados pelo controlador.

### FourStateController

Implementa a lei de controle final.

### BalanceTelemetry

Responsável pela saída de dados experimentais e telemetria.

### FurutaSystem

Integra:

-   sensores;
-   estados;
-   captura;
-   controle;
-   atuador;
-   segurança;
-   comandos seriais;
-   telemetria.

Na Fase 10, o `main.cpp` pôde ser reduzido essencialmente à
inicialização e atualização do objeto principal.

------------------------------------------------------------------------

# Medição do pêndulo

O AS5600 fornece a posição angular absoluta do eixo do pêndulo.

Durante o desenvolvimento foram validados:

-   comunicação I2C;
-   detecção do sensor;
-   presença do ímã;
-   continuidade angular;
-   referência de posição;
-   tratamento da passagem entre 0 e 360 graus.

O sensor é utilizado como fonte primária da posição do pêndulo.

------------------------------------------------------------------------

# Estimativa da velocidade

Calcular a velocidade apenas pela diferença entre duas amostras
consecutivas mostrou-se inadequado.

O problema ocorre porque:

-   o AS5600 possui resolução discreta;
-   o período de amostragem é pequeno;
-   uma única contagem pode produzir uma variação aparente significativa
    de velocidade.

Foi adotada uma estimativa baseada em **regressão linear sobre uma
janela de sete amostras**.

Essa abordagem reduz o efeito da quantização e produz uma estimativa de
velocidade mais apropriada para o controlador.

A escolha envolve um compromisso:

``` text
mais filtragem  -> menos ruído
mais filtragem  -> maior atraso
```

A janela adotada apresentou comportamento adequado nos ensaios
realizados.

------------------------------------------------------------------------

# Caracterização do atuador

Antes da identificação do sistema, o motor foi testado isoladamente.

Foram avaliados:

-   movimento angular;
-   conversão entre graus e passos;
-   velocidade;
-   aceleração;
-   reversões;
-   repetibilidade;
-   perda de passos;
-   limites operacionais.

Valores conservadores adotados no controlador:

``` text
velocidade máxima do braço: aproximadamente 180 deg/s
aceleração máxima comandada: aproximadamente 600 deg/s²
```

Esses limites não representam necessariamente o máximo absoluto do
motor. Eles foram escolhidos como limites de operação do sistema de
controle.

------------------------------------------------------------------------

# Referências do sistema

## Referência do braço

O braço não possui encoder absoluto.

Por isso, uma posição física escolhida é definida como:

``` text
phi = 0
```

por meio do comando `Z`.

Quando o motor é desabilitado e o braço pode ser movimentado
manualmente, essa referência deve ser considerada perdida.

## Referência do pêndulo

Posicionar manualmente o pêndulo exatamente na vertical mostrou-se pouco
prático.

Foi adotada uma solução mais robusta:

1.  deixar o pêndulo livre e parado para baixo;
2.  registrar essa posição;
3.  calcular automaticamente a posição superior como 180 graus em
    relação à referência inferior.

Assim:

``` text
TOP = DOWN + 180 graus
```

O comando `T` realiza essa calibração na implementação atual.

------------------------------------------------------------------------

# Identificação experimental

Não foi adotado inicialmente um modelo físico completo obtido apenas a
partir das dimensões, massas e inércias.

Em vez disso, a dinâmica próxima da vertical foi identificada
experimentalmente.

## Estratégia

Foram realizados ensaios com o pêndulo próximo da posição superior e
excitações conhecidas aplicadas ao braço.

Os dados utilizados incluíram:

``` text
tempo
posição do pêndulo
velocidade do pêndulo
comando aplicado ao braço
posição estimada do braço
velocidade comandada ao braço
```

Também foram utilizados ensaios com excitações de sinais opostos para
verificar a coerência da resposta.

## Modelo local identificado

O modelo experimental consolidado próximo da vertical foi:

``` text
betaDDot = 21.7*beta - 3.28*betaDot - 0.75*u
```

onde:

``` text
beta      = posição do pêndulo em relação ao topo
betaDot   = velocidade angular do pêndulo
betaDDot  = aceleração angular do pêndulo
u         = aceleração comandada ao braço
```

Esse é um **modelo local**, destinado à região próxima da posição
vertical.

Ele não deve ser interpretado como um modelo global válido para todas as
posições do pêndulo.

------------------------------------------------------------------------

# Evolução do controle

## Primeira abordagem: PD do pêndulo

A primeira estratégia de estabilização utilizou apenas:

``` text
posição do pêndulo
velocidade do pêndulo
```

Ganhos obtidos a partir do modelo identificado:

``` text
Kp = 60.93
Kd = 8.96
```

O PD demonstrou que era possível estabilizar localmente o pêndulo.

Entretanto, surgiu um problema:

> o pêndulo podia permanecer controlado enquanto o braço caminhava
> progressivamente em direção ao limite de sua área de trabalho.

Portanto, controlar somente o pêndulo não era suficiente.

## Tentativa de correção independente do braço

Foi estudada a inclusão de termos adicionais para recentralizar o braço.

Os experimentos mostraram que tratar a correção do braço como uma ação
praticamente independente podia degradar a estabilização.

Isso levou à reformulação do problema.

## Controle de quatro estados

O sistema passou a ser tratado de forma integrada, considerando:

``` text
posição do braço
velocidade do braço
posição do pêndulo
velocidade do pêndulo
```

Essa mudança foi fundamental para controlar simultaneamente o equilíbrio
e o movimento do braço.

------------------------------------------------------------------------

# Controlador atual

Para o projeto do controlador foi utilizada a aproximação:

``` text
phiDDot = u
```

em conjunto com o modelo identificado do pêndulo.

O controlador foi projetado por posicionamento de polos.

Polos utilizados:

``` text
-0.7
-1.0
-4.0
-6.0
```

Ganhos finais:

``` text
Kphi     = 0.77419
KphiDot  = 2.31979
Kbeta    = 95.71079
KbetaDot = 14.31971
```

Lei de controle implementada:

``` text
u =
    Kphi*phi
  + KphiDot*phiDot
  + Kbeta*beta
  + KbetaDot*betaDot
```

com os sinais definidos de acordo com as convenções internas do projeto.

A saída é posteriormente limitada pelos valores permitidos para o
atuador.

------------------------------------------------------------------------

# Captura e BALANCE

O controlador local só deve assumir o sistema quando o pêndulo estiver
suficientemente próximo da posição vertical.

A lógica utilizada é aproximadamente:

``` text
IDLE
  ↓
WAIT_CAPTURE
  ↓
CAPTURE_READY
  ↓
WAIT_RELEASE
  ↓
BALANCE
```

O sistema verifica:

-   proximidade angular do topo;
-   velocidade angular suficientemente pequena;
-   permanência nessa região durante um intervalo mínimo.

Quando as condições são atendidas, a captura é reconhecida.

Após a soltura do pêndulo, o controlador passa a atuar automaticamente.

Essa lógica reduz a dependência de posicionar manualmente o pêndulo
exatamente no ponto de equilíbrio.

------------------------------------------------------------------------

# Limites e proteções

O sistema possui limites destinados a proteger o experimento e evitar
movimentos incompatíveis com a região de operação estudada.

Entre os valores utilizados durante o desenvolvimento:

``` text
período de controle:       4 ms
frequência de controle:    250 Hz

u máximo:                  +/-600 deg/s²
velocidade do braço:       aproximadamente +/-180 deg/s

limite de beta em testes:  aproximadamente +/-8 graus
limite do braço:           definido conforme a fase/ensaio
```

Os limites do braço foram modificados durante o desenvolvimento.
Portanto, **o valor definitivo deve sempre ser conferido em
`FurutaConfig.h` na versão executada**.

O software também prevê comandos de parada e emergência.

------------------------------------------------------------------------

# Telemetria

A Fase 10 foi refatorada também para permitir melhor observação do
comportamento do sistema.

A telemetria registra o vetor de estados e o controle.

Formato conceitual:

``` text
tempo
phi
phiDot
beta
betaDot
uRaw
u
```

onde:

``` text
uRaw = saída calculada pelo controlador antes da saturação
u    = comando efetivamente aplicado após os limites
```

Essa distinção permite identificar situações nas quais o controlador
solicita uma ação que o atuador não consegue executar.

A telemetria é enviada em frequência inferior à frequência do controle
para reduzir a interferência da comunicação serial na malha em tempo
real.

------------------------------------------------------------------------

# Como compilar e executar

## Requisitos

-   Visual Studio Code
-   extensão PlatformIO
-   Arduino Nano
-   driver USB correspondente ao Nano utilizado

## PlatformIO

O ambiente ativo deve corresponder à fase desejada.

Para a versão integrada:

``` ini
[platformio]
default_envs = fase_10_integracao_final
```

O projeto utiliza:

``` ini
platform = atmelavr
board = nanoatmega328
framework = arduino
monitor_speed = 115200
```

Dependendo do Arduino Nano utilizado, pode ser necessário configurar o
protocolo/bootloader correspondente à placa.

## Compilação

No PlatformIO:

``` text
Build
```

ou pelo terminal:

``` bash
pio run
```

## Upload

``` bash
pio run --target upload
```

## Monitor serial

``` bash
pio device monitor
```

Velocidade:

``` text
115200 baud
```

------------------------------------------------------------------------

# Sequência básica de ensaio

A sequência consolidada utilizada nos ensaios de BALANCE é:

``` text
1. Centralizar fisicamente o braço.

2. D
   Desabilitar o motor.

3. Z
   Definir a posição atual do braço como phi = 0.

4. Deixar o pêndulo livre, parado e apontando para baixo.

5. T
   Registrar a referência inferior.
   O topo é calculado automaticamente a 180 graus.

6. E
   Habilitar o motor.

7. B
   Armar o modo BALANCE.

8. Levar manualmente o pêndulo para próximo da vertical.

9. Aguardar CAPTURE_READY.

10. Soltar o pêndulo.

11. Observar o ensaio e a telemetria.
```

> Antes de executar qualquer ensaio, confira no monitor serial as
> instruções da versão de firmware carregada.

------------------------------------------------------------------------

# Comandos seriais

Na versão integrada, os comandos principais são:

  Comando   Função
  --------- ------------------------------------------------------------
  `T`       Calibrar a referência do pêndulo com ele parado para baixo
  `Z`       Definir a posição atual do braço como zero
  `E`       Habilitar o motor
  `D`       Desabilitar o motor
  `B`       Armar captura e BALANCE
  `STOP`    Parar o controle
  `X`       Parada de emergência
  `S`       Exibir status
  `H`       Exibir ajuda

Os comandos podem variar em fases antigas do projeto.

------------------------------------------------------------------------

# Principais resultados

O projeto demonstrou experimentalmente que:

-   o AS5600 pode ser utilizado para medir a posição do pêndulo;
-   a velocidade angular pode ser estimada de forma adequada para o
    controle;
-   o NEMA 17 com A4988 possui autoridade suficiente para os ensaios
    locais realizados;
-   a dinâmica próxima da vertical pode ser identificada
    experimentalmente;
-   um PD baseado apenas no pêndulo consegue estabilizá-lo, mas não
    resolve adequadamente a deriva do braço;
-   a realimentação dos quatro estados melhora a formulação do problema;
-   o sistema conseguiu realizar ensaios sustentados de estabilização.

Entre os testes realizados houve um ensaio de aproximadamente:

``` text
100 segundos
```

mantendo o sistema em BALANCE.

Também foram registrados ensaios que terminaram por limite angular ou
saturação. Esses resultados são importantes porque mostram que a
estabilização obtida **não deve ser interpretada como robustez
ilimitada**.

A Fase 10 acrescenta telemetria justamente para permitir analisar em
detalhe o que ocorre antes de uma eventual perda de equilíbrio.

------------------------------------------------------------------------

# Seis decisões importantes

## 1. Desenvolver e validar por etapas

Não tentar resolver o sistema inteiro imediatamente.

## 2. Resolver primeiro o BALANCE

O swing-up foi deixado para depois da validação da estabilização local.

## 3. Não usar derivação simples para a velocidade

A velocidade do pêndulo passou a ser estimada por regressão sobre várias
amostras.

## 4. Identificar experimentalmente a planta

O controlador foi baseado no comportamento observado no equipamento
real.

## 5. Tratar braço e pêndulo como um único sistema

O controle evoluiu do PD do pêndulo para uma realimentação de quatro
estados.

## 6. Reconhecer que os estados do braço são inferidos

A ausência de encoder no braço é uma limitação explícita do projeto e
deve ser considerada na interpretação dos resultados.

------------------------------------------------------------------------

# Limitações atuais

## Ausência de encoder no braço

A principal limitação de instrumentação é que a posição real do braço
não é medida independentemente.

Se ocorrer perda de passos:

``` text
posição estimada != posição física
```

## Modelo local

O modelo identificado é válido para a região próxima da posição
vertical.

Não deve ser utilizado automaticamente como modelo global para o
swing-up.

## Estimativa da velocidade

A filtragem reduz o efeito da quantização, mas introduz atraso.

## Atuador

O motor de passo possui limites de:

-   velocidade;
-   aceleração;
-   torque;
-   frequência de passos.

Saturações podem reduzir a autoridade do controlador.

## Captura manual

A versão atual ainda depende do operador para levar o pêndulo à região
próxima da vertical.

------------------------------------------------------------------------

# Próximas etapas

As principais evoluções previstas são:

### 1. Analisar a telemetria da Fase 10

Estudar temporalmente:

``` text
phi
phiDot
beta
betaDot
uRaw
u
```

especialmente antes de episódios de saturação ou perda de equilíbrio.

### 2. Avaliar uma medição independente do braço

Adicionar encoder ou outro sensor permitiria medir diretamente a posição
real do braço.

### 3. Refinar a robustez do BALANCE

Investigar:

-   saturações;
-   atraso da estimativa;
-   perda de passos;
-   sensibilidade aos ganhos;
-   perturbações externas.

### 4. Desenvolver o swing-up

Após consolidar o BALANCE, implementar a estratégia para levar
automaticamente o pêndulo da posição inferior até a região de captura.

Arquitetura futura esperada:

``` text
IDLE
  ↓
SWING_UP
  ↓
CAPTURE
  ↓
BALANCE
```

### 5. Integrar swing-up e balance

A etapa final será realizar automaticamente a transição entre os dois
controladores.

------------------------------------------------------------------------

# Observação sobre segurança

O Pêndulo de Furuta é um sistema mecânico com partes móveis e movimentos
rápidos.

Durante os ensaios:

-   mantenha mãos e objetos afastados da trajetória do braço;
-   utilize limites de software;
-   mantenha acesso rápido à parada de emergência;
-   não aumente velocidade ou aceleração sem caracterizar previamente o
    atuador;
-   interrompa o teste se houver perda de passos, vibração excessiva ou
    comportamento inesperado.

------------------------------------------------------------------------

# Estado atual do projeto

``` text
[OK] Ambiente PlatformIO
[OK] AS5600
[OK] Medição de posição
[OK] Estimativa de velocidade
[OK] Acionamento do motor
[OK] Caracterização do atuador
[OK] Referências do braço e do pêndulo
[OK] Identificação experimental local
[OK] Controlador PD experimental
[OK] Controle de quatro estados
[OK] Captura automática próxima ao topo
[OK] BALANCE
[OK] Refatoração da arquitetura
[OK] Telemetria dos estados e controle

[ ] Análise extensa de robustez
[ ] Encoder independente do braço
[ ] Swing-up automático
[ ] Transição automática SWING_UP -> BALANCE
```

------------------------------------------------------------------------

# Conclusão

Este projeto adotou uma abordagem essencialmente experimental para o
desenvolvimento de um Pêndulo de Furuta.

Em vez de partir diretamente para um controlador completo, cada elemento
foi validado individualmente. A posição do pêndulo foi medida com o
AS5600, sua velocidade foi estimada com filtragem apropriada, o atuador
foi caracterizado e a dinâmica próxima da vertical foi identificada
experimentalmente.

O primeiro controlador PD demonstrou a possibilidade de estabilização,
mas também revelou a necessidade de considerar o movimento do braço. O
projeto então evoluiu para uma realimentação de quatro estados, tratando
braço e pêndulo como partes de um único sistema.

A arquitetura atual fornece uma base organizada para continuar o
desenvolvimento, especialmente em três direções:

**robustez, instrumentação do braço e swing-up automático.**

------------------------------------------------------------------------

## Licença

Defina aqui a licença escolhida para o projeto antes da publicação
pública do repositório.

Exemplos comuns para projetos de código aberto incluem MIT, BSD e GPL.

------------------------------------------------------------------------

## Documentação

Além deste README, o projeto possui documentação técnica detalhada do
processo de desenvolvimento, identificação, projeto do controlador,
resultados experimentais e limitações encontradas.

Esse material deve ser mantido junto ao repositório como registro das
decisões de engenharia e dos experimentos realizados.
