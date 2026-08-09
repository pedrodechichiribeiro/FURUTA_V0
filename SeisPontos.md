# As seis decisões mais importantes do projeto

Este texto registra as seis decisões que mais influenciaram o desenvolvimento do Pêndulo de Furuta. O objetivo não é detalhar cálculos, mas preservar as escolhas de engenharia que determinaram o caminho seguido no projeto.

---

## 1. Desenvolver e validar o sistema por etapas

A primeira decisão importante foi **não tentar resolver o Pêndulo de Furuta inteiro de uma vez**.

O projeto foi dividido em etapas, permitindo validar separadamente:

- o ambiente de desenvolvimento;
- o sensor de posição;
- a medida de velocidade do pêndulo;
- o acionamento do motor;
- os limites do atuador;
- a identificação experimental;
- o controlador;
- a integração final.

A estratégia geral foi simples:

**medir → validar → atuar → validar → identificar → controlar**

Essa organização foi fundamental para evitar que problemas diferentes fossem confundidos. Um erro de leitura do sensor, por exemplo, não deveria ser interpretado como um problema do controlador.

A arquitetura modular alcançada ao final do projeto é consequência direta dessa decisão.

**Decisão:** desenvolver e validar o sistema progressivamente, antes de integrar todas as funções.

---

## 2. Resolver primeiro o equilíbrio e deixar o swing-up para depois

O problema completo do Pêndulo de Furuta possui duas tarefas diferentes:

1. levar o pêndulo da posição inferior até próximo da vertical;
2. manter o pêndulo equilibrado na vertical.

Decidimos inicialmente trabalhar apenas com a segunda tarefa.

O pêndulo seria levado manualmente para próximo da posição vertical e, a partir daí, o controlador assumiria o sistema.

Isso permitiu estudar o problema de estabilização sem introduzir ao mesmo tempo a complexidade do swing-up.

Também surgiu dessa decisão a sequência de captura utilizada no programa:

**aguardar aproximação → reconhecer o topo → detectar a soltura → iniciar o controle**

Somente depois de demonstrar que o sistema consegue permanecer equilibrado faz sentido acrescentar o swing-up automático.

**Decisão:** resolver primeiro o BALANCE e deixar o SWING-UP para uma etapa posterior.

---

## 3. Não calcular a velocidade do pêndulo por uma derivada simples

O AS5600 fornece diretamente a posição angular do pêndulo, mas o controlador também precisa conhecer sua velocidade.

Inicialmente, a solução mais óbvia seria calcular a velocidade usando apenas a diferença entre duas leituras consecutivas.

Os testes mostraram que isso não era adequado.

Como o sensor possui resolução discreta e o sistema realiza leituras muito rapidamente, pequenas mudanças de uma contagem produziam variações excessivas na velocidade calculada.

Decidimos então utilizar uma estimativa baseada em **regressão linear sobre uma janela de sete amostras**.

Essa solução introduz um pequeno atraso, mas produz uma medida de velocidade muito mais útil para:

- identificação do sistema;
- controle;
- reconhecimento da condição de captura;
- análise dos experimentos.

Essa decisão foi especialmente importante porque uma velocidade mal estimada prejudicaria tanto a identificação quanto o controlador.

**Decisão:** utilizar uma estimativa filtrada da velocidade do pêndulo, em vez de uma derivada simples entre duas amostras.

---

## 4. Identificar experimentalmente o sistema

Outra decisão fundamental foi **não depender exclusivamente de um modelo físico teórico**.

Para construir um modelo completo a partir da mecânica seria necessário conhecer com boa precisão diversos parâmetros, como massas, inércias, atritos, características do motor e outros efeitos do conjunto real.

Em vez disso, decidimos identificar experimentalmente o comportamento do sistema próximo da posição vertical.

Foram realizados ensaios controlados, aplicando movimentos conhecidos ao braço e observando a resposta do pêndulo.

Desses ensaios foi obtido um modelo local relacionando:

- posição do pêndulo;
- velocidade do pêndulo;
- aceleração comandada ao braço;
- aceleração resultante do pêndulo.

Ao mesmo tempo, o motor também foi caracterizado experimentalmente para determinar limites de velocidade e aceleração que pudessem ser usados com segurança e repetibilidade.

Assim, o controlador passou a ser projetado para o **sistema que realmente construímos**, e não apenas para um modelo ideal.

**Decisão:** identificar experimentalmente a dinâmica local e respeitar os limites reais do atuador.

---

## 5. Tratar braço e pêndulo como um único sistema

O primeiro controlador utilizava apenas a posição e a velocidade do pêndulo.

Ele conseguiu estabilizar o pêndulo, mas revelou um problema importante: **o braço podia continuar se deslocando até alcançar seu limite mecânico**.

Portanto, manter apenas o pêndulo na vertical não era suficiente.

Foi considerada inicialmente a possibilidade de acrescentar um controle separado para recentralizar o braço. Os testes mostraram, porém, que essa combinação de controles podia interferir negativamente na estabilização.

A solução foi mudar a forma de enxergar o problema.

Passamos a considerar simultaneamente quatro informações:

- posição do braço;
- velocidade do braço;
- posição do pêndulo;
- velocidade do pêndulo.

Em vez de existir um controlador para o pêndulo e outro para o braço, passou a existir **um único controlador considerando o estado completo do sistema**.

Essa mudança foi decisiva para obter ensaios sustentados de equilíbrio e manter também o movimento do braço sob controle.

**Decisão:** tratar braço e pêndulo como partes de um único sistema dinâmico de quatro estados.

---

## 6. Reconhecer que a posição e a velocidade do braço não são medidas diretamente

A sexta decisão foi reconhecer explicitamente uma limitação importante do hardware atual.

O pêndulo possui o AS5600 e, portanto, sua posição é medida diretamente.

O braço, entretanto, **não possui encoder próprio**.

A posição do braço é estimada a partir da contagem dos passos comandados ao motor. Sua velocidade também é obtida a partir do movimento que foi solicitado ao motor.

Isso significa que o programa conhece o movimento que deveria ter ocorrido, mas não possui uma medição independente confirmando que o braço realmente realizou exatamente esse movimento.

Se o motor perder passos, por exemplo, a posição registrada pelo programa pode deixar de corresponder à posição física real.

Essa limitação explica vários cuidados adotados no projeto:

- quando o motor é liberado, a referência do braço é considerada perdida;
- o braço precisa ser novamente referenciado antes de determinados ensaios;
- foram adotados limites conservadores de velocidade e aceleração;
- os resultados precisam ser interpretados sabendo que o movimento do braço é estimado;
- uma futura versão poderá incorporar um sensor independente para o braço.

Reconhecer essa limitação é melhor do que tratá-la como se não existisse.

**Decisão:** assumir explicitamente que os estados do braço são inferidos pelo comando do motor e não medidos por um encoder independente.

---

# Resumo

| Nº | Decisão | Por que foi importante |
|---|---|---|
| 1 | Desenvolver por etapas | Permitiu validar cada componente antes da integração |
| 2 | Resolver primeiro o BALANCE | Separou estabilização e swing-up |
| 3 | Filtrar a medida de velocidade | Melhorou a qualidade da informação usada pelo controle |
| 4 | Identificar experimentalmente o sistema | Aproximou o modelo do equipamento real |
| 5 | Adotar um controle de quatro estados | Passou a considerar conjuntamente braço e pêndulo |
| 6 | Reconhecer a ausência de encoder no braço | Tornou explícita uma limitação importante do sistema |

---

# Conclusão

Essas seis decisões contam, de forma resumida, a evolução técnica do projeto.

O desenvolvimento começou pela validação individual dos componentes, passou pela construção de medidas confiáveis, caracterização do atuador e identificação experimental, e chegou finalmente a um controlador que considera conjuntamente o braço e o pêndulo.

Ao mesmo tempo, o projeto manteve explícitas suas limitações atuais, especialmente a ausência de uma medição independente da posição do braço.

Esse caminho criou uma base sólida para as próximas etapas: aumentar a robustez da estabilização, melhorar a instrumentação do braço e, posteriormente, implementar o swing-up automático.
