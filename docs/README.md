## Schedule
|                        | **1-2** | **3-4** | **5-6** | **7-8** | **9-10** | **11-12** |
| :--------------------: | :-----: | :-----: | :-----: | :-----: | :------: | :-------: |
|   literature review    |    ⬛    |    ⬜    |    ⬜    |    ⬜    |    ⬜     |           |
| pipeline configuration |    ⬛    |    ⬜    |         |    ⬜    |          |           |
|   baseline assembly    |    ⬜    |    ⬜    |         |         |          |           |
|   obstacle avoidance   |         |    ⬜    |    ⬜    |    ⬜    |          |           |
|   trajectory update    |         |    ⬜    |    ⬜    |    ⬜    |    ⬜     |           |
|      free flight       |         |         |         |    ⬜    |    ⬜     |     ⬜     |
| results dissemination  |         |         |         |         |    ⬜     |     ⬜     |


<p>
<details>
<summary>Revisão bibliográfica (`literature review`)</summary>
Esta etapa consiste em continuar nossos estudos sobre trabalhos estado-da-arte relacionados ao projeto, 
explorando em particular a literatura sobre: <em>path planning</em>, otimização de trajetórias de <em>drones</em>, odometria visual, 
trabalhos utilizando simuladores para veículos aéreos e aprendizado por reforço.
</p>
</details>

<details>
<summary>Configuração do <em>pipeline</em> de experimentos (`pipeline configuration`)</summary>
<p>
Prepararemos os programas envolvidos no projeto — AirSim (simulador), 
Meshroom (reconstrução 3D), Open3D (análise dos dados tridimensionais) —
para que possamos automatizar os experimentos, integrando as APIs Python providas por 
todas as bibliotecas mencionadas, de forma a acelerar o <em>feedback loop</em> de 
desenvolvimento → teste → avaliação → desenvolvimento. Esta etapa também envolve a avaliação do OpenDroneMap, e a familiarização com a Unreal Engine para podermos criar e alterar diferentes cenas de teste (p. ex. para utilizarmos <em>domain randomization</em>) na segunta metade do projeto.
</p>
</details>

<details>
<summary>Criação de uma <em>baseline</em> (`baseline assembly`)</summary>
<p>
Partiremos de trajetórias simples pré-programadas, comumente utilizadas (p. ex. em grade e em hélice), 
coletando imagens de um conjunto de cenas simuladas para podermos utilizá-las 
como base de referência para avaliar nosso trabalho, comparando as
reconstruções feitas.
</p>
</details>

<details>
<summary>Prevenção de obstáculos (`obstacle avoidance`)</summary>
<p>
Ainda seguindo um plano de voo predeterminado, introduziremos obstáculos de modo que será necessário que o <em>drone</em> desvie do caminho planejado (sem intervenção manual de um piloto, mantendo o voo autônomo).
Para isso, aplicaremos um algoritmo capaz de desviar da trajetória para evitar colisões e então retomá-la — fazendo o replanejamento 
e a detecção de obstáculos (a partir das imagens da câmera) em tempo real.
</p>
</details>

<details>
<summary>Atualização de trajetória a partir da nuvem de pontos (`trajectory update`)</summary>
<p>
Nesta etapa iniciaremos a análise em tempo real da nuvem de pontos reconstruída (pelo Meshroom a partir das imagens coletadas 
no AirSim), com o objetivo de determinar as regiões da estrutura sobrevoada que devem ser melhor capturadas, com o objetivo de 
aprimorar a reconstrução 3D final.
</p>
</details>

<details>
<summary>Voo livre (`free flight`)</summary>
<p>
Exploraremos como tornar o modelo implementado mais geral, removendo a limitação imposta por termos que prover um 
plano de voo inicial. Assim, pretendemos ter ao final desta fase um algoritmo que seja inteiramente responsável por 
traçar a rota de voo do <em>drone</em>, tendo conhecimento apenas de uma caixa delimitadora (<em>bounding box</em>) da área de interesse.
</p>
</details>

<details>
<summary>Disseminação de resultados (`results dissemination`)</summary>
<p>
Na etapa final do projeto de Iniciação Científica escreveremos e submeteremos artigos científicos (e relatórios técnicos) 
para conferências de relevância na área, melhoraremos a documentação dos algoritmos desenvolvidos, e publicaremos o trabalho 
desenvolvido de forma aberta (<em>open-source</em>).
</p>
</details>
<p>
</p>