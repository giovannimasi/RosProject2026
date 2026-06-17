# Human-Like Motion Planning Based on Game Theoretic Decision Making

**Authors:** Annemarie Turnwald, Dirk Wollherr
**Affiliation:** Chair of Automatic Control Engineering, Technical University of Munich, 80290 Munich, Germany
**Published in:** International Journal of Social Robotics (2019) 11:151–170
**DOI:** https://doi.org/10.1007/s12369-018-0487-2
**Accepted:** 3 July 2018 / Published online: 11 July 2018 / © The Author(s) 2018

---

## Abstract

Robot motion planners are increasingly being equipped with an intriguing property: **human likeness**. This property can enhance human–robot interactions and is essential for a convincing computer animation of humans. This paper presents a (multi-agent) motion planner for dynamic environments that generates human-like motion. The presented motion planner stands out against other motion planners by explicitly modeling human-like decision making and taking interdependencies between individuals into account, which is achieved by applying **game theory**. Non-cooperative games and the concept of a **Nash equilibrium** are used to formulate the decision process that describes human motion behavior while walking in a populated environment.

The approach is evaluated through two experiments designed as variations of the **Turing test**:
1. A **video study** showing simulated, moving pedestrians (participants are passive observers).
2. A **collision avoidance study**, wherein participants interact within virtual reality with an agent controlled by different motion planners.

Results of both studies coincide and show that participants could **not distinguish** between human motion behavior and the game-theoretic artificial behavior. In contrast, participants **could** distinguish human motions from motions based on established planners such as **reciprocal velocity obstacles (RVO)** or **social forces (SF)**.

**Keywords:** Game theory · Human-like motion planning · Variation of Turing test · Interaction awareness

---

## 1 Introduction

People assign human attributes to inanimate objects (beliefs, consciousness, intentions) — this is called **anthropomorphizing**. This can be exploited to enhance human–robot interaction. The aim: devise a robot motion planner that generates **human-like motions**, and that addresses the challenges of a *populated, yet not crowded* environment.

Key motivations:
- A human perceives an agent as more mindful if it appears similar to oneself; then it is treated as an entity that deserves respect and possesses apparent competence.
- Two aspects influence the scale of anthropomorphism: **similarity in appearance** and **similarity in motion**. This paper focuses on the latter: **human-like motions**.

In previous work [81], the authors showed that human decision making during navigation can be approximated with **Nash's equilibrium** from game theory. Building on this, they devise a motion planner that reproduces human-like motion behavior using game theory.

**Game theory** extends optimal control theory to a decentralized multi-agent decision problem. It models the decision-making process of individuals rather than merely reactive behavior. Its major advantage: it incorporates reasoning about possible actions of others and interdependencies — called **interaction awareness** [81]. Ignoring interdependencies leads to inaccurate motion prediction and results in detours, stop-and-go motions, or even collisions.

Two major challenges addressed:
1. Enabling robots to move in a human-like way.
2. Navigating fluently within a dynamic environment by considering interaction awareness.

**Paper structure:** Sect. 2 surveys related work; Sect. 3 defines the problem; Sect. 4 explains the game-theoretic background and implementation; Sects. 5 and 6 cover the two experimental setups, evaluation methods, and results.

---

## 2 Related Work

Combines methods from psychology, robot motion planning, and mathematics.

**Psychological studies on anthropomorphism + motion:**
- Heider and Simmel [26]: humans ascribe intentions to moving shapes (circles, triangles) if movements resemble social interactions.
- Humans read emotions from the gait of humanoid robots, a Roomba, even a simplistic moving stick.
- Epley et al. [21]: humans are likely to anthropomorphize agents if they move at speeds similar to human walking speeds. A robot that moves in a human-like manner is more likely to be anthropomorphized.

**Evaluation of human likeness — two main approaches:**
1. Define a set of rules/measurements considered human-like, then determine how well the planner fulfills them (e.g., passing on the correct side, smoothness, number of collisions, path irregularity, safety threat). *Limitation:* assumes which behavior is human-like.
2. **Human discrimination** — assessment made by/against humans via observations or questionnaires. (This paper's approach, via a Turing test variation.)

**Game theory in motion planning and coordination:**
- LaValle and Hutchinson [43]: among first to propose game theory for high-level multi-robot coordination.
- Applications: multi-robot search, shared exploration, coalition formation, pursuit-evasion (zero-sum/differential games).
- Most works focus on *groups of robots*. In contrast, this paper considers human–robot or human–human navigation.
- Two-agent human–robot works: Gabler et al. [22], Dragan [20], Nikolaidis et al. [58] (pick-and-place / handover). Sadigh et al. [71] and Bahram et al. [4]: merging/crossing for cars.
- Hoogendoorn and Bovy [30]: first to connect game theory with human motion (crowd flows as differential game). Mesmer and Bloebaum [50]: game theory + velocity obstacles. Mean-field game theory for large crowds (Dogbé [18], Lachapelle and Wolfram [42]).
- Ma et al. [46]: estimated person-specific behavior parameters, predicted motions by encoding coupling during multi-agent interactions with game theory.

**This paper's distinction:** Uses game theory not only for *prediction* but also for *planning* human-like motions. Focuses on the decision-making process of individuals, taking interdependencies into account. All agents are modeled as **interaction-aware individuals** that anticipate possible avoidance maneuvers — going beyond the popular constant-velocity assumption. Navigation is formulated as a **mathematical decision** between movements. Interested in *microscopic* behavior (human likeness) in busy-but-not-crowded areas. First time a motion planner for populated environments based on game theory is tested in an **online fashion**.

---

## 3 Problem Formulation

> **Goal:** A motion planner that generates human-like motion behavior for robots acting in populated environments.

**Fundamental definition** (used throughout): an artificially generated motion is a **human-like motion** if a human perceives no difference between a 'real' human motion and an artificially generated motion. No further assumptions made.

> **Human-like motion planning:** Consists of planning collision-free motions for one or more agents such that they behave equivalent to, or indistinguishable from, a human.

The robot acts in populated environments → solve a **kinodynamic planning problem within a dynamic workspace** (see Fig. 2).

**Notation:**
- State space $\mathcal{X}$ occupied by static objects and dynamic agents.
- $\mathcal{X}^{\text{obj}}$ = unified occupancy of all static objects.
- Admissible workspace: $\mathcal{X}^{\text{adm}} = \mathcal{X} \setminus \mathcal{X}^{\text{obj}}$ (deliberately disregards space occupied by dynamic agents).
- Find a trajectory $\boldsymbol{\tau}_n$ for each agent $A_n \in \mathcal{A}$ satisfying static-object constraints and local differential constraints.

**Differential constraints** (implicit form):

$$\dot{\mathbf{x}}_n = f(\mathbf{x}_n, \mathbf{u}_n) \tag{1}$$

where $\mathbf{x}_n \in \mathcal{X}$ is the agent state and $\mathbf{u}_n \in \mathcal{U}_n$ the control input of agent $A_n$.

**Kinodynamic planning problem** [44]: find a trajectory from an initial state $\mathbf{x}_n^{\text{init}} \in \mathcal{X}$ to a goal region $\mathcal{X}_n^{\text{goal}} \in \mathcal{X}$. A trajectory is a time-parameterized continuous path:

$$\boldsymbol{\tau}_n : [0, T] \rightarrow \mathcal{X}^{\text{adm}}$$

that fulfills constraints (1) and avoids static objects. A trajectory **segment** is described by $\boldsymbol{\tau}_n([t^0, t^1])$.

**Collision-avoidance constraint** (dynamic agents): To guarantee collision-free navigation, the following must hold at any time $t$:

$$\mathcal{X}_n^{\text{dyn}}(t) \cap \mathcal{X}_{n'}^{\text{dyn}}(t) = \emptyset \quad \forall\, A_n, A_{n'} \in \mathcal{A} \tag{2}$$

with $\mathcal{X}_n^{\text{dyn}}(t)$ being the subset of state space occupied by a dynamic agent $A_n$ at time $t$.

**Agent properties (toward human likeness):**
- An agent can be either a human or a controllable agent (robots, game characters, simulated particles).
- All agents are **interaction-aware** (reason about possible motions of others and interdependencies, from a navigational perspective).

> **Interaction-aware motion planning:** Defined as the planning of collision-free trajectories in dynamic environments that additionally considers possible reciprocal actions and influences between all other dynamic agents.

**Specializations:**
- All dynamic agents are robots → centralized multi-agent motion planning problem.
- Robot(s) among humans → robot must reason about possible human motions + interdependencies and decide its trajectory.

The approach addresses both.

---

## 4 Human-Like, Interaction-Aware Motion Planning Based on Game Theory

Goal: reproduce human behavior with a motion planner that takes interaction awareness of all agents into account. Previous works [79, 81] showed human interaction-aware decision making can be mathematically formulated as **searching for Nash equilibria in a static game**.

### 4.1 Modeling Navigation as a Game

Focus: **non-cooperative games**.

> **Non-cooperative game theory:** Handles how rational individuals make decisions when they are interdependent.

- *Non-cooperative* (vs. cooperative/coalitional) means the focus is on benefit to *individuals* rather than groups. Cooperation still occurs if beneficial to individuals (= rational behavior).
- Individuals behave **rationally** if they maximize expected utility / minimize expected cost. This paper implies navigating humans act rationally.

> **Footnote on rationality:** The assumption may not be perfectly true (assumes all agents know all actions/costs and anticipate choices rationally). However, it is a logical first step before more realistic models.

#### Definition 1 (Static Game)
A static, non-cooperative, finite, nonzero-sum game is defined by [45]:

1. **Finite set of $N$ agents** $\mathcal{A} = \{A_1, A_2, \dots, A_N\}$, $N = |\mathcal{A}|$.
2. **Finite set of action sets** $\mathcal{T} = \mathcal{T}_1 \cup \mathcal{T}_2 \cup \dots \cup \mathcal{T}_N$, where a set $\mathcal{T}_n$ is defined for each agent $A_n$. Each $\boldsymbol{\tau}_n^m \in \mathcal{T}_n$ is an **action** of $A_n$, with $m = \{1, 2, \dots, M_n\}$ and $M_n = |\mathcal{T}_n|$.
3. **Cost function** $J_n : \mathcal{T}_1 \times \mathcal{T}_2 \times \dots \times \mathcal{T}_N \rightarrow \mathbb{R} \cup \{\infty\}$ for each agent $A_n \in \mathcal{A}$.

Subscript $n$ = addressed agent; superscript $m$ = action. $\boldsymbol{\tau}_n^m$ is the $m$th action of $M_n$ actions of agent $A_n$. A game is **nonzero-sum** if the sum of each agent's costs can differ from zero.

Mapping to robotic motion planning: an action $\boldsymbol{\tau}_n^m$ is a **trajectory** leading $A_n$ from start $\mathbf{x}_n^{\text{init}}$ to goal region $\mathcal{X}_n^{\text{goal}}$.

**Example (Fig. 2/3):** Two agents $A_1, A_2$ walking on a sidewalk with a static object. Actions:
$$\mathcal{T}_1 = \{\boldsymbol{\tau}_1^1, \boldsymbol{\tau}_1^2, \boldsymbol{\tau}_1^3, \boldsymbol{\tau}_1^4\}, \quad \mathcal{T}_2 = \{\boldsymbol{\tau}_2^1, \boldsymbol{\tau}_2^2, \boldsymbol{\tau}_2^3, \boldsymbol{\tau}_2^4, \boldsymbol{\tau}_2^5\}$$

**Cost function** models interaction awareness — consists of an **independent** component $\hat{J}$ and an **interactive** component $\tilde{J}_n$:

$$J_n(\boldsymbol{\tau}_1^m, \dots, \boldsymbol{\tau}_n^{m'}, \dots, \boldsymbol{\tau}_N^{m''}) = \hat{J}_n(\boldsymbol{\tau}_n^{m'}) + \tilde{J}_n(\boldsymbol{\tau}_1^m, \dots, \boldsymbol{\tau}_n^{m'}, \dots, \boldsymbol{\tau}_N^{m''}) \tag{3}$$

- **Independent component** $\hat{J}_n(\boldsymbol{\tau}_n^{m'})$: depends only on agent $A_n$'s own action. If no interaction occurs, the game reduces to an independent set of optimal control problems.
- **Interactive component** $\tilde{J}_n$: the interdependency cost. Depends on its own and other agents' actions:

$$\tilde{J}_n(\boldsymbol{\tau}_1^m, \dots, \boldsymbol{\tau}_N^{m''}) := \begin{cases} \infty & \text{if a collision occurs} \\ 0 & \text{else} \end{cases} \tag{4}$$

So $\tilde{J}_n$ is infinity if action $\boldsymbol{\tau}_n^{m'}$ leads to a collision with another player's action; otherwise zero.

### 4.2 Solving Games: Solution Techniques

A combination of actions is called an **allocation** $(\boldsymbol{\tau}_1^m, \dots, \boldsymbol{\tau}_n^{m'}, \dots, \boldsymbol{\tau}_N^{m''})$.

> **Nash equilibrium:** An allocation where no agent can reduce its own cost by changing its action if the other agents stick to their actions. A Nash equilibrium is the best response for everyone.

Set of Nash equilibria: $\mathcal{E} = \{\epsilon^1, \dots, \epsilon^k, \dots, \epsilon^K\}$, with $K = |\mathcal{E}|$. Actions of an equilibrium allocation $\epsilon^k = (\boldsymbol{\tau}_1^*, \dots, \boldsymbol{\tau}_N^*)$ are marked with an asterisk.

#### Definition 2 (Nash equilibrium)
The $N$-tuple allocation of actions $(\boldsymbol{\tau}_1^*, \dots, \boldsymbol{\tau}_n^*, \dots, \boldsymbol{\tau}_N^*)$, with $\boldsymbol{\tau}_n^* \in \mathcal{T}_n$, constitutes a non-cooperative Nash equilibrium for an $N$-agent game if the following $N$ inequalities are satisfied for all actions $\boldsymbol{\tau}_n^m \in \mathcal{T}_n$:

$$
\begin{aligned}
J_1(\boldsymbol{\tau}_1^*, \boldsymbol{\tau}_2^*, \dots, \boldsymbol{\tau}_N^*) &\leq J_1(\boldsymbol{\tau}_1^m, \boldsymbol{\tau}_2^*, \dots, \boldsymbol{\tau}_N^*) \\
J_2(\boldsymbol{\tau}_1^*, \boldsymbol{\tau}_2^*, \dots, \boldsymbol{\tau}_N^*) &\leq J_2(\boldsymbol{\tau}_1^*, \boldsymbol{\tau}_2^m, \dots, \boldsymbol{\tau}_N^*) \\
&\vdots \\
J_N(\boldsymbol{\tau}_1^*, \boldsymbol{\tau}_2^*, \dots, \boldsymbol{\tau}_N^*) &\leq J_N(\boldsymbol{\tau}_1^*, \dots, \boldsymbol{\tau}_{N-1}^*, \boldsymbol{\tau}_N^m)
\end{aligned}
\tag{5}
$$

At least one solution $\epsilon^k$ exists if a cost function of form (3)–(4) is used.

> **Footnote:** Nash proved [57] that existence of at least one equilibrium is guaranteed for all types of cost functions if **mixed strategies** are considered.

**Example (Table 1, Fig. 3):** Cost matrix considering interdependence (collisions). The game has **four Nash equilibria**:
$$\mathcal{E} = \{\epsilon^1, \epsilon^2, \epsilon^3, \epsilon^4\} = \{(\boldsymbol{\tau}_1^{2*}, \boldsymbol{\tau}_2^{2*}), (\boldsymbol{\tau}_1^{1*}, \boldsymbol{\tau}_2^{3*}), (\boldsymbol{\tau}_1^{3*}, \boldsymbol{\tau}_2^{5*}), (\boldsymbol{\tau}_1^{4*}, \boldsymbol{\tau}_2^{4*})\}$$

**Table 1 — Static game cost pairs $J_1 | J_2$** (Nash equilibria in **bold**):

| $A_1 \backslash A_2$ | $\boldsymbol{\tau}_2^1$ | $\boldsymbol{\tau}_2^2$ | $\boldsymbol{\tau}_2^3$ | $\boldsymbol{\tau}_2^4$ | $\boldsymbol{\tau}_2^5$ |
|---|---|---|---|---|---|
| $\boldsymbol{\tau}_1^1$ | 5\|5 | 5\|4 | **5\|1** | ∞ | ∞ |
| $\boldsymbol{\tau}_1^2$ | 4\|5 | **4\|4** | ∞ | ∞ | ∞ |
| $\boldsymbol{\tau}_1^3$ | 1\|5 | ∞ | ∞ | ∞ | **1\|3** |
| $\boldsymbol{\tau}_1^4$ | ∞ | ∞ | ∞ | **2\|2** | 2\|3 |

(Independent costs from Fig. 3: $\hat{J}_1(\boldsymbol{\tau}_1^1)=5, \hat{J}_1(\boldsymbol{\tau}_1^2)=4, \hat{J}_1(\boldsymbol{\tau}_1^3)=1, \hat{J}_1(\boldsymbol{\tau}_1^4)=2$; $\hat{J}_2(\boldsymbol{\tau}_2^1)=5, \hat{J}_2(\boldsymbol{\tau}_2^2)=4, \hat{J}_2(\boldsymbol{\tau}_2^3)=1, \hat{J}_2(\boldsymbol{\tau}_2^4)=4, \hat{J}_2(\boldsymbol{\tau}_2^5)=3$.)

**Which equilibrium to choose?**

> **Pareto optimality:** A Pareto optimal outcome is an allocation in which it is impossible to reduce the cost of any player without raising the cost of at least one other player.

The cost pair (4\|4) is dominated by (2\|2) and (1\|3). Only **Pareto-optimal Nash allocations** are kept:
$$\mathcal{E}_{\text{pareto}} = \{(\boldsymbol{\tau}_1^{1*}, \boldsymbol{\tau}_2^{3*}), (\boldsymbol{\tau}_1^{3*}, \boldsymbol{\tau}_2^{5*}), (\boldsymbol{\tau}_1^{4*}, \boldsymbol{\tau}_2^{4*})\}, \quad \mathcal{E}_{\text{pareto}} \subseteq \mathcal{E}$$

These three are interpreted as avoidance maneuvers: agents avoid each other equally, or one gives way to the other.

### 4.3 Implementing a Game Theoretic Motion Planner

The problem is **decoupled** (see Fig. 4):
1. Solve the **kinodynamic planning problem independently** for each agent — repeatedly, to produce a set of trajectories (action sets) per agent.
2. **Game-theoretic reasoning** decides on a combination of trajectories.

The reasoning step differentiates between (a) all agents controllable, and (b) mixture of humans and robots.

#### 4.3.1 Trajectory Planning with Differential Constraints

A **control-based version of the RRT** (rapidly exploring random tree) [44] is used — chosen because it considers differential constraints and finds multiple trajectories. (Could also use RRT* or optimize trajectories; crucial that several *diverse* trajectories to the goal are found.)

The control-based RRT samples a state at random, finds the nearest neighbor, then in the extension step selects a control input $\mathbf{u} \in \mathcal{U}_n$ that extends the nearest vertex toward the sampled state. The control input is applied for a time interval $\delta t$, $\{\mathbf{u}(t') \mid t \leq t' \leq t + \delta t\}$, and the new state computed by numerical integration. Output: collision-free trajectory $\boldsymbol{\tau}_n$ **plus** a series of controls $\boldsymbol{\mu}_n : [0, T] \rightarrow \mathcal{U}_n$.

**Discrete-time approximation of (1):**

$$\mathbf{x}_n[t + 1] = f(\mathbf{x}_n[t], \mathbf{u}_n[t]) \tag{6}$$

**State and control vectors:**

$$\mathbf{x}_n[t] = \begin{pmatrix} x_n[t] \\ y_n[t] \\ \theta_n[t] \end{pmatrix}, \quad \mathbf{u}_n[t] = \begin{pmatrix} v_n[t] \\ w_n[t] \end{pmatrix}$$

where $(x_n, y_n, \theta_n)^\mathsf{T}$ describes global position and orientation of the center of mass, and control inputs $(v_n, w_n)^\mathsf{T}$ are **linear and angular velocities**.

Discrete trajectory: $\boldsymbol{\tau}_n = (\mathbf{x}_n[0], \mathbf{x}_n[1], \dots, \mathbf{x}_n[T])$; discrete control series: $\boldsymbol{\mu}_n = (\mathbf{u}_n[0], \mathbf{u}_n[1], \dots, \mathbf{u}_n[T])$.

**Discrete-time unicycle model** (to approximate an agent's motion):

$$
\begin{aligned}
x_n[t+1] &= x_n[t] + dt\, v_n[t] \cos(\theta_n[t]) \\
y_n[t+1] &= y_n[t] + dt\, v_n[t] \sin(\theta_n[t]) \\
\theta_n[t+1] &= \theta_n[t] + dt\, w_n[t]
\end{aligned}
\tag{7}
$$

$dt$ = magnitude of numerical integration time step. Note: $dt$ differs from the RRT time step $\delta t$ (the **propagation duration**, which can be larger).

**Finite set of control inputs:**

$$\mathcal{U}_n = \left\{ \begin{bmatrix} v_n \\ 0 \end{bmatrix}, \begin{bmatrix} v_n \\ w_n \end{bmatrix}, \begin{bmatrix} v_n \\ -w_n \end{bmatrix}, \begin{bmatrix} v_n \\ cw_n \end{bmatrix}, \begin{bmatrix} v_n \\ -cw_n \end{bmatrix} \right\} \tag{8}$$

with $w_n \in [w_n^{\min}, w_n^{\max}]$ randomly chosen every time before a new trajectory is planned. A factor $c$ allows for different curvatures within the resulting path; here $c = \tfrac{1}{2}$. The linear velocity $v_n$ is an agent-specific value.

To create diverse trajectories: angular velocity randomly chosen as above; propagation duration $\delta t$ not fixed but lies in interval $[\delta t^{\min}, \delta t^{\max}]$, varying at each state extension step. Upper/lower bounds randomly chosen from intervals $\Gamma^{\min}$ and $\Gamma^{\max}$ before each new trajectory plan (values in Table 2). The **Open Motion Planning Library** (http://ompl.kavrakilab.org/) served as the implementation basis.

#### 4.3.2 Choosing the Cost Function

Independent cost component $\hat{J}_n$ must be specified. A prior human motion analysis [81] found that considering only **length of a trajectory** plus possible collisions worked best. So:

$$\hat{J}_n(\boldsymbol{\tau}_n^m) := \text{Length}(\boldsymbol{\tau}_n^m) \tag{9}$$

Corroborated by research stating humans follow a minimization principle (minimize global path length / take shortest path). Acknowledged that length alone doesn't perfectly capture true navigation cost, but a more complex function doesn't necessarily improve performance [81]; length is a prevalent aim of humans.

#### 4.3.3 Multi-agent Motion Planning with Game Theory (all agents controllable)

A static game is constantly replayed every $\Delta t$ seconds (adapts to changes). Agents $A_n$, action sets $\mathcal{T}_n$, and costs $J_n$ change at every time step. Several RRT planners generate new trajectory sets for all agents.

**Stand-still action:** A default action $\boldsymbol{\tau}_n^0$ = "stand still for $\Delta t$ seconds" is added to each set $\mathcal{T}_n$ so an agent can stop immediately. It is tagged with an independent cost $\hat{J}$ that is **higher than each trajectory cost** in $\mathcal{T}_n$ but **lower than the cost for a collision**. This prevents "stand still" from always being the best option.

**Process:** After the game is set up, its set of Nash equilibria $\mathcal{E}$ is calculated and processed in the **coordination step** (Fig. 4). The Nash equilibrium that **Pareto-dominates** the others is chosen. If several Pareto-optimal equilibria exist, one is selected at random → allocation $\epsilon^*$. Each equilibrium trajectory $\boldsymbol{\tau}_n^*$ has a corresponding control series $\boldsymbol{\mu}_n^*$.

For the duration of $\Delta t$ seconds, the control inputs of the respective trajectories are transferred to each agent. The agents advance, the environment changes, and the next planning loop begins. The chosen equilibrium trajectories $\epsilon^*$ are **memorized** and used as actions in the static game of the following time step (see Fig. 4) — they lead to the goal and are promising because they were the 'winning' combination in the last loop.

**Example (Fig. 5/6):** Environment from the *BIWI Walking Pedestrians* dataset [62], five agents. Trajectory sets shown at different time steps; Nash equilibrium trajectories drawn in bold. The dark green agent's trajectory is constantly improved: at $t=0$ s the game found a relatively long and curvy solution; in general, the more often the game is played, the shorter the path becomes.

#### 4.3.4 Motion Planning Among Humans (mixture of controlled agents and humans)

A static game is repeatedly played; set of Nash equilibria $\mathcal{E}$ calculated each time step. Difference is in **deciding which Nash equilibrium is the 'winning' allocation $\epsilon^*$**:

- Only at the first planning loop ($t=0$) is the Pareto-optimal Nash equilibrium chosen.
- For subsequent steps, three sets are considered:
  - $\mathcal{E}[t]$ — Nash equilibria from the current time step.
  - $\mathcal{E}[t - \Delta t]$ — Nash equilibria from the previous time step.
  - $\mathcal{T}^{\text{obs}}$ — set of **observed trajectories** the agents actually walked in the previous time step, with elements $\boldsymbol{\tau}_n^{\text{obs}}([t - \Delta t, t])$.

**Reasoning step** (marked with dashed lines in Fig. 4):
1. Infer which of the previous equilibrium allocations $\epsilon^k \in \mathcal{E}[t - \Delta t]$ is most similar to the observed behavior $\mathcal{T}^{\text{obs}}$.
2. The most similar allocation in $\mathcal{E}[t - \Delta t]$ is compared to allocations in the new set $\mathcal{E}[t]$.
3. The allocation in $\mathcal{E}[t]$ with the highest resemblance is chosen as the 'winning' allocation $\epsilon^*$ of time step $t$.

By this, knowledge gained through observation is included. Similarity between two trajectories [80]: for this case it is sufficient to compute the **average Euclidean distance** (because only length is considered in the cost function). To obtain similarity between two allocations, the mean of all trajectory comparisons is calculated.

---

## 5 Evaluation: Multi-agent Motion Planning and Coordination (Online Video Study)

> **Hypothesis 1:** While watching a video that shows walking pedestrians, humans cannot distinguish between motions based on our game theoretic motion planner and motions based on human motions. They are perceived as equally human-like motions.

**Compared motion planning methods (abbreviations):**
- **HU** — human motions
- **GT** — game theoretic planner
- **RVO** — reciprocal velocity obstacles [84]
- **SF** — social forces [27, 31]

RVO and SF chosen because they are interaction-aware and often used for comparisons.

### 5.1 Experimental Setup: Online Video Study

- Questionnaire posted online. Participants watched videos of pedestrians in an urban environment (paths drawn in afterward, not visible during).
- Two motion generation methods: reproduce real human trajectories (HU), or generate artificial walking motions with the same start/goal using GT, RVO, or SF.
- After each video, participants decide whether motions are human or artificial (Turing-test inspired).
- Human trajectories from the **hotel sequence** of the *BIWI Walking Pedestrians* dataset [62]. Six sequences selected (≥4 pedestrians moving, some in different directions for interaction; 4–7 s long). Sequences start at times 160, 275, 404, 417, 454, 511 s.
- Agents' average speeds $\hat{v}_n$, initial states $\mathbf{x}_n^{\text{init}}$, goal regions $\mathcal{X}_n^{\text{goal}}$ extracted as input. GT used parameters in Table 2 column 'video'.
- Visualized with robot simulator **V-REP** (www.coppeliarobotics.com), compatible with ROS.

### 5.2 Statistical Data Analysis

- **227 participants**. 30% female / 70% male. Age 32.20 ± 9.43. Experience with robotics 2.17 ± 1.30 (scale 1–5). Study ~10 min.
- Each participant watched $6 \times 4 = 24$ videos (6 sequences, 4 methods) in random order. Rated human likeness per method (e.g., perceived 4/6 as human → 67%).

**Results (Fig. 8) — average human likeness:**

| Method | Perceived as human | Perceived as algorithm |
|---|---|---|
| **HU** (human recordings) | **71%** | 29% |
| **GT** (game theoretic) | **69%** | 31% |
| **RVO** | 43% | 57% |
| **SF** | 30% | 70% |

- GT (69%) perceived as **almost as human-like** as human recordings (71%). RVO (43%) and SF (30%) clearly lower.
- **Friedman test** (nonparametric; chosen because independent variable has >2 levels and dependent variable is ordinal, within-subject): $p \ll 0.001$ at 5% significance → at least one group differs.
- **Post hoc Wilcoxon-Nemenyi-McDonald-Thompson test** (Fig. 9): significant difference between **all** group comparisons **except GT-HU** ($p = 0.847$). Participants **could not distinguish** GT from human motions. → **Hypothesis 1 confirmed.**
- RVO-SF difference smaller yet still significant.

---

## 6 Evaluation: Motion Planning Among Humans (Virtual Reality Collision Avoidance Study)

Evaluates whether the GT planner generates human-like motions for an artificial agent moving in the **same environment** as a human (collision avoidance in VR).

> **Hypothesis 2:** While walking within virtual reality with another agent, humans cannot distinguish if the agents' motions are based on our game theoretic planner or on human motions. They are perceived as equally human-like motions.

- RVO planner also implemented; SF neglected (lowest human likeness previously).
- Participants moved within the same environment as the judged agent (named **Bill**), walked actively (no remote control), reacted to each other. Realized with head-mounted display + VR.

### 6.1 Experimental Setup: Walking Within Virtual Reality

- Combined: robot simulator **V-REP**, head-mounted display (**Oculus DK2**), motion capture system (**Qualisys**, vision-based, 250 Hz). Frame rate 17–25 fps.
- Environment reproduces a real laboratory (Fig. 11a). Carpet, colored start/goal markers, two agents.
- Task: repeatedly walk from a fixed start to a fixed goal while paying attention to Bill. Start/goal positions chosen so agents would most likely collide → forces interaction/avoidance.
- Participant's start/goal = yellow fields; Bill's = blue. Traffic-light countdown signals when to start.
- **Bill controlled by:** (a) human experimenter motions (HU) — pose matched via reflective markers; or (b) a motion planner (GT or RVO). GT used Table 2 'virtual' parameters; RVO used Table 7. Average velocity of each participant determined in a test run and forwarded to both planners.
- Each participant walked **30 rounds** (10 repetitions × 3 methods HU/GT/RVO), randomized order, ~1 hour.
- To conceal which mode: experimenter always walked start→goal; participant wore earplugs + elevator music (hide footstep sounds).

**Questionnaire (after each round):**
- **Q1:** Was Bill controlled by a real person or a computer program?
- **Q2:** How cooperative did Bill behave? (1 = very cooperative … 9 = not cooperative at all)
- **Q3:** How comfortable did you feel? (1 = very comfortable … 9 = not comfortable at all)

### 6.2 Statistical Data Analysis

- **27 volunteers**. 26% female / 74% male. Age 28.04 ± 4.20. Experience: robotics 2.93 ± 1.33, PC games 3.11 ± 1.45.

**Human likeness (Q1, Fig. 12):**

| Method | Perceived as human | Perceived as algorithm |
|---|---|---|
| **HU** | **66%** | 34% |
| **GT** | **60%** | 40% |
| **RVO** | 30% | 70% |

- Order identical to video study; percentages resemble each other (though lower).
- **Friedman test:** $p = 0.001$ at 5% significance.
- **Post hoc (Fig. 13a):** significant difference between all comparisons **except GT-HU**. Participants again **could not distinguish** GT from a real human, but **did** detect RVO. → confirms GT generates human-like behavior.

**Comfort (Q3) and Cooperation (Q2) — results inverted:**
- Levels of comfort and cooperation of the GT planner are comparatively high, but **not as high** as for human and RVO. Confirmed by two Friedman tests ($p \ll 0.001$).
- Post hoc comfort (Fig. 13b): participants felt similarly comfortable with a human or RVO, but **significantly less comfortable** with the GT planner.
- Cooperation (Fig. 13c): human and RVO rated **more cooperative** than GT.

**Trajectory analysis (Fig. 14, Table 4):**
- The human and particularly the RVO planner **start the avoidance maneuver earlier** than the GT planner.
- Computed average minimum distance, average velocity, average absolute curvature.
- Shapiro-Wilk normality test: all $p > 0.05$ → normally distributed → use **repeated measure ANOVA**.

**Table 4 — Means and SDs (VR study):**

| Variable | HU mean | GT mean | RVO mean | HU SD | GT SD | RVO SD |
|---|---|---|---|---|---|---|
| Human likeness (%) | 65.59 | 60.00 | 30.16 | 15.63 | 16.17 | 22.67 |
| Comfort* | 2.70 | 3.84 | 3.17 | 0.93 | 1.36 | 1.34 |
| Cooperation† | 2.66 | 4.13 | 3.00 | 0.99 | 1.58 | 1.44 |
| Distance (m) | 1.04 | 0.84 | 0.98 | 0.10 | 0.07 | 0.14 |
| Velocity (m/s) | 0.69 | 0.69 | 0.68 | 0.14 | 0.13 | 0.14 |
| Abs. curvature (1/m) | 1.51 | 1.48 | 1.54 | 0.57 | 0.50 | 0.52 |

*1 (very comfortable) … 9 (not comfortable at all). †1 (very cooperative) … 9 (not cooperative at all).

**Table 5 — Repeated measure ANOVA:**

| Variable | Mauchly $p$ | $F(2,52)$ | $p$ value | Effect size |
|---|---|---|---|---|
| Distance | 0.125 | 62.431 | < 0.001* | 0.396 |
| Velocity | 0.666 | 0.768 | 0.469 | 0.001 |
| Abs. curvature | 0.550 | 0.690 | 0.506 | 0.002 |

- Mauchly's sphericity test not significant for all ($p > 0.05$) → sphericity assumption met.
- Velocity and curvature: ANOVA $p > 0.05$ → **no significant difference** (participants did not adapt velocity/curvature to the planning method).
- **Distance:** $p \ll 0.001$ → significant. Pairwise paired $t$-tests (Table 6, Bonferroni): GT distance significantly **smaller** than the other two methods. No significant difference between HU and RVO.

**Table 6 — Pairwise distance comparisons ($p$, Bonferroni):**

| | GT | RVO |
|---|---|---|
| HU | < 0.001* | 0.009 |
| RVO | < 0.001* | — |

→ Human and RVO perceived as more cooperative/comfortable because both maintained a **greater distance**.

### 6.3 Remarks

- Distance and comfort are related. Notable: a human-like motion does **not** necessarily result in increased comfort (in line with [61]: humans prefer larger keeping distance but judge it unnatural).
- Possible reason: uncanny valley [55] — though no participant pointed in this direction; more likely comfort depends on cooperation + distance.
- Differences between HU and GT still exist (Fig. 14) but may be too small to be noticeable, possibly due to imperfections in the VR setup.
- Alternative setup: a robotic platform either remote-controlled (human) or motion-planner-controlled. But that introduces uncontrolled variables (different behavior toward a robot; impaired viewpoint of experimenter). VR rules these out.
- **Replanning time $\Delta t$:** Algorithm runs fluently up to **20 Hz** (two agents, 31 actions). Bottleneck = generating trajectories with the RRT path planner. Reduced frequency to **10 Hz** (Table 2) to stay comparable to RVO (which showed oscillating behavior if $\Delta t < 0.10$ s).
- With increasing number of agents, **calculation of Nash equilibria becomes the main bottleneck** (number of possible allocations increases exponentially). Implementation uses brute-force search; calculation time decreases significantly with an efficient Nash-equilibria search method [66, 88]. For large populations, mean field game theory [18, 42] may be more suitable.

---

## 7 Potential Bias in the Study Design

- Both studies set up like a Turing test, asking whether observed motions are based on human motion. Used a human avatar (rather than robotic-like) to make the task clear — possibly biased participants to consider motions human-generated.
- Believed not to affect performance of methods relative to each other, but a robotic-like agent might lower the percentage. Subsequent studies should consider this bias (e.g., add a robotic-like avatar in VR; videos could show simplistic moving shapes like triangles [12, 26]).
- Majority of participants male in both studies → possible bias (men may perceive motions/social norms differently than women).

---

## 8 Conclusions and Future Work

- Succeeded in devising a (multi-agent) motion planner that generates human-like trajectories — motions of artificial agent(s) are **indistinguishable from human motions**.
- Based on repeatedly playing a non-cooperative, static game and searching for **Nash equilibria**, which approximates human decision making during navigation [81].
- Two self-contained studies support human likeness: participants of both the online video study and the VR study could **not distinguish** GT from human motions, but **could** tell human motions apart from RVO or SF.
- High potential for robots navigating near/sharing workspaces with humans (museum guides, delivery robots); also computer animations (games, VR trainings).
- **Caveat:** Second study indicated humans feel slightly but noticeably **less comfortable** moving toward a GT-controlled agent. Motivates further investigation of the cost function and solution concept.
- Recently, Kuleshov and Schrijvers [41]: **inverse game theory** determines cost functions consistent with a given equilibrium allocation — a promising future direction. Learning-based approaches very promising.
- Future work: further experimental studies with a real robotic platform to evaluate efficiency and safety; clarify how humans judge motions differently in VR vs. facing a real robot.

**Compliance / Conflict of interest:** Dirk Wollherr receives funding from BMW Group and Siemens. Annemarie Turnwald is employee of BMW Group. Open Access (CC BY 4.0).

---

## Appendix: Implementation Details and Parameters

Implementation details for **RVO** (Table 7) and **SF** (Table 8). For RVO, the **RVO2 Library** (http://gamma.cs.unc.edu/RVO2/) was used; social forces based on [31]. An **elliptical specification** of pedestrian repulsion forces with symmetrical treatment of interacting agents reproduces human behavior best (also smoothest and most human-like by inspection).

### Table 2 — Parameters used by the game theoretic planner (GT)

| Parameter | Video | Virtual | Explanation |
|---|---|---|---|
| $\Delta t$ (s) | 0.10 | 0.10 | Time step, resolution, time used for replanning |
| $R$ (m) | 0.300 | 0.375 | Radius of an agent |
| $\mathcal{X}^{\text{goal}}$ (m²) | 0.30 × 1.0 | 0.30 × 0.50 | Size of the goal region in x and y (x × y) |
| $M_n$ | 16 | 31 | Maximum number of actions per agent in a game |
| $dt$ (s) | 0.05 | 0.05 | Numeric integrator time step |
| $\Gamma^{\min}$ (s) | [0.35, 0.65] | [0.35, 0.65] | Range for the lower bound of propagation duration $\delta t$ |
| $\Gamma^{\max}$ (s) | [0.75, 1.25] | [0.75, 1.25] | Range for the upper bound of propagation duration $\delta t$ |
| $w_n^{\min}$ (rad/s) | 0.10 | 0.10 | Minimum angular velocity of an agent |
| $w_n^{\max}$ (rad/s) | 0.50 | 0.55 | Maximum angular velocity of an agent |

### Table 7 — Parameters used for the reciprocal velocity obstacles (RVO) approach

| Parameter | Video | Virtual | Explanation |
|---|---|---|---|
| $\Delta t$ (s) | 0.10 | 0.05 | Time step, resolution, time used for replanning |
| maxNeighbors | 10 | 10 | Maximal number of other agents taken into account |
| maxSpeed (m/s) | 2.50 | 2.50 | Maximum speed of an agent |
| neighborDist (m) | 15 | 15 | Within this distance agents take other agents into account |
| $R$ (m) | 0.300 | 0.375 | Radius of an agent |
| timeHorizon (s) | 5.00 | 10.00 | Min. time for which velocities are safe w.r.t. other agents |
| timeHorizonObst (s) | 5.00 | 10.00 | Min. time for which velocities are safe w.r.t. static objects |

### Table 8 — Parameters used for the social forces (SF) approach

| Parameter | Video | Explanation |
|---|---|---|
| $\Delta t$ (s) | 0.10 | Time step, resolution, time used for replanning |
| neighborDist (m) | 15.00 | Within this distance agents take other agents/objects into account |
| $R$ (m) | 0.300 | Radius of an agent |
| $A$ | 4.30 | Parameter of the elliptical repulsive potential of pedestrian forces |
| $B$ | 1.07 | Parameter of the elliptical repulsive potential of pedestrian forces |
| $\lambda$ | 1 | Anisotropy parameter |

---

## Key Takeaways for Implementation

1. **Decoupled architecture** (Fig. 4): (a) parallel control-based RRT planners produce diverse trajectory sets per agent → (b) static non-cooperative game over those trajectories → (c) Nash-equilibria solving → (d) coordination (Pareto selection) → (e) trajectory combination $\epsilon^*$ → control series $\boldsymbol{\mu}_n^*$ applied for $\Delta t$.
2. **Replanned every $\Delta t$** (10–20 Hz feasible for 2 agents / 16–31 actions). Winning equilibrium memorized → seeds next loop's action set.
3. **Cost function:** $J_n = \hat{J}_n + \tilde{J}_n$ where $\hat{J}_n = \text{Length}(\boldsymbol{\tau}_n^m)$ and $\tilde{J}_n = \infty$ on collision, else $0$.
4. **Stand-still action** $\boldsymbol{\tau}_n^0$ added to each action set, cost between max trajectory cost and collision cost.
5. **Two reasoning modes:** all-controllable (Pareto-dominant Nash, random tie-break) vs. among-humans (match observed trajectories $\mathcal{T}^{\text{obs}}$ to previous equilibria, then to current set, via average Euclidean distance).
6. **Motion model:** discrete unicycle (Eq. 7), control inputs Eq. 8 with curvature factor $c = 1/2$.
7. **Libraries:** OMPL (RRT), V-REP/CoppeliaSim (simulation, ROS-compatible), RVO2 (baseline).
