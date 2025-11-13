# AlphaDesign System Flowcharts

## 1. System Architecture

```mermaid
graph LR
    A[alphadesign.py] --> B[Pipeline]
    B --> C[Genetic Algorithm]
    B --> D[Neural Network]
    B --> E[CFD Analysis]
    B --> F[Wing Generator]
    C --> E
    D --> C
    E --> F
```

## 2. Main Execution Flow

```mermaid
flowchart TD
    Start([Start]) --> Init[Initialize Components]
    Init --> Loop{More<br/>Generations?}
    Loop -->|Yes| Eval[Evaluate Population]
    Eval --> Select[Select & Breed]
    Select --> Guide[NN Guidance]
    Guide --> Train[Train NN]
    Train --> Loop
    Loop -->|No| Save[Save Results]
    Save --> End([End])
```

## 3. Single Generation

```mermaid
flowchart LR
    Pop[Population] --> CFD[CFD Analysis]
    CFD --> Fit[Fitness Scores]
    Fit --> Best[Find Best]
    Best --> Elite[Elite Selection]
    Elite --> Cross[Crossover]
    Cross --> Mut[Mutation]
    Mut --> NN[NN Tweaks]
    NN --> NewPop[New Population]
```

## 4. RL Flow (Policy-Value Network)

```mermaid
flowchart TD
    subgraph "Input State"
        S[Wing Parameters<br/>~50 values]
    end

    subgraph "Neural Network"
        S --> Shared[Shared Backbone<br/>3 Hidden Layers]
        Shared --> Policy[Policy Head<br/>Parameter Adjustments]
        Shared --> Value[Value Head<br/>Fitness Prediction]
    end

    subgraph "Actions & Rewards"
        Policy --> Tweak[Tweak Parameters<br/>±exploration noise]
        Tweak --> NewParams[New Wing Design]
        NewParams --> CFD[CFD Evaluation]
        CFD --> Reward[Actual Fitness]
    end

    subgraph "Learning"
        Reward --> Loss1[Policy Loss<br/>Improve adjustments]
        Reward --> Loss2[Value Loss<br/>Better prediction]
        Value --> Loss2
        Loss1 --> Update[Backprop<br/>Update Weights]
        Loss2 --> Update
    end

    Update --> Shared
```

## 5. RL Training Cycle

```mermaid
flowchart LR
    Collect[Collect Best<br/>Designs] --> Convert[Parameters<br/>→ Tensors]
    Convert --> Forward[Forward Pass<br/>Policy + Value]
    Forward --> CalcLoss[Calculate Loss<br/>Constraint + Performance]
    CalcLoss --> Backward[Backpropagation<br/>+ Grad Clip]
    Backward --> UpdateW[Update Weights<br/>AdamW + Cosine]
    UpdateW --> Apply[Apply to Next<br/>Generation]
```

## Key Components

| Component | Purpose |
|-----------|---------|
| **Genetic Algorithm** | Evolves wing designs through selection, crossover, mutation |
| **Policy Network** | Learns which parameter adjustments improve designs |
| **Value Network** | Predicts fitness score before expensive CFD |
| **CFD Analysis** | Evaluates aerodynamic performance from STL geometry |
| **Wing Generator** | Creates 3D STL models from parameters |
| **Constraints** | Enforces FIA 2024 F1 regulations |

## How It Works

1. **Initialize**: Create random population of wing designs
2. **Evaluate**: Run CFD analysis on each design → fitness scores
3. **Select**: Keep best designs (elites) + tournament selection
4. **Breed**: Crossover + mutation → new offspring
5. **RL Guidance**: Policy network suggests parameter tweaks based on learned patterns
6. **RL Training**: Value network learns to predict fitness, policy learns better adjustments
7. **Curriculum**: Early focus on constraints → later focus on performance
8. **Repeat**: Loop until max generations or time limit
9. **Output**: Best STL designs + trained RL agent

## Outputs

- `stl_outputs/generation_XXX_best_design.stl` - Best wing designs
- `checkpoints/checkpoint_gen_XXX.json` - Population snapshots
- `neural_networks/network_gen_XXX.pth` - NN weights
- `cfd_results/genX_indY_cfd_results.json` - Cached CFD data

