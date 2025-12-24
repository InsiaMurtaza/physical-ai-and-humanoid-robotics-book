# ROS 2 Communication Patterns Overview

```mermaid
graph TB
    subgraph "ROS 2 Nodes"
        A[Node 1]
        B[Node 2]
        C[Node 3]
        D[Node N]
    end

    subgraph "Communication Mechanisms"
        E[Topics<br/>Publish/Subscribe]
        F[Services<br/>Request/Response]
        G[Actions<br/>Goal/Feedback/Result]
    end

    A -->|Publish| E
    B -->|Subscribe| E
    C -->|Service<br/>Call| F
    D -->|Service<br/>Server| F
    A <--> G
    B <--> G

    style A fill:#4CAF50,stroke:#388E3C,color:#fff
    style B fill:#2196F3,stroke:#0D47A1,color:#fff
    style C fill:#FF9800,stroke:#EF6C00,color:#fff
    style D fill:#9C27B0,stroke:#510165,color:#fff
    style E fill:#FFC107,stroke:#FF8F00,color:#000
    style F fill:#FFC107,stroke:#FF8F00,color:#000
    style G fill:#FFC107,stroke:#FF8F00,color:#000
```

This diagram shows the different communication patterns available in ROS 2. Nodes can communicate using topics for asynchronous messaging, services for synchronous request-response, and actions for long-running tasks with feedback.