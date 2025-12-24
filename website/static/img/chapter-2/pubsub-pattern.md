# Publisher/Subscriber Communication Pattern

```mermaid
graph LR
    A[Publisher Node] -->|Publishes| B[Topic]
    B -->|Subscribes| C[Subscriber Node 1]
    B -->|Subscribes| D[Subscriber Node 2]
    B -->|Subscribes| E[Subscriber Node N]

    style A fill:#4CAF50,stroke:#388E3C,color:#fff
    style C fill:#2196F3,stroke:#0D47A1,color:#fff
    style D fill:#2196F3,stroke:#0D47A1,color:#fff
    style E fill:#2196F3,stroke:#0D47A1,color:#fff
    style B fill:#FFC107,stroke:#FF8F00,color:#000
```

The publisher/subscriber pattern allows for asynchronous communication where publishers send messages to topics without knowing which subscribers will receive them. Multiple subscribers can listen to the same topic, enabling broadcast communication.