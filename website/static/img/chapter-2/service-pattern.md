# Service/Client Communication Pattern

```mermaid
graph LR
    A[Client Node] -->|Request| B[Service]
    B -->|Response| A
    B -->|Serves| C[Service Server]

    style A fill:#2196F3,stroke:#0D47A1,color:#fff
    style C fill:#4CAF50,stroke:#388E3C,color:#fff
    style B fill:#FFC107,stroke:#FF8F00,color:#000
```

The service/client pattern implements synchronous request-response communication. The client sends a request to the service and waits for a response. This pattern is ideal for operations that require a specific result.