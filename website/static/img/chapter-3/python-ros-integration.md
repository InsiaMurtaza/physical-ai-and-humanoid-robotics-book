# Python-ROS Integration Architecture

```mermaid
graph TB
    subgraph "Python Application Layer"
        A[Python AI/ML Agent]
        B[Numpy/Pandas<br/>Data Processing]
        C[Scikit-learn/<br/>TensorFlow]
    end

    subgraph "rclpy Bridge Layer"
        D[rclpy Python<br/>Client Library]
        E[ROS Message<br/>Conversion]
    end

    subgraph "ROS 2 Core"
        F[ROS 2 Nodes]
        G[Topics & Services]
        H[DDS Middleware]
    end

    subgraph "Robot Hardware Layer"
        I[Robot Controller]
        J[Sensors<br/>LIDAR, Camera, etc.]
        K[Actuators<br/>Motors, etc.]
    end

    A --> D
    B --> D
    C --> D
    D --> F
    E --> G
    F --> H
    G --> I
    J --> G
    K --> G

    style A fill:#4CAF50,stroke:#388E3C,color:#fff
    style D fill:#2196F3,stroke:#0D47A1,color:#fff
    style F fill:#FFC107,stroke:#FF8F00,color:#000
    style H fill:#9C27B0,stroke:#510165,color:#fff
    style I fill:#FF9800,stroke:#EF6C00,color:#fff
```

This diagram illustrates how Python-based AI agents connect to ROS 2 systems through the rclpy client library, enabling integration between Python AI/ML ecosystems and robotic hardware systems.