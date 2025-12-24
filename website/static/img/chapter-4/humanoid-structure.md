# Humanoid Robot Structure Diagram

```mermaid
graph TD
    A[base_link<br/>Torso Base] --> B[torso_link<br/>Main Body]
    A --> E[left_thigh_link<br/>Left Leg Top]
    A --> F[right_thigh_link<br/>Right Leg Top]

    B --> C[head_link<br/>Head]
    B --> G[left_upper_arm_link<br/>Left Arm Top]
    B --> H[right_upper_arm_link<br/>Right Arm Top]

    G --> I[left_forearm_link<br/>Left Forearm]
    I --> J[left_hand_link<br/>Left Hand]

    H --> K[right_forearm_link<br/>Right Forearm]
    K --> L[right_hand_link<br/>Right Hand]

    E --> M[left_shin_link<br/>Left Shin]
    M --> N[left_foot_link<br/>Left Foot]

    F --> O[right_shin_link<br/>Right Shin]
    O --> P[right_foot_link<br/>Right Foot]

    style A fill:#FFD700,stroke:#333,stroke-width:2px
    style B fill:#87CEEB,stroke:#333,stroke-width:2px
    style C fill:#FFA07A,stroke:#333,stroke-width:2px
    style G fill:#87CEEB,stroke:#333,stroke-width:2px
    style H fill:#87CEEB,stroke:#333,stroke-width:2px
    style I fill:#87CEEB,stroke:#333,stroke-width:2px
    style K fill:#87CEEB,stroke:#333,stroke-width:2px
    style J fill:#FFA07A,stroke:#333,stroke-width:2px
    style L fill:#FFA07A,stroke:#333,stroke-width:2px
    style E fill:#98FB98,stroke:#333,stroke-width:2px
    style F fill:#98FB98,stroke:#333,stroke-width:2px
    style M fill:#98FB98,stroke:#333,stroke-width:2px
    style O fill:#98FB98,stroke:#333,stroke-width:2px
    style N fill:#DEB887,stroke:#333,stroke-width:2px
    style P fill:#DEB887,stroke:#333,stroke-width:2px
```

This diagram shows the kinematic structure of the minimal humanoid robot, with the base link at the center connecting to all major body parts: torso (with head and arms) and legs. Each link is labeled with its name and function.