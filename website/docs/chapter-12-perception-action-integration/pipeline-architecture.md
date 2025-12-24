---
sidebar_position: 2
---

# Perception-to-Action Pipeline Architecture

## The Integration Challenge

Connecting perception systems to action systems involves several critical challenges:

**Latency Management**: Ensuring that action decisions can be made and executed within required time constraints.

**Data Flow Coordination**: Managing the flow of information from sensors through perception algorithms to action selection.

**Uncertainty Propagation**: Handling uncertainty in perception results when making action decisions.

**System Synchronization**: Coordinating the timing of perception, planning, and action execution cycles.

### Latency Management in Perception-Action Systems

Managing latency in perception-action systems is crucial for real-time robotic applications. Here's an example of how to design a low-latency pipeline:

```python
# Example of a low-latency perception-action pipeline
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from collections import deque
from threading import Lock

class LowLatencyPerceptionActionPipeline(Node):
    """
    A low-latency perception-action pipeline that minimizes processing delays
    """
    def __init__(self):
        super().__init__('low_latency_pipeline')

        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()

        # Create subscribers with QoS for real-time performance
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile)

        # Create publisher for robot commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Buffer for recent sensor data to handle processing delays
        self.image_buffer = deque(maxlen=2)  # Keep only latest 2 images
        self.scan_buffer = deque(maxlen=2)   # Keep only latest 2 scans
        self.buffer_lock = Lock()

        # Timing statistics
        self.processing_times = deque(maxlen=100)
        self.callback_times = deque(maxlen=100)

        # Performance monitoring timer
        self.perf_timer = self.create_timer(1.0, self.print_performance_stats)

        # Action selection parameters
        self.declare_parameter('max_processing_time_ms', 50)
        self.declare_parameter('control_frequency_hz', 10.0)
        self.declare_parameter('safety_distance', 0.5)

        # Control loop timer
        control_freq = self.get_parameter('control_frequency_hz').value
        self.control_timer = self.create_timer(1.0/control_freq, self.control_loop)

        self.get_logger().info('Low-latency perception-action pipeline initialized')

    def image_callback(self, msg):
        """Process incoming image data with minimal latency"""
        start_time = time.time()

        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Store in buffer with timestamp for temporal consistency
            with self.buffer_lock:
                self.image_buffer.append({
                    'image': cv_image,
                    'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    'msg_timestamp': time.time()
                })

        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')

        # Record callback processing time
        callback_time = (time.time() - start_time) * 1000  # Convert to ms
        self.callback_times.append(callback_time)

    def scan_callback(self, msg):
        """Process incoming laser scan data with minimal latency"""
        start_time = time.time()

        try:
            # Store scan data with timestamp
            with self.buffer_lock:
                self.scan_buffer.append({
                    'ranges': np.array(msg.ranges),
                    'intensities': np.array(msg.intensities),
                    'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    'msg_timestamp': time.time(),
                    'angle_min': msg.angle_min,
                    'angle_max': msg.angle_max,
                    'angle_increment': msg.angle_increment
                })

        except Exception as e:
            self.get_logger().error(f'Error in scan callback: {e}')

        # Record callback processing time
        callback_time = (time.time() - start_time) * 1000  # Convert to ms
        self.callback_times.append(callback_time)

    def control_loop(self):
        """Main control loop that processes perception data and generates actions"""
        start_time = time.time()

        # Get the most recent sensor data
        with self.buffer_lock:
            if not self.image_buffer or not self.scan_buffer:
                return  # No data available yet

            latest_image_data = self.image_buffer[-1]
            latest_scan_data = self.scan_buffer[-1]

        # Perform perception processing
        perception_results = self.process_perception(
            latest_image_data['image'],
            latest_scan_data
        )

        # Generate action based on perception results
        action_cmd = self.select_action(perception_results)

        # Publish the action command
        self.cmd_pub.publish(action_cmd)

        # Record processing time
        processing_time = (time.time() - start_time) * 1000  # Convert to ms
        self.processing_times.append(processing_time)

    def process_perception(self, image, scan_data):
        """Process sensor data to extract relevant information for action selection"""
        results = {
            'obstacles': [],
            'targets': [],
            'navigation_state': 'safe',
            'processing_time': 0
        }

        start_time = time.time()

        # Process laser scan for obstacle detection
        obstacles = self.detect_obstacles_from_scan(scan_data)
        results['obstacles'] = obstacles

        # Process image for target detection (simplified)
        targets = self.detect_targets_from_image(image)
        results['targets'] = targets

        # Determine navigation state based on obstacles
        if any(obs['distance'] < 0.5 for obs in obstacles):
            results['navigation_state'] = 'obstacle_detected'
        elif any(t['confidence'] > 0.8 for t in targets):
            results['navigation_state'] = 'target_detected'
        else:
            results['navigation_state'] = 'safe'

        results['processing_time'] = (time.time() - start_time) * 1000
        return results

    def detect_obstacles_from_scan(self, scan_data):
        """Detect obstacles from laser scan data"""
        obstacles = []

        # Filter valid range readings
        valid_ranges = scan_data['ranges'][(scan_data['ranges'] > 0.1) &
                                          (scan_data['ranges'] < 10.0)]

        # Find obstacles within safety distance
        for i, range_val in enumerate(valid_ranges):
            if range_val < 1.0:  # Consider anything closer than 1m as potential obstacle
                angle = scan_data['angle_min'] + i * scan_data['angle_increment']
                obstacles.append({
                    'distance': range_val,
                    'angle': angle,
                    'x': range_val * np.cos(angle),
                    'y': range_val * np.sin(angle)
                })

        return obstacles

    def detect_targets_from_image(self, image):
        """Detect targets from image data (simplified implementation)"""
        targets = []

        # Convert to HSV for color-based detection (example: detecting red objects)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for red color
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2

        # Find contours of detected objects
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter small contours
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2

                # Calculate relative position (simplified)
                image_center_x = image.shape[1] // 2
                relative_x = (center_x - image_center_x) / image_center_x  # -1 to 1

                targets.append({
                    'x': center_x,
                    'y': center_y,
                    'width': w,
                    'height': h,
                    'relative_x': relative_x,
                    'confidence': min(0.9, cv2.contourArea(contour) / 1000.0)  # Normalize confidence
                })

        return targets

    def select_action(self, perception_results):
        """Select appropriate action based on perception results"""
        cmd = Twist()

        if perception_results['navigation_state'] == 'obstacle_detected':
            # Stop or turn to avoid obstacle
            obstacles = perception_results['obstacles']
            closest_obstacle = min(obstacles, key=lambda x: x['distance'])

            if closest_obstacle['distance'] < 0.3:
                # Emergency stop
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            elif closest_obstacle['distance'] < 0.5:
                # Turn away from obstacle
                cmd.linear.x = 0.1  # Slow forward
                cmd.angular.z = -0.5 * np.sign(closest_obstacle['angle'])  # Turn away
            else:
                # Slow down approach
                cmd.linear.x = 0.2
                cmd.angular.z = 0.0

        elif perception_results['navigation_state'] == 'target_detected':
            # Navigate toward target
            targets = [t for t in perception_results['targets'] if t['confidence'] > 0.7]
            if targets:
                closest_target = max(targets, key=lambda x: x['confidence'])
                cmd.linear.x = 0.3  # Move forward toward target

                # Turn toward target if not centered
                if abs(closest_target['relative_x']) > 0.1:
                    cmd.angular.z = -0.8 * closest_target['relative_x']  # Proportional control

        else:
            # Safe navigation - move forward
            cmd.linear.x = 0.4
            cmd.angular.z = 0.0

        return cmd

    def print_performance_stats(self):
        """Print performance statistics"""
        if self.processing_times:
            avg_processing = np.mean(self.processing_times)
            max_processing = np.max(self.processing_times)
            min_processing = np.min(self.processing_times)

            self.get_logger().info(
                f'Performance - Avg: {avg_processing:.2f}ms, '
                f'Max: {max_processing:.2f}ms, Min: {min_processing:.2f}ms, '
                f'Count: {len(self.processing_times)}'
            )

def main(args=None):
    rclpy.init(args=args)
    pipeline = LowLatencyPerceptionActionPipeline()

    try:
        rclpy.spin(pipeline)
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Data Flow Coordination

Coordinating data flow between perception and action systems requires careful management of message passing and temporal consistency:

```python
# Example of data flow coordination system
from dataclasses import dataclass
from typing import Dict, Any, Optional
import threading
import time
from collections import defaultdict

@dataclass
class SensorData:
    """Container for sensor data with metadata"""
    timestamp: float
    data: Any
    sensor_type: str
    frame_id: str
    seq: int

@dataclass
class PerceptionResult:
    """Container for perception algorithm results"""
    timestamp: float
    detections: list
    features: Dict[str, Any]
    confidence: float
    processing_time: float

@dataclass
class ActionCommand:
    """Container for action commands"""
    timestamp: float
    command_type: str
    parameters: Dict[str, Any]
    priority: int

class DataFlowCoordinator:
    """
    Coordinates data flow between perception and action systems
    """
    def __init__(self):
        self.sensors = {}
        self.perception_results = {}
        self.action_queue = []
        self.data_lock = threading.Lock()

        # Buffer for sensor data
        self.sensor_buffer_size = 10
        self.sensor_buffers = defaultdict(lambda: [])

        # Buffer for perception results
        self.perception_buffer_size = 5
        self.perception_buffers = defaultdict(lambda: [])

        # Callbacks for data processing
        self.perception_callbacks = {}
        self.action_callbacks = {}

    def register_sensor(self, sensor_name: str, callback_func):
        """Register a sensor with its callback function"""
        self.sensors[sensor_name] = callback_func

    def register_perception_module(self, module_name: str, callback_func):
        """Register a perception module with its callback function"""
        self.perception_callbacks[module_name] = callback_func

    def register_action_module(self, module_name: str, callback_func):
        """Register an action module with its callback function"""
        self.action_callbacks[module_name] = callback_func

    def process_sensor_data(self, sensor_name: str, sensor_data: SensorData):
        """Process incoming sensor data"""
        with self.data_lock:
            # Add to sensor buffer
            buffer = self.sensor_buffers[sensor_name]
            buffer.append(sensor_data)

            # Maintain buffer size
            if len(buffer) > self.sensor_buffer_size:
                buffer.pop(0)

        # Call sensor-specific callback
        if sensor_name in self.sensors:
            self.sensors[sensor_name](sensor_data)

    def process_perception_result(self, module_name: str, result: PerceptionResult):
        """Process perception result"""
        with self.data_lock:
            # Add to perception buffer
            buffer = self.perception_buffers[module_name]
            buffer.append(result)

            # Maintain buffer size
            if len(buffer) > self.perception_buffer_size:
                buffer.pop(0)

        # Call perception-specific callback
        if module_name in self.perception_callbacks:
            self.perception_callbacks[module_name](result)

    def queue_action(self, action_cmd: ActionCommand):
        """Queue an action command for execution"""
        with self.data_lock:
            self.action_queue.append(action_cmd)
            # Sort by priority (higher priority first)
            self.action_queue.sort(key=lambda x: x.priority, reverse=True)

    def get_synchronized_data(self, sensor_names: list, max_time_diff: float = 0.1):
        """Get synchronized sensor data within time tolerance"""
        with self.data_lock:
            if not all(name in self.sensor_buffers for name in sensor_names):
                return None

            # Get latest data from each sensor
            latest_data = {}
            for name in sensor_names:
                if self.sensor_buffers[name]:
                    latest_data[name] = self.sensor_buffers[name][-1]
                else:
                    return None

            # Check temporal consistency
            timestamps = [data.timestamp for data in latest_data.values()]
            time_diff = max(timestamps) - min(timestamps)

            if time_diff > max_time_diff:
                # Find data within time tolerance
                ref_time = max(timestamps)
                synchronized_data = {}
                for name, data in latest_data.items():
                    # Look for data within tolerance
                    for buffered_data in reversed(self.sensor_buffers[name]):
                        if abs(buffered_data.timestamp - ref_time) <= max_time_diff:
                            synchronized_data[name] = buffered_data
                            break
                    else:
                        return None  # No synchronized data found

                return synchronized_data

            return latest_data

    def execute_next_action(self):
        """Execute the next action in the queue"""
        with self.data_lock:
            if self.action_queue:
                action = self.action_queue.pop(0)
                return action
            return None

class PerceptionActionNode(Node):
    """
    Example node that uses the DataFlowCoordinator
    """
    def __init__(self):
        super().__init__('perception_action_node')

        # Initialize coordinator
        self.coordinator = DataFlowCoordinator()

        # Register callbacks
        self.coordinator.register_sensor('camera', self.camera_callback)
        self.coordinator.register_sensor('lidar', self.lidar_callback)
        self.coordinator.register_perception_module('object_detector', self.detection_callback)
        self.coordinator.register_action_module('motion_controller', self.motion_callback)

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_sub_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_sub_callback, 10)

        # Create publisher for commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for processing loop
        self.process_timer = self.create_timer(0.1, self.process_loop)

    def image_sub_callback(self, msg):
        """Handle incoming image messages"""
        sensor_data = SensorData(
            timestamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            data=msg,
            sensor_type='camera',
            frame_id=msg.header.frame_id,
            seq=msg.header.seq
        )
        self.coordinator.process_sensor_data('camera', sensor_data)

    def scan_sub_callback(self, msg):
        """Handle incoming laser scan messages"""
        sensor_data = SensorData(
            timestamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            data=msg,
            sensor_type='lidar',
            frame_id=msg.header.frame_id,
            seq=msg.header.seq
        )
        self.coordinator.process_sensor_data('lidar', sensor_data)

    def camera_callback(self, sensor_data: SensorData):
        """Process camera data through perception pipeline"""
        # Convert to OpenCV and run object detection
        cv_image = self.bridge.imgmsg_to_cv2(sensor_data.data, 'bgr8')

        # Simulate object detection (in real implementation, use actual detector)
        detections = self.run_object_detection(cv_image)

        result = PerceptionResult(
            timestamp=time.time(),
            detections=detections,
            features={'image_shape': cv_image.shape},
            confidence=0.9,
            processing_time=0.05
        )

        self.coordinator.process_perception_result('object_detector', result)

    def lidar_callback(self, sensor_data: SensorData):
        """Process lidar data through perception pipeline"""
        # Process laser scan for obstacle detection
        obstacles = self.process_laser_scan(sensor_data.data)

        result = PerceptionResult(
            timestamp=time.time(),
            detections=obstacles,
            features={'scan_range': (min(sensor_data.data.ranges), max(sensor_data.data.ranges))},
            confidence=0.95,
            processing_time=0.02
        )

        self.coordinator.process_perception_result('lidar_processor', result)

    def process_loop(self):
        """Main processing loop"""
        # Get synchronized sensor data
        synchronized_data = self.coordinator.get_synchronized_data(['camera', 'lidar'])

        if synchronized_data:
            # Fusion of sensor data
            fused_result = self.fuse_sensor_data(synchronized_data)

            # Generate action based on fused data
            action_cmd = self.generate_action(fused_result)

            # Queue the action
            action_wrapper = ActionCommand(
                timestamp=time.time(),
                command_type='motion',
                parameters=action_cmd,
                priority=1
            )
            self.coordinator.queue_action(action_wrapper)

        # Execute next action
        next_action = self.coordinator.execute_next_action()
        if next_action:
            self.cmd_pub.publish(next_action.parameters)

    def run_object_detection(self, image):
        """Run object detection on image (simplified)"""
        # This would be replaced with actual object detection
        return [{'class': 'person', 'confidence': 0.85, 'bbox': [100, 100, 200, 200]}]

    def process_laser_scan(self, scan_msg):
        """Process laser scan for obstacles (simplified)"""
        # This would be replaced with actual obstacle detection
        return [{'type': 'obstacle', 'distance': 1.5, 'angle': 0.1}]

    def fuse_sensor_data(self, sensor_data):
        """Fuse data from multiple sensors"""
        # Implement sensor fusion logic
        return {
            'objects': sensor_data['camera'].data if 'camera' in sensor_data else [],
            'obstacles': sensor_data['lidar'].data if 'lidar' in sensor_data else [],
            'timestamp': time.time()
        }

    def generate_action(self, fused_data):
        """Generate action based on fused sensor data"""
        cmd = Twist()
        # Implement action selection logic
        return cmd
```

## Isaac's Integration Architecture

The Isaac platform addresses these challenges through several architectural approaches:

**Modular Design**: Clear separation of concerns between perception, planning, and action components while maintaining efficient communication.

**Hardware Acceleration**: GPU acceleration for both perception processing and action planning to reduce system latency.

**Standardized Interfaces**: Consistent message formats and communication patterns that facilitate component integration.

**Real-Time Optimization**: Hardware-accelerated optimization of perception-action loops for time-critical applications.

### Isaac's Hardware Acceleration Architecture

Here's an example of how Isaac implements hardware acceleration for perception-action pipelines:

```cpp
// Example Isaac hardware-accelerated perception-action pipeline
#include <cuda_runtime.h>
#include <tensorrt/infer.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>

class IsaacPerceptionActionPipeline {
public:
    IsaacPerceptionActionPipeline() {
        // Initialize CUDA context
        cudaSetDevice(0);

        // Initialize TensorRT engine for perception
        initTensorRTEngine();

        // Initialize GPU memory pools
        initMemoryPools();

        // Create processing streams
        initProcessingStreams();

        // Start processing threads
        startProcessingThreads();
    }

    void processImage(const cv::Mat& input_image) {
        // Wait for available buffer
        std::unique_lock<std::mutex> lock(buffer_mutex_);
        buffer_cv_.wait(lock, [this] { return available_buffers_.size() > 0; });

        // Get available buffer
        auto buffer_id = available_buffers_.front();
        available_buffers_.pop();
        lock.unlock();

        // Copy image to GPU memory
        copyImageToGPU(input_image, buffer_id);

        // Process image using TensorRT
        processImageWithTensorRT(buffer_id);

        // Queue for action selection
        {
            std::lock_guard<std::mutex> lock(action_queue_mutex_);
            action_queue_.push(buffer_id);
        }
        action_queue_cv_.notify_one();
    }

private:
    void initTensorRTEngine() {
        // Initialize TensorRT engine for object detection
        // This would load a pre-trained model optimized for GPU
        // For this example, we'll simulate the initialization
        std::cout << "Initializing TensorRT engine for perception..." << std::endl;
    }

    void initMemoryPools() {
        // Pre-allocate GPU memory for image processing
        const size_t image_size = 640 * 480 * 3; // 640x480 RGB
        const int num_buffers = 4; // Double buffer for pipelining

        for (int i = 0; i < num_buffers; ++i) {
            void* gpu_ptr;
            cudaMalloc(&gpu_ptr, image_size);
            gpu_buffers_.push_back(gpu_ptr);
            available_buffers_.push(i);
        }
    }

    void initProcessingStreams() {
        // Create CUDA streams for overlapping operations
        cudaStreamCreate(&copy_stream_);
        cudaStreamCreate(&compute_stream_);
        cudaStreamCreate(&action_stream_);
    }

    void startProcessingThreads() {
        // Start perception processing thread
        perception_thread_ = std::thread([this]() {
            while (running_) {
                processPerceptionStep();
            }
        });

        // Start action selection thread
        action_thread_ = std::thread([this]() {
            while (running_) {
                processActionStep();
            }
        });
    }

    void copyImageToGPU(const cv::Mat& image, int buffer_id) {
        // Copy image from CPU to GPU using dedicated stream
        cudaMemcpyAsync(
            gpu_buffers_[buffer_id],
            image.data,
            image.total() * image.elemSize(),
            cudaMemcpyHostToDevice,
            copy_stream_
        );
    }

    void processImageWithTensorRT(int buffer_id) {
        // Process image using TensorRT engine
        // This would run the actual neural network inference
        // For this example, we'll simulate the processing

        // Synchronize the compute stream
        cudaStreamSynchronize(compute_stream_);

        // In a real implementation, this would:
        // 1. Bind input tensor to gpu_buffers_[buffer_id]
        // 2. Execute inference
        // 3. Process output tensors
    }

    void processPerceptionStep() {
        // Wait for a processed buffer
        // In real implementation, this would wait for TensorRT completion
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Simulated processing time

        // Mark buffer as ready for action processing
        perception_results_.push(buffer_id);
        perception_cv_.notify_one();
    }

    void processActionStep() {
        // Wait for perception results
        std::unique_lock<std::mutex> lock(action_queue_mutex_);
        action_queue_cv_.wait(lock, [this] { return !action_queue_.empty(); });

        auto buffer_id = action_queue_.front();
        action_queue_.pop();
        lock.unlock();

        // Generate action based on perception results
        generateAction(buffer_id);

        // Return buffer to available pool
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            available_buffers_.push(buffer_id);
        }
        buffer_cv_.notify_one();
    }

    void generateAction(int buffer_id) {
        // Generate action command based on processed perception data
        // This would analyze the TensorRT output and select appropriate action
        std::cout << "Generating action for buffer " << buffer_id << std::endl;

        // In a real implementation, this would:
        // 1. Analyze detection results from TensorRT
        // 2. Apply decision-making logic
        // 3. Generate motion commands
        // 4. Publish commands via ROS 2
    }

    // GPU memory management
    std::vector<void*> gpu_buffers_;
    std::queue<int> available_buffers_;
    std::mutex buffer_mutex_;
    std::condition_variable buffer_cv_;

    // Processing streams
    cudaStream_t copy_stream_;
    cudaStream_t compute_stream_;
    cudaStream_t action_stream_;

    // Processing queues
    std::queue<int> action_queue_;
    std::mutex action_queue_mutex_;
    std::condition_variable action_queue_cv_;

    std::queue<int> perception_results_;
    std::mutex perception_mutex_;
    std::condition_variable perception_cv_;

    // Processing threads
    std::thread perception_thread_;
    std::thread action_thread_;
    std::atomic<bool> running_{true};
};
```

## Pipeline Patterns

**Sequential Pipeline**: Perception results feed into planning, which generates action commands in a linear sequence.

**Parallel Pipeline**: Perception and planning occur simultaneously with action execution, enabling faster response times.

**Feedback-Integrated Pipeline**: Action outcomes feed back into perception systems to improve future performance.

**Hierarchical Pipeline**: Multiple levels of perception and action coordination, from low-level reflexes to high-level planning.

### Sequential Pipeline Implementation

```python
# Example of sequential perception-action pipeline
class SequentialPerceptionActionPipeline:
    """
    A sequential pipeline where perception -> planning -> action in a linear sequence
    """
    def __init__(self):
        self.perception_system = PerceptionSystem()
        self.planning_system = PlanningSystem()
        self.action_system = ActionSystem()

        # Message queue for sequential processing
        self.perception_queue = []
        self.planning_queue = []
        self.action_queue = []

        # Processing locks
        self.perception_lock = threading.Lock()
        self.planning_lock = threading.Lock()
        self.action_lock = threading.Lock()

        # Start processing threads
        self.perception_thread = threading.Thread(target=self.perception_worker)
        self.planning_thread = threading.Thread(target=self.planning_worker)
        self.action_thread = threading.Thread(target=self.action_worker)

        self.running = True

        # Start threads
        self.perception_thread.start()
        self.planning_thread.start()
        self.action_thread.start()

    def add_sensor_data(self, sensor_data):
        """Add sensor data to the perception queue"""
        with self.perception_lock:
            self.perception_queue.append(sensor_data)

    def perception_worker(self):
        """Worker thread for perception processing"""
        while self.running:
            with self.perception_lock:
                if self.perception_queue:
                    sensor_data = self.perception_queue.pop(0)
                else:
                    sensor_data = None

            if sensor_data is not None:
                # Process perception
                perception_result = self.perception_system.process(sensor_data)

                # Add to planning queue
                with self.planning_lock:
                    self.planning_queue.append(perception_result)

            time.sleep(0.01)  # Small delay to prevent busy waiting

    def planning_worker(self):
        """Worker thread for planning"""
        while self.running:
            with self.planning_lock:
                if self.planning_queue:
                    perception_result = self.planning_queue.pop(0)
                else:
                    perception_result = None

            if perception_result is not None:
                # Generate plan
                plan = self.planning_system.generate_plan(perception_result)

                # Add to action queue
                with self.action_lock:
                    self.action_queue.append(plan)

            time.sleep(0.01)  # Small delay to prevent busy waiting

    def action_worker(self):
        """Worker thread for action execution"""
        while self.running:
            with self.action_lock:
                if self.action_queue:
                    plan = self.action_queue.pop(0)
                else:
                    plan = None

            if plan is not None:
                # Execute action
                self.action_system.execute(plan)

            time.sleep(0.01)  # Small delay to prevent busy waiting

class PerceptionSystem:
    """Simple perception system"""
    def process(self, sensor_data):
        # Simulate perception processing
        return {'objects': [], 'features': [], 'timestamp': time.time()}

class PlanningSystem:
    """Simple planning system"""
    def generate_plan(self, perception_result):
        # Simulate planning
        return {'actions': [], 'trajectory': [], 'timestamp': time.time()}

class ActionSystem:
    """Simple action system"""
    def execute(self, plan):
        # Simulate action execution
        pass
```

### Parallel Pipeline Implementation

```python
# Example of parallel perception-action pipeline
class ParallelPerceptionActionPipeline:
    """
    A parallel pipeline where perception and planning occur simultaneously
    """
    def __init__(self):
        self.perception_system = PerceptionSystem()
        self.planning_system = PlanningSystem()
        self.action_system = ActionSystem()

        # Shared data structures
        self.sensor_data_buffer = []
        self.perception_results_buffer = []
        self.planning_results_buffer = []

        # Thread synchronization
        self.sensor_buffer_lock = threading.Lock()
        self.perception_buffer_lock = threading.Lock()
        self.planning_buffer_lock = threading.Lock()

        # Start processing threads
        self.sensor_thread = threading.Thread(target=self.sensor_processor)
        self.perception_thread = threading.Thread(target=self.perception_processor)
        self.planning_thread = threading.Thread(target=self.planning_processor)
        self.action_thread = threading.Thread(target=self.action_processor)

        self.running = True

        # Start threads
        self.sensor_thread.start()
        self.perception_thread.start()
        self.planning_thread.start()
        self.action_thread.start()

    def add_sensor_data(self, sensor_data):
        """Add sensor data to the buffer"""
        with self.sensor_buffer_lock:
            self.sensor_data_buffer.append(sensor_data)

    def sensor_processor(self):
        """Process incoming sensor data"""
        while self.running:
            # Process sensor data and send to perception
            with self.sensor_buffer_lock:
                if self.sensor_data_buffer:
                    sensor_data = self.sensor_data_buffer.pop(0)
                else:
                    sensor_data = None

            if sensor_data is not None:
                # Add to perception system for parallel processing
                self.perception_system.process_async(sensor_data)

            time.sleep(0.005)  # High frequency sensor processing

    def perception_processor(self):
        """Process perception tasks in parallel"""
        while self.running:
            # Process perception tasks
            perception_result = self.perception_system.get_result()
            if perception_result is not None:
                # Add to planning buffer
                with self.planning_buffer_lock:
                    self.planning_results_buffer.append(perception_result)

            time.sleep(0.01)

    def planning_processor(self):
        """Process planning tasks in parallel"""
        while self.running:
            # Process planning tasks
            with self.planning_buffer_lock:
                if self.planning_results_buffer:
                    perception_result = self.planning_results_buffer.pop(0)
                else:
                    perception_result = None

            if perception_result is not None:
                # Generate plan in parallel
                plan = self.planning_system.generate_plan(perception_result)

                # Execute action
                self.action_system.execute_async(plan)

            time.sleep(0.01)

    def action_processor(self):
        """Process action execution"""
        while self.running:
            # Check for completed actions
            action_result = self.action_system.get_result()
            if action_result is not None:
                # Process action result if needed
                pass

            time.sleep(0.01)

# Async versions of systems for parallel processing
class AsyncPerceptionSystem:
    """Asynchronous perception system"""
    def __init__(self):
        self.task_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.worker_thread = threading.Thread(target=self.worker)
        self.worker_thread.start()

    def process_async(self, sensor_data):
        """Add a perception task to the queue"""
        self.task_queue.put(sensor_data)

    def get_result(self):
        """Get a completed perception result"""
        try:
            return self.result_queue.get_nowait()
        except queue.Empty:
            return None

    def worker(self):
        """Background worker for perception processing"""
        while True:
            try:
                sensor_data = self.task_queue.get(timeout=0.1)
                # Process perception
                result = {'processed_data': sensor_data, 'timestamp': time.time()}
                self.result_queue.put(result)
            except queue.Empty:
                continue

class AsyncPlanningSystem:
    """Asynchronous planning system"""
    def __init__(self):
        self.task_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.worker_thread = threading.Thread(target=self.worker)
        self.worker_thread.start()

    def generate_plan_async(self, perception_result):
        """Add a planning task to the queue"""
        self.task_queue.put(perception_result)

    def get_result(self):
        """Get a completed plan"""
        try:
            return self.result_queue.get_nowait()
        except queue.Empty:
            return None

    def worker(self):
        """Background worker for planning"""
        while True:
            try:
                perception_result = self.task_queue.get(timeout=0.1)
                # Generate plan
                plan = {'actions': ['move_forward'], 'timestamp': time.time()}
                self.result_queue.put(plan)
            except queue.Empty:
                continue

class AsyncActionSystem:
    """Asynchronous action system"""
    def __init__(self):
        self.task_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.worker_thread = threading.Thread(target=self.worker)
        self.worker_thread.start()

    def execute_async(self, plan):
        """Add an action task to the queue"""
        self.task_queue.put(plan)

    def get_result(self):
        """Get a completed action result"""
        try:
            return self.result_queue.get_nowait()
        except queue.Empty:
            return None

    def worker(self):
        """Background worker for action execution"""
        while True:
            try:
                plan = self.task_queue.get(timeout=0.1)
                # Execute action
                result = {'status': 'completed', 'timestamp': time.time()}
                self.result_queue.put(result)
            except queue.Empty:
                continue
```

These examples demonstrate different architectural approaches to perception-action integration, each with their own trade-offs in terms of latency, complexity, and resource utilization. The Isaac platform leverages these patterns while adding hardware acceleration to optimize performance.