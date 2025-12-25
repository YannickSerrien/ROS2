/**
 * @file smart_pointers_demo.cpp
 * @brief Comprehensive demonstration of smart pointers in C++
 *
 * This example demonstrates:
 * - std::unique_ptr usage
 * - std::shared_ptr usage
 * - std::weak_ptr usage
 * - Smart pointers with polymorphism
 * - RAII pattern
 * - Common patterns for ROS2
 *
 * Compile: g++ -std=c++17 -Wall smart_pointers_demo.cpp -o smart_pointers_demo
 * Run: ./smart_pointers_demo
 */

#include <iostream>
#include <memory>
#include <vector>
#include <string>

// ============================================================================
// Example Classes
// ============================================================================

/**
 * @brief Base Sensor class demonstrating RAII
 */
class Sensor {
public:
    Sensor(const std::string& name) : name_(name) {
        std::cout << "[Sensor] " << name_ << " created\n";
    }

    virtual ~Sensor() {
        std::cout << "[Sensor] " << name_ << " destroyed\n";
    }

    virtual void read() const {
        std::cout << "[Sensor] Reading from " << name_ << "\n";
    }

    std::string get_name() const { return name_; }

protected:
    std::string name_;
};

/**
 * @brief GPS Sensor - derived class
 */
class GPSSensor : public Sensor {
public:
    GPSSensor() : Sensor("GPS") {}

    void read() const override {
        std::cout << "[GPS] Latitude: 37.7749, Longitude: -122.4194\n";
    }
};

/**
 * @brief IMU Sensor - derived class
 */
class IMUSensor : public Sensor {
public:
    IMUSensor() : Sensor("IMU") {}

    void read() const override {
        std::cout << "[IMU] Accel: (0.1, 0.2, 9.8), Gyro: (0.01, -0.02, 0.00)\n";
    }
};

/**
 * @brief Temperature Sensor - derived class
 */
class TempSensor : public Sensor {
public:
    TempSensor() : Sensor("Temperature") {}

    void read() const override {
        std::cout << "[Temp] Temperature: 23.5°C\n";
    }
};

// ============================================================================
// unique_ptr Examples
// ============================================================================

/**
 * @brief Demonstrates basic unique_ptr usage
 */
void demo_unique_ptr_basics() {
    std::cout << "\n=== unique_ptr Basics ===\n";

    // Create unique_ptr (preferred way)
    auto sensor = std::make_unique<Sensor>("Lidar");
    sensor->read();

    // Automatically destroyed when scope ends

    std::cout << "End of function scope\n";
}

/**
 * @brief Demonstrates unique_ptr ownership transfer
 */
void demo_unique_ptr_ownership() {
    std::cout << "\n=== unique_ptr Ownership Transfer ===\n";

    auto sensor1 = std::make_unique<GPSSensor>();
    std::cout << "sensor1 owns GPS\n";

    // Transfer ownership with std::move
    auto sensor2 = std::move(sensor1);
    std::cout << "Ownership transferred to sensor2\n";

    // sensor1 is now nullptr
    if (!sensor1) {
        std::cout << "sensor1 is nullptr\n";
    }

    sensor2->read();
}

/**
 * @brief Factory function returning unique_ptr
 */
std::unique_ptr<Sensor> create_sensor(const std::string& type) {
    if (type == "GPS") {
        return std::make_unique<GPSSensor>();
    } else if (type == "IMU") {
        return std::make_unique<IMUSensor>();
    } else if (type == "Temp") {
        return std::make_unique<TempSensor>();
    }
    return nullptr;
}

/**
 * @brief Demonstrates unique_ptr with polymorphism (common ROS2 pattern)
 */
void demo_unique_ptr_polymorphism() {
    std::cout << "\n=== unique_ptr Polymorphism ===\n";

    // Vector of unique_ptr to base class
    std::vector<std::unique_ptr<Sensor>> sensors;

    // Add different sensor types
    sensors.push_back(std::make_unique<GPSSensor>());
    sensors.push_back(std::make_unique<IMUSensor>());
    sensors.push_back(std::make_unique<TempSensor>());

    // Use polymorphically
    std::cout << "Reading from all sensors:\n";
    for (const auto& sensor : sensors) {
        sensor->read();
    }

    // All sensors automatically destroyed when vector goes out of scope
}

/**
 * @brief Demonstrates unique_ptr with factory pattern
 */
void demo_unique_ptr_factory() {
    std::cout << "\n=== unique_ptr Factory Pattern ===\n";

    auto gps = create_sensor("GPS");
    if (gps) {
        gps->read();
    }

    auto imu = create_sensor("IMU");
    if (imu) {
        imu->read();
    }
}

// ============================================================================
// shared_ptr Examples
// ============================================================================

/**
 * @brief Demonstrates basic shared_ptr usage
 */
void demo_shared_ptr_basics() {
    std::cout << "\n=== shared_ptr Basics ===\n";

    auto sensor1 = std::make_shared<Sensor>("Camera");
    std::cout << "sensor1 use_count: " << sensor1.use_count() << "\n";

    {
        // Create another shared_ptr sharing ownership
        auto sensor2 = sensor1;
        std::cout << "sensor1 use_count: " << sensor1.use_count() << "\n";
        std::cout << "sensor2 use_count: " << sensor2.use_count() << "\n";
    }  // sensor2 destroyed, count decreases

    std::cout << "sensor1 use_count: " << sensor1.use_count() << "\n";
}  // sensor1 destroyed, object deleted

/**
 * @brief Demonstrates shared_ptr in multiple owners scenario
 */
void demo_shared_ptr_multiple_owners() {
    std::cout << "\n=== shared_ptr Multiple Owners ===\n";

    auto shared_sensor = std::make_shared<GPSSensor>();

    std::vector<std::shared_ptr<Sensor>> owner1;
    std::vector<std::shared_ptr<Sensor>> owner2;

    // Multiple owners share the same sensor
    owner1.push_back(shared_sensor);
    owner2.push_back(shared_sensor);

    std::cout << "Shared sensor use_count: " << shared_sensor.use_count() << "\n";  // 3

    owner1.clear();
    std::cout << "After owner1 cleared: " << shared_sensor.use_count() << "\n";  // 2

    owner2.clear();
    std::cout << "After owner2 cleared: " << shared_sensor.use_count() << "\n";  // 1

    // Sensor still alive because shared_sensor owns it
    shared_sensor->read();
}

// ============================================================================
// weak_ptr Examples
// ============================================================================

/**
 * @brief Example classes for circular reference
 */
class Parent;

class Child {
public:
    Child(const std::string& name) : name_(name) {
        std::cout << "[Child] " << name_ << " created\n";
    }

    ~Child() {
        std::cout << "[Child] " << name_ << " destroyed\n";
    }

    void set_parent(std::shared_ptr<Parent> parent) {
        parent_ = parent;  // Use weak_ptr to avoid circular reference
    }

    std::string name_;
    std::weak_ptr<Parent> parent_;  // Weak reference!
};

class Parent {
public:
    Parent(const std::string& name) : name_(name) {
        std::cout << "[Parent] " << name_ << " created\n";
    }

    ~Parent() {
        std::cout << "[Parent] " << name_ << " destroyed\n";
    }

    void add_child(std::shared_ptr<Child> child) {
        children_.push_back(child);
        child->set_parent(shared_from_this());
    }

    std::string name_;
    std::vector<std::shared_ptr<Child>> children_;
};

/**
 * @brief Demonstrates weak_ptr breaking circular references
 */
void demo_weak_ptr() {
    std::cout << "\n=== weak_ptr Breaking Circular References ===\n";

    // Note: In real code, Parent would inherit from std::enable_shared_from_this
    auto parent = std::make_shared<Parent>("Parent1");
    auto child = std::make_shared<Child>("Child1");

    // Create relationship (no circular reference due to weak_ptr)
    parent->add_child(child);

    std::cout << "Parent use_count: " << parent.use_count() << "\n";
    std::cout << "Child use_count: " << child.use_count() << "\n";

    // Both will be properly destroyed
}

/**
 * @brief Demonstrates weak_ptr usage pattern
 */
void demo_weak_ptr_usage() {
    std::cout << "\n=== weak_ptr Usage Pattern ===\n";

    std::weak_ptr<Sensor> weak_sensor;

    {
        auto shared_sensor = std::make_shared<Sensor>("Radar");
        weak_sensor = shared_sensor;  // weak_ptr observes

        // To use weak_ptr, convert to shared_ptr
        if (auto locked = weak_sensor.lock()) {
            std::cout << "Sensor is alive\n";
            locked->read();
            std::cout << "Use count: " << locked.use_count() << "\n";
        }
    }  // shared_sensor destroyed

    // Now weak_ptr points to destroyed object
    if (weak_sensor.expired()) {
        std::cout << "Sensor has been destroyed\n";
    }

    if (auto locked = weak_sensor.lock()) {
        std::cout << "This won't execute\n";
    } else {
        std::cout << "Cannot lock weak_ptr - object is gone\n";
    }
}

// ============================================================================
// ROS2-Style Patterns
// ============================================================================

/**
 * @brief Simulates ROS2-style publisher pattern
 */
class SimplePublisher {
public:
    void publish(std::unique_ptr<std::string> message) {
        std::cout << "[Publisher] Publishing (moved): " << *message << "\n";
        // Message ownership transferred, will be deleted here
    }
};

/**
 * @brief Demonstrates ROS2-style message publishing with move semantics
 */
void demo_ros2_style_publishing() {
    std::cout << "\n=== ROS2-Style Publishing ===\n";

    SimplePublisher pub;

    // Create message with unique_ptr
    auto msg = std::make_unique<std::string>("Hello ROS2!");

    // Publish with move (zero-copy, transfers ownership)
    pub.publish(std::move(msg));

    // msg is now nullptr
    if (!msg) {
        std::cout << "Message ownership transferred\n";
    }
}

/**
 * @brief Simulates ROS2-style node with shared_ptr
 */
class SimpleNode : public std::enable_shared_from_this<SimpleNode> {
public:
    SimpleNode(const std::string& name) : name_(name) {
        std::cout << "[Node] " << name_ << " created\n";
    }

    ~SimpleNode() {
        std::cout << "[Node] " << name_ << " destroyed\n";
    }

    std::shared_ptr<SimpleNode> get_shared() {
        return shared_from_this();
    }

    std::string name_;
};

/**
 * @brief Demonstrates ROS2-style node usage
 */
void demo_ros2_style_node() {
    std::cout << "\n=== ROS2-Style Node ===\n";

    // Create node with shared_ptr (ROS2 style)
    auto node = std::make_shared<SimpleNode>("my_node");

    std::cout << "Node use_count: " << node.use_count() << "\n";  // 1

    // Get shared_ptr to self (common in ROS2)
    auto self = node->get_shared();

    std::cout << "Node use_count: " << node.use_count() << "\n";  // 2
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "========================================\n";
    std::cout << "Smart Pointers Demonstration\n";
    std::cout << "========================================\n";

    // unique_ptr examples
    demo_unique_ptr_basics();
    demo_unique_ptr_ownership();
    demo_unique_ptr_polymorphism();
    demo_unique_ptr_factory();

    // shared_ptr examples
    demo_shared_ptr_basics();
    demo_shared_ptr_multiple_owners();

    // weak_ptr examples
    demo_weak_ptr();
    demo_weak_ptr_usage();

    // ROS2-style patterns
    demo_ros2_style_publishing();
    demo_ros2_style_node();

    std::cout << "\n========================================\n";
    std::cout << "All demonstrations complete!\n";
    std::cout << "========================================\n";

    return 0;
}

/*
Expected Output:
========================================
Smart Pointers Demonstration
========================================

=== unique_ptr Basics ===
[Sensor] Lidar created
[Sensor] Reading from Lidar
End of function scope
[Sensor] Lidar destroyed

=== unique_ptr Ownership Transfer ===
[Sensor] GPS created
sensor1 owns GPS
Ownership transferred to sensor2
sensor1 is nullptr
[GPS] Latitude: 37.7749, Longitude: -122.4194
[Sensor] GPS destroyed

... (and so on)

========================================
All demonstrations complete!
========================================
*/
