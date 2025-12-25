/**
 * @file ex01-solution.cpp
 * @brief Solution for Exercise 1: Sensor Class Hierarchy
 *
 * Compile: g++ -std=c++17 -Wall ex01-solution.cpp -o ex01
 * Run: ./ex01
 */

#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <cstdlib>
#include <ctime>

// ============================================================================
// Base Sensor Class
// ============================================================================

class Sensor {
public:
    // Constructor
    Sensor(const std::string& name) : name_(name), is_active_(true) {
        std::cout << "Sensor '" << name_ << "' created\n";
    }

    // Virtual destructor (IMPORTANT!)
    virtual ~Sensor() {
        std::cout << "Sensor '" << name_ << "' destroyed\n";
    }

    // Pure virtual function - must be implemented by derived classes
    virtual double read() = 0;

    // Virtual function with default implementation
    virtual std::string getType() const {
        return "Generic";
    }

    // Regular member functions
    std::string getName() const {
        return name_;
    }

    bool isActive() const {
        return is_active_;
    }

    void setActive(bool active) {
        is_active_ = active;
    }

protected:
    std::string name_;
    bool is_active_;
};

// ============================================================================
// Temperature Sensor
// ============================================================================

class TemperatureSensor : public Sensor {
public:
    TemperatureSensor(const std::string& name) : Sensor(name) {}

    double read() override {
        // Simulate temperature reading between 20-30°C
        double temp = 20.0 + (std::rand() % 1000) / 100.0;  // 20.00 to 30.00
        return temp;
    }

    std::string getType() const override {
        return "Temperature";
    }
};

// ============================================================================
// Distance Sensor
// ============================================================================

class DistanceSensor : public Sensor {
public:
    DistanceSensor(const std::string& name) : Sensor(name) {}

    double read() override {
        // Simulate distance reading between 0-100 cm
        double distance = std::rand() % 10000 / 100.0;  // 0.00 to 100.00
        return distance;
    }

    std::string getType() const override {
        return "Distance";
    }
};

// ============================================================================
// Camera Sensor
// ============================================================================

class CameraSensor : public Sensor {
public:
    CameraSensor(const std::string& name) : Sensor(name) {}

    double read() override {
        // Simulate image capture (1.0 = captured, 0.0 = failed)
        return 1.0;  // Always successful for simplicity
    }

    std::string getType() const override {
        return "Camera";
    }
};

// ============================================================================
// Main Program
// ============================================================================

int main() {
    // Initialize random seed for sensor readings
    std::srand(std::time(nullptr));

    std::cout << "=== Sensor System Demo ===\n\n";

    // Create vector of sensors using smart pointers
    std::vector<std::unique_ptr<Sensor>> sensors;

    // Add different sensor types
    sensors.push_back(std::make_unique<TemperatureSensor>("temp1"));
    sensors.push_back(std::make_unique<DistanceSensor>("dist1"));
    sensors.push_back(std::make_unique<CameraSensor>("cam1"));

    std::cout << "\n=== Reading from all sensors ===\n\n";

    // Read from all sensors using polymorphism
    for (const auto& sensor : sensors) {
        std::cout << "Sensor: " << sensor->getName() << "\n";
        std::cout << "  Type: " << sensor->getType() << "\n";
        std::cout << "  Reading: " << sensor->read() << "\n";
        std::cout << "  Status: " << (sensor->isActive() ? "Active" : "Inactive") << "\n";
        std::cout << "\n";
    }

    std::cout << "=== Demonstrating deactivation ===\n\n";

    // Deactivate first sensor
    sensors[0]->setActive(false);

    // Check all sensors
    for (const auto& sensor : sensors) {
        std::cout << sensor->getName() << " is "
                  << (sensor->isActive() ? "ACTIVE" : "INACTIVE") << "\n";
    }

    std::cout << "\n=== Program ending (sensors auto-destroyed) ===\n";

    // Sensors are automatically destroyed when vector goes out of scope
    // Thanks to unique_ptr!

    return 0;
}

/*
Key Concepts Demonstrated:

1. Inheritance: TemperatureSensor, DistanceSensor, CameraSensor inherit from Sensor

2. Polymorphism: Different sensor types called through base class pointer/reference

3. Virtual Functions: read() and getType() can be overridden

4. Pure Virtual: read() MUST be implemented by derived classes

5. Virtual Destructor: Essential for proper cleanup through base class pointer

6. Const Correctness: Methods that don't modify use const

7. Smart Pointers: unique_ptr for automatic memory management

8. Protected Members: name_ and is_active_ accessible to derived classes

Output Example:
================
=== Sensor System Demo ===

Sensor 'temp1' created
Sensor 'dist1' created
Sensor 'cam1' created

=== Reading from all sensors ===

Sensor: temp1
  Type: Temperature
  Reading: 25.34
  Status: Active

Sensor: dist1
  Type: Distance
  Reading: 67.89
  Status: Active

Sensor: cam1
  Type: Camera
  Reading: 1.00
  Status: Active

=== Demonstrating deactivation ===

temp1 is INACTIVE
dist1 is ACTIVE
cam1 is ACTIVE

=== Program ending (sensors auto-destroyed) ===
Sensor 'temp1' destroyed
Sensor 'dist1' destroyed
Sensor 'cam1' destroyed
*/
