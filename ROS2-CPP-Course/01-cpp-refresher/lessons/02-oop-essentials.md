# Lesson 2: OOP Essentials

## Learning Objectives

- Understand classes and objects in C++
- Master constructors and destructors
- Use inheritance and polymorphism
- Implement virtual functions
- Apply OOP concepts in ROS2 nodes

## Introduction

Object-Oriented Programming (OOP) is fundamental to ROS2. Every ROS2 node is a class, and understanding OOP is essential for building robotic systems. This lesson covers the OOP features you'll use constantly in ROS2 development.

## Classes and Objects

### Python vs C++ Classes

**Python:**
```python
class Robot:
    def __init__(self, name):
        self.name = name
        self.speed = 0.0

    def move(self, speed):
        self.speed = speed
        print(f"{self.name} moving at {speed}")

robot = Robot("R2D2")
robot.move(1.5)
```

**C++:**
```cpp
class Robot {
public:
    Robot(const std::string& name) : name_(name), speed_(0.0) {}

    void move(double speed) {
        speed_ = speed;
        std::cout << name_ << " moving at " << speed << std::endl;
    }

private:
    std::string name_;
    double speed_;
};

Robot robot("R2D2");
robot.move(1.5);
```

### Class Definition

```cpp
class Sensor {
public:
    // Constructor
    Sensor(const std::string& name) : name_(name) {
        std::cout << "Sensor " << name_ << " initialized\n";
    }

    // Destructor
    ~Sensor() {
        std::cout << "Sensor " << name_ << " destroyed\n";
    }

    // Public methods
    void read() const {
        std::cout << "Reading from " << name_ << "\n";
    }

    std::string getName() const { return name_; }

private:
    // Private member variables (trailing underscore convention)
    std::string name_;
    double last_reading_;
};
```

### Access Specifiers

```cpp
class MyClass {
public:
    // Accessible from anywhere
    void publicMethod() {}

protected:
    // Accessible from this class and derived classes
    void protectedMethod() {}

private:
    // Accessible only from this class
    void privateMethod() {}
    int private_data_;
};
```

## Constructors and Destructors

### Default Constructor

```cpp
class Point {
public:
    // Default constructor
    Point() : x_(0.0), y_(0.0) {
        std::cout << "Point created at origin\n";
    }

private:
    double x_, y_;
};

Point p;  // Calls default constructor
```

### Parameterized Constructor

```cpp
class Point {
public:
    // Parameterized constructor
    Point(double x, double y) : x_(x), y_(y) {
        std::cout << "Point created at (" << x_ << ", " << y_ << ")\n";
    }

private:
    double x_, y_;
};

Point p(3.0, 4.0);  // Calls parameterized constructor
```

### Member Initializer List (Preferred)

```cpp
class Robot {
public:
    // Good: Use initializer list
    Robot(const std::string& name, double speed)
        : name_(name), speed_(speed), active_(true) {
        // Body for additional initialization
    }

    // Bad: Assignment in body (less efficient)
    Robot(const std::string& name, double speed) {
        name_ = name;      // Calls default constructor, then assignment
        speed_ = speed;
        active_ = true;
    }

private:
    std::string name_;
    double speed_;
    bool active_;
};
```

### Destructor (RAII)

```cpp
class FileLogger {
public:
    FileLogger(const std::string& filename) {
        file_.open(filename);
        std::cout << "File opened\n";
    }

    ~FileLogger() {
        if (file_.is_open()) {
            file_.close();
            std::cout << "File closed\n";
        }
    }

    void log(const std::string& message) {
        file_ << message << std::endl;
    }

private:
    std::ofstream file_;
};

// Usage - automatic cleanup!
{
    FileLogger logger("log.txt");
    logger.log("Message");
}  // Destructor automatically closes file
```

## Inheritance

### Basic Inheritance

```cpp
// Base class
class Sensor {
public:
    Sensor(const std::string& name) : name_(name) {}

    void printName() const {
        std::cout << "Sensor: " << name_ << std::endl;
    }

protected:
    std::string name_;
};

// Derived class
class TemperatureSensor : public Sensor {
public:
    TemperatureSensor(const std::string& name)
        : Sensor(name), temperature_(0.0) {}

    void readTemperature() {
        temperature_ = 23.5;  // Simulated reading
        std::cout << name_ << " temperature: " << temperature_ << "°C\n";
    }

private:
    double temperature_;
};

// Usage
TemperatureSensor temp("DHT22");
temp.printName();         // Inherited method
temp.readTemperature();   // Own method
```

### Calling Base Constructor

```cpp
class Vehicle {
public:
    Vehicle(const std::string& model) : model_(model) {
        std::cout << "Vehicle created: " << model_ << "\n";
    }

protected:
    std::string model_;
};

class Car : public Vehicle {
public:
    // Call base constructor in initializer list
    Car(const std::string& model, int doors)
        : Vehicle(model), doors_(doors) {
        std::cout << "Car with " << doors_ << " doors\n";
    }

private:
    int doors_;
};

Car car("Tesla Model 3", 4);
// Output:
// Vehicle created: Tesla Model 3
// Car with 4 doors
```

## Polymorphism and Virtual Functions

### Virtual Functions

```cpp
class Shape {
public:
    virtual ~Shape() = default;  // Virtual destructor (important!)

    // Pure virtual function (abstract method)
    virtual double area() const = 0;

    // Virtual function with default implementation
    virtual void draw() const {
        std::cout << "Drawing shape\n";
    }
};

class Circle : public Shape {
public:
    Circle(double radius) : radius_(radius) {}

    // Override virtual function
    double area() const override {
        return 3.14159 * radius_ * radius_;
    }

    void draw() const override {
        std::cout << "Drawing circle with radius " << radius_ << "\n";
    }

private:
    double radius_;
};

class Rectangle : public Shape {
public:
    Rectangle(double width, double height)
        : width_(width), height_(height) {}

    double area() const override {
        return width_ * height_;
    }

    void draw() const override {
        std::cout << "Drawing rectangle " << width_ << "x" << height_ << "\n";
    }

private:
    double width_, height_;
};
```

### Polymorphic Usage

```cpp
void processShape(const Shape& shape) {
    shape.draw();
    std::cout << "Area: " << shape.area() << std::endl;
}

Circle circle(5.0);
Rectangle rect(4.0, 6.0);

processShape(circle);    // Calls Circle::draw() and Circle::area()
processShape(rect);      // Calls Rectangle::draw() and Rectangle::area()

// With pointers
std::vector<std::unique_ptr<Shape>> shapes;
shapes.push_back(std::make_unique<Circle>(3.0));
shapes.push_back(std::make_unique<Rectangle>(2.0, 4.0));

for (const auto& shape : shapes) {
    shape->draw();  // Polymorphic call
}
```

### Virtual Destructor (Critical!)

```cpp
class Base {
public:
    Base() { data_ = new int[100]; }

    // Without virtual: memory leak if deleted through base pointer!
    ~Base() { delete[] data_; }

    // Correct: virtual destructor
    virtual ~Base() { delete[] data_; }

private:
    int* data_;
};

class Derived : public Base {
public:
    Derived() { more_data_ = new int[100]; }
    ~Derived() { delete[] more_data_; }

private:
    int* more_data_;
};

// Problem without virtual destructor
Base* ptr = new Derived();
delete ptr;  // Only calls Base::~Base(), leaks Derived::more_data_!

// Solution: Make Base destructor virtual
```

## ROS2 Node Example

Every ROS2 node uses OOP principles:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// ROS2 node inherits from rclcpp::Node
class MyNode : public rclcpp::Node {
public:
    // Constructor
    MyNode() : Node("my_node"), count_(0) {
        // Initialize publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

        // Initialize timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyNode::timerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Node initialized");
    }

    // Destructor
    ~MyNode() {
        RCLCPP_INFO(this->get_logger(), "Node destroyed");
    }

private:
    // Private callback method
    void timerCallback() {
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Hello " + std::to_string(count_++);
        publisher_->publish(std::move(msg));
    }

    // Private member variables
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();  // Create node object
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Const Correctness

Mark methods that don't modify the object as `const`:

```cpp
class Point {
public:
    Point(double x, double y) : x_(x), y_(y) {}

    // Const methods (don't modify object)
    double getX() const { return x_; }
    double getY() const { return y_; }
    double distance() const {
        return std::sqrt(x_ * x_ + y_ * y_);
    }

    // Non-const methods (modify object)
    void setX(double x) { x_ = x; }
    void setY(double y) { y_ = y; }
    void move(double dx, double dy) {
        x_ += dx;
        y_ += dy;
    }

private:
    double x_, y_;
};

const Point p(3.0, 4.0);
double x = p.getX();    // OK: const method
// p.setX(5.0);         // ERROR: can't call non-const method on const object
```

## Best Practices

### Rule of Five

If you define any of these, define all five:

```cpp
class Resource {
public:
    // 1. Destructor
    ~Resource() { cleanup(); }

    // 2. Copy constructor
    Resource(const Resource& other) { copy(other); }

    // 3. Copy assignment
    Resource& operator=(const Resource& other) {
        if (this != &other) {
            cleanup();
            copy(other);
        }
        return *this;
    }

    // 4. Move constructor
    Resource(Resource&& other) noexcept { move(other); }

    // 5. Move assignment
    Resource& operator=(Resource&& other) noexcept {
        if (this != &other) {
            cleanup();
            move(other);
        }
        return *this;
    }

private:
    void cleanup() { /* release resources */ }
    void copy(const Resource& other) { /* copy */ }
    void move(Resource& other) { /* move and reset other */ }
};
```

### Or Use Rule of Zero

```cpp
// Prefer: Use smart pointers and STL containers (Rule of Zero)
class BetterResource {
public:
    // No need to define special members!
    // Compiler-generated ones work correctly
private:
    std::unique_ptr<Data> data_;
    std::vector<int> values_;
    std::string name_;
};
```

## Common Patterns

### Singleton Pattern

```cpp
class Logger {
public:
    // Get singleton instance
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    // Delete copy and move
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void log(const std::string& message) {
        std::cout << "[LOG] " << message << std::endl;
    }

private:
    // Private constructor
    Logger() = default;
};

// Usage
Logger::getInstance().log("Message");
```

### Factory Pattern

```cpp
class Sensor {
public:
    virtual ~Sensor() = default;
    virtual void read() = 0;
};

class GPSSensor : public Sensor {
public:
    void read() override { /* GPS reading */ }
};

class IMUSensor : public Sensor {
public:
    void read() override { /* IMU reading */ }
};

class SensorFactory {
public:
    static std::unique_ptr<Sensor> createSensor(const std::string& type) {
        if (type == "GPS") return std::make_unique<GPSSensor>();
        if (type == "IMU") return std::make_unique<IMUSensor>();
        return nullptr;
    }
};

// Usage
auto sensor = SensorFactory::createSensor("GPS");
sensor->read();
```

## Summary

**Key Takeaways:**
- Classes encapsulate data and behavior
- Constructors initialize, destructors clean up (RAII)
- Inheritance creates "is-a" relationships
- Virtual functions enable polymorphism
- Always use virtual destructors in base classes
- ROS2 nodes are classes inheriting from `rclcpp::Node`
- Use const correctness for better design

**Critical for ROS2:**
- Every node is a class
- Inheritance from `rclcpp::Node`
- Virtual functions for lifecycle callbacks
- RAII for resource management

## Practice Exercises

1. Create a sensor class hierarchy (Base + 3 derived types)
2. Implement a ROS2 node using OOP principles
3. Practice const correctness in getter methods

## What's Next?

- **Next Lesson**: [Pointers and References](03-pointers-references.md)
- **Related**: [Smart Pointers](04-smart-pointers.md) - Modern memory management

---

**Master OOP and you'll master ROS2 node design!**
