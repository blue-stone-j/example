/*
create an alias for a member variable in a C++ class
*/

#include <iostream>

class Reference {
public:
    int memberVariable;

    // Constructor
    Reference(int value) : memberVariable(value), alias(memberVariable) {}

    // Alias as a reference
    int& alias;
};

class Using {
public:
    using AliasType = int; // Alias for int

    AliasType memberVariable;

    Using(int value) : memberVariable(value) {}
};

class Decltype {
public:
    int memberVariable;

    Decltype(int value) : memberVariable(value) {}

    // Alias using decltype(auto)
    decltype(auto) alias() { return (memberVariable); }
};

// Lazy Initialization with a Pointer
class Pointer {
public:
    std::vector<int>* alias; // Use a pointer for deferred initialization
    std::vector<int> memberVector;

    Pointer() : alias(nullptr) {} // Alias starts as nullptr

    void initialize(const std::vector<int>& vec) {
        memberVector = vec;
        alias = &memberVector; // Bind alias to memberVector after initialization
    }
};

class Optional {
public:
    std::optional<std::vector<int>> memberVector; // Optional for deferred initialization
    std::vector<int>* alias; // Pointer to memberVector

    Optional() : alias(nullptr) {}

    void initialize(const std::vector<int>& vec) {
        memberVector = vec;       // Initialize memberVector
        alias = &(*memberVector); // Bind alias to memberVector
    }
};

// Use a default dummy value
class DummyValue{
public:
    std::vector<int> memberVector;  // Always initialized
    std::vector<int>& alias;        // Reference to memberVector

    DummyValue() : memberVector({}), alias(memberVector) {} // Initialized with a dummy vector

    void initialize(const std::vector<int>& vec) {
        memberVector = vec; // Update memberVector
    }
};

// Use a setter method
class SetterMethod {
public:
    std::vector<int> memberVector;

    SetterMethod() {}

    void initialize(const std::vector<int>& vec) {
        memberVector = vec;
    }

    std::vector<int>& alias() {
        return memberVector; // Alias via function
    }
};

int main() {
    Reference obj(42);

    // Access via alias
    std::cout << "Original: " << obj.memberVariable << ", Alias: " << obj.alias << "\n";

    // Modify via alias
    obj.alias = 100;
    std::cout << "Modified: " << obj.memberVariable << ", Alias: " << obj.alias << "\n";

    Using obj2(42);
    std::cout << "Value: " << obj2.memberVariable << "\n";

    Decltype obj3(42);
    // Access and modify via alias
    obj3.alias() = 100;
    std::cout << "Modified: " << obj3.memberVariable << "\n";

    Pointer obj4;
    // Deferred initialization
    obj4.initialize({1, 2, 3});
    obj4.alias->push_back(4);
    for (int val : obj4.memberVector) {
        std::cout << val << " ";
    }

    Optional obj5;
    // Deferred initialization
    obj5.initialize({1, 2, 3});
    obj5.alias->push_back(4);
    for (int val : *(obj5.alias)) {
        std::cout << val << " ";
    }

    DummyValue obj6;
    // Initialize later
    obj6.initialize({1, 2, 3});
    obj6.alias.push_back(4);
    for (int val : obj6.alias) {
        std::cout << val << " ";
    }

    SetterMethod obj7;
    // Initialize later
    obj7.initialize({1, 2, 3});
    obj7.alias().push_back(4);
    for (int val : obj7.alias()) {
        std::cout << val << " ";
    }


    return 0;
}
