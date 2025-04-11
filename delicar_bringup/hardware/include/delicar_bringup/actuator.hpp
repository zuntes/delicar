#ifndef DELICAR_HARDWARE_ACTUATOR_H
#define DELICAR_HARDWARE_ACTUATOR_H

#include <string>

class Actuator
{
  public:
    std::string name = "";
    std::string type = "";
    int enc = 0;
    double cmd = 0.0;
    double pos = 0.0;
    double vel = 0.0;
    double eff = 0.0;

    Actuator() = default;
    
    Actuator(const std::string &actuator_name, const std::string &actuator_type)
    {
      setup(actuator_name, actuator_type);
    }
    
    void setup(const std::string &actuator_name, const std::string &actuator_type)
    {
      name = actuator_name;
      type = actuator_type;
      
      // Initialize values
      enc = 0;
      cmd = 0.0;
      pos = 0.0;
      vel = 0.0;
      eff = 0.0;
    }
};


#endif // DELICAR_HARDWARE_ACTUATOR_H