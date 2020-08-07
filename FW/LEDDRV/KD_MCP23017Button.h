class MCP23017Button 
{
  private:
    Adafruit_MCP23017 *_mcp_chip;
    bool _button_pin;
    bool released = true;
    bool inverted = false;
    
  public: 
    MCP23017Button(Adafruit_MCP23017 *mcp_chip, int _pin, bool _pullup, bool _inverted);
    MCP23017Button(void);
    int init(void);
    
    MCP23017Button(Adafruit_MCP23017 *mcp_chip, int _pin, bool _pullup, bool _inverted);
    MCP23017Button(Adafruit_MCP23017 *mcp_chip, int _pin, bool _pullup);
    MCP23017Button(Adafruit_MCP23017 *mcp_chip, int _pin);
    MCP23017Button(void) { }
    bool setDevice(Adafruit_MCP23017);
    void setPin(int);
    void setPullup(bool);
    void setInverted(bool);
    bool state(void);
    bool pressed(void);
    bool isReleased(void);
};
