class MCP23017Button 
{
  private:
    Adafruit_MCP23017 *_mcp_chip;
    int _button_pin;
    int _command;
    bool _state_old;
    bool _state;

    bool released = true;
    bool inverted = false;
    
  public: 
    MCP23017Button(Adafruit_MCP23017 *mcp_chip, int buttonpin, int command);
    MCP23017Button(void);
    int init(void);
    int readstate(void);
    
    MCP23017Button(int, bool, bool);
    MCP23017Button(int, bool);
    MCP23017Button(int);
    MCP23017Button(void) { }
    void setPin(int _pin) { pin = _pin; }
    void setPullup(bool);
    void setInverted(bool _invert) { inverted = _invert; released = !state(); }
    bool pressed(void);
    bool state(void);
    bool isReleased(void);
};
