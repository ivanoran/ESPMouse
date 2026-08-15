#define BUTTON1_PIN 4
#define BUTTON2_PIN 2
#define BUTTON3_PIN 3
#define BUTTON4_PIN 5
#define BUTTON5_PIN 6
#define BUTTON6_PIN 7

enum button_func
{
    left,
    right,
    middle,
    pan,
    macro_1,
    macro_2
};

class Buttons
{
    public:

    Buttons()
    {        
        _buttons[button_func::left] = BUTTON1_PIN;
        _buttons[button_func::middle] = BUTTON2_PIN;
        _buttons[button_func::macro_1] = BUTTON3_PIN;
        _buttons[button_func::macro_2] = BUTTON4_PIN;
        _buttons[button_func::pan] = BUTTON5_PIN;
        _buttons[button_func::right] = BUTTON6_PIN;

    };

    void init()
    {
        pinMode(_buttons[0], INPUT_PULLUP);
        pinMode(_buttons[1], INPUT_PULLUP);
        pinMode(_buttons[2], INPUT_PULLUP);
        pinMode(_buttons[3], INPUT_PULLUP);
        pinMode(_buttons[4], INPUT_PULLUP);
        pinMode(_buttons[5], INPUT_PULLUP);
    };

    bool read(button_func button)
    {
        return !digitalRead(_buttons[button]);
    };

    void set_pin_to_func(button_func button, uint8_t pin) 
    {
        _buttons[button] = pin;
    };

    private:
    uint8_t _buttons[6];
};