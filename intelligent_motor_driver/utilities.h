enum mode_t 
    {
    IDLE,
    PWM,
    ITEST,
    HOLD,
    TRACK} mode;

int get_mode(void);
void set_mode(int);

