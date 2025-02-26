
//
void oled_init (void);
void oled_clear(void);
void send_command (char);
void set_addressing (char, char);
void oled_wr_str_s(char, char, char*, char);
void oled_wr_str(char, char, char*, char);
void oled_bat (void);
void oled_voltage (int);
void oled_clear (void);

//
