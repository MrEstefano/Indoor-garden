# Indoor-garden
The code operates on Queuing architecture to manage water monitoring and irrigation. The system measures a soil moisture by capacitive sensor. The values are computed by PID alghoritm to calculate a pump runtime acordingly. The sampling rate of 2 minutes provides favorable soil condition to mantain healthy growth of the plant. The pump is active LOW. The pump is limited to 24h water supply allowance, which is monitored via flow sensor. The flow sensortriggers hardware interrup. The Countdown for each task currently running in a queue is displayed, same as soil moisture on a TFT display On the TFT display user can intercat with encoder push button, which is checked  evry time the Timer 1 overflows. There is a menu where user can acces and change values for soil moisture treshold setpoint, dayly water limit, PID coeficients, overwrite pump On or Off, reset to default 

FYI: Flow sensor coeficient was calibrated, same as  PID Coeficients there tuned using Ziegler Nichols 1 method, to suit the kind of soil    

![image](https://github.com/user-attachments/assets/c296bf6b-3757-4cc2-9958-aeb7355ce43b)
