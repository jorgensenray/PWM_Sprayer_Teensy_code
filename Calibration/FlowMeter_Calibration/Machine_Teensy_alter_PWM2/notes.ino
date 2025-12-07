/*

added PinNum to outputs.ino  comment out digitalWrite statement
add PWMDefinitions.h to Machine_Teensy_alter_PWM  at the top
Add PWM_Setup(); to setup
add PWM_loop(); to loop

add to Machine_Teensy_alter_PWM2.ino
    uint16_t udpListenPort = 8888;  // General UDP listener port

    uint16_t udpSPRAYERListenPort7777 = 7777;  // Port to listen for Sprayer C# app
    uint16_t udpSPRAYERSendPort7777 = 7777;    // Port to send to Sprayer C# app

    uint16_t udpAOGListenPort9999 = 9999;  // AOG mirrored listener port - recieve 253 PGN where we get (actualSteerAngle)
    uint16_t udpSendPort = 9999;  
*/


